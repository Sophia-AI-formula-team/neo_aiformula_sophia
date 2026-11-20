"""Sensor preprocessing and checkpoint load, independent of ROS."""
import inspect
import os
from pathlib import Path
import time
import warnings

import cv2
import numpy as np
import torch

from .model import B14
from .weights import verify


def canonical_bgr(image):
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('Expected an 8-bit BGR camera frame')
    h, w = image.shape[:2]
    if w * 9 != h * 16:
        raise ValueError('Camera image must match the calibrated 16:9 optical view')
    if (w, h) != (640, 360):
        image = cv2.resize(image, (640, 360), interpolation=cv2.INTER_AREA)
    return np.ascontiguousarray(image)


def native_fusion_channels(device, setting):
    if setting not in ('auto', 'stock', 'native112', 'native112_160'):
        raise ValueError('Expected cuda_fusion auto, stock, native112 or native112_160')
    if device.type != 'cuda' or setting == 'stock':
        return ()
    if setting != 'auto':
        return (112,) if setting == 'native112' else (112, 160)
    backend = (torch.backends.cudnn.version(), torch.cuda.get_device_capability(device))
    # Preserve the Orin setting and the lower-tail-latency option measured
    # with the full driver at camera cadence on this specific workstation.
    if backend == (8600, (8, 7)):
        return (112,)
    if backend == (8700, (8, 9)) and torch.cuda.get_device_name(device) == 'NVIDIA GeForce RTX 4070 Ti':
        return (112,)
    return ()


class FixedCudaGraph:
    """Replay the fixed-size network and leave detection selection on the caller stream."""

    def __init__(self, model, batch, native):
        self.device = batch.tensors.device
        self.batch = type(batch)(batch.tensors.clone(), list(batch.image_sizes))
        self.native = native
        self.graph = torch.cuda.CUDAGraph()
        caller = torch.cuda.current_stream(self.device)
        warmup = torch.cuda.Stream(device=self.device)
        warmup.wait_stream(caller)
        with torch.cuda.stream(warmup):
            for _ in range(3):
                model.forward_fixed(self.batch, native)
        caller.wait_stream(warmup)
        with torch.cuda.graph(self.graph, stream=warmup):
            self.fixed = model.forward_fixed(self.batch, native)
        caller.wait_stream(warmup)
        self.grid = model.detection_grid(self.batch, self.fixed[2])

    def matches(self, batch, native):
        inputs = batch.tensors
        static = self.batch.tensors
        return (inputs.shape == static.shape and inputs.stride() == static.stride()
                and inputs.dtype == static.dtype and inputs.device == static.device
                and batch.image_sizes == self.batch.image_sizes and native == self.native)

    def predict(self, model, batch, native):
        self.batch.tensors.copy_(batch.tensors)
        self.graph.replay()
        return model.decode_detections(batch, native, self.fixed, self.grid)


class Predictor:
    def __init__(self, path, device='cuda', cuda_fusion='auto'):
        self.device = torch.device(device)
        torch.set_num_threads(4)
        torch.backends.cudnn.benchmark = False
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.allow_tf32 = False
        torch.backends.cuda.matmul.allow_tf32 = False
        cv2.setNumThreads(1)
        kwargs = {'weights_only': True} if 'weights_only' in inspect.signature(torch.load).parameters else {}
        with Path(path).expanduser().open('rb') as stream:
            verify(stream)
            checkpoint = torch.load(stream, map_location='cpu', **kwargs)
        if checkpoint['architecture'] != 'yolop_b1_honda_three_head_v2':
            raise ValueError('Unexpected checkpoint architecture')
        self.native_fusion_channels = native_fusion_channels(self.device, cuda_fusion)
        self.model = B14(checkpoint['model_config'],
                         native_fusion=112 in self.native_fusion_channels,
                         native_deep_fusion=160 in self.native_fusion_channels)
        self.model.load_state_dict(checkpoint['model'], strict=True)
        self.model.to(self.device).eval()
        self._try_cuda_graph = self.device.type == 'cuda' and os.environ.get('CONCEPT_CUDA_GRAPH') == '1'
        self._cuda_graph = None
        self.cuda_graph_error = None

    @property
    def cuda_graph_active(self):
        return self._cuda_graph is not None

    def _forward(self, tensor):
        if not self._try_cuda_graph and self._cuda_graph is None:
            return self.model(tensor)
        native = tensor.shape[-2:]
        batch, _ = self.model.detector.transform([tensor])
        if self._try_cuda_graph:
            self._try_cuda_graph = False
            try:
                self._cuda_graph = FixedCudaGraph(self.model, batch, native)
            except (RuntimeError, AttributeError, TypeError) as exc:
                self.cuda_graph_error = str(exc)
                warnings.warn('Concept CUDA graph unavailable; using eager inference: %s' % exc,
                              RuntimeWarning)
        if self._cuda_graph is not None and self._cuda_graph.matches(batch, native):
            return self._cuda_graph.predict(self.model, batch, native)
        return self.model.decode_detections(batch, native, self.model.forward_fixed(batch, native))

    @torch.no_grad()
    def predict(self, bgr, timings=None):
        if timings is not None:
            began = time.perf_counter_ns()
        frame = canonical_bgr(bgr)
        rgb = frame[:, :, ::-1].transpose(2, 0, 1).copy()
        host = torch.from_numpy(rgb)
        if timings is not None:
            prepared = time.perf_counter_ns()
        tensor = host.to(self.device)
        if timings is not None:
            uploaded = time.perf_counter_ns()
        tensor = tensor.float() / 255
        if timings is not None:
            events = None
            if self.device.type == 'cuda':
                events = tuple(torch.cuda.Event(enable_timing=True) for _ in range(2))
                events[0].record()
            forward_began = time.perf_counter_ns()
            forward_cpu_began = time.thread_time_ns()
        road, lane, detections = self._forward(tensor)
        if timings is not None:
            forward_ended = time.perf_counter_ns()
            forward_cpu_ended = time.thread_time_ns()
            if events is not None:
                events[1].record()
        # The two-class argmax returns int64. Move one byte per mask pixel
        # across the device boundary and keep the original comparison and ties.
        output = (frame, road.bool().cpu().numpy(), lane.bool().cpu().numpy(),
                  {k: v.cpu().numpy() for k, v in detections.items()})
        if timings is not None:
            finished = time.perf_counter_ns()
            timings.update(inference_host_prepare=(prepared - began) / 1e6,
                           inference_input_upload=(uploaded - prepared) / 1e6,
                           inference_forward_wall=(forward_ended - forward_began) / 1e6,
                           inference_forward_thread_cpu=(forward_cpu_ended - forward_cpu_began) / 1e6,
                           inference_output_copy_and_wait=(finished - forward_ended) / 1e6)
            # The existing blocking result copies complete work on this stream.
            if events is not None:
                timings['inference_forward_cuda_stream'] = events[0].elapsed_time(events[1])
                timings['inference_forward_cuda_graph'] = int(self.cuda_graph_active)
        return output
