"""Compare B1.4 CUDA settings on the same five frozen images."""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import torch

from concept_yolop.inference import Predictor
from concept_yolop.weights import default_path


class TensorTimings:
    def __init__(self, model):
        self.handles = []
        self.events = {}
        for name, layer in [
                ('encoder', model.encoder), ('road', model.road_head),
                ('lane', model.lane_head), ('pyramid', model.detector.backbone),
                ('detector_head', model.detector.head)]:
            if name == 'encoder':
                self.handles.append(layer[0].register_forward_pre_hook(self.enter(name)))
                self.handles.append(layer[-1].register_forward_hook(self.leave(name)))
            else:
                self.handles.append(layer.register_forward_pre_hook(self.enter(name)))
                self.handles.append(layer.register_forward_hook(self.leave(name)))

    def enter(self, name):
        def run(*_):
            a = torch.cuda.Event(enable_timing=True)
            b = torch.cuda.Event(enable_timing=True)
            self.events[name] = a, b
            a.record()
        return run

    def leave(self, name):
        def run(*_):
            self.events[name][1].record()
        return run

    def milliseconds(self):
        return {key: a.elapsed_time(b) for key, (a, b) in self.events.items()}


def median(values):
    return round(float(np.median(values)), 3)


def run(predictor, frames):
    timing = TensorTimings(predictor.model)
    outputs, elapsed, layers = [], [], []
    for frame in frames:
        begin = time.monotonic_ns()
        outputs.append(predictor.predict(frame))
        elapsed.append((time.monotonic_ns() - begin) / 1e6)
        layers.append(timing.milliseconds())
    for handle in timing.handles:
        handle.remove()
    return outputs, dict(predict_ms=[round(x, 3) for x in elapsed], median_ms=median(elapsed),
                         stages_median_ms={key: median([row[key] for row in layers]) for key in layers[0]})


def check(results, reference):
    same = dict(road_pixel_diff=0, lane_pixel_diff=0, frames_labels_differ=0,
                largest_box_diff=0., largest_score_diff=0.)
    for (_, road, lane, det), (_, expected_road, expected_lane, expected_det) in zip(results, reference):
        same['road_pixel_diff'] += int(np.count_nonzero(road != expected_road))
        same['lane_pixel_diff'] += int(np.count_nonzero(lane != expected_lane))
        if not np.array_equal(det['labels'], expected_det['labels']):
            same['frames_labels_differ'] += 1
        if len(det['boxes']) == len(expected_det['boxes']) and len(det['boxes']):
            for field, key in [('boxes', 'largest_box_diff'), ('scores', 'largest_score_diff')]:
                same[key] = max(same[key], float(np.max(np.abs(det[field] - expected_det[field]))))
    return same


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('frame_dir', type=Path)
    parser.add_argument('--modes', nargs='+', default=['stock', 'heuristic', 'autotune', 'autotune-heuristic'])
    args = parser.parse_args()
    frames = [cv2.imread(str(path)) for path in sorted(args.frame_dir.glob('*_rgb.png'))]
    if not frames:
        raise SystemExit('No frozen frames')
    print(json.dumps(dict(torch=torch.__version__, cudnn=torch.backends.cudnn.version(),
                          gpu=torch.cuda.get_device_name(), n_frames=len(frames))), flush=True)
    reference = None
    for mode in args.modes:
        predictor = Predictor(default_path())
        torch.backends.cudnn.deterministic = 'heuristic' not in mode
        torch.backends.cudnn.benchmark = 'autotune' in mode
        torch.backends.cudnn.allow_tf32 = 'tf32' in mode and 'seg' not in mode
        if 'native' in mode or 'tf32-seg' in mode:
            def wrap(original):
                def decode(*inputs):
                    with torch.backends.cudnn.flags(enabled='native' not in mode,
                                                    deterministic=True, allow_tf32='tf32-seg' in mode):
                        return original(*inputs)
                return decode

            for decoder in [predictor.model.road_head, predictor.model.lane_head]:
                layer = decoder.fuse[2][0][0] if 'single' in mode else decoder
                layer.forward = wrap(layer.forward)
        if 'nhwc' in mode:
            predictor.model.to(memory_format=torch.channels_last)

            def layout(_, inputs, output):
                batch, _ = output
                batch.tensors = batch.tensors.contiguous(memory_format=torch.channels_last)

            predictor.model.detector.transform.register_forward_hook(layout)
        if 'half' in mode:
            original_predict = predictor.predict

            def half_predict(image):
                with torch.autocast('cuda', dtype=torch.float16):
                    return original_predict(image)

            predictor.predict = half_predict
        predictor.predict(frames[0])
        outputs, result = run(predictor, frames)
        reference = outputs if reference is None else reference
        print(json.dumps(dict(mode=mode, **result, parity=check(outputs, reference))), flush=True)
        del predictor
        torch.cuda.empty_cache()


if __name__ == '__main__':
    main()
