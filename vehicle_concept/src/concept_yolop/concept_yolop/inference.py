"""Sensor preprocessing and checkpoint load, independent of ROS."""
import inspect
from pathlib import Path

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


class Predictor:
    def __init__(self, path, device='cuda'):
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
        native_fusion = (self.device.type == 'cuda' and torch.backends.cudnn.version() == 8600
                         and torch.cuda.get_device_capability(self.device) == (8, 7))
        self.model = B14(checkpoint['model_config'], native_fusion=native_fusion)
        self.model.load_state_dict(checkpoint['model'], strict=True)
        self.model.to(self.device).eval()

    @torch.no_grad()
    def predict(self, bgr):
        frame = canonical_bgr(bgr)
        rgb = frame[:, :, ::-1].transpose(2, 0, 1).copy()
        road, lane, detections = self.model(torch.from_numpy(rgb).to(self.device).float() / 255)
        # The two-class argmax returns int64. Move one byte per mask pixel
        # across the device boundary and keep the original comparison and ties.
        return (frame, road.bool().cpu().numpy(), lane.bool().cpu().numpy(),
                {k: v.cpu().numpy() for k, v in detections.items()})
