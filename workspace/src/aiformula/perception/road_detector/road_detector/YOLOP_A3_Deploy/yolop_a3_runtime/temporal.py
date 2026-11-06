"""Pointwise convex fusion of current and past floating lane probabilities."""
import math
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F
from .geometry import sampling_grid, validate_matrix
from .shared_fcos import SharedPerception

ARCHITECTURE = "a3_shared_fcos_temporal_probability_v76"
MODEL_CONFIG = dict(min_size=640, max_size=1152, width=32, depth=2)
INPUT_CONTRACT = dict(version=2, name="a3_v76_native", **MODEL_CONFIG,
                      size_divisible=32, resize="torch_bilinear", input="uint8_bgr",
                      padding="bottom_right_zero_normalized", normalize="imagenet",
                      restore="torch_logits_crop_resize_align_corners_false", threshold=.5,
                      temporal="previous_raw_probability", max_gap_seconds=.25,
                      precision="float32", near=.05, far=5000.)


class ProbabilityFusion(nn.Module):
    def __init__(self):
        super().__init__()
        self.gate = nn.Sequential(nn.Conv2d(4, 8, 1), nn.ReLU(), nn.Conv2d(8, 1, 1))
        nn.init.zeros_(self.gate[-1].weight)
        nn.init.constant_(self.gate[-1].bias, -4.)

    def forward(self, current, previous=None, grid=None, valid=None, enabled=True):
        if not enabled or previous is None:
            return current
        if current.ndim != 4 or current.shape[1] != 1 or previous.shape != current.shape:
            raise ValueError('Matching B1HW probability tensors required')
        batch, _, height, width = current.shape
        if grid is None or grid.shape != (batch, height, width, 2) or valid is None or valid.shape != current.shape:
            raise ValueError('Matching BHW2 sampling grid and B1HW validity required')
        warped = F.grid_sample(previous.detach(), grid, mode='bilinear',
                               padding_mode='zeros', align_corners=False)
        inputs = torch.cat((current, warped, warped - current, valid), dim=1)
        weight = self.gate(inputs).sigmoid() * valid
        return current + weight * (warped - current)


class TemporalPerception(nn.Module):
    """One camera stream per instance; caller supplies capture-time geometry."""
    def __init__(self):
        super().__init__()
        self.base = SharedPerception(**MODEL_CONFIG)
        self.memory = ProbabilityFusion()
        self.reset()

    def reset(self):
        self.previous = self.previous_plane = None
        self.sequence = self.timestamp = None
        self.status = "single_frame"

    def forward(self, images, *, detect=True):
        # Mixing independent images into a stream must never reuse stale history.
        self.reset()
        return self.base(images, detect=detect)

    def forward_frame(self, image, *, sequence, timestamp, plane_to_image, detect=True):
        if not isinstance(sequence, str) or not sequence:
            raise ValueError("sequence must identify a nonempty camera/calibration epoch")
        if not math.isfinite(timestamp):
            raise ValueError("Finite capture timestamp required")
        plane = validate_matrix(plane_to_image)
        status = "warmup"
        if self.previous is not None:
            if sequence != self.sequence:
                status = "sequence_reset"
            elif not 0 < timestamp - self.timestamp <= .25:
                status = "timestamp_reset"
            elif tuple(image.shape[-2:]) != tuple(self.previous.shape[-2:]):
                status = "shape_reset"
            else:
                status = "aligned"
        lanes, objects = self.base([image], detect=detect)
        raw = lanes[0].softmax(0)[1][None, None]
        if status == "aligned":
            grid, valid = sampling_grid(self.previous_plane, plane, *raw.shape[-2:], device=raw.device)
            fused = self.memory(raw, self.previous, grid, valid).clamp(1e-7, 1 - 1e-7)[0, 0]
            lanes = [torch.stack((torch.log1p(-fused), fused.log()))]
        # Store raw, not fused probability: no recursive persistence of old errors.
        self.previous = raw.detach()
        self.previous_plane = np.array(plane, copy=True)
        self.sequence, self.timestamp, self.status = sequence, timestamp, status
        return lanes, objects


def load_model(path, device="cpu"):
    saved = torch.load(path, map_location="cpu", weights_only=True)
    if saved.get("architecture") != ARCHITECTURE:
        raise ValueError("Expected the V76 shared MobileNet/FCOS temporal model")
    if saved.get("input_contract") != INPUT_CONTRACT:
        raise ValueError("V76 input contract mismatch; runtime and weights must be updated together")
    if set(saved) != {"architecture", "input_contract", "model", "provenance"}:
        raise ValueError("Expected a model-only deployment checkpoint")
    model = TemporalPerception()
    model.load_state_dict(saved["model"], strict=True)
    return model.to(device).eval()
