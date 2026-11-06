"""Temporal state, geometry and public API checks without a ROS installation."""
import sys
from pathlib import Path
import numpy as np
import pytest
import torch
from torch import nn

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from yolop_a3_runtime.temporal import TemporalPerception, ProbabilityFusion
from yolop_a3_runtime.geometry import sampling_grid, validate_matrix


class FixedBase(nn.Module):
    def forward(self, images, *, detect=True):
        image = images[0]
        return [image[:2]], [dict(scores=torch.tensor([.9]))] if detect else []


@pytest.fixture
def model():
    value = TemporalPerception()
    value.base = FixedBase()
    return value.eval()


@torch.inference_mode()
def test_raw_history_and_object_outputs(model):
    first = torch.rand(3, 12, 20)
    second = torch.rand_like(first)
    kwargs = dict(sequence="camera/calibration-1", plane_to_image=np.eye(3))
    lanes, objects = model.forward_frame(first, timestamp=0., **kwargs)
    assert torch.equal(lanes[0], first[:2]) and model.status == "warmup"
    lanes, following = model.forward_frame(second, timestamp=.1, **kwargs)
    assert model.status == "aligned"
    assert torch.equal(following[0]["scores"], objects[0]["scores"])
    assert torch.equal(model.previous, second[:2].softmax(0)[1][None, None])
    assert not torch.equal(lanes[0].softmax(0)[1], model.previous[0, 0])
    assert not model.previous.requires_grad


@pytest.mark.parametrize("change,status", [
    ({"sequence": "different"}, "sequence_reset"),
    ({"timestamp": 0.}, "timestamp_reset"),
    ({"timestamp": -.1}, "timestamp_reset"),
    ({"timestamp": .251}, "timestamp_reset"),
])
@torch.inference_mode()
def test_discontinuity_uses_exact_current_logits(model, change, status):
    image = torch.rand(3, 12, 20)
    kwargs = dict(sequence="a", timestamp=0., plane_to_image=np.eye(3))
    model.forward_frame(image, **kwargs)
    kwargs["timestamp"] = .1
    kwargs.update(change)
    lanes, _ = model.forward_frame(image, **kwargs)
    assert model.status == status and torch.equal(lanes[0], image[:2])


@torch.inference_mode()
def test_shape_reset_single_frame_and_explicit_reset(model):
    kwargs = dict(sequence="a", plane_to_image=np.eye(3))
    model.forward_frame(torch.rand(3, 12, 20), timestamp=0., **kwargs)
    image = torch.rand(3, 10, 20)
    lanes, _ = model.forward_frame(image, timestamp=.1, **kwargs)
    assert model.status == "shape_reset" and torch.equal(lanes[0], image[:2])
    model([image])
    assert model.previous is None and model.status == "single_frame"
    model.forward_frame(image, timestamp=.2, **kwargs)
    model.reset()
    assert model.previous is None and model.timestamp is None


@pytest.mark.parametrize("matrix", [None, [], np.zeros((3, 3)), np.full((3, 3), np.nan)])
def test_invalid_geometry_rejected(matrix):
    with pytest.raises(ValueError):
        validate_matrix(matrix)


@pytest.mark.parametrize("change", [{"sequence": ""}, {"timestamp": float("nan")}, {"plane_to_image": None}])
def test_invalid_frame_metadata_does_not_update_state(model, change):
    kwargs = dict(sequence="a", timestamp=0., plane_to_image=np.eye(3))
    kwargs.update(change)
    with pytest.raises(ValueError):
        model.forward_frame(torch.zeros(3, 12, 20), **kwargs)
    assert model.previous is None


def test_identity_warp_and_invalid_regions():
    image = torch.rand(1, 1, 12, 20)
    grid, valid = sampling_grid(np.eye(3), np.eye(3), 12, 20, device="cpu")
    result = ProbabilityFusion()(image, image, grid, valid)
    torch.testing.assert_close(result, image, atol=1e-7, rtol=1e-6)
    assert torch.equal(ProbabilityFusion()(image, image, grid, valid * 0), image)
    assert ProbabilityFusion()(image, enabled=False) is image
