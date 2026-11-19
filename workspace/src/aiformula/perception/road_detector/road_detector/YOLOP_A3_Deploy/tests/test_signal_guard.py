from pathlib import Path

import numpy as np
import pytest
import torch

from yolop_a3_runtime import PerceptionModel
from yolop_a3_runtime.color_semantics import classify_traffic_light, summarize_signal


@pytest.mark.parametrize("red_pixels, expected", [(9, "red"), (2, "unknown"), (0, "green")])
def test_green_background_cannot_outvote_red(red_pixels, expected):
    rgb = np.full((20, 30, 3), (0, 220, 0), dtype=np.uint8)
    rgb.reshape(-1, 3)[100:100 + red_pixels] = (0, 0, 230)
    evidence = classify_traffic_light(rgb, (0, 0, 30, 20))
    assert evidence.counts["green"] > evidence.counts["red"]
    assert evidence.pixel_count == red_pixels
    assert evidence.label == expected


def test_warm_amber_or_reflective_white_is_not_green():
    rgb = np.full((12, 18, 3), (0, 210, 0), dtype=np.uint8)
    rgb[4:7, 4:7] = (0, 170, 240)
    assert classify_traffic_light(rgb, (0, 0, 18, 12)).label == "unknown"
    rgb[:] = 245
    assert classify_traffic_light(rgb, (0, 0, 18, 12)).label == "unknown"


@pytest.mark.parametrize("labels, state", [
    ([], "unknown"), (["green"], "green"), (["green", "green"], "green"),
    (["green", "unknown"], "unknown"), (["green", "yellow"], "unknown"),
    (["green", "red", "unknown"], "red"),
])
def test_conflict_never_becomes_frame_green(labels, state):
    assert summarize_signal(labels) == state


@pytest.mark.parametrize("lean", [False, True])
def test_public_signal_payload_uses_red_evidence_not_green_background(monkeypatch, lean):
    weights = Path(__file__).resolve().parents[1] / "weights/shared_best.pth"
    runtime = PerceptionModel(weights, device="cpu", lean_runtime=lean)
    image = np.full((24, 40, 3), (0, 180, 0), dtype=np.uint8)
    image[5:8, 4:7] = (0, 0, 230)
    result = dict(boxes=torch.tensor([[1., 2., 18., 20.], [20., 2., 39., 20.]]),
                  labels=torch.tensor([10, 10]), scores=torch.tensor([.8, .8]))
    monkeypatch.setattr(runtime.model, "forward",
                        lambda _: ([torch.zeros(2, 24, 40)], [result]))
    output = runtime.predict(image)
    assert [d.traffic_light_state for d in output.detections] == ["red", "green"]
    assert output.detections_payload()["traffic_light_state"] == "red"
    assert output.red_pixels_count == 9
    assert not output.obstacle_mask.any()
