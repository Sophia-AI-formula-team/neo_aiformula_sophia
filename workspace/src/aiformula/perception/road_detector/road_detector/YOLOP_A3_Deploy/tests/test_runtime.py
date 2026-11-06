import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from yolop_a3_runtime import PerceptionModel, SharedFPNPerception, Detection, ROS_TOPICS
from yolop_a3_runtime import RELEASE_NAME, __version__
from yolop_a3_runtime.outputs import render
from yolop_a3_runtime.color_semantics import classify_traffic_cone
from yolop_a3_runtime.temporal import ARCHITECTURE, INPUT_CONTRACT, load_model
from yolop_a3_runtime.lane_processing import LaneRuntimeConfig, postprocess_lane_probability


@pytest.fixture(scope="module")
def runtime():
    torch.set_num_threads(4)
    result = PerceptionModel(ROOT / "weights/shared_best.pth", device="cpu")
    result.model.base.detector.transform.min_size = (128,)
    result.model.base.detector.transform.max_size = 256
    return result


def test_public_entrypoint_is_current_shared_architecture(runtime):
    assert PerceptionModel is SharedFPNPerception
    assert runtime.model.base.detector.__class__.__name__ == "FCOS"
    assert "yolop_a3_runtime.model" not in sys.modules
    assert "yolop_a3_runtime.runtime" not in sys.modules


def test_single_encoder_visit_and_output_contract(runtime):
    calls = []
    hook = runtime.model.base.encoder[0].register_forward_hook(lambda *args: calls.append(1))
    image = np.zeros((90, 160, 3), np.uint8)
    before = image.copy()
    result = runtime.predict(image)
    hook.remove()
    assert calls == [1]
    assert np.array_equal(image, before)
    for mask in (result.lane_mask, result.obstacle_mask):
        assert mask.shape == (90, 160) and mask.dtype == np.uint8
        assert set(np.unique(mask)) <= {0, 255}
    assert result.combined_overlay.shape == image.shape
    assert result.traffic_light_state in {"red", "green", "unknown"}
    assert result.temporal_status == "single_frame"
    json.dumps(result.detections_payload(), allow_nan=False)


@pytest.mark.parametrize("image", [None, np.zeros((0, 5, 3), np.uint8),
                                    np.zeros((4, 5), np.uint8), np.zeros((4, 5, 3), np.float32)])
def test_invalid_input_rejected(runtime, image):
    with pytest.raises(ValueError):
        runtime.predict(image)


@pytest.mark.parametrize("threshold", [-.1, 1.1, float("nan")])
def test_invalid_threshold_rejected_before_loading_weights(threshold):
    with pytest.raises(ValueError):
        PerceptionModel("missing.pth", score_threshold=threshold)


def test_bundled_checkpoint_is_single_model_without_training_state():
    path = ROOT / "weights/shared_best.pth"
    manifest = json.loads((ROOT / "weights/manifest.json").read_text())
    assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest["sha256"]
    payload = torch.load(path, map_location="cpu", weights_only=True)
    assert set(payload) == {"architecture", "model", "input_contract", "provenance"}
    assert payload["architecture"] == ARCHITECTURE
    assert payload["input_contract"] == INPUT_CONTRACT
    assert payload["provenance"]["step"] == 1024
    assert payload["provenance"]["arm"] == "aligned"
    assert payload["provenance"]["source_deployment_candidate"] is False
    assert list((ROOT / "weights").glob("*.pth")) == [path]
    assert not any("teacher" in name for name in payload["model"])


def test_a4_identity_preserves_selected_weight_and_public_interfaces():
    import ast
    import xml.etree.ElementTree as ET
    manifest = json.loads((ROOT / "weights/manifest.json").read_text())
    assert RELEASE_NAME == manifest["release"] == "A4"
    assert __version__ == manifest["package_version"] == "4.0.0"
    assert manifest["tag"] == "a4" and manifest["previous_tag"] == "a3-v76"
    assert manifest["sha256"] == "b4d3556ded29ada232682b607a11296d723ffe74814a85cc4f2bcbf573e5b1e3"
    assert 'version = "4.0.0"' in (ROOT / "pyproject.toml").read_text()
    ros = ROOT / "ros2/yolop_a3_perception"
    package = ET.parse(ros / "package.xml").getroot()
    assert package.findtext("name") == "yolop_a3_perception"
    assert package.findtext("version") == __version__
    tree = ast.parse((ros / "setup.py").read_text())
    setup = next(node for node in ast.walk(tree) if isinstance(node, ast.Call)
                 and isinstance(node.func, ast.Name) and node.func.id == "setup")
    assert next(ast.literal_eval(v.value) for v in setup.keywords if v.arg == "version") == __version__
    assert PerceptionModel is SharedFPNPerception


def test_wrong_architecture_fails_closed(tmp_path):
    path = tmp_path / "old.pth"
    torch.save(dict(architecture="obsolete_ssd", model={}), path)
    with pytest.raises(ValueError, match="shared MobileNet"):
        load_model(path)


@pytest.mark.parametrize("contract", [None, {}, dict(INPUT_CONTRACT, min_size=320),
                                     dict(INPUT_CONTRACT, threshold=.4)])
def test_changed_or_missing_input_contract_fails_before_model_load(tmp_path, contract):
    path = tmp_path / "wrong_contract.pth"
    torch.save(dict(architecture=ARCHITECTURE, input_contract=contract, model={}), path)
    with pytest.raises(ValueError, match="input contract"):
        load_model(path)


def test_default_lane_output_keeps_raw_pixels_and_gaps():
    probability = np.zeros((32, 32), np.float32)
    probability[4:14, 15] = .7
    probability[17:28, 15] = .7
    probability[1, 1] = .5
    expected = (probability >= .5).astype(np.uint8)
    config = LaneRuntimeConfig()
    assert not config.postprocess_enabled
    assert np.array_equal(postprocess_lane_probability(probability, config), expected)


def test_bundled_input_transform_is_native_not_square():
    model = load_model(ROOT / "weights/shared_best.pth")
    assert model.base.detector.transform.min_size == (640,)
    assert model.base.detector.transform.max_size == 1152
    assert model.base.detector.transform.size_divisible == 32


def test_obstacle_mask_uses_boxes_and_excludes_lights():
    image = np.zeros((40, 80, 3), np.uint8)
    boxes = [(1, (2, 2, 10, 10)), (3, (12, 2, 20, 10)),
             (91, (22, 2, 30, 10)), (10, (50, 2, 60, 10))]
    detections = [Detection(box, category, "target", .9, "target") for category, box in boxes]
    _, mask = render(image, detections)
    assert all(mask[5, x] == 255 for x in (5, 15, 25))
    assert mask[5, 55] == 0
    assert ROS_TOPICS.obstacle_mask == "/cone_mask"
    assert ROS_TOPICS.object_overlay == "/cone_bbox_image"


@pytest.mark.parametrize("color,label", [((0, 0, 160), "red"), ((0, 160, 160), "yellow"),
                                        ((0, 50, 0), "green"), ((160, 0, 0), "blue")])
def test_cone_body_color_survives_reflective_band(color, label):
    image = np.full((100, 80, 3), color, np.uint8)
    image[25:65] = 240
    assert classify_traffic_cone(image, (0, 0, 80, 100)).label == label


def test_white_cone_on_green_background():
    image = np.full((100, 80, 3), (0, 140, 0), np.uint8)
    image[:, 20:60] = 230
    assert classify_traffic_cone(image, (0, 0, 80, 100)).label == "white"


def test_empty_and_dark_cones_are_unknown():
    image = np.zeros((30, 30, 3), np.uint8)
    for box in ((0, 0, 30, 30), (2, 2, 2, 2)):
        assert classify_traffic_cone(image, box).label == "unknown"
