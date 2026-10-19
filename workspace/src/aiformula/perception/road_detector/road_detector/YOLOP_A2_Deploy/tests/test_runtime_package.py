from pathlib import Path

import numpy as np

from yolop_a2_runtime.contracts import ROS_ENCODINGS, ROS_TOPICS
from yolop_a2_runtime.object_model import TRAFFIC_CONE_ID
from yolop_a2_runtime.runtime import Detection, render_object_outputs


def _detection(class_id: int, name: str) -> Detection:
    return Detection(
        box_xyxy=(10.0, 10.0, 40.0, 50.0),
        class_id=class_id,
        class_name=name,
        score=0.9,
        semantic_label=name.replace(" ", "_"),
    )


def test_object_avoidance_topics_match_reference_contract() -> None:
    assert ROS_TOPICS.object_overlay == "/cone_bbox_image"
    assert ROS_TOPICS.obstacle_mask == "/cone_mask"
    assert ROS_ENCODINGS["object_overlay"] == "bgr8"
    assert ROS_ENCODINGS["obstacle_mask"] == "mono8"


def test_obstacle_mask_separates_cones_from_traffic_lights() -> None:
    image = np.zeros((64, 64, 3), dtype=np.uint8)
    _, cone_mask = render_object_outputs(
        image,
        (_detection(TRAFFIC_CONE_ID, "traffic cone"),),
    )
    _, light_mask = render_object_outputs(image, (_detection(10, "traffic light"),))
    assert np.count_nonzero(cone_mask) > 1000
    assert np.count_nonzero(light_mask) == 0


def test_runtime_source_has_no_training_modules() -> None:
    package_root = Path(__file__).resolve().parents[1] / "yolop_a2_runtime"
    forbidden_names = ("train", "dataset", "loss", "optimizer", "evaluation")
    source_names = [path.stem.lower() for path in package_root.glob("*.py")]
    assert not any(
        forbidden in source_name
        for source_name in source_names
        for forbidden in forbidden_names
    )

    source = "\n".join(
        path.read_text(encoding="utf-8") for path in package_root.glob("*.py")
    )
    assert "torch.optim" not in source
    assert "torch.utils.data" not in source
    assert "save_object_checkpoint" not in source
    assert "trainable_object_parameters" not in source
