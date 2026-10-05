import numpy as np
import pytest

from yolop_a2_runtime.lane_processing import LaneRuntimeConfig, preprocess_lane_image
from yolop_a2_runtime.runtime import DualPerceptionModel


@pytest.mark.parametrize(
    "image",
    (
        None,
        np.zeros((10, 10), dtype=np.uint8),
        np.zeros((10, 10, 3), dtype=np.float32),
    ),
)
def test_predict_rejects_invalid_image_contract(image: object) -> None:
    model = DualPerceptionModel.__new__(DualPerceptionModel)
    with pytest.raises(ValueError, match="uint8 HxWx3"):
        model.predict(image)


def test_predict_rejects_zero_height_before_model_execution() -> None:
    model = DualPerceptionModel.__new__(DualPerceptionModel)
    image = np.empty((0, 10, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match="dimensions must be positive"):
        model.predict(image)


def test_lane_preprocess_rejects_empty_image_with_context() -> None:
    image = np.empty((10, 0, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match=r"shape \(10, 0, 3\)"):
        preprocess_lane_image(image, LaneRuntimeConfig())
