from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
import torch


IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(1, 1, 3)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(1, 1, 3)


@dataclass(frozen=True)
class LaneRuntimeConfig:
    image_size: tuple[int, int] = (640, 640)
    prediction_threshold: float = 0.5
    postprocess_enabled: bool = False
    vertical_close: bool = True
    vertical_kernel: tuple[int, int] = (3, 11)
    component_filter: bool = True
    minimum_component_area: int = 60
    minimum_component_height: int = 24
    maximum_component_aspect_ratio: float = 1.8
    bottom_touch_margin: int = 48


def preprocess_lane_image(
    image_bgr: np.ndarray,
    config: LaneRuntimeConfig,
) -> tuple[
    torch.Tensor,
    tuple[tuple[int, int], tuple[tuple[float, float], tuple[float, float]]],
]:
    if not isinstance(image_bgr, np.ndarray) or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        shape = getattr(image_bgr, "shape", None)
        raise ValueError(f"expected a BGR HxWx3 image, got shape {shape}")
    if image_bgr.shape[0] < 1 or image_bgr.shape[1] < 1:
        raise ValueError(f"expected a non-empty BGR image, got shape {image_bgr.shape}")

    original_shape = image_bgr.shape[:2]
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    letterboxed, ratio, pad = _letterbox(
        image_rgb,
        new_shape=config.image_size,
        scaleup=False,
    )
    normalized = letterboxed.astype(np.float32) / 255.0
    normalized = (normalized - IMAGENET_MEAN) / IMAGENET_STD
    tensor = torch.from_numpy(
        np.ascontiguousarray(normalized.transpose(2, 0, 1))
    ).unsqueeze(0)
    return tensor, (original_shape, (ratio, pad))


def lane_probability_from_logits(lane_logits: torch.Tensor) -> torch.Tensor:
    if lane_logits.ndim != 4:
        raise ValueError("lane logits must have shape [B, C, H, W]")
    if lane_logits.size(1) == 1:
        foreground_logits = lane_logits[:, 0]
    elif lane_logits.size(1) == 2:
        foreground_logits = lane_logits[:, 1] - lane_logits[:, 0]
    else:
        raise ValueError("lane logits must contain one or two channels")
    return torch.sigmoid(foreground_logits)


def restore_lane_map(
    array: np.ndarray | torch.Tensor,
    shape: tuple[tuple[int, int], tuple[tuple[float, float], tuple[float, float]]],
) -> np.ndarray:
    if isinstance(array, torch.Tensor):
        array = array.detach().cpu().numpy()
    if array.dtype == np.float16:
        array = array.astype(np.float32, copy=False)

    (original_height, original_width), (_, pad) = shape
    pad_width, pad_height = (int(round(value)) for value in pad)
    cropped = array[
        pad_height : array.shape[0] - pad_height,
        pad_width : array.shape[1] - pad_width,
    ]
    if cropped.size == 0:
        raise ValueError(
            f"lane restoration produced an empty crop: array={array.shape}, shape={shape}"
        )
    return cv2.resize(
        cropped,
        (original_width, original_height),
        interpolation=cv2.INTER_LINEAR,
    )


def postprocess_lane_probability(
    probability: np.ndarray,
    config: LaneRuntimeConfig,
) -> np.ndarray:
    binary_mask = (probability >= config.prediction_threshold).astype(np.uint8)
    if not config.postprocess_enabled:
        return binary_mask

    processed = binary_mask
    if config.vertical_close:
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, config.vertical_kernel)
        processed = cv2.morphologyEx(processed, cv2.MORPH_CLOSE, kernel)
    if not config.component_filter:
        return processed.astype(np.uint8)
    return _filter_lane_components(processed, config)


def overlay_lane_mask(
    image_bgr: np.ndarray,
    lane_mask: np.ndarray,
    *,
    color: tuple[int, int, int] = (0, 0, 255),
    alpha: float = 0.5,
) -> np.ndarray:
    overlay = image_bgr.copy()
    selected = lane_mask.astype(bool)
    if selected.any():
        color_array = np.array(color, dtype=np.float32)
        overlay[selected] = (
            overlay[selected].astype(np.float32) * (1.0 - alpha)
            + color_array * alpha
        )
    return overlay.astype(np.uint8)


def _letterbox(
    image: np.ndarray,
    *,
    new_shape: tuple[int, int],
    color: tuple[int, int, int] = (114, 114, 114),
    scaleup: bool,
) -> tuple[np.ndarray, tuple[float, float], tuple[float, float]]:
    shape = image.shape[:2]
    ratio_value = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:
        ratio_value = min(ratio_value, 1.0)
    ratio = (ratio_value, ratio_value)
    resized_shape = (
        int(round(shape[1] * ratio_value)),
        int(round(shape[0] * ratio_value)),
    )
    pad_width = (new_shape[1] - resized_shape[0]) / 2
    pad_height = (new_shape[0] - resized_shape[1]) / 2
    if shape[::-1] != resized_shape:
        image = cv2.resize(image, resized_shape, interpolation=cv2.INTER_AREA)
    top, bottom = int(round(pad_height - 0.1)), int(round(pad_height + 0.1))
    left, right = int(round(pad_width - 0.1)), int(round(pad_width + 0.1))
    image = cv2.copyMakeBorder(
        image,
        top,
        bottom,
        left,
        right,
        cv2.BORDER_CONSTANT,
        value=color,
    )
    return image, ratio, (pad_width, pad_height)


def _filter_lane_components(
    binary_mask: np.ndarray,
    config: LaneRuntimeConfig,
) -> np.ndarray:
    filtered = np.zeros_like(binary_mask, dtype=np.uint8)
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        binary_mask.astype(np.uint8),
        connectivity=8,
    )
    for component_index in range(1, component_count):
        y = stats[component_index, cv2.CC_STAT_TOP]
        width = stats[component_index, cv2.CC_STAT_WIDTH]
        height = stats[component_index, cv2.CC_STAT_HEIGHT]
        area = stats[component_index, cv2.CC_STAT_AREA]
        aspect_ratio = width / max(height, 1)
        touches_bottom = (
            y + height >= binary_mask.shape[0] - config.bottom_touch_margin
        )
        keep = area >= config.minimum_component_area and (
            height >= config.minimum_component_height
            or aspect_ratio <= config.maximum_component_aspect_ratio
            or touches_bottom
        )
        if keep:
            filtered[labels == component_index] = 1
    return filtered
