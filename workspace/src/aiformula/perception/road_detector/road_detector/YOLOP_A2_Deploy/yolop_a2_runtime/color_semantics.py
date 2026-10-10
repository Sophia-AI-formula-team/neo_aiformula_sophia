from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class ColorEvidence:
    label: str
    pixel_count: int
    counts: dict[str, int]


def classify_traffic_light(
    image_bgr: np.ndarray,
    box_xyxy: tuple[float, float, float, float],
) -> ColorEvidence:
    crop = _box_crop(image_bgr, box_xyxy)
    counts = _hsv_counts(crop)
    candidates = {name: counts[name] for name in ("red", "yellow", "green")}
    label, pixel_count = max(candidates.items(), key=lambda item: item[1])
    minimum = max(3, int(crop.shape[0] * crop.shape[1] * 0.002))
    if pixel_count < minimum:
        return ColorEvidence("unknown", counts["red"], counts)
    return ColorEvidence(label, counts["red"], counts)


def classify_traffic_cone(
    image_bgr: np.ndarray,
    box_xyxy: tuple[float, float, float, float],
) -> ColorEvidence:
    crop = _box_crop(image_bgr, box_xyxy)
    if crop.size:
        height, width = crop.shape[:2]
        crop = crop[
            int(height * 0.15) :,
            int(width * 0.15) : max(int(width * 0.85), 1),
        ]

    counts = _hsv_counts(crop)
    candidates = {
        name: counts[name]
        for name in ("red", "orange", "yellow", "green", "blue", "white")
    }
    label, pixel_count = max(candidates.items(), key=lambda item: item[1])
    minimum = max(6, int(crop.shape[0] * crop.shape[1] * 0.01))
    if pixel_count < minimum:
        label = "unknown"
    return ColorEvidence(label, pixel_count, counts)


def _box_crop(
    image_bgr: np.ndarray,
    box_xyxy: tuple[float, float, float, float],
) -> np.ndarray:
    height, width = image_bgr.shape[:2]
    x1, y1, x2, y2 = box_xyxy
    left = min(max(int(np.floor(x1)), 0), width)
    top = min(max(int(np.floor(y1)), 0), height)
    right = min(max(int(np.ceil(x2)), 0), width)
    bottom = min(max(int(np.ceil(y2)), 0), height)
    if right <= left or bottom <= top:
        return np.empty((0, 0, 3), dtype=np.uint8)
    return image_bgr[top:bottom, left:right]


def _hsv_counts(crop_bgr: np.ndarray) -> dict[str, int]:
    color_names = ("red", "orange", "yellow", "green", "blue", "white")
    if crop_bgr.size == 0:
        return {name: 0 for name in color_names}

    hue, saturation, value = cv2.split(cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV))
    vivid = (saturation >= 75) & (value >= 65)
    masks = {
        "red": vivid & ((hue <= 9) | (hue >= 170)),
        "orange": vivid & (hue >= 10) & (hue <= 21),
        "yellow": vivid & (hue >= 22) & (hue <= 37),
        "green": vivid & (hue >= 38) & (hue <= 90),
        "blue": vivid & (hue >= 91) & (hue <= 135),
        "white": (saturation <= 45) & (value >= 150),
    }
    return {name: int(np.count_nonzero(mask)) for name, mask in masks.items()}
