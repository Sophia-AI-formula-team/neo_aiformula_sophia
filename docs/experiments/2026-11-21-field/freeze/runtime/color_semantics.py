from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


from .traffic_light_policy import (
    GREEN_LIGHT_MAX_RELATIVE_ROW,
    GREEN_LIGHT_MIN_SATURATION,
    GREEN_LIGHT_TRACK_SURFACE_MIN_SATURATION,
    SMALL_PANEL_GREEN_MIN_SATURATION,
    SMALL_PANEL_GREEN_MIN_VALUE,
    SMALL_PANEL_MAX_SIDE_AT_360P,
)

RED_LIGHT_MIN_PIXELS = 15


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
    # A pale reflection near the bottom of the image can be the green track surface.
    _, top, _, bottom = box_xyxy
    green_floor = GREEN_LIGHT_MIN_SATURATION if (top + bottom) * 0.5 <= image_bgr.shape[0] * GREEN_LIGHT_MAX_RELATIVE_ROW else (
        GREEN_LIGHT_TRACK_SURFACE_MIN_SATURATION
    )
    counts = _hsv_counts(crop, green_min_saturation=green_floor)
    minimum = max(3, int(crop.shape[0] * crop.shape[1] * 0.002))
    # A few lit red pixels can be surrounded by a much larger green background.
    # Weak or ambiguous warm evidence is not permission to report green.
    label = "unknown"
    if counts["red"] >= max(RED_LIGHT_MIN_PIXELS, minimum):
        label = "red"
    elif not counts["red"]:
        if counts["green"] >= minimum and not (counts["orange"] or counts["yellow"]):
            label = "green"
        elif counts["yellow"] >= minimum and counts["green"] < minimum:
            label = "yellow"
    return ColorEvidence(label, counts["red"], counts)


def summarize_signal(labels):
    states = set(labels)
    if "red" in states:
        return "red"
    # Detections do not identify which lamp controls our lane.
    return "green" if states == {"green"} else "unknown"


def classify_traffic_cone(
    image_bgr: np.ndarray,
    box_xyxy: tuple[float, float, float, float],
) -> ColorEvidence:
    crop = _box_crop(image_bgr, box_xyxy)
    if crop.size:
        height, width = crop.shape[:2]
        crop = crop[
            int(height * 0.15) : max(int(height * 0.85), 1),
            int(width * 0.35) : max(int(width * 0.65), 1),
        ]

    counts = _hsv_counts(crop, minimum_value=35)
    candidates = {
        name: counts[name]
        for name in ("red", "orange", "yellow", "green", "blue")
    }
    label, pixel_count = max(candidates.items(), key=lambda item: item[1])
    # Reflective white bands are not the cone's body color.
    minimum = max(6, int(crop.shape[0] * crop.shape[1] * 0.12))
    if pixel_count < minimum:
        label = "white" if counts["white"] >= minimum else "unknown"
        pixel_count = counts["white"] if label == "white" else pixel_count
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


def _hsv_counts(
    crop_bgr: np.ndarray, *, minimum_value: int = 65, green_min_saturation: int = 75,
) -> dict[str, int]:
    color_names = ("red", "orange", "yellow", "green", "blue", "white")
    if crop_bgr.size == 0:
        return {name: 0 for name in color_names}

    hue, saturation, value = cv2.split(cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV))
    vivid = (saturation >= 75) & (value >= minimum_value)
    masks = {
        "red": vivid & ((hue <= 9) | (hue >= 170)),
        "orange": vivid & (hue >= 10) & (hue <= 21),
        "yellow": vivid & (hue >= 22) & (hue <= 37),
        "green": (saturation >= green_min_saturation) & (value >= minimum_value) & (hue >= 38) & (hue <= 90),
        "blue": vivid & (hue >= 91) & (hue <= 135),
        "white": (saturation <= 45) & (value >= 150),
    }
    return {name: int(np.count_nonzero(mask)) for name, mask in masks.items()}


# Front-facing square LED panel used on the course, at the ZED's nHD geometry.
TRACK_RED_MIN_SATURATION = 45
TRACK_GREEN_MIN_SATURATION = 45
TRACK_MIN_VALUE = 160
TRACK_MIN_COLOR_FILL = 0.62
TRACK_SMALL_GREEN_MIN_SATURATION = SMALL_PANEL_GREEN_MIN_SATURATION
TRACK_SMALL_GREEN_MIN_VALUE = SMALL_PANEL_GREEN_MIN_VALUE
TRACK_SMALL_GREEN_MAX_SIDE = SMALL_PANEL_MAX_SIDE_AT_360P  # pixels at 360 image rows
_TRACK_CLOSE = np.ones((3, 3), dtype=np.uint8)


def classify_track_signal(image_bgr):
    """Return a validated panel, with red taking precedence over green."""
    height, width = image_bgr.shape[:2]
    top, bottom = round(height * 0.38), round(height * 0.56)
    if bottom - top < 10 or width < 10:
        return None
    hue, saturation, value = cv2.split(cv2.cvtColor(image_bgr[top:bottom], cv2.COLOR_BGR2HSV))
    bright = value >= TRACK_MIN_VALUE
    red = bright & (saturation >= TRACK_RED_MIN_SATURATION) & ((hue <= 9) | (hue >= 170))
    green_hue = (hue >= 38) & (hue <= 90)
    green = bright & (saturation >= TRACK_GREEN_MIN_SATURATION) & green_hue
    scale = height / 360
    small_green = (
        (value >= TRACK_SMALL_GREEN_MIN_VALUE)
        & (saturation >= TRACK_SMALL_GREEN_MIN_SATURATION)
        & green_hue
    )
    # The distant LED face covers about 10x11 pixels at the red stop line.
    # Keep the panel shape, fill, dark border and red-first checks for it.
    passes = (
        ("red", red, 35, None),
        ("green", green, 70, None),
        ("green", small_green, 70, TRACK_SMALL_GREEN_MAX_SIDE * scale),
    )
    for state, mask, minimum_pixels, maximum_side in passes:
        closed = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, _TRACK_CLOSE)
        _, labels, boxes, _ = cv2.connectedComponentsWithStats(closed)
        best = None
        for component, (x, y, w, h, _) in enumerate(boxes[1:], start=1):
            center_y = top + y + h / 2
            if not (height * 0.41 <= center_y <= height * 0.52):
                continue
            if not (6 * scale <= h <= 55 * scale and 6 * scale <= w <= 70 * scale
                    and 0.78 * h <= w <= 1.32 * h):
                continue
            if maximum_side is not None and max(w, h) > maximum_side:
                continue
            face = mask[y:y+h, x:x+w] & (labels[y:y+h, x:x+w] == component)
            count = int(np.count_nonzero(face))
            fill = count / (w * h)
            required_pixels = minimum_pixels * scale**2
            if state == "red":
                required_pixels = max(RED_LIGHT_MIN_PIXELS, required_pixels)

            if fill < TRACK_MIN_COLOR_FILL or count < required_pixels:
                continue
            if not _has_dark_panel_border(value, face, x, y, w, h):
                continue
            if best is None or count > best[2]:
                best = ((int(x), int(top+y), int(x+w), int(top+y+h)), state, count, fill)
        if best is not None:
            return best
    return None


def _has_dark_panel_border(value, face, x, y, width, height):
    edge = max(2, round(min(width, height) * 0.20))
    right, bottom = x + width, y + height
    if x < edge or y < edge or right + edge > value.shape[1] or bottom + edge > value.shape[0]:
        return False
    panel_value = np.median(value[y:bottom, x:right][face])
    sides = (value[y-edge:y, x:right], value[bottom:bottom+edge, x:right],
             value[y:bottom, x-edge:x], value[y:bottom, right:right+edge])
    return sum(np.mean(side <= panel_value - 25) >= 0.5 for side in sides) >= 3
