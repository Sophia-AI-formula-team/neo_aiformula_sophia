from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Tuple


class PrimitiveName(str, Enum):
    STOP = "STOP"
    SMALL_LEFT = "SMALL_LEFT"
    MEDIUM_LEFT = "MEDIUM_LEFT"
    STRONG_LEFT = "STRONG_LEFT"
    LEFT_THEN_ALIGN = "LEFT_THEN_ALIGN"
    SMALL_RIGHT = "SMALL_RIGHT"
    MEDIUM_RIGHT = "MEDIUM_RIGHT"
    STRONG_RIGHT = "STRONG_RIGHT"
    RIGHT_THEN_ALIGN = "RIGHT_THEN_ALIGN"
    SLOW_FORWARD = "SLOW_FORWARD"


PathPoint = Tuple[float, float]


@dataclass(frozen=True)
class Primitive:
    name: str
    linear_velocity: float
    angular_velocity: float
    duration: float
    path_points: Tuple[PathPoint, ...] = ()

    @property
    def is_stop(self) -> bool:
        return self.name == PrimitiveName.STOP.value


def default_primitives(
    duration: float = 1.0,
    slow_forward_v: float = 0.15,
    diversion_v: float = 0.08,
    small_left_w: float = 0.30,
    medium_left_w: float = 0.55,
    strong_left_w: float = 0.85,
) -> list[Primitive]:
    # Image coordinates are normalized: x grows left-to-right, y grows top-to-bottom.
    # This is front-camera perspective, not BEV: image bottom is near the robot,
    # image middle/top is farther ahead, and paths are visually compressed toward
    # the horizon/vanishing region as y decreases. Moving primitives intentionally
    # fan out toward different directions instead of converging back to the center.
    return [
        Primitive(PrimitiveName.STOP.value, 0.0, 0.0, duration, ()),
        Primitive(
            PrimitiveName.SMALL_LEFT.value,
            diversion_v,
            small_left_w,
            duration,
            ((0.49, 0.92), (0.44, 0.78), (0.39, 0.64), (0.34, 0.52), (0.30, 0.42)),
        ),
        Primitive(
            PrimitiveName.MEDIUM_LEFT.value,
            diversion_v,
            medium_left_w,
            duration,
            ((0.48, 0.92), (0.40, 0.78), (0.32, 0.64), (0.25, 0.52), (0.20, 0.42)),
        ),
        Primitive(
            PrimitiveName.STRONG_LEFT.value,
            diversion_v,
            strong_left_w,
            duration,
            ((0.47, 0.92), (0.35, 0.80), (0.24, 0.66), (0.15, 0.54), (0.08, 0.44)),
        ),
        Primitive(
            PrimitiveName.LEFT_THEN_ALIGN.value,
            diversion_v,
            medium_left_w * 0.6,
            duration,
            ((0.48, 0.92), (0.39, 0.78), (0.31, 0.65), (0.26, 0.54), (0.24, 0.43)),
        ),
        Primitive(
            PrimitiveName.SMALL_RIGHT.value,
            diversion_v,
            -small_left_w,
            duration,
            ((0.51, 0.92), (0.56, 0.78), (0.61, 0.64), (0.66, 0.52), (0.70, 0.42)),
        ),
        Primitive(
            PrimitiveName.MEDIUM_RIGHT.value,
            diversion_v,
            -medium_left_w,
            duration,
            ((0.52, 0.92), (0.60, 0.78), (0.68, 0.64), (0.75, 0.52), (0.80, 0.42)),
        ),
        Primitive(
            PrimitiveName.STRONG_RIGHT.value,
            diversion_v,
            -strong_left_w,
            duration,
            ((0.53, 0.92), (0.65, 0.80), (0.76, 0.66), (0.85, 0.54), (0.92, 0.44)),
        ),
        Primitive(
            PrimitiveName.RIGHT_THEN_ALIGN.value,
            diversion_v,
            -medium_left_w * 0.6,
            duration,
            ((0.52, 0.92), (0.61, 0.78), (0.69, 0.65), (0.74, 0.54), (0.76, 0.43)),
        ),
        Primitive(
            PrimitiveName.SLOW_FORWARD.value,
            slow_forward_v,
            0.0,
            duration,
            ((0.50, 0.92), (0.50, 0.78), (0.50, 0.64), (0.50, 0.50), (0.50, 0.38)),
        ),
    ]
