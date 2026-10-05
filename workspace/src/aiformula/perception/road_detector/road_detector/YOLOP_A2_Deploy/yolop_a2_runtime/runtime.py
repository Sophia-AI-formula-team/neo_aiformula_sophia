from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch

from .color_semantics import classify_traffic_cone, classify_traffic_light
from .lane_model import load_lane_checkpoint
from .lane_processing import (
    LaneRuntimeConfig,
    lane_probability_from_logits,
    overlay_lane_mask,
    postprocess_lane_probability,
    preprocess_lane_image,
    restore_lane_map,
)
from .object_model import (
    OBJECT_CATEGORIES,
    TRAFFIC_CONE_ID,
    load_object_checkpoint,
)


TRAFFIC_LIGHT_ID = 10
OBSTACLE_CLASS_IDS = frozenset({1, 2, 3, 4, 6, 8, TRAFFIC_CONE_ID})


@dataclass(frozen=True)
class Detection:
    box_xyxy: tuple[float, float, float, float]
    class_id: int
    class_name: str
    score: float
    semantic_label: str
    color: str | None = None
    traffic_light_state: str | None = None

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "box_xyxy": [round(value, 3) for value in self.box_xyxy],
            "class_id": self.class_id,
            "class_name": self.class_name,
            "score": round(self.score, 6),
            "semantic_label": self.semantic_label,
        }
        if self.color is not None:
            payload["color"] = self.color
        if self.traffic_light_state is not None:
            payload["traffic_light_state"] = self.traffic_light_state
        return payload


@dataclass(frozen=True)
class PerceptionOutput:
    lane_mask: np.ndarray
    lane_overlay: np.ndarray
    obstacle_mask: np.ndarray
    object_overlay: np.ndarray
    combined_overlay: np.ndarray
    detections: tuple[Detection, ...]
    traffic_light_state: str
    red_pixels_count: int
    timings_ms: dict[str, float]

    def detections_payload(self) -> dict[str, Any]:
        return {
            "traffic_light_state": self.traffic_light_state,
            "red_pixels_count": self.red_pixels_count,
            "detections": [detection.to_dict() for detection in self.detections],
            "timings_ms": {
                name: round(value, 3) for name, value in self.timings_ms.items()
            },
        }


def render_object_outputs(
    image_bgr: np.ndarray,
    detections: tuple[Detection, ...],
) -> tuple[np.ndarray, np.ndarray]:
    overlay = image_bgr.copy()
    obstacle_mask = np.zeros(image_bgr.shape[:2], dtype=np.uint8)
    palette = {
        TRAFFIC_CONE_ID: (0, 165, 255),
        TRAFFIC_LIGHT_ID: (255, 255, 0),
        1: (255, 0, 255),
        3: (0, 255, 255),
    }

    for detection in detections:
        bounds = _clamp_box(detection.box_xyxy, image_bgr.shape[:2])
        if bounds is None:
            continue
        left, top, right, bottom = bounds
        color = palette.get(detection.class_id, (255, 180, 0))
        cv2.rectangle(overlay, (left, top), (right, bottom), color, 2)
        _draw_label(
            overlay,
            f"{detection.semantic_label} {detection.score:.2f}",
            left,
            top,
            color,
        )
        if detection.class_id in OBSTACLE_CLASS_IDS:
            cv2.rectangle(obstacle_mask, (left, top), (right, bottom), 255, -1)
    return overlay, obstacle_mask


class DualPerceptionModel:
    def __init__(
        self,
        lane_weights: str | Path,
        object_weights: str | Path,
        *,
        device: str | torch.device | None = None,
        object_score_threshold: float = 0.50,
        cone_score_threshold: float = 0.50,
        lane_config: LaneRuntimeConfig | None = None,
    ) -> None:
        if not 0.0 <= object_score_threshold <= 1.0:
            raise ValueError("object_score_threshold must be between 0 and 1")
        if not 0.0 <= cone_score_threshold <= 1.0:
            raise ValueError("cone_score_threshold must be between 0 and 1")

        self.device = self._resolve_device(device)
        self.object_score_threshold = object_score_threshold
        self.cone_score_threshold = cone_score_threshold
        self.lane_config = lane_config or LaneRuntimeConfig()
        self.lane_model = load_lane_checkpoint(lane_weights, device=self.device)
        self.lane_uses_half = self.device.type == "cuda"
        if self.lane_uses_half:
            self.lane_model.half()
        self.object_model, self.object_checkpoint = load_object_checkpoint(
            object_weights,
            device=self.device,
        )

    @staticmethod
    def _resolve_device(device: str | torch.device | None) -> torch.device:
        if device is None or str(device).lower() == "auto":
            return torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        resolved = torch.device(device)
        if resolved.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError(f"CUDA device {resolved} was requested but CUDA is unavailable")
        return resolved

    @torch.inference_mode()
    def predict(self, image_bgr: np.ndarray) -> PerceptionOutput:
        if not isinstance(image_bgr, np.ndarray) or (
            image_bgr.ndim != 3
            or image_bgr.shape[2] != 3
            or image_bgr.dtype != np.uint8
        ):
            shape = getattr(image_bgr, "shape", None)
            dtype = getattr(image_bgr, "dtype", None)
            raise ValueError(
                f"image_bgr must be a uint8 HxWx3 BGR image; "
                f"got shape={shape}, dtype={dtype}"
            )
        if image_bgr.shape[0] < 1 or image_bgr.shape[1] < 1:
            raise ValueError(f"image_bgr dimensions must be positive: {image_bgr.shape}")

        started = time.perf_counter()
        lane_mask = self._predict_lane(image_bgr)
        lane_finished = time.perf_counter()
        detections, red_pixels_count = self._predict_objects(image_bgr)
        object_finished = time.perf_counter()

        lane_overlay = overlay_lane_mask(image_bgr, lane_mask)
        object_overlay, obstacle_mask = render_object_outputs(image_bgr, detections)
        combined_overlay, _ = render_object_outputs(lane_overlay, detections)
        traffic_light_state = self._aggregate_traffic_light_state(detections)
        finished = time.perf_counter()
        return PerceptionOutput(
            lane_mask=lane_mask.astype(np.uint8) * 255,
            lane_overlay=lane_overlay,
            obstacle_mask=obstacle_mask,
            object_overlay=object_overlay,
            combined_overlay=combined_overlay,
            detections=detections,
            traffic_light_state=traffic_light_state,
            red_pixels_count=red_pixels_count,
            timings_ms={
                "lane": (lane_finished - started) * 1000.0,
                "objects": (object_finished - lane_finished) * 1000.0,
                "render": (finished - object_finished) * 1000.0,
                "total": (finished - started) * 1000.0,
            },
        )

    def _predict_lane(self, image_bgr: np.ndarray) -> np.ndarray:
        lane_input, shape = preprocess_lane_image(image_bgr, self.lane_config)
        lane_input = lane_input.to(self.device)
        lane_input = lane_input.half() if self.lane_uses_half else lane_input.float()
        lane_logits = self.lane_model(lane_input)
        lane_probability = lane_probability_from_logits(lane_logits)[0]
        restored = restore_lane_map(lane_probability, shape).astype(np.float32)
        return postprocess_lane_probability(restored, self.lane_config)

    def _predict_objects(
        self,
        image_bgr: np.ndarray,
    ) -> tuple[tuple[Detection, ...], int]:
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        object_input = torch.from_numpy(
            np.ascontiguousarray(image_rgb.transpose(2, 0, 1))
        )
        object_input = object_input.to(self.device, dtype=torch.float32).div_(255.0)
        prediction = self.object_model([object_input])[0]

        detections: list[Detection] = []
        red_pixels_count = 0
        for box_tensor, label_tensor, score_tensor in zip(
            prediction["boxes"],
            prediction["labels"],
            prediction["scores"],
        ):
            class_id = int(label_tensor.item())
            score = float(score_tensor.item())
            threshold = (
                self.cone_score_threshold
                if class_id == TRAFFIC_CONE_ID
                else self.object_score_threshold
            )
            if score < threshold or class_id <= 0 or class_id >= len(OBJECT_CATEGORIES):
                continue

            box = tuple(float(value) for value in box_tensor.tolist())
            class_name = OBJECT_CATEGORIES[class_id]
            semantic_label = class_name.replace(" ", "_")
            color: str | None = None
            traffic_light_state: str | None = None
            if class_id == TRAFFIC_CONE_ID:
                evidence = classify_traffic_cone(image_bgr, box)
                color = evidence.label
                semantic_label = (
                    f"{color}_traffic_cone" if color != "unknown" else "traffic_cone"
                )
            elif class_id == TRAFFIC_LIGHT_ID:
                evidence = classify_traffic_light(image_bgr, box)
                traffic_light_state = evidence.label
                red_pixels_count += evidence.pixel_count
                semantic_label = f"{traffic_light_state}_traffic_light"

            detections.append(
                Detection(
                    box_xyxy=box,
                    class_id=class_id,
                    class_name=class_name,
                    score=score,
                    semantic_label=semantic_label,
                    color=color,
                    traffic_light_state=traffic_light_state,
                )
            )
        return tuple(detections), red_pixels_count

    @staticmethod
    def _aggregate_traffic_light_state(detections: tuple[Detection, ...]) -> str:
        states = {
            detection.traffic_light_state
            for detection in detections
            if detection.traffic_light_state is not None
        }
        for priority_state in ("red", "green"):
            if priority_state in states:
                return priority_state
        return "unknown"


def _clamp_box(
    box_xyxy: tuple[float, float, float, float],
    image_shape: tuple[int, int],
) -> tuple[int, int, int, int] | None:
    height, width = image_shape
    x1, y1, x2, y2 = box_xyxy
    left = min(max(int(round(x1)), 0), width - 1)
    top = min(max(int(round(y1)), 0), height - 1)
    right = min(max(int(round(x2)), 0), width - 1)
    bottom = min(max(int(round(y2)), 0), height - 1)
    if right <= left or bottom <= top:
        return None
    return left, top, right, bottom


def _draw_label(
    image: np.ndarray,
    label: str,
    left: int,
    top: int,
    color: tuple[int, int, int],
) -> None:
    text_size, baseline = cv2.getTextSize(
        label,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        1,
    )
    label_top = max(top - text_size[1] - baseline - 4, 0)
    cv2.rectangle(
        image,
        (left, label_top),
        (min(left + text_size[0] + 4, image.shape[1] - 1), top),
        color,
        -1,
    )
    cv2.putText(
        image,
        label,
        (left + 2, max(top - baseline - 2, text_size[1])),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 0),
        1,
        cv2.LINE_AA,
    )
