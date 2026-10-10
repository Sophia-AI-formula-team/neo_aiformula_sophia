"""Independent lane and object outputs shared by Python and ROS callers."""
from dataclasses import asdict, dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    box_xyxy: tuple
    class_id: int
    class_name: str
    score: float
    semantic_label: str
    color: str | None = None
    traffic_light_state: str | None = None

    def to_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class PerceptionOutput:
    lane_mask: np.ndarray
    lane_overlay: np.ndarray
    obstacle_mask: np.ndarray
    object_overlay: np.ndarray
    combined_overlay: np.ndarray
    detections: tuple
    traffic_light_state: str
    red_pixels_count: int
    timings_ms: dict

    def detections_payload(self):
        return dict(detections=[item.to_dict() for item in self.detections],
                    traffic_light_state=self.traffic_light_state,
                    red_pixels_count=self.red_pixels_count, timings_ms=self.timings_ms)


def render(image, detections):
    overlay = image.copy()
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    for item in detections:
        left, top, right, bottom = (int(round(value)) for value in item.box_xyxy)
        color = (0, 255, 255) if item.class_id == 10 else (0, 165, 255)
        cv2.rectangle(overlay, (left, top), (right, bottom), color, 2)
        label = f"{item.semantic_label} {item.score:.2f}"
        cv2.putText(overlay, label, (max(0, left), max(16, top - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(overlay, label, (max(0, left), max(16, top - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
        if item.class_id in {1, 3, 91}:
            cv2.rectangle(mask, (left, top), (right, bottom), 255, -1)
    return overlay, mask
