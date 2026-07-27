from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Mapping, Optional

from .constraints import SemanticConstraints


IGNORE_LANE = "ignore_lane"
CANDIDATE = "candidate"
RECOVERABLE = "recoverable"


@dataclass
class LaneRecoveryGate:
    stable_required_frames: int = 5
    min_confidence: float = 0.65
    horizontal_angle_threshold_deg: float = 15.0
    max_center_jump_ratio: float = 0.18
    history: Deque[dict] = field(default_factory=lambda: deque(maxlen=20))

    def update(
        self,
        lane_result: Mapping | None,
        constraints: SemanticConstraints,
        current_road_angle_deg: Optional[float] = None,
    ) -> str:
        if constraints.diversion_active or not lane_result:
            self.history.clear()
            return IGNORE_LANE

        lane = dict(lane_result)
        if not self._basic_valid(lane, current_road_angle_deg):
            self.history.clear()
            return IGNORE_LANE

        if self.history and self._center_jump_too_large(self.history[-1], lane):
            self.history.clear()
            return CANDIDATE

        self.history.append(lane)
        if len(self.history) >= self.stable_required_frames:
            recent = list(self.history)[-self.stable_required_frames :]
            if all(self._basic_valid(item, current_road_angle_deg) for item in recent):
                return RECOVERABLE
        return CANDIDATE

    def _basic_valid(self, lane: Mapping, current_road_angle_deg: Optional[float]) -> bool:
        if not bool(lane.get("stable", False)):
            return False
        if float(lane.get("confidence", 0.0)) < self.min_confidence:
            return False

        angle = abs(float(lane.get("angle_deg", 0.0)))
        if angle < self.horizontal_angle_threshold_deg:
            return False
        if abs(angle - 180.0) < self.horizontal_angle_threshold_deg:
            return False

        if current_road_angle_deg is not None:
            road_diff = abs(float(lane.get("angle_deg", 0.0)) - current_road_angle_deg)
            if road_diff > 35.0:
                return False
        return True

    def _center_jump_too_large(self, prev: Mapping, curr: Mapping) -> bool:
        prev_x = float(prev.get("center_x", 0.5))
        curr_x = float(curr.get("center_x", 0.5))
        return abs(curr_x - prev_x) > self.max_center_jump_ratio
