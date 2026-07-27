from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional

import numpy as np

from .constraints import SemanticConstraints
from .heading import angle_diff
from .primitives import Primitive, PrimitiveName, default_primitives


@dataclass
class PlannerWeights:
    heading: float = 0.8
    smoothness: float = 0.4
    curvature: float = 0.3
    semantic_cost: float = 1.0
    progress: float = 0.25


@dataclass
class LocalPrimitivePlanner:
    lethal_cost: int = 255
    weights: PlannerWeights = field(default_factory=PlannerWeights)
    primitives: Optional[list[Primitive]] = None
    red_forward_center_x_ratio: float = 0.5
    red_forward_width_ratio: float = 0.20
    red_forward_top_y_ratio: float = 0.25
    red_forward_bottom_y_ratio: float = 0.62

    def plan(
        self,
        costmap: np.ndarray,
        constraints: SemanticConstraints,
        current_yaw: float = 0.0,
        theta_ref: Optional[float] = None,
        previous_primitive_name: Optional[str] = None,
    ) -> tuple[Primitive, Dict[str, Dict[str, float | bool | str]]]:
        if costmap is None or np.asarray(costmap).size == 0:
            stop = self._stop()
            return stop, {stop.name: self._stop_info("missing_costmap")}

        details: Dict[str, Dict[str, float | bool | str]] = {}
        valid_moving: list[tuple[float, Primitive]] = []

        for primitive in self._primitives():
            info = self._score_primitive(
                primitive,
                np.asarray(costmap),
                constraints,
                current_yaw,
                theta_ref,
                previous_primitive_name,
            )
            details[primitive.name] = info
            if primitive.is_stop:
                continue
            if info["valid"]:
                valid_moving.append((float(info["total"]), primitive))

        if not valid_moving:
            stop = self._stop()
            details[stop.name] = self._stop_info("fallback_stop")
            return stop, details

        valid_moving.sort(key=lambda item: item[0])
        return valid_moving[0][1], details

    def _score_primitive(
        self,
        primitive: Primitive,
        costmap: np.ndarray,
        constraints: SemanticConstraints,
        current_yaw: float,
        theta_ref: Optional[float],
        previous_primitive_name: Optional[str],
    ) -> Dict[str, float | bool | str]:
        if primitive.is_stop:
            return self._stop_info("stop")

        if constraints.red_forward_forbidden and primitive.name == PrimitiveName.SLOW_FORWARD.value:
            return {
                "valid": False,
                "total": float("inf"),
                "semantic": 0.0,
                "lethal_hit": False,
                "red_gate_hit": True,
                "curvature": abs(primitive.angular_velocity),
                "smoothness": 0.0,
                "heading": 0.0,
                "progress": self._progress_reward(primitive.path_points),
                "reason": "red_forward_gate",
            }

        costs = self._sample_costs(costmap, primitive.path_points)
        lethal_hit = any(cost >= self.lethal_cost for cost in costs)
        red_gate_hit = constraints.red_forward_forbidden and self._path_hits_red_forward_gate(primitive.path_points)
        curvature = abs(primitive.angular_velocity)
        smoothness = 0.0 if previous_primitive_name in (None, primitive.name) else 1.0
        heading_cost = 0.0
        if theta_ref is not None:
            predicted_yaw = current_yaw + primitive.angular_velocity * primitive.duration
            heading_cost = abs(angle_diff(predicted_yaw, theta_ref))
        progress = self._progress_reward(primitive.path_points)

        if lethal_hit:
            return {
                "valid": False,
                "total": float("inf"),
                "semantic": 1.0,
                "lethal_hit": True,
                "red_gate_hit": red_gate_hit,
                "curvature": curvature,
                "smoothness": smoothness,
                "heading": heading_cost,
                "progress": progress,
                "reason": "red_forward_gate" if red_gate_hit else "lethal_cost",
            }

        semantic = float(np.mean(costs)) / max(1.0, float(self.lethal_cost))
        total = (
            self.weights.semantic_cost * semantic
            + self.weights.curvature * curvature
            + self.weights.smoothness * smoothness
            + self.weights.heading * heading_cost
            - self.weights.progress * progress
        )
        if constraints.red_forward_forbidden and "LEFT" in primitive.name:
            total -= 0.12

        return {
            "valid": True,
            "total": float(total),
            "semantic": semantic,
            "lethal_hit": False,
            "red_gate_hit": False,
            "curvature": curvature,
            "smoothness": smoothness,
            "heading": heading_cost,
            "progress": progress,
            "reason": "ok",
        }

    def _sample_costs(self, costmap: np.ndarray, points: Iterable[tuple[float, float]]) -> list[int]:
        height, width = costmap.shape[:2]
        costs: list[int] = []
        for x_ratio, y_ratio in points:
            x = int(round(max(0.0, min(1.0, x_ratio)) * (width - 1)))
            y = int(round(max(0.0, min(1.0, y_ratio)) * (height - 1)))
            costs.append(int(costmap[y, x]))
        return costs

    @staticmethod
    def _progress_reward(points: Iterable[tuple[float, float]]) -> float:
        pts = list(points)
        if len(pts) < 2:
            return 0.0
        return max(0.0, pts[0][1] - pts[-1][1])

    def _path_hits_red_forward_gate(self, points: Iterable[tuple[float, float]]) -> bool:
        half_width = 0.5 * self.red_forward_width_ratio
        x_min = self.red_forward_center_x_ratio - half_width
        x_max = self.red_forward_center_x_ratio + half_width
        y_min = self.red_forward_top_y_ratio
        y_max = self.red_forward_bottom_y_ratio
        return any(x_min <= x <= x_max and y_min <= y <= y_max for x, y in points)

    def _primitives(self) -> list[Primitive]:
        return self.primitives if self.primitives is not None else default_primitives()

    def _stop(self) -> Primitive:
        for primitive in self._primitives():
            if primitive.is_stop:
                return primitive
        return Primitive(PrimitiveName.STOP.value, 0.0, 0.0, 1.0, ())

    @staticmethod
    def _stop_info(reason: str) -> Dict[str, float | bool | str]:
        return {
            "valid": True,
            "total": 0.0,
            "semantic": 0.0,
            "lethal_hit": False,
            "red_gate_hit": False,
            "curvature": 0.0,
            "smoothness": 0.0,
            "heading": 0.0,
            "progress": 0.0,
            "reason": reason,
        }
