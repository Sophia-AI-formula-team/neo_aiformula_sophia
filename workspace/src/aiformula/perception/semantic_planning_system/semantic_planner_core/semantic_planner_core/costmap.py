from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .constraints import SemanticConstraints


@dataclass
class SemanticCostmapBuilder:
    low_cost: int = 20
    unknown_cost: int = 180
    lethal_cost: int = 255
    lane_guidance_cost: int = 10
    center_x_ratio: float = 0.5
    top_y_ratio: float = 0.25
    bottom_y_ratio: float = 0.62
    width_ratio: float = 0.20

    def build(
        self,
        road_mask: np.ndarray,
        offroad_mask: np.ndarray,
        constraints: SemanticConstraints,
        lane_guidance_mask: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        road = self._as_bool_mask(road_mask)
        offroad = self._as_bool_mask(offroad_mask, shape=road.shape)

        costmap = np.full(road.shape, self.unknown_cost, dtype=np.uint8)
        costmap[road] = np.uint8(self.low_cost)
        costmap[offroad] = np.uint8(self.lethal_cost)

        if (
            lane_guidance_mask is not None
            and constraints.allow_lane_follow
            and not constraints.diversion_active
        ):
            lane = self._as_bool_mask(lane_guidance_mask, shape=road.shape)
            lane_cells = lane & ~offroad
            costmap[lane_cells] = np.minimum(costmap[lane_cells], self.lane_guidance_cost)

        if constraints.red_forward_forbidden:
            self._insert_red_forward_region(costmap)

        return costmap.astype(np.uint8, copy=False)

    def _insert_red_forward_region(self, costmap: np.ndarray) -> None:
        height, width = costmap.shape[:2]
        cx = int(round(width * self.center_x_ratio))
        half_w = max(1, int(round(width * self.width_ratio * 0.5)))
        x0 = max(0, cx - half_w)
        x1 = min(width, cx + half_w)
        y0 = max(0, min(height, int(round(height * self.top_y_ratio))))
        y1 = max(0, min(height, int(round(height * self.bottom_y_ratio))))
        if y1 > y0 and x1 > x0:
            costmap[y0:y1, x0:x1] = np.uint8(self.lethal_cost)

    @staticmethod
    def _as_bool_mask(mask: np.ndarray, shape: tuple[int, int] | None = None) -> np.ndarray:
        arr = np.asarray(mask)
        if arr.ndim == 3:
            arr = arr[..., 0]
        if shape is not None and arr.shape != shape:
            raise ValueError(f"Mask shape {arr.shape} does not match expected {shape}")
        return arr > 0
