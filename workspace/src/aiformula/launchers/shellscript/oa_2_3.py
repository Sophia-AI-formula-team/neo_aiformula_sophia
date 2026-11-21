#!/usr/bin/env python3
"""Use the legacy oa_neo_nurbs window on the OA 2.2 planning and IO pipeline."""

import math

import cv2
import numpy as np

from oa_2_2 import ConeMaskAvoider as BaseAvoider, main as run_avoider


class ConeMaskAvoider(BaseAvoider):
    VERSION = 'oa_2_3'
    REVISION = 'nurbs_window_1'
    # During the drain, the slots move from A/peak/end to adjacent samples.
    # Run the jump check when replacing the path, where it tests a route change.
    CHECK_SAME_PATH_WINDOW_JUMP = False

    def _select_window(self):
        points = self._path_in_vehicle()
        next_index = int(np.searchsorted(self.path_cumulative, self.path_progress_m, side='right'))
        if not self.obstacle_cleared and next_index <= self.path_peak_index:
            start = self._point_at_progress(self.path_progress_m, points)
            remaining = np.vstack((start, points[next_index:]))
            a = self._point_at_lookahead_x(remaining)
            b, c = points[self.path_peak_index], points[-1]
            if (1e-6 < a[0] < b[0] < c[0]
                    and math.dist(a, b) >= self.min_point_spacing_m
                    and math.dist(b, c) >= self.min_point_spacing_m):
                return (a, b, c)
            if self.path_progress_m <= 1e-6:
                return None

        # As in oa_neo_nurbs, drain the original samples after B on clear.
        # Use measured progress and the current vehicle frame for the drain.
        if self.obstacle_cleared:
            next_index = max(next_index, self.path_peak_index + 1)
        min_forward = max(1e-6, min(self.desired_x_m, self.remaining_m / 2.0))
        targets = []
        for point in points[next_index:]:
            if not targets:
                if point[0] < min_forward:
                    continue
            elif (point[0] <= targets[-1][0]
                  or math.dist(targets[-1], point) < self.min_point_spacing_m):
                continue
            targets.append(point)
            if len(targets) == 3:
                return targets
        return None

    def draw_path(self, vis):
        if (self.world_path is not None and self.odom is not None
                and self.path_frame == self.odom[3]):
            pixels = self._path_pixels(self._path_in_vehicle())
            cv2.polylines(vis, [pixels], False, (255, 255, 0), 2)


def main(args=None):
    run_avoider(args, avoider_cls=ConeMaskAvoider)


if __name__ == '__main__':
    main()
