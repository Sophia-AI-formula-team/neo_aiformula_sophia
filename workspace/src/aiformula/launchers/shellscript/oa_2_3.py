#!/usr/bin/env python3
"""Use the legacy oa_neo_nurbs window on the OA 2.2 planning and IO pipeline."""

import cv2
import numpy as np

from oa_2_2 import ConeMaskAvoider as BaseAvoider, main as run_avoider


class ConeMaskAvoider(BaseAvoider):
    VERSION = 'oa_2_3'
    REVISION = 'nurbs_window_2'
    DEFAULT_CONE_THRESH_PX = 1000
    RELEASE_ON_CONFIRMED_CLEAR = True
    # During the drain, the slots move from A/peak/end to adjacent samples.
    # Run the jump check when replacing the path, where it tests a route change.
    CHECK_SAME_PATH_WINDOW_JUMP = False

    def __init__(self):
        self.fault = None
        self.last_window_ns = None
        super().__init__()

    def tick(self):
        if self.fault is not None:
            self._clear_fault_when_safe()
            return
        super().tick()
        if self.lane_forward:
            return
        if self.latest_mask is None or self.latest_vis_img is None or self.odom is None:
            reason = 'missing_input'
        elif not self._inputs_fresh():
            reason = self.execution_state
        else:
            now = self.get_clock().now().nanoseconds
            if self.last_window_ns is None:
                self.last_window_ns = now
            # Let the existing stationary rebase finish before giving up ownership.
            deadline = (self.sensor_timeout + self.switch_confirm_seconds) * 1e9
            if now - self.last_window_ns < deadline:
                return
            reason = 'window_timeout'
        self.fault = reason
        self._finish_avoidance('oa_fault')

    def _clear_fault_when_safe(self):
        if (self.odom is None or self.latest_mask is None or self.latest_vis_img is None
                or not self._inputs_fresh()):
            self.clear_frames = 0
            return
        if self.mask_sequence == self.counted_mask_sequence:
            return
        self.counted_mask_sequence = self.mask_sequence
        self.h, self.w = self.latest_vis_img.shape[:2]
        self.cx = self.w // 2
        _, pixels, _, _, _ = self._compute_roi_counts(self.latest_vis_img, self.latest_mask)
        # After the route is discarded, there is no corridor to validate.
        self.clear_frames = self.clear_frames + 1 if pixels == 0 else 0
        if self.clear_frames >= self.clear_confirm_frames:
            self.fault = None
            self._finish_avoidance('fault_cleared')

    def _finish_avoidance(self, state='idle'):
        self.last_window_ns = None
        if state in ('manual_takeover', 'manual_waiting_fresh_mask'):
            self.fault = None
        super()._finish_avoidance(state)

    def _path_end_row(self):
        # A nonempty planning mask has at least one row in the effective ROI.
        _, roi = self._roi_geometry()
        return int(np.argmax(np.any(roi, axis=1)))

    def _maybe_replan(self, mask, vis):
        # Use the legacy triplet timer to refresh A/peak/end while occupied.
        # The base planner checks route clearance and continuity before publishing.
        self.replan_reason = 'nurbs_rolling_window'
        self.compute_and_store_path(mask, vis, publish_window=True)

    def _select_window(self):
        points = self._path_in_vehicle()
        if not self.obstacle_cleared:
            peak, end = points[self.path_peak_index], points[-1]
            if self.path_progress_m >= self.path_cumulative[self.path_peak_index]:
                return None
            a = self._point_at_lookahead_x(points)
            if a[0] >= peak[0] and peak[0] > 1e-6:
                # A short effective ROI can put B inside the normal lookahead.
                # Place A on the rising segment before B.
                first_arc = self.path_cumulative[self.path_peak_index]
                a = self._point_at_progress((self.path_progress_m + first_arc) / 2.0, points)
            if a[0] > 1e-6 and a[0] + 1e-6 < peak[0] < end[0]:
                return (a, peak, end)
            return None

        # The legacy drain starts after B and takes adjacent curve samples.
        # Anchor those groups to measured progress and publish in the current vehicle frame.
        for index in range(self.path_peak_index + 1, len(points), 3):
            start = min(index, len(points) - 3)
            if self.path_cumulative[start] <= self.path_progress_m:
                continue
            group = points[start:start + 3]
            if group[0, 0] > 1e-6 and np.all(np.diff(group[:, 0]) > 1e-6):
                return group
        return None

    def _publish_window_unchecked(self, points):
        a, b, c = points
        super()._publish_window_unchecked(((a[0], a[1] * 0.5), b, c))
        self.last_window_ns = self.get_clock().now().nanoseconds

    def draw_path(self, vis):
        if (self.world_path is not None and self.odom is not None
                and self.path_frame == self.odom[3]):
            pixels = self._path_pixels(self._path_in_vehicle())
            cv2.polylines(vis, [pixels], False, (255, 255, 0), 2)


def main(args=None):
    run_avoider(args, avoider_cls=ConeMaskAvoider)


if __name__ == '__main__':
    main()
