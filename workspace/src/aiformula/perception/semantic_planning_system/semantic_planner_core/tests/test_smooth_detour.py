"""Geometric regression tests; these are not vehicle or road-boundary tests."""
import math
import unittest

from semantic_planner_core.rough_route import RouteReference, RouteSample
from semantic_planner_core.smooth_detour import SmoothDetourTracker, build_smooth_detour


class SmoothDetourTests(unittest.TestCase):
    def setUp(self):
        self.reference = RouteReference([
            RouteSample(-20, 0, 0), RouteSample(40, 0, 0),
            RouteSample(40, 50, 0), RouteSample(-20, 50, 0),
        ], 0.2, 0.0)
        self.route = [(0., 0.), (0., 30.), (40., 30.)]

    def build(self, pose=(-12., 0., 0.), radius=4.):
        return build_smooth_detour(self.reference, self.route, 90.0, pose, radius, 12.0)

    def test_ubs_limits_curvature_through_entry_inside_and_exit(self):
        path = self.build()
        self.assertLessEqual(path.max_curvature, 1.0 / 4.0 + 0.001)
        self.assertAlmostEqual(path.points[0][0], -12.0)
        self.assertAlmostEqual(path.points[0][1], 0.0)
        # The handoff is 2 m after the exit curve returns to the outer route.
        join = min(range(len(path.cumulative_m)), key=lambda i: abs(path.cumulative_m[i] - path.join_progress_m))
        expected = self.reference.point_at(90.0 + path.trim_m + 2.0)
        self.assertLess(math.dist(path.points[join], expected[:2]), 0.2)
        a, b = path.points[join - 1], path.points[join + 1]
        self.assertLess(abs(math.atan2(b[1] - a[1], b[0] - a[0]) - math.pi / 2), 0.05)

    def test_exit_requires_heading_alignment_before_switching_to_vision(self):
        path = self.build()
        tracker = SmoothDetourTracker(path, 2.0, 1.5, 0.5, 65.0)
        tracker.progress_m = path.join_progress_m - 1.0
        point = tracker._point_at(path.join_progress_m + 0.05)
        # At the main-line rejoin, a 30-degree heading mismatch must not finish.
        result = tracker.update(point[0], point[1], math.pi / 2 - math.radians(30.0))
        self.assertFalse(result.done)
        result = tracker.update(point[0], point[1], math.pi / 2)
        self.assertTrue(result.done)

    def test_ordered_tracker_completes_and_keeps_targets_in_front(self):
        path = self.build()
        tracker = SmoothDetourTracker(path, 2.0, 1.5, 0.5, 65.0)
        self.assertTrue(tracker.update(-12.0, 0.0, 0.0).available)
        complete = False
        for index, point in enumerate(path.points[1:-1], 1):
            following = path.points[index + 1]
            yaw = math.atan2(following[1] - point[1], following[0] - point[0])
            result = tracker.update(point[0], point[1], yaw)
            if result.done:
                complete = True
                self.assertGreaterEqual(result.progress_m, path.join_progress_m)
                break
            self.assertTrue(result.available, result.reason)
            for x, y in result.points:
                self.assertGreater(x, 0.0)
                self.assertLessEqual(abs(math.degrees(math.atan2(y, x))), 65.0)
        self.assertTrue(complete)

    def test_impossibly_large_vehicle_radius_does_not_return_a_route(self):
        with self.assertRaisesRegex(ValueError, 'no_curvature_feasible_spline'):
            self.build(radius=100.)

    def test_off_path_or_backward_pose_does_not_get_moving_targets(self):
        path = self.build()
        tracker = SmoothDetourTracker(path, 2.0, 1.5, 0.5, 65.0)
        self.assertFalse(tracker.update(-12., 4., 0.).available)
        self.assertFalse(tracker.update(-12., 0., math.pi).available)
        self.assertTrue(tracker.update(-12., 0., 0.).available)


if __name__ == '__main__':
    unittest.main()
