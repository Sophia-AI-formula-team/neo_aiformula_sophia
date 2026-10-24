"""Offline geometry/unit evidence, not a simulated vehicle or controller run.

Run with the original core package parent on PYTHONPATH, for example:
    python -B -m unittest -v test_continuous_detour_tracker
No models, Unity, fitting, synthetic learning, or core source writes occur.
"""

import math
import unittest

from semantic_planner_core.traffic_route import DetourResult, DetourTracker
from continuous_detour_tracker import ContinuousDetourTracker


def snapshot(tracker):
    return (tracker.started, tracker.done, tracker.segment_index,
            tracker.progress_m, tracker._accepted_position)


class ContinuousDetourTrackerTests(unittest.TestCase):
    def assertRejectedUnchanged(self, tracker, pose, reason):
        before = snapshot(tracker)
        result = tracker.update(*pose)
        self.assertFalse(result.available)
        self.assertFalse(result.done)
        self.assertEqual(result.reason, reason)
        self.assertEqual(snapshot(tracker), before)
        self.assertEqual(result.progress_m, before[3])
        return result

    def test_original_api_and_real_result_type(self):
        tracker = ContinuousDetourTracker([(0, 0), (10, 0)], .8, 2, .5, .5, 3)
        result = tracker.update(0, 0, 0)
        self.assertIs(type(result), DetourResult)
        self.assertEqual(result.points, [(.5, 0), (1, 0), (1.5, 0)])
        self.assertEqual((result.reason, result.segment_index, result.progress_m),
                         ("tracking", 0, 0))

    def test_dense_segments_do_not_compress_preview(self):
        route = [(i / 100, 0) for i in range(501)]
        tracker = ContinuousDetourTracker(route, waypoint_radius_m=.1)
        result = tracker.update(0, 0, 0)
        self.assertEqual(result.points, [(.5, 0), (1, 0), (1.5, 0)])
        result = tracker.update(1.25, 0, 0)
        self.assertAlmostEqual(result.progress_m, 1.25)
        self.assertEqual(result.segment_index, 125)
        for point, expected in zip(result.points, (.5, 1, 1.5)):
            self.assertAlmostEqual(point[0], expected)

    def test_preview_crosses_corner_before_vehicle_reaches_it(self):
        tracker = ContinuousDetourTracker([(0, 0), (5, 0), (5, 5)], waypoint_radius_m=.8)
        for x in (0, 1, 2, 3):
            self.assertTrue(tracker.update(x, 0, 0).available)
        result = tracker.update(4, 0, 0)
        self.assertEqual(result.points, [(.5, 0), (1, 0), (1, .5)])
        self.assertEqual(result.segment_index, 0)
        self.assertEqual(result.progress_m, 4)

    def test_original_right_angle_negative_and_no_pose_repair(self):
        route = [(0, 0), (5, 0), (5, 5)]
        old = DetourTracker(route, .8, 2, .5, .5, 3)
        new = ContinuousDetourTracker(route, .8, 2, .5, .5, 3)
        for x in (0, 1, 2, 3, 4, 4.3):
            self.assertTrue(old.update(x, 0, 0).available)
            self.assertTrue(new.update(x, 0, 0).available)
        self.assertEqual(old.update(5.1, 0, 0).reason, "target_behind_vehicle")
        for unused in range(3):
            self.assertRejectedUnchanged(new, (5.1, 0, 0), "target_behind_vehicle")
        # Both fail this overshot unsmoothed corner. No synthetic yaw alignment
        # is applied to make either tracker appear to recover or drive through.

    def test_r3_curve_supplied_poses_offline_only(self):
        # A test fixture, not geometry produced by the tracker or a vehicle trace.
        straight = [(i / 4, 0) for i in range(13)]
        arc = [(3 + 3 * math.sin(i * math.pi / 120),
                3 - 3 * math.cos(i * math.pi / 120)) for i in range(1, 61)]
        exit_points = [(6, 3 + i / 4) for i in range(1, 13)]
        route = straight + arc + exit_points
        yaws = [0.0] * len(straight) + [i * math.pi / 120 for i in range(1, 61)]
        yaws += [math.pi / 2] * len(exit_points)
        tracker = ContinuousDetourTracker(route, waypoint_radius_m=.05)
        previous = 0
        observed_curvature = False
        for pose, yaw in zip(route, yaws):
            result = tracker.update(*pose, yaw)
            self.assertGreaterEqual(result.progress_m, previous)
            self.assertLessEqual(result.progress_m - previous, 2 + 1e-9)
            previous = result.progress_m
            self.assertTrue(result.available or result.done, result)
            if result.available:
                self.assertTrue(all(p[0] >= -1e-6 for p in result.points))
                if 2.5 < result.progress_m < 6:
                    observed_curvature |= result.points[-1][1] > .05
                    self.assertGreater(math.dist(result.points[0], result.points[-1]), .9)
        self.assertTrue(observed_curvature)
        self.assertTrue(tracker.done)
        self.assertAlmostEqual(tracker.progress_m, tracker.length_m)

    def test_global_future_crossing_not_selected_or_done(self):
        route = [(0, 0), (5, 0), (5, 5), (0, 5), (0, 0), (.5, 0)]
        tracker = ContinuousDetourTracker(route, waypoint_radius_m=1)
        result = tracker.update(.25, 0, 0)
        self.assertTrue(result.available)
        self.assertFalse(result.done)
        self.assertAlmostEqual(result.progress_m, .25)
        self.assertEqual(result.segment_index, 0)
        result = tracker.update(.5, 0, 0)
        self.assertFalse(result.done)  # exactly at a much later endpoint
        self.assertAlmostEqual(result.progress_m, .5)

    def test_interior_self_crossing_keeps_current_route_branch(self):
        route = [(-5, 0), (5, 0), (5, 5), (0, 5), (0, -5)]
        tracker = ContinuousDetourTracker(route, waypoint_radius_m=.1)
        for x in (-5, -4, -3, -2, -1, 0):
            result = tracker.update(x, 0, 0)
            self.assertTrue(result.available)
        self.assertEqual(result.progress_m, 5)
        self.assertEqual(result.segment_index, 0)
        self.assertEqual(result.points, [(.5, 0), (1, 0), (1.5, 0)])

    def test_hard_bound_and_repeated_teleport_cannot_ratchet(self):
        tracker = ContinuousDetourTracker([(0, 0), (20, 0)])
        tracker.update(0, 0, 0)
        for unused in range(3):
            self.assertRejectedUnchanged(tracker, (5, 0, 0), "pose_step_exceeded")
        self.assertTrue(tracker.update(2, 0, 0).available)
        self.assertEqual(tracker.progress_m, 2)
        with self.assertRaises(ValueError):
            ContinuousDetourTracker([(0, 0), (20, 0)], max_advance_m=2.00001)

    def test_lower_speed_dependent_configured_bound(self):
        tracker = ContinuousDetourTracker([(0, 0), (20, 0)], max_advance_m=.25)
        tracker.update(0, 0, 0)
        self.assertRejectedUnchanged(tracker, (.3, 0, 0), "pose_step_exceeded")
        self.assertTrue(tracker.update(.25, 0, 0).available)

    def test_projection_window_exhaustion_despite_small_chord(self):
        # A U geometry makes Euclidean translation shorter than route progress.
        # Local projection must not claim its later leg or ratchet to it.
        route = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 3)]
        tracker = ContinuousDetourTracker(route, entry_radius_m=2,
                                         lookahead_m=.1, spacing_m=.1)
        tracker.update(0, 0, 0)
        self.assertRejectedUnchanged(tracker, (.9, 1.1, math.pi),
                                     "projection_window_exhausted")

    def test_backwards_motion_does_not_decrease_progress(self):
        tracker = ContinuousDetourTracker([(0, 0), (10, 0)])
        for x in (0, 1, 2):
            tracker.update(x, 0, 0)
        result = tracker.update(1.8, 0, 0)
        self.assertTrue(result.available)
        self.assertEqual(result.progress_m, 2)
        repeated = tracker.update(1.8, 0, 0)
        self.assertEqual(repeated.progress_m, 2)

    def test_entry_cross_track_and_heading_rejections_are_atomic(self):
        tracker = ContinuousDetourTracker([(0, 0), (10, 0)],
                                         entry_radius_m=.5, max_cross_track_m=.2)
        self.assertRejectedUnchanged(tracker, (1, 0, 0), "entry_not_reached")
        tracker.update(0, 0, 0)
        self.assertRejectedUnchanged(tracker, (.1, .3, 0), "cross_track_exceeded")
        self.assertRejectedUnchanged(tracker, (.1, 0, math.pi), "target_behind_vehicle")

    def test_end_completion_bounded_and_short_preview_distinct(self):
        tracker = ContinuousDetourTracker([(0, 0), (5, 0)], waypoint_radius_m=.05)
        for x in (0, 1, 2, 3, 4):
            self.assertTrue(tracker.update(x, 0, 0).available)
        result = tracker.update(4.8, 0, 0)
        self.assertTrue(result.available)
        self.assertEqual(len(set(result.points)), 3)
        result = tracker.update(4.96, 0, 0)
        self.assertTrue(result.done)
        self.assertEqual(result.points, [])
        self.assertEqual(result.progress_m, 5)
        self.assertFalse(result.available)
        self.assertTrue(tracker.update(100, 100, 0).done)

    def test_terminal_overshoot_outside_radius_cannot_claim_done(self):
        tracker = ContinuousDetourTracker([(0, 0), (5, 0)], waypoint_radius_m=.1)
        for x in (0, 1, 2, 3, 4):
            tracker.update(x, 0, 0)
        self.assertRejectedUnchanged(tracker, (5.2, 0, 0), "terminal_not_reached")

    def test_invalid_poses_leave_state_unchanged_even_after_done(self):
        tracker = ContinuousDetourTracker([(0, 0), (1, 0)], waypoint_radius_m=.01)
        tracker.update(0, 0, 0)
        for pose in ((math.nan, 0, 0), (0, math.inf, 0), (0, 0, -math.inf),
                     (None, 0, 0), ("bad", 0, 0)):
            self.assertRejectedUnchanged(tracker, pose, "invalid_pose")
        self.assertTrue(tracker.update(1, 0, 0).done)
        self.assertRejectedUnchanged(tracker, (math.nan, 0, 0), "invalid_pose")

    def test_invalid_geometry_and_limits(self):
        for route in ([], [(0, 0)], [(0, 0), (0, 0)], [(0, 0), (1e-7, 0)],
                      [(0, 0), (math.inf, 0)], [(0, 0), (1, math.nan)],
                      [(0, 0), (1, 2, 3)], [(0, 0), None],
                      [(1e308, 0), (-1e308, 0)]):
            with self.subTest(route=route), self.assertRaises(ValueError):
                ContinuousDetourTracker(route)
        for name in ("waypoint_radius_m", "max_cross_track_m", "lookahead_m",
                     "spacing_m", "entry_radius_m", "max_advance_m"):
            for value in (0, -1, math.nan, math.inf, "bad"):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    ContinuousDetourTracker([(0, 0), (1, 0)], **{name: value})

    def test_route_snapshot_prevents_stale_cached_geometry(self):
        route = [[0, 0], [10, 0]]
        tracker = ContinuousDetourTracker(route)
        route[1][:] = [0, 10]
        route.append([10, 10])
        self.assertEqual(tracker.points, ((0, 0), (10, 0)))
        self.assertEqual(tracker.update(0, 0, 0).points, [(.5, 0), (1, 0), (1.5, 0)])

    def test_reset_clears_progress_and_pose_history(self):
        tracker = ContinuousDetourTracker([(0, 0), (2, 0)], waypoint_radius_m=.01)
        tracker.update(0, 0, 0)
        self.assertTrue(tracker.update(2, 0, 0).done)
        tracker.reset()
        self.assertEqual(snapshot(tracker), (False, False, 0, 0, None))
        self.assertTrue(tracker.update(0, 0, 0).available)


if __name__ == "__main__":
    unittest.main(verbosity=2)
