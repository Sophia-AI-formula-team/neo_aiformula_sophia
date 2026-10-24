import math
import unittest

from semantic_planner_core.rough_route import HeadingGuard, RouteReference, RouteSample


def rectangle():
    samples = []
    corners = [(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0), (0.0, 0.0)]
    for start, end in zip(corners, corners[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        count = round(distance / 0.2)
        for step in range(count):
            fraction = step / count
            samples.append(RouteSample(start[0] + fraction * (end[0] - start[0]),
                                       start[1] + fraction * (end[1] - start[1]),
                                       math.atan2(end[1] - start[1], end[0] - start[0])))
    samples.append(samples[0])
    return samples


class RouteReferenceTests(unittest.TestCase):
    def test_closed_progress_wraps_and_matches_across_lap(self):
        route = RouteReference(rectangle(), smooth_window_m=0.0)
        self.assertAlmostEqual(route.length_m, 32.0)
        self.assertEqual(len(route.cumulative_m), len(route.points) + 1)
        self.assertAlmostEqual(route.point_at(route.length_m + 0.4)[0], 0.4)
        matched = route.match(0.4, 0.0, route.length_m - 0.2, 0.6, 1.0, 0.3)
        self.assertAlmostEqual(matched, route.length_m + 0.4)

    def test_match_cannot_jump_to_nearby_wrong_crossing(self):
        route = RouteReference([RouteSample(x, y, 0.0) for x, y in
                                [(0, 0), (4, 4), (0, 4), (4, 0), (0, 0)]],
                               smooth_window_m=0.0)
        first_crossing = math.sqrt(8.0)
        matched = route.match(2.0, 2.0, first_crossing - 0.1, 0.1, 0.5, 0.3)
        self.assertAlmostEqual(matched, first_crossing, places=2)
        self.assertIsNone(route.match(2.0, 2.0, 0.0, 0.0, 0.3, 0.2))
        self.assertIsNone(route.match(2.0, 2.0, None, 0.0, 0.5, 0.3))

    def test_short_detour_is_straightened_but_never_trusted(self):
        samples = [RouteSample(0, 0, 0), RouteSample(1, 0, 0),
                   RouteSample(1.5, 0.4, 0, True), RouteSample(2, 0, 0),
                   RouteSample(6, 0, 0), RouteSample(6, 6, 0),
                   RouteSample(0, 6, 0), RouteSample(0, 0, 0)]
        route = RouteReference(samples, smooth_window_m=0.0, max_detour_gap_m=2.0)
        middle_x, middle_y, trusted = route.point_at(1.5)
        self.assertAlmostEqual(middle_x, 1.5)
        self.assertAlmostEqual(middle_y, 0.0)
        self.assertFalse(trusted)
        # A trusted destination beyond the detour cannot bypass the gap check.
        self.assertEqual(route.direction_candidates(0, 0, 0, 0, 3.0, 0.0), [])

    def test_long_detour_remains_excluded_and_unsmoothed(self):
        samples = [RouteSample(0, 0, 0), RouteSample(1, 0, 0),
                   RouteSample(2, 3, 0, True), RouteSample(3, 0, 0),
                   RouteSample(8, 0, 0), RouteSample(8, 8, 0),
                   RouteSample(0, 8, 0), RouteSample(0, 0, 0)]
        route = RouteReference(samples, smooth_window_m=1.0, max_detour_gap_m=2.0)
        self.assertTrue(any(y > 2.5 and not trusted for (x, y), trusted in zip(route.points, route.trusted)
                            if 1.0 < x < 3.0))

    def test_smoothing_reduces_small_oscillations_and_preserves_loop(self):
        samples = []
        for index in range(301):
            angle = 2.0 * math.pi * index / 300
            radius = 8.0 + 0.07 * math.sin(20.0 * angle)
            samples.append(RouteSample(radius * math.cos(angle), radius * math.sin(angle), angle + math.pi / 2))
        raw = RouteReference(samples, spacing_m=0.1, smooth_window_m=0.0)
        smooth = RouteReference(samples, spacing_m=0.1, smooth_window_m=1.4)
        def radial_spread(route):
            radii = [math.hypot(x, y) for x, y in route.points]
            mean = sum(radii) / len(radii)
            return sum((radius - mean) ** 2 for radius in radii) / len(radii)
        self.assertLess(radial_spread(smooth), radial_spread(raw) * 0.7)
        self.assertLess(math.hypot(smooth.points[-1][0] - smooth.points[0][0],
                                   smooth.points[-1][1] - smooth.points[0][1]), 0.15)
        self.assertEqual(smooth.point_at(0), smooth.point_at(smooth.length_m))

    def test_actual_position_produces_return_bearing(self):
        route = RouteReference(rectangle(), smooth_window_m=0.0)
        angles = route.direction_candidates(2.0, 0.5, 0.0, 2.0, 2.0, 0.0)
        self.assertEqual(len(angles), 1)
        self.assertAlmostEqual(angles[0], math.atan2(-0.5, 2.0))

    def test_large_smoothing_window_on_three_points_keeps_usable_loop(self):
        route = RouteReference(
            [RouteSample(0, 0, 0), RouteSample(2, 0, 0),
             RouteSample(1, math.sqrt(3), 0), RouteSample(0, 0, 0)],
            spacing_m=2.0, smooth_window_m=1000.0,
        )
        self.assertEqual(len(route.points), 3)
        self.assertGreater(route.length_m, 1.0)
        self.assertTrue(all(math.isfinite(value) for value in route.point_at(1.0)[:2]))
        self.assertEqual(route.point_at(0), route.point_at(route.length_m))

    def test_exclusion_at_lap_seam_blocks_targets_beyond_seam(self):
        samples = rectangle()
        samples[0] = RouteSample(0, 0, 0, excluded=True)
        samples[-1] = samples[0]
        route = RouteReference(samples, smooth_window_m=1.0)
        start_s = route.length_m - 1.0
        x, y, trusted = route.point_at(start_s)
        self.assertTrue(trusted)
        # The endpoint after the seam is trusted, but reaching it crosses the
        # excluded origin. This also exercises unfolded progress beyond L.
        self.assertTrue(route.point_at(start_s + 3.0)[2])
        self.assertEqual(route.direction_candidates(x, y, -math.pi / 4,
                                                   start_s, 3.0, 0.0), [])
        self.assertEqual(route.direction_candidates(x, y, -math.pi / 4,
                                                   start_s + route.length_m, 3.0, 0.0), [])

    def test_progress_tolerance_accepts_early_turn_without_global_search(self):
        route = RouteReference(rectangle(), smooth_window_m=0.0)
        angles = route.direction_candidates(8.0, 0.0, 0.0, 8.0, 2.0, 1.0)
        self.assertAlmostEqual(min(angles), 0.0)
        self.assertGreater(max(angles), math.radians(20))
        self.assertEqual(route.direction_candidates(2, 0, math.pi, 2, 2, 0), [])

    def test_duplicates_and_invalid_input(self):
        samples = rectangle()
        samples.insert(5, samples[5])
        route = RouteReference(samples)
        self.assertGreater(route.length_m, 30.0)
        with self.assertRaises(ValueError):
            RouteReference([RouteSample(0, 0, 0)] * 3)
        with self.assertRaises(ValueError):
            RouteReference(rectangle(), spacing_m=0)
        self.assertIsNone(route.match(float("nan"), 0, 0, 0, 1, 1))


class HeadingGuardTests(unittest.TestCase):
    def guard(self, frames=3):
        return HeadingGuard(math.radians(10), frames, math.radians(15), math.radians(3))

    def test_persistence_bounded_slew_and_no_accumulation(self):
        guard = self.guard()
        results = [guard.update(math.radians(50), [0.0]) for _ in range(20)]
        self.assertEqual(results[:2], [0.0, 0.0])
        self.assertAlmostEqual(results[2], math.radians(-3))
        self.assertAlmostEqual(results[-1], math.radians(-15))
        self.assertAlmostEqual(results[-2], results[-1])
        self.assertTrue(all(abs(result) <= math.radians(15) + 1e-12 for result in results))

    def test_corrects_only_to_tolerance_edge(self):
        guard = self.guard(frames=1)
        for _ in range(10):
            correction = guard.update(math.radians(14), [0.0])
        self.assertAlmostEqual(correction, math.radians(-4))
        self.assertEqual(guard.update(math.radians(9), [0.0]), 0.0)
        self.assertEqual(guard.violation_count, 0)

    def test_wrap_pi_and_nearest_candidate(self):
        guard = self.guard(frames=1)
        self.assertEqual(guard.update(math.radians(179), [math.radians(-179)]), 0.0)
        self.assertEqual(guard.update(math.radians(25), [0.0, math.radians(20)]), 0.0)
        correction = guard.update(math.radians(170), [math.radians(-160)])
        self.assertGreater(correction, 0.0)

    def test_disabling_clears_all_pending_and_active_correction(self):
        guard = self.guard()
        for _ in range(5):
            guard.update(1.0, [0.0])
        self.assertEqual(guard.update(1.0, [0.0], enabled=False), 0.0)
        self.assertEqual(guard.reason, "disabled")
        self.assertEqual(guard.violation_count, 0)
        self.assertEqual(guard.update(1.0, [0.0]), 0.0)
        self.assertEqual(guard.reason, "pending")
        self.assertEqual(guard.update(1.0, []), 0.0)
        self.assertEqual(guard.reason, "unavailable")

    def test_opposing_outliers_do_not_accumulate_persistence(self):
        guard = self.guard()
        for angle in [1.0, -1.0] * 10:
            self.assertEqual(guard.update(angle, [0.0]), 0.0)
        self.assertEqual(guard.violation_count, 1)


if __name__ == "__main__":
    unittest.main()
