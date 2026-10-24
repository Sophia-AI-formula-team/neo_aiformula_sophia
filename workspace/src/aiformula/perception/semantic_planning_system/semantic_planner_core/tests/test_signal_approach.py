"""Pure transport/envelope checks; the motion example is synthetic, not Unity."""
import json
import math
import unittest

from semantic_planner_core.signal_approach import parse_signal_observation, stop_envelope


class SignalApproachTests(unittest.TestCase):
    @staticmethod
    def envelope(distance=12.0, speed=0.2, age=0.0, uncertainty=0.0, bound=None):
        return stop_envelope(distance, uncertainty, age, speed if bound is None else bound,
                             speed, 8.5, 0.8, 1.0, 0.2)

    def test_color_without_range_remains_real_color(self):
        for state in ("red", "green", "unknown"):
            result = parse_signal_observation(json.dumps(dict(state=state, sequence=0, age_sec=0.1)))
            self.assertEqual(result.state, state)
            self.assertIsNone(result.distance_m)
            self.assertEqual(result.range_reason, "signal_range_missing")

    def test_bad_frame_metadata_is_rejected(self):
        good = dict(state="green", sequence=4, age_sec=0.1, distance_m=9.0)
        for field, value in (("sequence", True), ("sequence", 1.5), ("sequence", -1),
                             ("state", "yellow"), ("age_sec", -1),
                             ("age_sec", float("nan")), ("age_sec", "0")):
            with self.subTest(field=field, value=value):
                with self.assertRaises(ValueError):
                    parse_signal_observation(json.dumps(dict(good, **{field: value})))

    def test_invalid_distance_does_not_invent_a_different_color(self):
        for field, value in (("distance_m", float("inf")), ("distance_m", True),
                             ("distance_m", 0), ("uncertainty_m", -1)):
            data = dict(state="red", sequence=2, age_sec=0.0, distance_m=20.0)
            data[field] = value
            result = parse_signal_observation(json.dumps(data))
            self.assertEqual(result.state, "red")
            self.assertIsNone(result.distance_m)
            self.assertEqual(result.range_reason, "signal_range_invalid")

    def test_speed_cap_obeys_stop_inequality_and_is_monotonic(self):
        previous = 0.0
        for remaining in (0.0, 0.001, 0.01, 0.1, 0.3, 0.5, 1.0, 10.0):
            profile = self.envelope(distance=8.5 + remaining)
            speed = profile["speed_limit_mps"]
            self.assertGreaterEqual(speed, previous)
            self.assertLessEqual(speed, 0.8)
            self.assertLessEqual(speed * 0.2 + speed * speed / 2.0, remaining + 1e-12)
            previous = speed

    def test_negative_remaining_never_permits_reverse(self):
        result = self.envelope(distance=8.0, speed=-0.4, age=0.2)
        self.assertEqual(result["speed_limit_mps"], 0.0)
        self.assertEqual(result["distance_lower_m"], 8.0)

    def test_capture_age_uses_speed_bound_and_distance_uncertainty(self):
        profile = self.envelope(distance=10.0, speed=0.2, age=0.3, uncertainty=0.5, bound=2.0)
        self.assertAlmostEqual(profile["distance_lower_m"], 8.9)
        self.assertLess(profile["speed_limit_mps"], 0.8)

    def test_actual_speed_can_require_braking_despite_small_next_command(self):
        self.assertTrue(self.envelope(distance=9.0, speed=1.5)["braking_required"])
        self.assertFalse(self.envelope(distance=9.0, speed=0.1)["braking_required"])

    def test_synthetic_decelerating_plant_reaches_stop_band_without_early_latch(self):
        # A labelled toy plant checks envelope integration, not vehicle acceptance.
        for initial_speed in (0.2, 0.8, 2.5):
            with self.subTest(initial_speed=initial_speed):
                distance, speed, stopped, moving_steps = 24.0, initial_speed, False, 0
                dt = 0.05
                for _ in range(1600):
                    profile = self.envelope(distance=distance, speed=speed)
                    if profile["remaining_m"] <= 0.1:
                        stopped = True
                    command = 0.0 if stopped or profile["braking_required"] else profile["speed_limit_mps"]
                    moving_steps += command > 0.0
                    previous = speed
                    acceleration = 1.0 if command > speed else 1.9
                    speed += max(-acceleration * dt, min(acceleration * dt, command - speed))
                    distance -= 0.5 * (previous + speed) * dt
                    if stopped and speed <= 1e-9:
                        break
                self.assertTrue(stopped)
                self.assertGreater(moving_steps, 50)
                self.assertGreaterEqual(distance, 8.3)
                self.assertLessEqual(distance, 8.7)


if __name__ == "__main__":
    unittest.main()
