import math
import unittest

from semantic_planner_core.traffic_light_filter import TrafficLightFilter


class TrafficLightFilterTests(unittest.TestCase):
    def test_no_messages_never_decides(self):
        vote = TrafficLightFilter()
        self.assertEqual(vote.evaluate(0.0), "unknown")
        self.assertEqual(vote.evaluate(10.0), "unknown")
        self.assertEqual(vote.snapshot(10.0)["sample_count"], 0)
        self.assertIsNone(vote.snapshot(10.0)["last_message_age_sec"])

    def test_red_and_green_both_wait_for_complete_window(self):
        for state in ("red", "green"):
            with self.subTest(state=state):
                vote = TrafficLightFilter()
                self.assertEqual(vote.add(state, 0.0), "unknown")
                self.assertEqual(vote.add(state, 0.2), "unknown")
                self.assertEqual(vote.add(state, 0.4), "unknown")
                self.assertEqual(vote.evaluate(0.499), "unknown")
                self.assertEqual(vote.evaluate(0.5), state)

    def test_one_frame_and_timer_ticks_do_not_become_many_votes(self):
        vote = TrafficLightFilter()
        vote.add("red", 0.0)
        for index in range(51):
            self.assertEqual(vote.evaluate(index / 100.0), "unknown")
        self.assertEqual(vote.snapshot(0.5)["sample_count"], 1)
        self.assertFalse(vote.snapshot(0.5)["ready"])

    def test_minimum_real_frames_required(self):
        vote = TrafficLightFilter(min_samples=4)
        for now in (0.0, 0.2, 0.4):
            vote.add("green", now)
        self.assertEqual(vote.evaluate(0.5), "unknown")
        self.assertEqual(vote.add("green", 0.5), "green")

    def test_unknown_observations_count_in_majority_denominator(self):
        vote = TrafficLightFilter()
        for state, now in (("red", 0.0), ("red", 0.1), ("green", 0.2),
                           ("unknown", 0.3), ("unknown", 0.4)):
            vote.add(state, now)
        self.assertEqual(vote.evaluate(0.5), "unknown")
        self.assertEqual(vote.snapshot(0.5)["unknown_count"], 2)

    def test_strict_majority_not_plurality_or_tie(self):
        for states, expected in ((["red", "red", "green"], "red"),
                                 (["green", "green", "unknown"], "green"),
                                 (["red", "red", "green", "green"], "unknown"),
                                 (["red", "red", "unknown", "unknown"], "unknown")):
            with self.subTest(states=states):
                vote = TrafficLightFilter()
                for index, state in enumerate(states):
                    vote.add(state, index * 0.1)
                self.assertEqual(vote.evaluate(0.5), expected)

    def test_sliding_window_changes_red_to_green(self):
        vote = TrafficLightFilter()
        for now in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5):
            vote.add("red", now)
        self.assertEqual(vote.evaluate(0.5), "red")
        vote.add("green", 0.6)
        self.assertEqual(vote.add("green", 0.7), "red")
        vote.add("green", 0.8)
        self.assertEqual(vote.add("green", 0.9), "green")
        self.assertGreater(vote.snapshot(0.9)["green_count"], vote.snapshot(0.9)["red_count"])

    def test_aged_out_frames_no_longer_vote(self):
        vote = TrafficLightFilter()
        vote.add("red", 0.0)
        vote.add("green", 0.3)
        vote.add("green", 0.4)
        self.assertEqual(vote.evaluate(0.5), "green")
        self.assertEqual(vote.evaluate(0.51), "unknown")
        self.assertEqual(vote.snapshot(0.51)["red_count"], 0)
        self.assertEqual(vote.add("green", 0.52), "green")

    def test_no_message_gap_expires_vote_and_restarts_full_window(self):
        vote = TrafficLightFilter()
        for now in (0.0, 0.2, 0.4, 0.5):
            vote.add("red", now)
        self.assertEqual(vote.evaluate(0.5), "red")
        self.assertEqual(vote.evaluate(1.01), "unknown")
        status = vote.snapshot(1.01)
        self.assertFalse(status["ready"])
        self.assertEqual(status["sample_count"], 0)
        self.assertAlmostEqual(status["last_message_age_sec"], 0.51)
        for now in (1.1, 1.2, 1.4):
            self.assertEqual(vote.add("green", now), "unknown")
        self.assertEqual(vote.add("green", 1.6), "green")

    def test_gap_restarts_window_even_without_intervening_evaluate(self):
        vote = TrafficLightFilter()
        for now in (0.0, 0.2, 0.4, 0.5):
            vote.add("red", now)
        self.assertEqual(vote.add("green", 2.0), "unknown")
        vote.add("green", 2.1)
        self.assertEqual(vote.add("green", 2.4), "unknown")
        self.assertEqual(vote.evaluate(2.5), "green")

    def test_clear_discards_decision_and_previous_times(self):
        vote = TrafficLightFilter()
        for now in (4.0, 4.2, 4.5):
            vote.add("red", now)
        vote.clear()
        self.assertEqual(vote.evaluate(0.0), "unknown")
        self.assertIsNone(vote.snapshot(0.0)["last_message_age_sec"])
        for now in (0.0, 0.1, 0.2):
            self.assertEqual(vote.add("green", now), "unknown")
        self.assertEqual(vote.evaluate(0.5), "green")

    def test_invalid_colors_become_unknown_and_strings_are_normalized(self):
        vote = TrafficLightFilter()
        for state, now in ((" RED ", 0.0), ("Red", 0.1), (None, 0.2),
                           ("yellow", 0.3), ("", 0.4)):
            vote.add(state, now)
        status = vote.snapshot(0.5)
        self.assertEqual(status["red_count"], 2)
        self.assertEqual(status["unknown_count"], 3)
        self.assertEqual(vote.evaluate(0.5), "unknown")

    def test_rejects_invalid_configuration(self):
        for window in (0, -1, math.nan, math.inf, -math.inf, None, "bad"):
            with self.subTest(window=window), self.assertRaises(ValueError):
                TrafficLightFilter(window_sec=window)
        for count in (0, 1, -1, 2.5, 3.0, True, False, None, "3"):
            with self.subTest(count=count), self.assertRaises(ValueError):
                TrafficLightFilter(min_samples=count)

    def test_rejects_nonfinite_or_backward_times_without_losing_votes(self):
        vote = TrafficLightFilter()
        vote.add("red", 1.0)
        for now in (math.nan, math.inf, -math.inf, None, "bad", 0.9):
            for method in (vote.evaluate, vote.snapshot):
                with self.subTest(now=now, method=method.__name__), self.assertRaises(ValueError):
                    method(now)
            with self.subTest(now=now, method="add"), self.assertRaises(ValueError):
                vote.add("green", now)
        self.assertEqual(vote.snapshot(1.0)["sample_count"], 1)


if __name__ == "__main__":
    unittest.main()
