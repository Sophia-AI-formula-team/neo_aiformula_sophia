"""Red-light auto-turn and fallback checks using real ROS callbacks in a stub."""
import json
import math
import unittest
from unittest.mock import patch

import test_map_assist as fixtures
import test_ubs_map_assist as ubs


class AutoTurnTests(unittest.TestCase):
    def setUp(self):
        self.case = ubs.StopFlagAndUbsTests(methodName='runTest')
        self.case.setUp()
        self.addCleanup(self.case.doCleanups)
        self.h = self.case.h
        self.node = self.case.node
        self.case.set_fused_map()

    def test_first_red_frame_previews_but_does_not_commit_without_vote(self):
        self.node.p['detour_trigger_margin_m'] = 100.0
        self.assertIsNone(self.node.stop_flag)
        self.node._on_light_state(fixtures.DataMessage('red'))
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'prepare')
        self.assertTrue(self.node.red_preview_unconfirmed)
        self.assertEqual(self.node.traffic_light, 'unknown')
        self.assertTrue(self.node.smooth_path, self.node.smooth_plan_reason)
        self.assertEqual(self.node.traffic_reason, 'red_preview_spline_ready')
        self.h.confirm_light('red')
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'detour')
        self.assertTrue(self.node.detour_motion_ready)

    def test_previous_green_vote_does_not_cancel_first_red_preview(self):
        self.h.confirm_light('green')
        self.node._on_light_state(fixtures.DataMessage('red'))
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'prepare')
        self.assertTrue(self.node.red_preview_unconfirmed)
        self.assertTrue(self.node.smooth_path)
        self.h.confirm_light('green')
        self.assertEqual(self.node.traffic_mode, 'normal')
        self.assertIsNone(self.node.smooth_path)

    def test_only_confirmed_green_releases_first_lap_wait(self):
        self.node.reference = None
        self.h.confirm_light("red", stop_flag=True)
        self.assertEqual(self.node.traffic_reason, "red_wait_first_lap")
        for state in ("off", "yellow", "unknown"):
            self.h.confirm_light(state)
            self.node._on_controller_cmd(self.h.command())
            self.assertEqual(self.node.traffic_mode, "wait")
            self.h.assert_stopped()
        self.h.confirm_light("green", refresh_points=False)
        self.assertEqual(self.node.traffic_mode, "normal")
        self.assertEqual(self.node.traffic_reason, "green_resume")
        self.node._on_controller_cmd(self.h.command())
        self.h.assert_stopped()

    def test_green_majority_from_aging_votes_requires_a_new_green_frame(self):
        self.node.reference = None
        self.h.confirm_light("red", stop_flag=True)
        self.assertEqual(self.node.traffic_mode, "wait")
        for _ in range(3):
            self.h.advance(0.02)
            self.node._on_light_state(fixtures.DataMessage("green"))
            self.assertEqual(self.node.traffic_mode, "wait")
        self.h.advance(0.38)
        self.node._tick()
        self.assertEqual(self.node.traffic_light, "green")
        self.assertEqual(self.node.traffic_mode, "wait")
        self.node._on_light_state(fixtures.DataMessage("unknown"))
        self.node._on_controller_cmd(self.h.command())
        self.assertEqual(self.node.traffic_mode, "wait")
        self.h.assert_stopped()
        self.h.confirm_light("green")
        self.assertEqual(self.node.traffic_reason, "green_resume")

    def test_off_does_not_cancel_a_committed_detour(self):
        self.node.p["detour_trigger_margin_m"] = 100.0
        self.h.confirm_light("red")
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, "detour")
        self.h.confirm_light("off")
        self.assertEqual(self.node.traffic_mode, "detour")

    def test_unknown_and_recovered_distance_cannot_reopen_red_stop(self):
        self.node.reference = None
        self.node.p["traffic_stop_flag_timeout_sec"] = 0.5
        self.h.confirm_light("red", stop_flag=True)
        self.h.advance(0.6)
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, "wait")
        self.node._on_light_state(fixtures.DataMessage("unknown"))
        self.assertEqual(self.node.traffic_mode, "wait")
        self.h.advance(0.11)
        for _ in range(6):
            self.node._on_light_state(fixtures.DataMessage("unknown"))
            self.h.advance(0.11)
        self.node._on_traffic_stop_flag(fixtures.DataMessage(False))
        self.node._tick()
        self.assertEqual(self.node.traffic_light, "unknown")
        self.assertEqual(self.node.traffic_mode, "wait")
        self.node._on_controller_cmd(self.h.command())
        self.h.assert_stopped()

    def test_margin_triggers_turn_without_flag_stop(self):
        self.case.red_far()
        self.assertEqual(self.node.traffic_mode, 'prepare')
        self.assertEqual(self.node.auto_trigger_reason, 'radius_has_margin')
        self.node._on_traffic_stop_flag(fixtures.DataMessage(False))
        self.node.p['detour_trigger_margin_m'] = 100.0
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'detour')
        self.assertEqual(self.node.auto_trigger_reason, 'current_radius_near_limit')
        self.assertFalse(self.node.stop_flag)
        self.assertLessEqual(self.node.smooth_path.max_curvature, 1.0 / 4.0 + 1e-3)

    def test_unavailable_predicted_plan_commits_current_feasible_plan(self):
        self.case.red_far()
        path = self.node.smooth_path
        self.node.p['detour_trigger_margin_m'] = 0.0
        self.node.smooth_plan_pose = None
        current = self.node.odom
        def plan(pose):
            if math.dist(pose[:2], current[:2]) > 0.5:
                raise ValueError('forecast_reaches_limit')
            return path
        with patch.object(self.node, '_smooth_plan_at', side_effect=plan):
            self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'detour')
        self.assertEqual(self.node.auto_trigger_reason, 'predicted_plan_unavailable')
        self.assertIs(self.node.smooth_path, path)

    def test_no_feasible_route_continues_until_flag_then_waits(self):
        self.node.p['detour_min_turn_radius_m'] = 100.0
        self.case.red_far()
        self.assertEqual(self.node.traffic_mode, 'prepare')
        self.assertEqual(self.node.traffic_reason, 'red_prepare_spline_unavailable')
        self.assertIsNone(self.node.smooth_path)
        visual = self.h.triplet()
        self.node._tick()
        self.h.assert_last_points_equal(visual)
        self.node._on_traffic_stop_flag(fixtures.DataMessage(True))
        # The callback itself must revoke velocity, without waiting for a tick.
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertTrue(self.node.auto_stop_latched)
        self.assertTrue(self.node.traffic_reason.startswith('red_wait_spline_'))
        self.h.assert_stopped()
        self.node._on_traffic_stop_flag(fixtures.DataMessage(False))
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.h.confirm_light('green')
        self.assertEqual(self.node.traffic_mode, 'normal')

    def test_no_distance_message_and_no_plan_holds_until_green(self):
        self.node.p['detour_min_turn_radius_m'] = 100.0
        self.h.confirm_light('red')
        self.node._tick()
        self.assertEqual(self.node.traffic_reason, 'red_wait_flag_unavailable')
        self.assertTrue(self.node.auto_stop_latched)
        self.h.assert_stopped()
        self.node._on_traffic_stop_flag(fixtures.DataMessage(False))
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertEqual(self.node.traffic_reason, 'red_wait_flag_unavailable')
        self.h.confirm_light('unknown')
        count = len(self.node.cmd_pub.messages)
        for _ in range(20):
            self.h.advance(.05)
            self.node._red_hold_tick()
            self.h.assert_stopped()
        self.assertEqual(len(self.node.cmd_pub.messages), count + 20)
        self.h.confirm_light('green')
        self.assertEqual(self.node.traffic_mode, 'normal')

    def test_unconfirmed_or_unknown_light_never_starts_turn(self):
        self.case.red_far()
        self.node._on_light_state(fixtures.DataMessage('unknown'))
        self.node.p['detour_trigger_margin_m'] = 100.0
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertEqual(self.node.traffic_reason, 'red_wait_signal_lost')
        self.node._on_traffic_stop_flag(fixtures.DataMessage(True))
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertEqual(self.node.traffic_reason, 'red_wait_signal_lost')
        self.h.confirm_light('green')
        self.assertEqual(self.node.traffic_mode, 'normal')

    def test_lost_red_before_pixel_threshold_requires_green_for_straight(self):
        self.node.reference = None
        self.case.red_far()
        self.assertIs(self.node.stop_flag, False)
        self.node._on_light_state(fixtures.DataMessage('unknown'))
        self.h.assert_stopped()
        self.assertTrue(self.node.stop_committed)
        self.h.confirm_light('red', stop_flag=False)
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.h.confirm_light('green')
        self.assertEqual(self.node.traffic_reason, 'green_resume')

    def test_matched_route_only_turns_after_fresh_red_reconfirmation(self):
        self.case.red_far()
        self.node._on_light_state(fixtures.DataMessage('unknown'))
        self.node.p['detour_trigger_margin_m'] = 100.0
        self.h.advance(0.02)
        self.node._on_light_state(fixtures.DataMessage('red'))
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertIsNone(self.node.detour_tracker)
        self.h.confirm_light('red')
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'detour')
        self.assertTrue(self.node.detour_motion_ready)

    def test_detector_silence_stops_red_approach(self):
        self.case.red_far()
        self.h.advance(0.51)
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertEqual(self.node.traffic_reason, 'red_wait_signal_lost')
        self.h.assert_stopped()

    def test_lost_next_signal_holds_at_detour_exit(self):
        self.case.start_detour()
        self.h.confirm_light('green')
        self.h.confirm_light('red')
        self.assertTrue(self.node.detour_next_red_pending)
        self.node._on_light_state(fixtures.DataMessage('unknown'))
        self.node._on_light_state(fixtures.DataMessage('red'))
        self.assertEqual(self.node.traffic_mode, 'detour')
        self.h.emit_detour_completion()
        self.node._on_traffic_stop_flag(fixtures.DataMessage(False))
        self.node._on_controller_cmd(self.h.command())
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertEqual(self.node.traffic_reason, 'red_wait_signal_lost')
        self.h.assert_stopped()

    def test_greens_before_red_stop_cannot_release_it(self):
        self.node.reference = None
        self.node._on_traffic_stop_flag(fixtures.DataMessage(True))
        start = self.h.now + 2.1
        for offset, color in ((0, 'red'), (.08, 'red'), (.19, 'green'), (.22, 'green'),
                              (.40, 'red'), (.501, 'red')):
            self.h.now = start + offset
            self.node._on_light_state(fixtures.DataMessage(color))
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.h.now = start + .60
        self.node._on_light_state(fixtures.DataMessage('green'))
        self.assertEqual(self.node.traffic_light, 'green')
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.h.confirm_light('green')
        self.assertEqual(self.node.traffic_reason, 'green_resume')

    def test_uncertain_lamp_at_detour_exit_requires_new_red_votes(self):
        self.case.start_detour()
        self.h.confirm_light('green')
        self.h.confirm_light('red')
        self.node._on_light_state(fixtures.DataMessage('green'))
        self.assertTrue(self.node.detour_next_red_pending)
        self.h.emit_detour_completion()
        self.node._on_light_state(fixtures.DataMessage('red'))
        self.assertFalse(self.node._red_recently_confirmed(self.h.now))
        self.assertEqual(self.node.traffic_mode, 'wait')

    def test_brief_missing_vote_uses_only_a_fresh_recently_confirmed_red(self):
        self.case.red_far()
        self.assertTrue(self.node._red_recently_confirmed(self.h.now))
        self.h.advance(0.35)
        self.node._update_light_vote(self.h.now)
        self.assertEqual(self.node.traffic_light, 'unknown')
        self.assertTrue(self.node._red_recently_confirmed(self.h.now))
        self.h.advance(0.3)
        self.node._update_light_vote(self.h.now)
        self.assertFalse(self.node._red_recently_confirmed(self.h.now))

    def test_red_fallback_stop_preempts_oa(self):
        self.node.p['detour_min_turn_radius_m'] = 100.0
        self.case.red_far()
        self.node._on_avoidance(fixtures.DataMessage(True))
        count, commands = self.h.output_count(), len(self.node.cmd_pub.messages)
        self.node._on_traffic_stop_flag(fixtures.DataMessage(True))
        self.node._tick()
        self.assertEqual(self.h.output_count(), count)
        self.assertGreater(len(self.node.cmd_pub.messages), commands)
        self.assertEqual(self.node.traffic_reason, 'red_wait_maneuver_release')
        self.h.assert_stopped()
        self.node._on_avoidance(fixtures.DataMessage(False))
        self.h.advance(0.25)
        self.h.triplet()
        self.h.odom(*self.node.odom)
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertEqual(self.node.traffic_reason, 'red_wait_maneuver_release')
        self.h.assert_stopped()

    def test_status_exposes_configured_radius_and_fallback_role(self):
        self.node._publish_status()
        status = json.loads(self.node.status_pub.messages[-1].data)
        self.assertEqual(status['traffic_stop_flag_role'], 'red_fallback_only')
        self.assertEqual(status['auto_radius_trigger_m'], 4.5)
        self.assertEqual(status['auto_trigger_preview_m'], 1.0)
        self.assertEqual(status['traffic_planner_version'], 'ubs_auto_turn_v1')

    def test_invalid_trigger_parameters_are_rejected(self):
        for key, value in (('detour_min_turn_radius_m', 0.0),
                           ('detour_trigger_margin_m', -0.1),
                           ('detour_trigger_preview_m', 0.0),
                           ('traffic_red_rearm_sec', 0.0)):
            with self.subTest(key=key):
                old = self.node.p[key]
                self.node.p[key] = value
                with self.assertRaises(ValueError):
                    self.node._validate_parameters()
                self.node.p[key] = old


if __name__ == '__main__':
    unittest.main()
