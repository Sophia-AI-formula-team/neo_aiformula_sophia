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

    def test_no_distance_message_and_no_plan_holds_until_a_state_arrives(self):
        self.node.p['detour_min_turn_radius_m'] = 100.0
        self.h.confirm_light('red')
        self.node._tick()
        self.assertEqual(self.node.traffic_reason, 'red_wait_flag_unavailable')
        self.assertFalse(self.node.auto_stop_latched)
        self.h.assert_stopped()
        self.node._on_traffic_stop_flag(fixtures.DataMessage(False))
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'prepare')
        self.assertEqual(self.node.traffic_reason, 'red_prepare_spline_unavailable')

    def test_unconfirmed_or_unknown_light_never_starts_turn(self):
        self.case.red_far()
        self.node._on_light_state(fixtures.DataMessage('unknown'))
        self.node.p['detour_trigger_margin_m'] = 100.0
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'prepare')
        self.assertEqual(self.node.traffic_reason, 'red_prepare_light_confirmation')
        self.node._on_traffic_stop_flag(fixtures.DataMessage(True))
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertEqual(self.node.traffic_reason, 'red_wait_light_confirmation')
        self.h.confirm_light('green')
        self.assertEqual(self.node.traffic_mode, 'normal')

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

    def test_oa_owns_control_even_when_fallback_stop_is_requested(self):
        self.node.p['detour_min_turn_radius_m'] = 100.0
        self.case.red_far()
        self.node._on_avoidance(fixtures.DataMessage(True))
        count, commands = self.h.output_count(), len(self.node.cmd_pub.messages)
        self.node._on_traffic_stop_flag(fixtures.DataMessage(True))
        self.node._tick()
        self.assertEqual(self.h.output_count(), count)
        self.assertEqual(len(self.node.cmd_pub.messages), commands)
        self.node._on_avoidance(fixtures.DataMessage(False))
        self.h.advance(0.25)
        self.h.triplet()
        self.h.odom(*self.node.odom)
        self.node._tick()
        self.assertEqual(self.node.traffic_mode, 'wait')
        self.assertTrue(self.node.traffic_reason.startswith('red_wait_spline_'))
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
