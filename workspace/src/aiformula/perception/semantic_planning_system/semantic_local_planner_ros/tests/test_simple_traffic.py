"""Production MapAssist callbacks with upstream ROS stubs; no DDS or vehicle commands."""
import json
import unittest
import test_map_assist as fixtures


class SimpleTrafficTests(unittest.TestCase):
    def setUp(self):
        self.t = fixtures.MapAssistTests()
        self.t.setUp()
        self.addCleanup(self.t.doCleanups)
        self.t.parameter_overrides['traffic_policy'] = 'stop_wait'
        self.t.traffic_node()
        self.t.signal_sequence = 0

    def test_default_and_atomic_subscription_do_not_require_distance_approach(self):
        t = self.t
        del t.parameter_overrides['traffic_policy']
        node = t.module.MapAssist()
        self.assertEqual(node.p['traffic_policy'], 'stop_wait')
        self.assertFalse(node.p['enable_signal_approach'])
        self.assertIn(node.p['signal_observation_topic'], node.subscriptions)

    def test_null_range_red_stops_and_null_range_green_releases(self):
        t = self.t
        t.confirm_signal('red', distance=None)
        self.assertEqual(t.node.traffic_mode, 'wait')
        self.assertEqual(t.node.traffic_reason, 'red_wait_green')
        t.node._on_controller_cmd(t.command())
        t.assert_stopped()
        t.confirm_signal('green', distance=None)
        self.assertEqual(t.node.traffic_mode, 'normal')
        self.assertEqual(t.node.traffic_reason, 'green_resume')

    def test_distance_value_cannot_enable_approach(self):
        t = self.t
        t.confirm_signal('red', distance=100.0)
        self.assertEqual(t.node.traffic_mode, 'wait')
        t.node._on_controller_cmd(t.command())
        t.assert_stopped()

    def test_single_red_and_timer_do_not_invent_stable_detection(self):
        t = self.t
        t.signal_frame('red', distance=None)
        for _ in range(6):
            t.advance(.1)
            t.node._tick()
        self.assertEqual(t.node.traffic_mode, 'normal')

    def test_unknown_loss_and_one_green_cannot_release_red(self):
        t = self.t
        t.confirm_signal('red', distance=None)
        t.signal_frame('green', distance=None)
        t.signal_frame('unknown', distance=None)
        t.advance(1.0)
        t.node._tick()
        t.node._on_controller_cmd(t.command())
        self.assertEqual(t.node.traffic_mode, 'wait')
        t.assert_stopped()

    def test_duplicate_stale_out_of_order_and_malformed_green_cannot_release(self):
        t = self.t
        t.confirm_signal('red', distance=None)
        sequence = t.signal_sequence
        payloads = [dict(state='green',sequence=sequence,age_sec=0,distance_m=None),
                    dict(state='green',sequence=sequence-1,age_sec=0,distance_m=None),
                    dict(state='green',sequence=sequence+1,age_sec=2,distance_m=None)]
        for payload in payloads:
            for _ in range(7):
                t.advance(.1)
                t.node._on_signal_observation(fixtures.DataMessage(json.dumps(payload)))
                t.node._tick()
        t.node._on_signal_observation(fixtures.DataMessage('{'))
        t.node._on_controller_cmd(t.command())
        self.assertEqual(t.node.traffic_mode,'wait')
        t.assert_stopped()

    def test_green_requires_resume_delay_fresh_abc_and_new_command(self):
        # Exercise the same behavioral contract under stop_wait, not legacy.
        self.t.test_first_lap_red_waits_unknown_stays_stopped_and_green_needs_new_points_and_command()

    def test_red_overrides_active_oa_commands(self):
        t = self.t
        t.ready_visual_motion()
        t.node._on_avoidance(fixtures.DataMessage(True))
        t.confirm_signal('red', distance=None)
        t.node._on_controller_cmd(t.command(.4,-.6))
        self.assertEqual(t.node.traffic_mode,'wait')
        t.assert_stopped()
        t.node._tick()
        t.assert_stopped()

    def test_learned_aligned_map_does_not_automatically_detour_on_red(self):
        t = self.t
        t.learned_official_loop()
        reference = t.node.reference
        self.assertTrue(t.node.fusion_status['available'])
        t.confirm_light('red')
        t.node._tick()
        self.assertEqual(t.node.traffic_mode,'wait')
        self.assertIsNone(t.node.detour_tracker)
        self.assertIs(t.node.reference,reference)
        t.node._on_controller_cmd(t.command())
        t.assert_stopped()

    def test_map_reset_does_not_release_red_wait(self):
        t = self.t
        t.confirm_signal('red', distance=None)
        t.advance(.6)
        t.node._reset_learning('test_reset')
        t.node._on_controller_cmd(t.command())
        self.assertEqual(t.node.traffic_mode,'wait')
        t.assert_stopped()

    def test_conflicting_distance_policy_and_unknown_policy_are_rejected(self):
        t = self.t
        t.parameter_overrides['enable_signal_approach'] = True
        with self.assertRaisesRegex(ValueError,'stop_wait'):
            t.module.MapAssist()
        t.parameter_overrides['enable_signal_approach'] = False
        t.parameter_overrides['traffic_policy'] = 'other'
        with self.assertRaisesRegex(ValueError,'traffic_policy'):
            t.module.MapAssist()


if __name__ == '__main__':
    unittest.main()
