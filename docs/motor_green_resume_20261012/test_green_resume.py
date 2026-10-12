"""Production MapAssist + MotorGate callbacks; ROS is stubbed, not DDS/hardware."""
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2] / 'workspace/src/aiformula'
sys.path[:0] = [str(ROOT / 'control/motor_controller'),
               str(ROOT / 'perception/semantic_planning_system/semantic_local_planner_ros/tests')]
from motor_controller.gate import MotorGate
import test_simple_traffic as fixtures


class GreenResumeTests(unittest.TestCase):
    def setUp(self):
        fixture = fixtures.SimpleTrafficTests()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.t = fixture.t
        self.gate = MotorGate()
        self.sequence = 0

    def transfer(self, command=(1., .3)):
        t = self.t
        self.sequence += 1
        stamp = round(t.now * 1e9)
        self.gate.session_message(json.dumps(dict(session='regression', sequence=self.sequence,
            stamp_ns=stamp, active=True)), stamp, t.now)
        t.node._tick()
        if command is not None:
            t.node._on_controller_cmd(t.command(*command))
        t.node._publish_status()
        status = json.loads(t.node.status_pub.messages[-1].data)
        self.gate.status(json.dumps(status), stamp, t.now)
        t.advance(.01)
        if command is not None:
            self.gate.command(command, t.now)
        return self.gate.output(round(t.now*1e9), t.now), status

    def test_three_red_green_cycles_without_any_topo_points(self):
        for _ in range(3):
            self.t.confirm_signal('red', distance=None)
            output, status = self.transfer()
            self.assertEqual(output, (0., 0.))
            self.assertTrue(status['traffic_stop_required'])
            self.t.confirm_signal('green', distance=None)
            self.t.advance(1.)
            output, status = self.transfer()
            self.assertEqual(status['traffic_mode'], 'normal')
            self.assertEqual(status['traffic_reason'], 'green_resume')
            self.assertEqual(status['velocity_gate_reason'], 'waiting_fresh_visual_triplet')
            self.assertFalse(status['recording_points_valid'])
            self.assertEqual(output, (1., .3))

    def test_green_releases_even_while_topo_resume_delay_is_active(self):
        self.t.confirm_signal('red', distance=None)
        self.transfer()
        self.t.confirm_signal('green', distance=None)
        self.t.node.resume_after = self.t.now + 10.
        output, status = self.transfer()
        self.assertFalse(status['path_output_allowed'])
        self.assertEqual(status['velocity_gate_reason'], 'resume_delay')
        self.assertEqual(output, (1., .3))

    def test_green_never_replays_pre_red_command(self):
        self.transfer()
        self.t.confirm_signal('red', distance=None)
        self.transfer()
        self.t.confirm_signal('green', distance=None)
        self.t.advance(1.)
        output, status = self.transfer(command=None)
        self.assertFalse(status['traffic_stop_required'])
        self.assertEqual(output, (0., 0.))
        output, _ = self.transfer()
        self.assertEqual(output, (1., .3))

    def test_green_with_new_lya_zero_stays_zero(self):
        self.t.confirm_signal('red', distance=None)
        self.transfer()
        self.t.confirm_signal('green', distance=None)
        output, _ = self.transfer(command=(0., 0.))
        self.assertEqual(output, (0., 0.))

    def test_one_green_unknown_and_loss_do_not_release_red(self):
        self.t.confirm_signal('red', distance=None)
        self.transfer()
        self.t.signal_frame('green', distance=None)
        self.t.signal_frame('unknown', distance=None)
        self.t.advance(1.)
        output, status = self.transfer()
        self.assertTrue(status['traffic_stop_required'])
        self.assertEqual(output, (0., 0.))

    def test_manual_still_wins_red_and_returns_to_red_hold_on_release(self):
        self.t.confirm_signal('red', distance=None)
        self.gate.joy(True, (-.2, .3), self.t.now)
        output, _ = self.transfer()
        self.assertEqual(output, (-.2, .3))
        self.gate.joy(False, (0., 0.), self.t.now)
        output, _ = self.transfer()
        self.assertEqual(output, (0., 0.))


if __name__ == '__main__':
    unittest.main(verbosity=2)
