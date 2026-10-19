import json
import unittest
from motor_controller.gate import MotorGate, ZERO, joystick_command


class GateTests(unittest.TestCase):
    def setUp(self):
        self.g=MotorGate(); self.t=100.; self.sequence=0

    def command(self, value=(1.,.3)):
        self.g.command(value,self.t)

    def out(self):
        return self.g.output(round(self.t*1e9),self.t)

    def session(self, active=True, identity='one'):
        self.sequence+=1
        data=dict(session=identity, sequence=self.sequence, stamp_ns=round(self.t*1e9),active=active)
        self.g.session_message(json.dumps(data),round(self.t*1e9),self.t)
        return data

    def status(self, go=True):
        data=dict(ros_time_sec=self.t,traffic_policy='stop_wait',traffic_mode='normal' if go else 'wait',
                  traffic_stop_required=not go,traffic_reason='normal' if go else 'red_wait_green',
                  velocity_gate_reason='forwarding_controller' if go else 'red_wait_green',path_output_allowed=go)
        self.g.status(json.dumps(data),round(self.t*1e9),self.t)
        return data

    def enabled(self):
        self.session();self.status();self.t+=.01;self.command()

    def test_default_legacy_without_semantic_inputs(self):
        self.command();self.assertEqual(self.out(),(1.,.3));self.assertEqual(self.g.reason,'legacy_passthrough')

    def test_legacy_ignores_red_status_without_session(self):
        self.status(False);self.command();self.assertEqual(self.out(),(1.,.3))

    def test_legacy_does_not_need_camera_odom_points_or_clock(self):
        self.command();self.assertEqual(self.g.output(0,self.t),(1.,.3))

    def test_existing_large_finite_command_not_silently_rejected(self):
        self.command((4.1,3.1));self.assertEqual(self.out(),(4.1,3.1))

    def test_zero_and_command_timeout(self):
        self.command(ZERO);self.assertEqual(self.out(),ZERO)
        self.command();self.t+=.51;self.assertEqual(self.out(),ZERO)

    def test_nonfinite_command_stops(self):
        self.command();self.command((float('nan'),0.));self.assertEqual(self.out(),ZERO)

    def test_semantic_start_waits_for_status_and_new_command(self):
        self.command();self.session();self.assertEqual(self.out(),ZERO)
        self.status();self.assertEqual(self.out(),ZERO)
        self.t+=.01;self.command();self.assertEqual(self.out(),(1.,.3))

    def test_semantic_red_stop_green_needs_new_command(self):
        self.enabled();self.assertEqual(self.out(),(1.,.3))
        self.t+=.01;self.status(False);self.command();self.assertEqual(self.out(),ZERO)
        self.t+=.01;self.status();self.assertEqual(self.out(),ZERO)
        self.t+=.01;self.command();self.assertEqual(self.out(),(1.,.3))

    def test_clean_exit_returns_to_legacy_without_extra_flag(self):
        self.enabled();self.t+=.01;self.status(False);self.session(False)
        self.assertFalse(self.g.semantic_active);self.assertEqual(self.out(),ZERO)
        self.t+=.01;self.command();self.assertEqual(self.out(),(1.,.3));self.assertEqual(self.g.reason,'legacy_passthrough')

    def test_crash_does_not_bypass_red(self):
        self.enabled();self.t+=.7;self.command();self.assertEqual(self.out(),ZERO)
        self.assertTrue(self.g.semantic_active)

    def test_status_loss_stops_even_with_session_heartbeat(self):
        self.enabled();self.t+=.7;self.session();self.command();self.assertEqual(self.out(),ZERO)

    def test_old_status_cannot_resume(self):
        self.enabled();old=self.status();self.t+=.02;self.status(False)
        self.g.status(json.dumps(old),round(self.t*1e9),self.t);self.command();self.assertEqual(self.out(),ZERO)

    def test_wrong_policy_stops(self):
        self.enabled();self.t+=.01;d=self.status();d['traffic_policy']='legacy';d['ros_time_sec']+=.001
        self.g.status(json.dumps(d),round((self.t+.001)*1e9),self.t);self.command();self.assertEqual(self.out(),ZERO)

    def test_stale_or_foreign_release_cannot_bypass_gate(self):
        self.enabled();d=self.session(False,'foreign');self.assertTrue(self.g.semantic_active)
        d['session']='one';d['stamp_ns']-=10**9
        self.g.session_message(json.dumps(d),round(self.t*1e9),self.t)
        self.assertTrue(self.g.semantic_active)

    def test_delayed_active_after_clean_stop_cannot_rearm(self):
        self.enabled();self.session(False);self.session(True)
        self.assertFalse(self.g.semantic_active)

    def test_new_semantic_session_after_clean_stop(self):
        self.enabled();self.session(False);self.t+=.01;self.session(True,'two')
        self.assertTrue(self.g.semantic_active);self.assertEqual(self.out(),ZERO)

    def test_manual_priority_in_all_three_modes(self):
        for semantic in (False,True):
            if semantic:self.session();self.status(False)
            self.g.joy(True,(-.2,.3),self.t);self.command((3.,-1.))
            self.assertEqual(self.out(),(-.2,.3))
            self.g.joy(False,ZERO,self.t)

    def test_manual_zero_still_owns_control(self):
        self.g.joy(True,ZERO,self.t);self.command();self.assertEqual(self.out(),ZERO)
        self.assertEqual(self.g.reason,'manual_priority')

    def test_manual_loss_stops_instead_of_falling_back(self):
        self.g.joy(True,(.2,.3),self.t);self.t+=.51;self.command();self.assertEqual(self.out(),ZERO)
        self.assertTrue(self.g.manual_held)

    def test_manual_release_requires_new_lya(self):
        self.command();self.g.joy(True,(.2,.3),self.t);self.t+=.01;self.g.joy(False,ZERO,self.t)
        self.assertEqual(self.out(),ZERO);self.t+=.01;self.command();self.assertEqual(self.out(),(1.,.3))

    def test_manual_can_take_over_semantic_crash(self):
        self.enabled();self.t+=1;self.g.joy(True,(.1,.1),self.t);self.assertEqual(self.out(),(.1,.1))

    def test_topo_diagnostic_hold_does_not_block_traffic_go(self):
        self.session()
        for reason in ('waiting_fresh_visual_triplet', 'controller_command_timeout', 'resume_delay', 'avoidance_active'):
            self.t += .01
            d=self.status();d.update(velocity_gate_reason=reason,path_output_allowed=False)
            d['ros_time_sec'] += .001
            self.g.status(json.dumps(d),round((self.t+.001)*1e9),self.t)
            self.t += .002;self.command()
            self.assertEqual(self.out(),(1.,.3),reason)

    def test_old_or_inconsistent_semantic_protocol_stops(self):
        self.enabled()
        for invalid in (None, 'false', 0, True):
            self.t += .01
            d=self.status();d.update(traffic_stop_required=invalid,ros_time_sec=self.t+.001)
            self.g.status(json.dumps(d),round((self.t+.001)*1e9),self.t)
            self.command();self.assertEqual(self.out(),ZERO)
        self.t += .01
        d=self.status();del d['traffic_stop_required'];d['ros_time_sec']+=.001
        self.g.status(json.dumps(d),round((self.t+.001)*1e9),self.t)
        self.command();self.assertEqual(self.out(),ZERO)


class JoystickTests(unittest.TestCase):
    def setUp(self):
        self.p=dict(enable_button=6,enable_turbo_button=4,axis_linear=1,axis_angular=3,
                    scale_linear=3.5,scale_linear_turbo=3.6,scale_angular=.7,scale_angular_turbo=1.)
        self.axes=[0.,.4,0.,-.5];self.buttons=[0]*13

    def test_no_enable(self):
        self.assertEqual(joystick_command(self.axes,self.buttons,self.p),(False,ZERO))

    def test_l2_axes_match_teleop(self):
        self.buttons[6]=1;held,cmd=joystick_command(self.axes,self.buttons,self.p)
        self.assertTrue(held);self.assertAlmostEqual(cmd[0],1.4);self.assertAlmostEqual(cmd[1],-.35)

    def test_l1_turbo(self):
        self.buttons[4]=1;held,cmd=joystick_command(self.axes,self.buttons,self.p)
        self.assertTrue(held);self.assertAlmostEqual(cmd[0],1.44);self.assertAlmostEqual(cmd[1],-.5)

    def test_invalid_joy_rejected(self):
        with self.assertRaises(ValueError):joystick_command([],[],self.p)
        self.buttons[6]=1;self.axes[1]=float('nan')
        with self.assertRaises(ValueError):joystick_command(self.axes,self.buttons,self.p)


if __name__=='__main__':unittest.main()
