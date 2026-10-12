import json
import unittest
from motor_controller.gate import MotorGate, ZERO


class GateTests(unittest.TestCase):
    def setUp(self):
        self.g = MotorGate('camera')
        self.t = 100.0
        self.seq = 0

    def signal(self, state='unknown', age=0.0):
        self.seq += 1
        ns = round((self.t-age)*1e9) + self.seq
        # Tiny sequence step preserves distinct images at the same simulated instant.
        payload = dict(header=dict(frame_id='camera', stamp=dict(sec=ns//10**9, nanosec=ns%10**9)),
                       sequence=ns, state=state, distance_m=None, uncertainty_m=None)
        self.g.signal(json.dumps(payload), round(self.t*1e9)+10000, self.t)
        return payload

    def status(self, valid=True):
        self.g.status(json.dumps(dict(ros_time_sec=self.t, traffic_policy='stop_wait',
            odom_age_sec=0.01, point_age_sec=dict(a=0.01,b=0.01,c=0.01),
            recording_points_valid=valid)), round(self.t*1e9)+10000, self.t)

    def ready(self):
        self.signal(); self.status(); self.g.command((1.0,0.3), False, self.t)

    def out(self, enabled=True, owners=True):
        return self.g.output(round(self.t*1e9)+10000, self.t, enabled, owners)

    def stable(self, state):
        for _ in range(4):
            self.t += 0.1; self.signal(state)

    def test_startup_stops(self):
        self.assertEqual(self.out(), ZERO)

    def test_deployment_inhibit(self):
        self.ready(); self.assertEqual(self.out(False), ZERO)
        self.assertEqual(self.out(), ZERO)  # Cached command cannot be released.

    def test_fresh_unknown_no_prior_red_allows_lane_driving(self):
        self.ready(); self.assertEqual(self.out(), (1.0,0.3))

    def test_first_red_stops_and_stable_red_latches(self):
        self.ready(); self.t+=0.01; self.signal('red')
        self.assertEqual(self.out(), ZERO); self.assertFalse(self.g.red_latched)
        self.stable('red'); self.assertTrue(self.g.red_latched)

    def test_red_unknown_and_loss_do_not_release(self):
        self.ready(); self.stable('red'); self.stable('unknown')
        self.status(); self.g.command((1.,0.),False,self.t)
        self.assertEqual(self.out(),ZERO); self.assertTrue(self.g.red_latched)
        self.t+=1; self.assertEqual(self.out(),ZERO)

    def test_stable_green_requires_new_command(self):
        self.ready(); self.stable('red'); self.stable('green'); self.status()
        self.assertFalse(self.g.red_latched); self.assertEqual(self.out(), ZERO)
        self.t+=0.01; self.g.command((1.,0.),False,self.t)
        self.assertEqual(self.out(),(1.,0.))

    def test_one_green_does_not_release(self):
        self.stable('red'); self.t+=0.1; self.signal('green')
        self.assertTrue(self.g.red_latched)

    def test_duplicates_never_count_as_green_votes(self):
        self.stable('red'); self.t+=0.1; p=self.signal('green')
        for _ in range(20): self.g.signal(json.dumps(p),round(self.t*1e9)+10000,self.t)
        self.assertTrue(self.g.red_latched)
        self.assertEqual(sum(s=='green' for _,s in self.g.votes),1)

    def test_stale_green_does_not_release(self):
        self.stable('red'); self.t+=2; self.signal('green',1)
        self.assertTrue(self.g.red_latched)

    def test_command_timeout(self):
        self.ready(); self.t+=0.26; self.signal(); self.status()
        self.assertEqual(self.out(),ZERO)

    def test_signal_timeout(self):
        self.ready(); self.t+=0.51; self.status(); self.g.command((1.,0.),False,self.t)
        self.assertEqual(self.out(),ZERO)

    def test_source_age_includes_elapsed_time_after_receipt(self):
        self.signal(age=0.45); self.status(); self.g.command((1.,0.),False,self.t)
        self.t+=0.06; self.assertEqual(self.out(),ZERO)

    def test_topo_timeout(self):
        self.ready(); self.t+=0.61; self.signal(); self.g.command((1.,0.),False,self.t)
        self.assertEqual(self.out(),ZERO)

    def test_invalid_lane_or_odom_stops(self):
        self.ready(); self.t+=0.01; self.status(False)
        self.assertEqual(self.out(),ZERO)

    def test_manual_priority_even_during_red(self):
        self.stable('red'); self.g.joy(True,self.t)
        self.t+=0.01; self.g.command((-.2,-.3),True,self.t)
        self.assertEqual(self.out(),(-.2,-.3))
        self.assertTrue(self.g.red_latched)

    def test_manual_zero_is_not_overridden_by_auto(self):
        self.ready(); self.g.joy(True,self.t); self.t+=0.01
        self.g.command(ZERO,True,self.t); self.g.command((1.,.2),False,self.t)
        self.assertEqual(self.out(),ZERO); self.assertEqual(self.g.reason,'manual_priority')

    def test_held_joystick_loss_never_falls_back_to_auto(self):
        self.ready(); self.g.joy(True,self.t); self.t+=0.01
        self.g.command((.2,0.),True,self.t); self.t+=0.26
        self.signal(); self.status(); self.g.command((1.,0.),False,self.t)
        self.assertEqual(self.out(),ZERO); self.assertTrue(self.g.manual_held)

    def test_release_requires_new_auto_command(self):
        self.ready(); self.g.joy(True,self.t); self.t+=0.01
        self.g.command((.2,0.),True,self.t); self.assertEqual(self.out(),(.2,0.))
        self.t+=0.01; self.g.joy(False,self.t); self.assertEqual(self.out(),ZERO)
        self.t+=0.01; self.g.command((1.,0.),False,self.t)
        self.assertEqual(self.out(),(1.,0.))

    def test_nonfinite_or_out_of_range_command_stops(self):
        for command in [(float('nan'),0.),(0.,float('inf')),(5.,0.),(0.,4.)]:
            self.ready(); self.g.command(command,False,self.t)
            self.assertEqual(self.out(),ZERO)

    def test_multiple_automatic_publishers_stop(self):
        self.ready(); self.assertEqual(self.out(owners=False),ZERO)

    def test_manual_priority_even_if_multiple_auto_publishers(self):
        self.ready(); self.g.joy(True,self.t); self.t+=0.01
        self.g.command((.2,.3),True,self.t)
        self.assertEqual(self.out(owners=False),(.2,.3))

    def test_ros_clock_rollback_stops_auto(self):
        self.ready(); self.assertEqual(self.out(),(1.,.3)); self.t-=1
        self.assertEqual(self.out(),ZERO)

    def test_bad_frame_cannot_vote(self):
        p=self.signal(); p['header']['frame_id']='other'; p['state']='red'
        self.g.signal(json.dumps(p),round(self.t*1e9)+10000,self.t)
        self.assertFalse(self.g.red_latched)


if __name__ == '__main__': unittest.main()
