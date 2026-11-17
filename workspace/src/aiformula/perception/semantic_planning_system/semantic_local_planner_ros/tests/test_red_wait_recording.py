"""Red holds speed without poisoning next-lap reference; unsafe inputs excluded."""
import unittest
import test_map_assist as fixtures

class RedWaitRecordingTests(unittest.TestCase):
    def setUp(self):
        self.h = fixtures.MapAssistTests(methodName='runTest')
        self.h.setUp()
        self.addCleanup(self.h.doCleanups)
        self.h.node.close_fusion_worker()
        self.h.traffic_node()
        self.addCleanup(self.h.node.close_fusion_worker)
        self.h.triplet()
        self.h.odom()
        self.h.confirm_light('red')
        self.assertEqual(self.h.node.traffic_mode, 'wait')

    def sample(self):
        self.h.advance(.1)
        self.h.triplet()
        self.h.odom(.2,0.,0.)
        self.h.advance(.1)
        self.h.triplet()
        self.h.odom(.4,0.,0.)
        return self.h.node.samples[-1]

    def test_red_wait_keeps_valid_recording_and_velocity_hold(self):
        self.assertFalse(self.sample().excluded)
        self.assertEqual(self.h.node._velocity_hold_reason(self.h.now), self.h.node.traffic_reason)

    def test_branch_remains_excluded(self):
        self.h.node.branch_active = True
        self.assertTrue(self.sample().excluded)

    def test_avoidance_remains_excluded(self):
        self.h.node.avoidance_active = True
        self.assertTrue(self.sample().excluded)

    def test_stale_points_remain_excluded(self):
        self.h.advance(2.)
        self.h.odom(.2,0.,0.)
        self.assertTrue(self.h.node.samples[-1].excluded)



    def test_resume_delay_does_not_poison_valid_recording(self):
        self.h.node.traffic_mode = 'normal'
        self.h.node.resume_after = self.h.now + 10.
        self.assertFalse(self.sample().excluded)
        self.assertEqual(self.h.node._velocity_hold_reason(self.h.now), 'resume_delay')

    def test_resume_delay_does_not_hide_active_branch(self):
        self.h.node.resume_after = self.h.now + 10.
        self.h.node.branch_active = True
        self.assertTrue(self.sample().excluded)
