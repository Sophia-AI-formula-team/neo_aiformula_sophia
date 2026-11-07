"""Behavior contracts for the new candidate rollout; no Unity-pass claim."""
import unittest
import numpy as np
from v34_road import RoadObservation,EvidenceRoad
from v34_controller import Planner34,PersistentTracker

class Contracts(unittest.TestCase):
    def observation(self,objects=()):
        z=np.linspace(1.,12.,40);road=np.c_[z,np.full(len(z),-2.),np.full(len(z),2.),np.full(len(z),.04)]
        return RoadObservation(road,np.asarray(objects,float).reshape(-1,6),2.,0.,.2,np.zeros(len(z)))
    def test_no_feasible_action_cannot_accelerate(self):
        p=Planner34();p.command=np.array([3.,.4])
        action=p.step(self.observation([[.4,0.,0.,0.,1.5,.1]]))
        self.assertEqual(p.last['reason'],'no_feasible_trajectory')
        self.assertEqual(action[0],0.)
    def test_missing_near_cone_not_deleted_after_point_six_seconds(self):
        t=PersistentTracker();t.update([[1.5,0.]],0,0,.2)
        for _ in range(4):objects=t.update([],0,0,.2)
        self.assertEqual(len(objects),1)
        self.assertGreater(objects[0,5],.07)
    def test_passed_obstacle_can_retire(self):
        t=PersistentTracker();t.update([[.5,0]],0,0,.2)
        for _ in range(8):objects=t.update([],2,0,.2)
        self.assertEqual(len(objects),0)
    def test_audit_is_actual_action(self):
        p=Planner34();action=p.step(self.observation())
        table=np.array(p.last['audit']['table']);pick=p.last['audit']['selected']
        np.testing.assert_allclose(action,table[pick,:2],atol=1e-6)
        self.assertEqual(pick,int(np.argmax(table[:,7])))
    def test_no_global_state_contract(self):
        with self.assertRaises(TypeError):Planner34().step(dict(global_position=[0,0],map=[]))
    def test_far_uncertain_object_does_not_shrink_near_road(self):
        p=Planner34();action=p.step(self.observation([[100,50,0,0,.34,8.]]))
        self.assertGreater(action[0],0.)
    def test_dark_seam_and_wide_white_paint_are_not_road_edges(self):
        image=np.full((360,640,3),95,np.uint8)
        image[295:,250:275]=0
        image[235:255,280:400]=245
        road,pixels,_=EvidenceRoad().read(image,np.zeros((360,640),np.uint8))
        self.assertTrue(all(p[0]==0 and p[1]==639 for p in pixels))
        self.assertTrue(np.all(road[:,1]<-.9))
    def test_first_command_slew_uses_sensor_interval(self):
        obs=self.observation();obs.heading[:]=.5
        p=Planner34();action=p.step(obs)
        self.assertLessEqual(abs(action[1]),p.p['slew']*obs.dt+1e-7)

if __name__=='__main__':unittest.main()
