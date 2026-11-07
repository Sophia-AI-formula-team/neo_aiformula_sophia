import dataclasses
import unittest
import numpy as np
from v3_model import Observation,Planner,LocalTracker,make_scenario,BODY_HALF_WIDTH,candidate_targets,DEFAULT_PARAMETERS
from v3_train import rank

class Contracts(unittest.TestCase):
    def test_no_global_fields_and_no_scenario_api(self):
        self.assertEqual({f.name for f in dataclasses.fields(Observation)}, {"road","objects","speed","yaw","dt"})
        with self.assertRaises(TypeError):Planner().step(make_scenario(8))

    def test_missing_road_does_not_fabricate_an_outer_reference(self):
        planner=Planner();planner.command=np.array([4.,.3])
        command=planner.step(Observation(np.empty((0,4)),np.empty((0,6)),4.,0.))
        self.assertLess(command[0],4.)
        self.assertNotIn("target",planner.last)
        self.assertEqual(planner.last["reason"],"unobserved_road")

    def test_stationary_object_compensated_from_odometry_only(self):
        tracker=LocalTracker();tracker.update([[6.,1.]],2.,0.,.2)
        out=tracker.update([[5.6,1.]],2.,0.,.2)
        self.assertAlmostEqual(out[0,2],0.,places=6)
        self.assertAlmostEqual(out[0,3],0.,places=6)

    def test_generated_parallel_cones_have_declared_gap_classes(self):
        for seed in range(15):
            for kind in ("A","B","C"):
                s=make_scenario(seed,kind);left,right=s.obstacles[:2]
                gap=right[1]-left[1]-.68
                outside=max(s.width/2-right[1]-.34,left[1]+s.width/2-.34)
                self.assertEqual(gap>2*BODY_HALF_WIDTH,kind!="B")
                if kind=="B":self.assertGreater(outside,2*BODY_HALF_WIDTH)
                if kind=="C":self.assertGreater(outside,2*BODY_HALF_WIDTH)

    def test_fast_crash_cannot_win_over_complete_lap(self):
        done=dict(completed=True,time=45,lap_length=100,progress=1,steering_tv=4)
        crash=dict(completed=False,time=.1,lap_length=100,progress=.1,steering_tv=0)
        self.assertLess(rank([done]),rank([crash]))

    def test_off_grid_narrow_opening_is_offered(self):
        road=np.array([[z,-3.,3.,.03] for z in (1.,2.,3.,4.,6.,9.)])
        cones=np.array([[3.7,.35-1.04,0,0,.34,.06],[3.7,.35+1.04,0,0,.34,.06]])
        p={**DEFAULT_PARAMETERS,"clearance":.04,"uncertainty":.8}
        look,targets=candidate_targets(Observation(road,cones,2.,0.),p)
        self.assertAlmostEqual(look,3.7)
        self.assertTrue(np.any(abs(targets-.35)<1e-8),"The real gap centre must not disappear between grid candidates")

if __name__=="__main__":unittest.main()
