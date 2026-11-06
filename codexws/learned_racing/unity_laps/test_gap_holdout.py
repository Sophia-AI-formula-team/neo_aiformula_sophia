import unittest
import numpy as np
from gap_holdout_score import gate


def fixture(x, max_t=6.):
    t=np.arange(0.,max_t+.0001,.1)
    rows=np.zeros((len(t),15))
    rows[:,0]=t;rows[:,1]=x;rows[:,2]=2*t-5.;rows[:,4]=2.;rows[:,7]=2*t;rows[:,8]=x
    run=dict(rows=rows.tolist(),props=[[-1.4,0,1,0,.15,0,.5,5],[1.2,0,1,0,.12,.6,.7,5]],propMin=[.5,.4])
    spec=dict(index=0,propIndices=[0,1],role="synthetic")
    return gate(run,spec)


class TrueGapPass(unittest.TestCase):
    def test_body_through_moving_gap_and_exact_geometry_times(self):
        state=fixture(0.)
        self.assertEqual(state["route"],"middle")
        self.assertAlmostEqual(state["frontEnterS"],(5-.28-.34)/2.)
        self.assertAlmostEqual(state["rearClearS"],(5+.88+.34)/2.)
        self.assertAlmostEqual(state["entryBodyMps"],2.)
        self.assertGreater(state["minLeftProjectedM"],.45)

    def test_outside_is_not_middle_even_when_the_sector_completes(self):
        self.assertEqual(fixture(2.8)["route"],"outside_outer_lane_cone")
        self.assertEqual(fixture(-2.8)["route"],"outside_inner_lane_cone")
        self.assertEqual(fixture(1.15)["route"],"straddle_or_uncertain")

    def test_entering_and_stopping_does_not_get_a_success(self):
        self.assertEqual(fixture(0.,2.7)["route"],"unfinished")
        self.assertEqual(fixture(0.,1.4)["route"],"not_reached")


if __name__=="__main__":
    unittest.main()
