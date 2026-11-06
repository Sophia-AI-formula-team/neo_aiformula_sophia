"""Small acceptance-regression tests using transparent synthetic telemetry, without Unity."""
import unittest
import numpy as np
from smooth_window_metrics import abs_integral, high_speed_distance, intervals_report, reversals
from smooth_holdout_score import criteria, bin_margin

def trace(t,s,speed):
    rows=np.zeros((len(t),15));rows[:,0]=t;rows[:,1]=s;rows[:,7]=s;rows[:,4]=speed
    return rows


class HoldoutSignals(unittest.TestCase):
    def test_measured_threshold_and_interpolated_gate(self):
        rows=trace([0,1,2],[0,4,8],[3,5,3])
        self.assertEqual(high_speed_distance(rows),(4.,4.))
        window=intervals_report(rows,[[2,6]])["totals"]
        self.assertEqual(window["progressSpeedMps"],4.)
        self.assertEqual(window["fractionPlannedAtLeast4"],1.)
        self.assertAlmostEqual(window["measuredBodyMeanMps"],4.5)
        self.assertAlmostEqual(window["measuredBodyMedianMps"],4.5)
        rows[:,5]=99.  # a command cannot improve measured-speed attainment
        self.assertEqual(high_speed_distance(rows),(4.,4.))

    def test_failure_keeps_missing_gate_distance_and_stops(self):
        rows=trace([0,1,2,3],[0,4,4,8],[4,4,0,4])
        w=intervals_report(rows,[[0,16]])["totals"]
        self.assertEqual(w["coverage"],.5)
        self.assertEqual(w["seconds"],3.)
        self.assertEqual(w["fractionPlannedAtLeast4"],.25)
        missing=intervals_report(rows,[[12,16]])["totals"]
        self.assertEqual(missing["coverage"],0.)
        self.assertIsNone(missing["progressSpeedMps"])

    def test_backtracking_is_never_credited_twice(self):
        rows=trace([0,1,2,3],[0,4,2,6],[4,4,4,4])
        g=intervals_report(rows,[[0,6]])["totals"]
        self.assertEqual(g["progressM"],6.)
        self.assertEqual(g["atLeast4M"],6.)
        self.assertEqual(g["bodyPathM"],10.)
        self.assertEqual(g["excessPathM"],4.)

    def test_zero_crossing_integral_and_flip_deadband(self):
        self.assertEqual(abs_integral(np.array([1.,-1.]),np.array([0.,1.])),.5)
        self.assertEqual(reversals(np.array([0.,.2,.01,-.02,-.14,.02,.17])),2)

    def test_thin_positive_margin_counts_as_physical_success(self):
        case=dict(scope="lap",limitM=280.87656260722264)
        run=dict(status="lap",length=280.87656260722264,gate=32,progress=281.,
                 seconds=75.,minRoad=0.,propMin=[.004],zoneEnd=[10.],rows=[[0.]*7+[281.]],
                 props=[[0.]*7+[30.]])
        self.assertTrue(criteria(run,case)["physicalSuccess"])
        self.assertEqual(bin_margin(.004),"[0,.10)")
        run["propMin"]=[0.]
        self.assertFalse(criteria(run,case)["physicalSuccess"])


if __name__=="__main__":
    unittest.main()
