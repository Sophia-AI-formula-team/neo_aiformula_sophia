import json
from pathlib import Path
import unittest

import numpy as np

from course import Course
from driving import BUFFER, Conditions, HALF_LENGTH, HALF_WIDTH, OnlineResponse, simulate
from dynamics import fit, sample, step
from inference import wheels


DATA = Path(__file__).parent/"demo"


def load(name):
    return json.loads((DATA/name).read_text(encoding="utf8"))


class RacingTests(unittest.TestCase):
    def test_map_change_outside_sensor_horizon_is_invisible(self):
        geometry = load("course.json")
        standard, gated = Course(geometry), Course(geometry, gates=True)
        zero = np.zeros(1)
        np.testing.assert_allclose(standard.view(zero, zero, zero), gated.view(zero, zero, zero))
        at_gate = np.array([23.])
        self.assertFalse(np.allclose(standard.view(at_gate, zero, zero), gated.view(at_gate, zero, zero)))

    def test_exported_driver_matches_live_simulation_without_the_course_file(self):
        course = Course(load("course.json"), gates=True, tight=True)
        driver = load("driver.json")
        tr = simulate(course, load("dynamics.json"), driver["weights"], trace=True)[-1]
        for index in (0, 97, 421, len(tr)-3):
            _, s, offset, heading, v, yaw = tr[index,:6]
            mid, gap = course.view(np.array([s]), np.array([offset]), np.array([heading]))
            left, right = (mid+gap)[0]*course.width, (mid-gap)[0]*course.width
            actual = wheels(driver, left, right, v, yaw)
            np.testing.assert_allclose([actual["right_rpm"],actual["left_rpm"]], tr[index+1,6:8], atol=1e-7)
        with self.assertRaises(ValueError):
            wheels(driver, [1]*5, [2]*5, 0, 0)
        with self.assertRaises(ValueError):
            wheels(driver, [1]*5, [-2]*5, float("nan"), 0)

    def test_inference_uses_last_observation_and_rejects_gaps(self):
        stream = np.array([[0,0,10],[0,.1,20],[0,.4,30]])
        values, valid = sample(stream, [2], np.array([-.1,.05,.1,.31]), .11)
        np.testing.assert_array_equal(valid, [False, True, True, False])
        np.testing.assert_array_equal(values[1:,0], [10,20,20])
        with self.assertRaises(ValueError):
            fit(Path("."), Path("."), ["duplicate"], ["duplicate"])

    def test_online_estimate_moves_after_observing_changed_response(self):
        p = np.array(load("dynamics.json")["parameters"])
        other = p.copy(); other[0] *= .85; other[6] += .055
        estimate = OnlineResponse(1, p)
        wheel = np.array([185.])
        np.testing.assert_allclose(estimate.compensate(wheel,wheel), [wheel,wheel])
        v, w = np.zeros(1), np.zeros(1)
        common, diff = np.array([1.85]), np.zeros(1)
        for _ in range(160):
            nv,nw = step(v,w,common,diff,other)
            estimate.observe(v,w,common,diff,nv,nw)
            v,w = nv,nw
        self.assertAlmostEqual(estimate.speed_delta[0], other[0]-p[0], places=3)
        self.assertAlmostEqual(estimate.yaw_delta[0], .055, places=3)

    def test_finished_traces_meet_the_proxy_wheel_margin(self):
        course=Course(load("course.json"), gates=True, tight=True)
        model, actor=load("dynamics.json"), load("driver.json")["weights"]
        _, finish, minimum, tr=simulate(course, model, actor, Conditions(), trace=True)
        self.assertTrue(np.isfinite(finish[0]))
        clearance=course.clearance(tr[:,1], tr[:,2], tr[:,3], HALF_LENGTH, HALF_WIDTH)
        self.assertGreaterEqual(clearance.min(), BUFFER)
        self.assertAlmostEqual(clearance.min(), minimum[0])
        self.assertGreaterEqual(tr[-1,1], course.length)
        self.assertTrue(np.all((tr[:,6:8]>=0) & (tr[:,6:8]<=405)))
