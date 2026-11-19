import numpy as np

from concept_candidates.field_controls import (
    DIFFERENTIAL_SLEW, MAX_YAW, forward_ceiling, forward_steering)
from concept_feasibility.response61515 import can_command, motor_profile, motor_steering


def test_field_ramp_uses_applied_zero_and_cannot_wind_up_while_stalled():
    applied = 0.
    for expected in (.2, .4, .5, .5):
        with motor_profile(True, (applied, 0.)):
            applied = forward_ceiling(0., .2, 4.)
        np.testing.assert_allclose(applied, expected)
    with motor_profile(True):
        assert forward_ceiling(3., .49, 4.) == .2
    with motor_profile(True, (3.9, .1)):
        assert forward_ceiling(3.9, .2, 4.) == 4.
        assert forward_ceiling(1.2, .2, 1.3) == 1.3


def test_field_launch_checks_both_real_wheels_and_turn_slew():
    drive = np.array([.2, .2, .2])
    desired = np.array([-1.55, 0., 1.55])
    previous = np.zeros_like(drive)
    u = forward_steering(desired, drive, 0., previous, .2)
    raw = motor_steering(-u, physical=True)
    assert (abs(raw) <= DIFFERENTIAL_SLEW * .2 + 1e-10).all()
    right, left = can_command(drive, u, physical=True)
    assert np.all(right > 0) and np.all(left > 0)
    assert np.allclose(u[1], 0.)
    assert (right[0] > left[0]) and (right[2] < left[2])
    assert np.max(np.abs(u)) < MAX_YAW


def test_field_turn_speed_cap_and_unrepresentable_request():
    for speed in (.05, .2, .5, 1., 4.):
        q = np.full(3, speed)
        desired = np.array([-.06211189553141594, .001, 0.])
        assert np.all(forward_steering(desired, q, speed, np.zeros(3), .2) == 0.)
        for sign in (-1., 1.):
            last = np.zeros(3)
            for _ in range(20):
                last = forward_steering(np.full(3, sign*2.), q, speed, last, .2)
            assert np.all(np.abs(last) <= min(.65, 1./speed) + 1e-8)
            assert np.all(last*sign >= 0)
            assert np.all(forward_steering(last, np.zeros(3), speed, last, .2) == 0.)
