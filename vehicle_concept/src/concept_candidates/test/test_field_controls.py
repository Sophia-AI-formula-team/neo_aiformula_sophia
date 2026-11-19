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

def test_speed_drop_at_observed_bend_changes_both_wheels_on_the_checked_profile():
    # A short extract of the body-frame November simulated frame 28.
    from concept_candidates.curve_road617 import RoadAwareCurvePlanner
    from concept_candidates.rear_reference617 import RecoveryReference
    from concept_feasibility.response61515 import motor_profile

    z = np.array([-3, 0, 1, 2.2, 2.7, 3.2, 3.46, 4.72, 6.5, 8., 9.5, 12.])
    left = [-2,-1.4,-1.4,-2.86,-2.85,-3.23,-3.4,-4.32,-4.55,-4.83,-4.96,-1.6]
    right = [1.5,1.4,1.4,1.73,-.81,3.21,-1.37,-2.06,-3.21,-4.14,2.66,2.25]
    road = np.column_stack((z, left, right, np.full(len(z),.06), np.full(len(z),.06)))
    obs = RecoveryReference(road, np.empty((0,77)), 2.359, -.145, .2, np.zeros(len(z)), road[:,:4], -2.96)
    engine = RoadAwareCurvePlanner(dict(pace=2., uncertainty=1.))
    with motor_profile(True, (2., -.423905)):
        engine.step(obs)
        audit = engine.last['audit']
        short = [i for i,plan in enumerate(audit['schedule']) if plan == 'drive_0.2s_then_stop_and_wait']
        assert short
        i = short[0]
        assert audit['retimed_profile_valid'][i] and audit['no_observation_brake_feasible'][i]
        assert not audit['no_observation_pass_then_stop_feasible'][i]
        engine.commit(i)
        controls = engine.reserved['controls']
        assert np.allclose(controls[0], audit['table'][i][:2], atol=1e-6, rtol=0)
        assert np.all(controls[:4,0] > 1.5) and np.all(controls[4:,0] == 0.)
        right,left = can_command(*controls[0],physical=True)
        assert int(right-left) == 160
        assert engine.last['contingency']['lower_m'] > 0.
