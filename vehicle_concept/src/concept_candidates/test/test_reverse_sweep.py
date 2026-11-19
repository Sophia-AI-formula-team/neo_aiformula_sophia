from types import SimpleNamespace

import numpy as np

from concept_candidates.backtrack_forward617 import checked_retreat
from concept_feasibility.response61515 import features, MODEL, motor_profile
from concept_feasibility.reverse_model617 import DATA, step


def test_recovery_checks_all_release_ticks_and_requires_supported_rear():
    obs = SimpleNamespace(road=np.array([[-3., -2., 2., .04], [10., -2., 2., .04]]),
                          objects=[], speed=0., yaw=0., dt=.2, rear_observed_limit=-3.)
    for duration in (0., .2, .4, .6, .8, 1.):
        result = checked_retreat(obs, duration, .3, 4.2, -.1, .18)
        assert result['valid']
        assert len(result['path']) == 21
        controls = result['controls']
        ticks = round(duration/.05)
        np.testing.assert_array_equal(controls[:ticks, 0], -.2)
        np.testing.assert_array_equal(controls[ticks:, 0], 0.)
    obs.road[0, 0] = -.08
    assert not checked_retreat(obs, 1., .3, 4.2)['known']
    obs.road[0, 0] = -3.
    obs.rear_observed_limit = None
    assert not checked_retreat(obs, 1., .3, 4.2)['valid']
    assert checked_retreat(obs, 0., .3, 4.2)['valid']


def test_retreat_cache_separates_motor_profiles():
    speed, yaw, dt = -.1, .13, .05
    for physical in (False, True, False):
        basis = features(-.2, 0., *MODEL['yaw'][4:6], physical=physical)
        expected = []
        for model, initial in (('v', speed), ('w', yaw)):
            tau, *gains = DATA['models'][model]
            target = float(basis @ np.asarray(gains))
            expected.append(target + (initial - target) * np.exp(-dt/tau))
        with motor_profile(physical):
            np.testing.assert_array_equal(step(speed, yaw, -.2, 0., dt, .3), expected + [0.])
