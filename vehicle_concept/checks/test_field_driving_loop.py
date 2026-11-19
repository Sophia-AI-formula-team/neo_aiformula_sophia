from unittest.mock import patch
import math
import numpy as np

from field_driving_loop import camera, Drive, EMPTY, motor_profile


def test_extra_shape_is_checked_and_keeps_all_75_slot_ids_after_grouping():
    image, road, lines = camera(0., -2.25, math.radians(-14))
    rpm = 1.4 * 30. / (math.pi * .127)
    sensor = dict(t=0., leftRpm=rpm, rightRpm=rpm, wheelRadius=.127, gyro=0.,
                  field_ground_projection=True, source='synthetic_case')
    args = (image, road, lines, EMPTY, sensor)
    driver, control = Drive(2.), Drive(2.)

    def ungrouped(targets, commands):
        indices = np.arange(len(commands))
        return indices, indices

    with motor_profile(True, (1.4, 0.)):
        measured = driver.step(*args)
        with patch('concept_candidates.curve_road617.distinct_requests', ungrouped):
            repeated = control.step(*args)
    assert len(measured['candidates']) == 75
    assert measured['edge_candidate_index'] is not None
    assert measured['edge_candidate_index'] == repeated['edge_candidate_index']
    assert measured['selected_index'] == repeated['selected_index']
    for a,b in zip(measured['candidates'], repeated['candidates']):
        for k in ('allowed','selectable','index'):
            assert a[k] == b[k]
        for k in ('speed_mps','yaw_right_radps','clearance_m','score','path'):
            np.testing.assert_allclose(a[k], b[k], rtol=0., atol=1e-8)
