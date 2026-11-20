"""Golden values come from the 11/19 installed LYA callback, including its dx term."""
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from concept_candidates.tracking import controller_record, first_lya_depth, neo_lya, tracking_request
from concept_drive.core import Drive
from concept_feasibility.response61515 import motor_profile


@pytest.mark.parametrize('inputs, expected', [
    ((2.5, 2.2, 0., 0., 0., 2.), (2.00374, -.04000044567373051)),
    ((1.7, 1.3, .17, -.24, .07, 2.), (1.916849965404755, .45)),
    ((2., -1.4, -.23, .07, -.32, 1.1), (1.0660927488555654, -.28588261972606654)),
    ((1.5, 1.5, 0., .4, 1., 2.), (1.8446719880057703, -.45)),
])
def test_installed_lya_formula(inputs, expected):
    np.testing.assert_allclose(neo_lya(*inputs), expected, atol=1e-13, rtol=0)


def test_lya_uses_target_pose_and_its_own_reference_speed_and_honours_stop():
    assert first_lya_depth(.95, 3.) == 1.
    assert first_lya_depth(1.05, 3.5) == 1.5
    assert first_lya_depth(1.05, 3.4) is None
    assert first_lya_depth(None, None) is None
    zero = np.zeros(3)
    requested = np.array([0., .12, 2.])
    params = dict(tracking_controller='lya', lya_odom_heading_right_rad=np.pi)
    trace = {}
    drive, yaw = tracking_request(zero, zero, zero, zero, zero, requested, zero, zero, zero, 3.,
                                  params, 1., trace)
    assert yaw[0] == 0.
    np.testing.assert_allclose(trace['vref'], 2.)
    np.testing.assert_allclose(drive, [0., 1.9983, 1.9983], atol=1e-12)
    assert yaw[2] > 0.  # Installed ROS bias turns to the car's right.
    assert [float(p[0][0]) for p in trace['points']] == [1., 2., 3.]
    trace.clear()
    drive, yaw = tracking_request(zero, zero, zero, zero, zero, requested, zero, zero, zero, 3.,
                                  params, None, trace)
    assert not trace and not drive.any() and not yaw.any()


@pytest.mark.parametrize('minimum, pace, expected', [
    (.5, 2., [0., .5, 1., 2.]), (0., 2., [0., .12, 1., 2.]),
    (.5, .4, [0., 0., 0., 0.]), (.8, 2., [0., .8, 1., 2.]),
])
def test_configured_forward_requests_preserve_stop_and_ceiling(minimum, pace, expected):
    from concept_candidates.field_controls import forward_request
    np.testing.assert_array_equal(forward_request(np.array([0., .12, 1., 3.]), minimum, pace), expected)


def test_lya_reference_speed_and_final_checked_limits_are_independent():
    zero = np.zeros(2)
    trace = {}
    params = dict(tracking_controller='lya', lya_odom_heading_right_rad=0., lya_v_t_mps=1.4,
                  min_nonzero_speed_mps=.5, forward_command_ceiling=.6)
    speed, yaw = tracking_request(zero, zero, zero, zero, zero, np.array([0., .2]), zero, zero, zero,
                                  3., params, 1., trace)
    np.testing.assert_array_equal(trace['vref'], 1.4)
    assert trace['raw_speed'][1] > 1.4 and yaw[0] == 0.
    np.testing.assert_array_equal(speed, [0., .6])


@pytest.mark.parametrize('mode,count', [('pure_pursuit',1), ('lya',3)])
def test_actual_first_control_and_sampled_points_match_selected_checked_candidate(mode, count):
    path = Path(__file__).parent/'data'/'moving-lap-start'
    frame = path/'00000'
    image = cv2.imread(str(frame)+'_rgb.png')
    sensor = json.loads((path/'expected.json').read_text())[0]['sensor']
    driver = Drive(2., mode)
    with np.load(str(frame)+'_perception.npz') as det, motor_profile(False):
        d = driver.step(image, det['raw'], det['lane'], det, sensor)
    info = d['tracking_controller']
    assert info['active'] and info['owner'] == 'candidate' and len(info['points_zx_m']) == count
    assert info['target_zx_m'] == info['points_zx_m'][0]
    assert [info['checked_speed_mps'], info['checked_yaw_right_radps']] == d['command_native']
    chosen = d['candidates'][d['selected_index']]
    assert chosen['selectable'] and chosen['path'] == d['selected_path']
    np.testing.assert_allclose(d['command_native'], [chosen['speed_mps'], chosen['yaw_right_radps']], atol=1.e-6)
    if mode == 'lya':
        a,b,c = np.asarray(info['points_zx_m'])
        np.testing.assert_allclose(np.diff([a[0],b[0],c[0]]), 1.)
        h1, h2 = np.arctan2(b[1]-a[1], b[0]-a[0]), np.arctan2(c[1]-b[1], c[0]-b[0])
        assert np.isclose(info['target_heading_right_rad'], h1)
        assert np.isclose(info['reference_yaw_right_radps'], (h2-h1)/.1)
    json.dumps(d, allow_nan=False)


@pytest.mark.parametrize('owner', ['edge_recovery','reverse','contingency','stop','unavailable'])
def test_independent_actions_do_not_inherit_a_tracking_target(owner):
    record = controller_record('lya', [0., 0.], owner)
    assert not record['active'] and not record['points_zx_m'] and record['target_zx_m'] is None

def test_only_a_superseded_point_hands_steering_to_the_edge_guard():
    from concept_candidates.edge_recovery import preview_control

    # Construct two requests against the 11/19 frame-25 exterior fit.
    # One points outwards; the other already asks for a stronger inward turn.
    edge = (-1, .19, -1.73, 5.72)
    desire = np.array([-.15, 2.])
    base = np.zeros(2)
    drive = np.full(2, .5)
    args = (edge, desire, base, base, base, np.full(2, 1.9), np.full(2, -.09),
            np.full(2, -.218), drive, .2, 2.)
    capture = dict(points=((np.ones(2), base),), requested=desire, vref=drive)
    with motor_profile(True):
        ordinary = preview_control(*args)
        measured = preview_control(*args, capture=capture)
    np.testing.assert_array_equal(ordinary, measured)
    assert list(capture['edge_override']) == [True, False]
    # Exercise the real index mapping: each request can represent several slots.
    frame = capture, [1, 0, 1]
    overridden = controller_record('lya', [measured[0][0], measured[1][0]], 'candidate', frame, 1)
    limited = controller_record('pure_pursuit', [measured[0][1], measured[1][1]], 'candidate', frame, 2)
    assert overridden['owner'] == 'edge_recovery' and not overridden['active']
    assert not overridden['points_zx_m'] and overridden['target_zx_m'] is None
    assert limited['owner'] == 'candidate' and limited['active']
    assert limited['requested_yaw_right_radps'] != limited['checked_yaw_right_radps']
