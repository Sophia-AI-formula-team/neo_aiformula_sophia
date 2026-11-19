import math

import numpy as np
import pytest

from concept_candidates.edge_recovery import corroborated_previous_edge, edge_control, fit_edge, preview_audit, preview_control
from concept_candidates.field_controls import DIFFERENTIAL_SLEW, OUTWARD_RELEASE_SLEW, HALF_TREAD, WHEEL_SPEED_FRACTION, forward_steering
from concept_feasibility.response61515 import motor_steering, motor_profile


def observations(tangent, near_side):
    z = np.arange(1., 6.01, .25)
    left, right = (-1.3, 5.) if near_side == -1 else (-5., 1.3)
    road = np.column_stack((z, left/np.cos(tangent)+z*np.tan(tangent),
                            right/np.cos(tangent)+z*np.tan(tangent),
                            np.full_like(z, .05), np.full_like(z, .05)))
    return road, [[float(p), 'paint_material_edge', 'connected_material_edge'] for p in z]


@pytest.mark.parametrize('side', [-1, 1])
def test_nearest_observed_exterior_and_no_prescribed_width(side):
    tangent = math.radians(-14*side)
    road, observed = observations(tangent, side)
    fit = fit_edge(road, observed)
    np.testing.assert_allclose(fit, (side, tangent, side*1.3), atol=1e-10)
    # An unrelated change of the far boundary cannot move the fitted edge.
    road[:, 2 if side < 0 else 1] -= side*3.
    np.testing.assert_allclose(fit_edge(road, observed), fit, atol=1e-10)


def test_no_edge_from_lane_mask_divider_distant_or_disconnected_samples():
    road, observed = observations(.2, -1)
    assert fit_edge(road, [[r[0], 'road_head', 'paint_connected_asphalt'] for r in observed]) is None
    assert fit_edge(road, observed[-8:]) is None
    assert fit_edge(road, observed[::4]) is None
    assert fit_edge(road, []) is None


def left_observations(z, left):
    return (np.column_stack((z, left, np.full_like(z, 4.))),
            [[float(depth), 'paint_material_edge', 'road_head'] for depth in z])


def test_measured_bend_can_supply_a_local_tangent_for_the_proposal():
    # Rounded confirmed left-edge samples in the 17:08:17 field recording,
    # frame 22. The visible bend is not straight over its entire 3 m extent.
    z = np.array([2.21, 2.46, 2.71, 2.96, 3.21, 3.46, 3.71, 4.21, 4.47, 4.72, 4.97, 5.22])
    x = np.array([-2.563, -2.186, -1.943, -1.706, -1.551, -1.423,
                  -1.275, -1.133, -1.073, -1.019, -1.012, -.967])
    result = fit_edge(*left_observations(z, x))
    assert result is not None
    side, angle, signed_distance = result
    assert side == -1 and .20 < angle < .45 and -3 < signed_distance < -1.5


def test_isolated_wrong_marking_does_not_remove_a_measured_near_edge():
    z = np.arange(1., 3.01, .25)
    x = -1.5 + .25*z
    x[4] += .8
    fit = fit_edge(*left_observations(z, x))
    angle = np.arctan(.25)
    np.testing.assert_allclose(fit, [-1, angle, -1.5*np.cos(angle)], atol=1e-9)


@pytest.mark.parametrize('alternating', [False, True])
def test_two_incompatible_edges_do_not_supply_a_turn_direction(alternating):
    z = np.arange(1., 4.01, .25)
    x = -1.5 + .20*z
    branches = np.arange(len(z)) % 2 == 0 if alternating else z >= 2.75
    x[branches] -= .95
    assert fit_edge(*left_observations(z, x)) is None


def test_samples_from_the_other_side_of_the_car_remain_ineligible():
    z = np.arange(1., 4.01, .25)
    assert fit_edge(*left_observations(z, 1.2+.20*z)) is None


@pytest.mark.parametrize('side', [-1, 1])
def test_edge_shape_builds_differential_inside_the_actual_motor_limits(side):
    angle = math.radians(-14*side)
    edge = (side, angle, side*1.3)
    last_q, last_w = 1.4, 0.
    with motor_profile(True):
        for _ in range(6):
            q, w = edge_control(edge, 0., 0., 0., 1.4, 0., last_q, last_w, .2, 2.)
            old_raw = motor_steering(-np.asarray(last_w), physical=True)
            raw = motor_steering(-np.asarray(w), physical=True)
            assert w*side < 0 and 0. < q <= min(last_q+.2, 1.9)
            assert abs(raw-old_raw) <= DIFFERENTIAL_SLEW*.2+1e-9
            assert abs(raw)*HALF_TREAD <= WHEEL_SPEED_FRACTION*q+1e-9
            last_q, last_w = q, w
    assert last_q > .9 and abs(last_w) > .02


def test_vectorized_rollout_matches_individual_shapes():
    edge = (-1, .24, -1.3)
    samples = [(0., 0., 0., 1.4, 0., 1.4, 0.), (1., .05, .2, .9, .1, .55, .03)]
    with motor_profile(True):
        expected = [edge_control(edge, *s, .2, 2.) for s in samples]
        command = edge_control(edge, *np.asarray(samples).T, .2, 2.)
    np.testing.assert_allclose(np.array(command).T, expected)

@pytest.mark.parametrize('side', [-1, 1])
def test_diverging_camera_edge_releases_outward_differential_before_full_tyre_exit(side):
    # Same relative geometry on either side, 1.5 m now but converging at 18бу.
    road, observed = observations(-.31*side, side)
    edge = fit_edge(road, observed, include_extent=True)
    assert edge[-1] == pytest.approx(6.)
    previous = side*.2
    for _ in range(10):
        _, result = preview_control(edge, side*.2, 0., 0., 0., 1.9, side*.1, previous, 1.6, .2)
        old = motor_steering(-previous, True)
        new = motor_steering(-result, True)
        assert abs(new-old) <= OUTWARD_RELEASE_SLEW*.2 + 1e-8
        assert abs(new)*.30 <= .5*1.6 + 1e-10
        if side*result < 0.:
            break
        assert abs(new) < abs(old)
        previous = result
    else:
        pytest.fail("Outward differential never crossed through equal wheel speeds")


@pytest.mark.parametrize('edge,z,heading', [(None, 0., 0.), ((-1,.30,-1.4,2.), 1.9, 0.),
                                             ((-1,.30,-1.4,5.), 0., .45),
                                             ((-1,.01,-2.,5.), 0., 0.)])
def test_unobserved_beyond_support_open_or_already_correcting_uses_ordinary_steer(edge,z,heading):
    desired, previous = -.12, -.1
    assert preview_control(edge, desired, z, 0., heading, 1.7, 0., previous, 1.5, .2)[1] == (
        forward_steering(desired, 1.5, 1.7, previous, .2))


def test_near_internal_lane_line_cannot_trigger_preview_guard():
    road, observed = observations(.3, -1)
    observed = [[z, 'paint_connected_asphalt', 'road_head'] for z, *_ in observed]
    assert fit_edge(road, observed, include_extent=True) is None

def test_transported_tangent_needs_this_photos_independent_matching_material_rows():
    road, observed = observations(.3, -1)
    edge = fit_edge(road, observed, include_extent=True)
    road = road.copy()
    # Advance 0.2 m in the previous camera frame. The old line has moved
    # closer by sin(angle)*0.2 m. Only its far strip is visible in this photo.
    road[:, 1] += .2*np.tan(.3)
    far = [row for row in observed if 3.5 <= row[0] <= 5.5]
    assert fit_edge(road, far, include_extent=True) is None
    confirmed = corroborated_previous_edge(edge, road, far, 1., 0., .2)
    np.testing.assert_allclose(confirmed, [-1, .3, edge[2]+.2*np.sin(.3), 5.5])
    assert corroborated_previous_edge(edge, road, [], 1., 0., .2) is None
    assert corroborated_previous_edge(edge, road, far, 1., 0., .8) is None
    road[:, 1] -= .5
    assert corroborated_previous_edge(edge, road, far, 1., 0., .2) is None


def test_audit_carries_signed_observed_geometry_and_distinguishes_an_absent_edge():
    edge = (-1, .3, -1.3, 5.)
    trace = preview_audit(edge, 'camera', -.05)
    assert trace['triggered'] and trace['side'] == 'left' and trace['normal_distance_m'] == -1.3
    assert trace['projected_center_clearance_m'] < trace['desired_center_clearance_m']
    assert preview_audit(None, '', 0.) == dict(source='no_supported_exterior', side=None, triggered=False)

def test_saved_forward_then_brake_rechecks_steering_against_the_actual_motor():
    from types import SimpleNamespace
    from concept_candidates.curve_road617 import RoadAwareCurvePlanner

    planner = RoadAwareCurvePlanner(dict(pace=2.))
    planner.current_observation = SimpleNamespace(road=np.zeros((4,5)), speed=1., dt=.2)
    planner.reserved = dict(issued_at=0., controls=np.array([[.8, .55], [0., 0.]]))
    with motor_profile(True, (.8, 0.)):
        assert planner.resume_contingency() is False
    assert planner.reserved is None

def test_early_release_uses_the_real_wheel_difference_and_a_coordinated_drive_floor():
    # Frame 25 of the field approach had a substantial outward command.
    # A 0.5 m/s slot would otherwise erase more than the whole release budget.
    edge, previous, dt = (-1,.19,-1.73,5.72), -.218, .2
    drive, yaw = preview_control(edge, -.15, 0., 0., 0., 1.9, -.09, previous, .5, dt, 2.)
    assert drive > .5 and yaw < 0.
    assert abs(motor_steering(-yaw, True)-motor_steering(-previous, True)) == pytest.approx(OUTWARD_RELEASE_SLEW*dt)
    assert preview_control(edge, -.15, 0., 0., 0., 1.9, -.09, previous, 0., dt, 2.) == (0., 0.)
