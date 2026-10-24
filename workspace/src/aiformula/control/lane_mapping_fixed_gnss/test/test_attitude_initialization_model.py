"""R9 startup model tests, independent of recorded trials and Unity truth.

The NumPy quaternion oracle below does not call production rotation helpers.
Larger helper-only rotations check order, not startup admission permissions.
"""
from dataclasses import FrozenInstanceError
import math

import numpy as np
import pytest

from lane_mapping_fixed_gnss.attitude import (
    CausalRawAttitude, DEFAULT_CONFIG, StationaryEvidence, _transport_directions,
)


T, R, G = 1_000_000_000, 9_000_000_000, 9.80665
BUDGET = 2.5e-5


def rotation_vector(vector):
    angle = np.linalg.norm(vector)
    if angle == 0.:
        return np.eye(3)
    axis = np.asarray(vector) / angle
    w = math.cos(angle / 2.)
    x, y, z = axis * math.sin(angle / 2.)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def make_window(rates, bias=(0., 0., 0.), dt=.05, initial=(0., 0., 1.)):
    rates, bias = np.asarray(rates, dtype=float), np.asarray(bias, dtype=float)
    up = np.asarray(initial, dtype=float)
    up /= np.linalg.norm(up)
    rows = []
    for index, rate in enumerate(rates):
        stamp, receipt = T + round(index*dt*1e9), R + round(index*dt*1e9)
        rows.append((stamp, receipt, tuple(rate), tuple(G*up)))
        if index + 1 < len(rates):
            up = rotation_vector(-(rate-bias)*dt) @ up
    return rows, up


def feed(estimator, rows):
    sample = None
    for stamp, receipt, gyro, force in rows:
        sample = estimator.accept_imu(stamp, receipt, gyro, force,
            StationaryEvidence(stamp, receipt, 0., True),
            now_source_ns=stamp, now_steady_ns=receipt)
    return sample


def constant_window():
    return make_window(np.zeros((21, 3)))[0]


def test_budget_derivation_is_explicit_conditional_geometry_not_sensor_calibration():
    assert DEFAULT_CONFIG["initialization_direction_error_rad"] == BUDGET
    h, distance = .5, 30.
    error = (h*h+distance*distance)*math.tan(BUDGET)/(h-distance*math.tan(BUDGET))
    assert .045 < error < .05
    # Existing camera/range and every raw-motion time gate are untouched.
    from lane_mapping_fixed_gnss.motion import DEFAULT_CONFIG as motion
    assert (motion["max_gyro_age_s"], motion["max_receipt_age_s"], motion["max_dt_s"]) == (.15, .2, .2)


def test_endpoint_transport_matches_independent_noncommuting_matrix_oracle():
    bias = np.array([.01, -.02, .03])
    rates = np.array([[.4, 0., 0.], [0., .6, 0.], [0., 0., -.5],
                      [.2, -.3, .1], [-.1, .2, .3], [8., 9., 10.]]) + bias
    rows, endpoint = make_window(rates, bias, dt=.1, initial=(.2, -.3, 1.))
    actual = _transport_directions(rows, tuple(bias))
    for direction in actual:
        np.testing.assert_allclose(direction, endpoint, atol=6e-16, rtol=0)
    # A summed-rate rotation is deliberately NOT the oracle for varying axes.
    start = np.asarray(rows[0][3])/G
    wrong = rotation_vector(-np.sum(rates[:-1]-bias, axis=0)*.1) @ start
    assert np.linalg.norm(wrong-endpoint) > 1e-4
    # The last rate has no previous interval: changing it cannot affect output.
    changed = list(rows)
    changed[-1] = (*rows[-1][:2], (-999., 999., 1000.), rows[-1][3])
    assert _transport_directions(changed, tuple(bias)) == actual


def test_admitted_noncommuting_window_returns_endpoint_not_untransported_average():
    bias = np.array([.003, -.002, .001])
    physical = np.zeros((21, 3))
    physical[:5] = [.0008, 0., 0.]
    physical[5:10] = [0., .0006, 0.]
    physical[10:15] = [-.0008, -.0006, 0.]
    rows, endpoint = make_window(physical+bias, bias, initial=(.2, -.1, 1.))
    sample = feed(CausalRawAttitude(), rows)
    assert sample is not None
    np.testing.assert_allclose(sample.up_body, endpoint, atol=7e-16, rtol=0)
    mean = np.mean([row[3] for row in rows], axis=0)
    mean /= np.linalg.norm(mean)
    assert np.linalg.norm(mean-endpoint) > 1e-5
    np.testing.assert_allclose(sample.gyro_bias, bias, atol=5e-19, rtol=0)
    assert sample.source_stamp_ns == rows[-1][0]
    assert sample.initialization.model_check.maximum_block_direction_residual_rad < 1e-14


@pytest.mark.parametrize("fraction, accepted", [(.999, True), (1.001, False)])
def test_new_direction_budget_is_stricter_without_relaxing_old_spread_gates(fraction, accepted):
    rows = constant_window()
    angle = BUDGET*fraction
    for index in range(15, len(rows)):
        rows[index] = (*rows[index][:3], (G*math.sin(angle), 0., G*math.cos(angle)))
    force = np.array([row[3] for row in rows])
    assert np.max(np.linalg.norm(force-force.mean(0), axis=1)) < .001 < .05
    estimator = CausalRawAttitude()
    sample = feed(estimator, rows)
    assert (sample is not None) is accepted
    assert estimator.last_source_stamp_ns == rows[-1][0]
    assert estimator.fault_reason is None
    check = estimator.last_initialization_model_check
    assert check.passed is accepted
    assert check.maximum_block_direction_residual_rad == pytest.approx(angle, abs=1e-19)
    if not accepted:
        assert not estimator._window
        assert check.reason == "initialization_direction_model_inconsistent"


def test_bias_nonstationarity_is_rejected_even_when_gravity_transport_is_exact():
    rates = np.zeros((21, 3))
    rates[15:, 0] = 1e-4
    rows, _ = make_window(rates)
    estimator = CausalRawAttitude()
    assert feed(estimator, rows) is None
    check = estimator.last_initialization_model_check
    assert check.maximum_block_direction_residual_rad < 1e-15
    assert check.heldout_direction_residual_rad < 1e-15
    assert check.perpendicular_bias_angle_rad == pytest.approx(1e-4, abs=1e-18)
    assert check.reason == "initialization_bias_not_stationary"


def test_holdout_is_not_refitted_and_gravity_parallel_rotation_remains_unobservable():
    rates = np.zeros((21, 3))
    rates[15:, 2] = .001
    rows, _ = make_window(rates)
    sample = feed(CausalRawAttitude(), rows)
    assert sample is not None
    assert sample.gyro_bias == (0., 0., 0.)  # NOT whole-window gyro mean.
    assert sample.initialization.gyro_mean[2] > 0.
    check = sample.initialization.model_check
    assert check.training_gyro_mean == sample.gyro_bias
    assert check.heldout_gyro_mean == (0., 0., .001)
    assert check.perpendicular_bias_difference_radps == 0.
    assert "rotation about gravity" in check.unobservable
    assert sample.held_rate == (0., 0., .001)


def test_deterministic_settling_like_transient_rejects_then_new_still_window_initializes():
    # A declared synthetic input, not a trial27-fitted signal or sensor noise model.
    rows = []
    for index in range(101):
        seconds = index*.01
        gyro = (0., .0008*math.exp(-seconds/.15), 0.)
        angle = .0004*math.exp(-seconds/.2)
        force = (G*math.sin(angle), 0., G*math.cos(angle))
        rows.append((T+index*10_000_000, R+index*10_000_000, gyro, force))
    values = np.array([row[3] for row in rows])
    assert np.max(np.linalg.norm(values-values.mean(0), axis=1)) < .05
    estimator = CausalRawAttitude()
    assert feed(estimator, rows) is None
    assert not estimator.ready and estimator.fault_reason is None
    rejected = estimator.last_initialization_model_check
    assert not rejected.passed
    next_rows = [(T+i*10_000_000, R+i*10_000_000, (0., 0., 0.), (0., 0., G))
                 for i in range(101, 202)]
    for row in next_rows[:-1]:
        assert feed(estimator, [row]) is None
    sample = feed(estimator, next_rows[-1:])
    assert sample is not None
    assert sample.initialization.first_source_stamp_ns == next_rows[0][0]
    assert sample.source_stamp_ns == next_rows[-1][0]
    assert sample.gyro_bias == (0., 0., 0.)
    assert rejected.endpoint_source_stamp_ns == rows[-1][0]


def test_source_quarter_boundaries_are_disjoint_with_irregular_sampling():
    offsets = [0, 30, 70, 120, 180, 240, 250, 300, 400, 490,
               500, 510, 600, 690, 740, 750, 800, 900, 950, 990, 1000]
    rows = [(T+i*1_000_000, R+i*1_000_000, (.001, .002, .003), (0., 0., G)) for i in offsets]
    sample = feed(CausalRawAttitude(), rows)
    assert sample is not None
    blocks = sample.initialization.model_check.blocks
    assert [b.sample_count for b in blocks] == [6, 4, 5, 6]
    assert sum(b.sample_count for b in blocks) == len(rows)
    assert all(a.last_source_stamp_ns < b.first_source_stamp_ns for a, b in zip(blocks, blocks[1:]))
    assert blocks[1].first_source_stamp_ns == T+250_000_000
    assert blocks[2].first_source_stamp_ns == T+500_000_000
    assert blocks[3].first_source_stamp_ns == T+750_000_000


def test_missing_temporal_blocks_wait_without_clearing_frontier_or_restarting_timer():
    estimator = CausalRawAttitude({"initialization_min_samples": 2,
                                  "initialization_duration_s": .1})
    rows = [(T+i, R+i, (0., 0., 0.), (0., 0., G)) for i in (0, 100_000_000, 110_000_000)]
    assert feed(estimator, rows) is None
    assert estimator.last_source_stamp_ns == rows[-1][0]
    assert estimator.initialization_wait_reason == "initialization_source_quarters_incomplete"
    assert estimator._window[0][0] == T and len(estimator._window) == 3
    assert estimator.window_resets == 0 and estimator.fault_reason is None


def test_zero_force_is_an_unqualified_window_not_an_accidental_terminal_exception():
    estimator = CausalRawAttitude()
    rows = constant_window()
    feed(estimator, rows[:10])
    row = (*rows[10][:3], (0., 0., 0.))
    assert feed(estimator, [row]) is None
    assert estimator.last_source_stamp_ns == row[0]
    assert estimator.initialization_wait_reason == "initialization_zero_specific_force"
    assert estimator.fault_reason is None and not estimator._window


def test_endpoint_last_rate_is_held_only_forward_and_moving_force_never_fuses():
    rows = constant_window()
    estimator = CausalRawAttitude()
    other = CausalRawAttitude()
    changed = list(rows)
    changed[-1] = (*rows[-1][:2], (1e-4, 0., 0.), rows[-1][3])
    first, second = feed(estimator, rows), feed(other, changed)
    assert first is not None and second is not None
    assert first.up_body == second.up_body
    assert second.gyro_bias == (0., 0., 0.) and second.held_rate == (1e-4, 0., 0.)
    for index in range(1, 4):
        stamp, receipt = rows[-1][0]+index*10_000_000, rows[-1][1]+index*10_000_000
        second = other.accept_imu(stamp, receipt, (1e-4, 0., 0.), (1e80, -8., 0.), None,
            now_source_ns=stamp, now_steady_ns=receipt)
    oracle = rotation_vector(np.array([-1e-4, 0., 0.])*.03) @ [0., 0., 1.]
    np.testing.assert_allclose(second.up_body, oracle, atol=3e-16, rtol=0)
    assert not other.snapshot()["dynamic_acceleration_used"]


def test_new_model_evidence_is_immutable_and_snapshot_does_not_alias_it():
    estimator = CausalRawAttitude()
    sample = feed(estimator, constant_window())
    check = sample.initialization.model_check
    with pytest.raises(FrozenInstanceError):
        check.passed = False
    with pytest.raises(FrozenInstanceError):
        check.blocks[0].sample_count = 0
    snapshot = estimator.snapshot()
    snapshot["last_initialization_model_check"]["blocks"][0]["sample_count"] = 999
    assert check.blocks[0].sample_count == 5
    assert len(check.blocks) == 4 and not estimator._window


@pytest.mark.parametrize("budget", [0., -1., float("nan"), float("inf"), True, math.pi/2])
def test_new_budget_rejects_invalid_values(budget):
    with pytest.raises(ValueError, match="initialization_direction_error_rad"):
        CausalRawAttitude({"initialization_direction_error_rad": budget})
