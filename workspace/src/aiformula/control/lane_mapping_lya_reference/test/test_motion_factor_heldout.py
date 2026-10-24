"""First evaluation of predeclared synthetic conditions after fixing q_lat=.05.

The generator family is intentionally shared with the development scenarios;
these are held-out CONDITIONS, not a new simulator, pixel pipeline, trajectory
dataset or guarantee of generalization. No recorded trial25 data is accessed.
Speeds 1.4/2.6 m/s, pitches +/-.35 degree, gyro biases +/-.0015 rad/s,
durations 5/7/11/15 s and their combinations were fixed before running them.
Thresholds are imported unchanged from the development acceptance design.
No global speed mutation and no result cache are used: every run owns its
100 Hz motion history, localizer and explicit speed argument.

Combined unknown biases are tested only with TWO visible boundaries. A single
line's unknown slope and an unknown gyro bias can be indistinguishable; this
suite does not claim a truth-accuracy guarantee for that unidentifiable case.
All new failures must remain failures, without choosing a new input or bound.
"""

import math

import numpy as np
import pytest

from test_motion_factor_scenarios import (
    CLEAN_LATERAL_RMSE_M, CLEAN_YAW_RMSE_RAD,
    FAN_LATERAL_RMSE_M, FAN_YAW_RMSE_RAD, GYRO_ERROR_RATIO,
    MIN_ACCEPTED_FRACTION, MOTION_MODEL, MOTION_PERIOD_NS, START_NS,
    _localizer, _scan, _scores, _tilted_camera_projection,
)


SPEEDS_MPS = (1.4, 2.6)
PITCHES_DEGREES = (-0.35, 0.35)
GYRO_BIASES_RAD_S = (-0.0015, 0.0015)


def _motion_history(duration_s, speed_mps, gyro_bias_rad_s):
    """Independent midpoint integration with speed explicitly in every call."""
    count = int(round(duration_s * 100))
    raw = np.zeros((count + 1, 3), dtype=np.float64)
    delta_yaw = gyro_bias_rad_s * 0.01
    for tick in range(1, count + 1):
        previous = raw[tick - 1]
        mid_yaw = previous[2] + delta_yaw / 2
        raw[tick, :2] = previous[:2] + speed_mps * 0.01 * np.array(
            [math.cos(mid_yaw), math.sin(mid_yaw)])
        raw[tick, 2] = previous[2] + delta_yaw
    return raw


def _heldout_series(speed_mps, rate_hz, duration_s, sides, pitch_degrees=0.0,
                    gyro_bias_rad_s=0.0):
    assert speed_mps in SPEEDS_MPS
    assert rate_hz in (10, 20)
    assert MOTION_MODEL["motion_lateral_noise_m_sqrt_s"] == .05
    raw = _motion_history(duration_s, speed_mps, gyro_bias_rad_s)
    localizer = _localizer()
    clean = _scan(sides)
    rows = []
    for tick in range(0, len(raw), 100 // rate_hz):
        time_s = tick * 0.01
        ramp = min(1.0, max(0.0, (time_s - 1.0) / 0.5))
        observed = _tilted_camera_projection(clean, math.radians(pitch_degrees) * ramp)
        source_stamp = START_NS + tick * MOTION_PERIOD_NS
        result = localizer.match(observed, raw[tick], source_stamp,
                                 prior_stamp_ns=source_stamp,
                                 motion_source_stamp_ns=source_stamp)
        if result.accepted:
            forward = np.array([math.cos(raw[tick, 2]), math.sin(raw[tick, 2])])
            assert abs(float((result.pose_xyyaw[:2] - raw[tick, :2]) @ forward)) <= 1e-8
            assert result.metrics["observable_rank"] == 2
            assert not result.metrics["longitudinal_observable"]
            assert result.metrics["motion_factor_enabled"]
            assert np.all(np.isfinite(result.pose_xyyaw))
        truth = (speed_mps * time_s, 0.0, 0.0)
        rows.append((tick, result.accepted, result.reason, tuple(result.pose_xyyaw),
                     tuple(raw[tick]), truth))
    return tuple(rows)


def _quality(rows):
    score = _scores(rows)
    assert score["accepted_fraction"] >= MIN_ACCEPTED_FRACTION, score
    assert score["last_accepted_tick"] == rows[-1][0], score
    return score


def test_motion_generator_does_not_reuse_other_speed_or_modify_an_existing_history():
    slow = _motion_history(5, 1.4, .0015)
    saved = slow.copy()
    fast = _motion_history(5, 2.6, .0015)
    np.testing.assert_array_equal(slow, saved)
    np.testing.assert_array_equal(slow[:, 2], fast[:, 2])
    np.testing.assert_allclose(fast[:, :2], slow[:, :2] * (2.6 / 1.4), rtol=2e-14, atol=1e-14)
    assert not np.shares_memory(slow, fast)


@pytest.mark.parametrize("speed_mps", SPEEDS_MPS)
@pytest.mark.parametrize("sides", [(-1.2,), (1.2,), (-1.2, 1.2)], ids=["right", "left", "both"])
def test_new_speeds_and_clean_duration(speed_mps, sides):
    rate_hz = 10 if speed_mps == 1.4 else 20
    score = _quality(_heldout_series(speed_mps, rate_hz, 5, sides))
    assert score["lateral_rmse"] <= CLEAN_LATERAL_RMSE_M, score
    assert score["yaw_rmse"] <= CLEAN_YAW_RMSE_RAD, score


@pytest.mark.parametrize("speed_mps", SPEEDS_MPS)
@pytest.mark.parametrize("pitch_degrees", PITCHES_DEGREES)
@pytest.mark.parametrize("sides", [(-1.2,), (1.2,), (-1.2, 1.2)], ids=["right", "left", "both"])
def test_new_fan_pitch_speed_and_duration(speed_mps, pitch_degrees, sides):
    rate_hz = 10 if speed_mps == 1.4 else 20
    score = _quality(_heldout_series(speed_mps, rate_hz, 7, sides, pitch_degrees))
    assert score["lateral_rmse"] <= FAN_LATERAL_RMSE_M, score
    assert score["yaw_rmse"] <= FAN_YAW_RMSE_RAD, score


@pytest.mark.parametrize("speed_mps", SPEEDS_MPS)
@pytest.mark.parametrize("gyro_bias", GYRO_BIASES_RAD_S)
def test_new_real_gyro_bias_speed_and_duration(speed_mps, gyro_bias):
    rate_hz = 20 if speed_mps == 1.4 else 10
    score = _quality(_heldout_series(speed_mps, rate_hz, 15, (-1.2, 1.2),
                                     gyro_bias_rad_s=gyro_bias))
    assert score["lateral_rmse"] <= GYRO_ERROR_RATIO * score["raw_lateral_rmse"], score
    assert score["yaw_rmse"] <= GYRO_ERROR_RATIO * score["raw_yaw_rmse"], score


@pytest.mark.parametrize("speed_mps", SPEEDS_MPS)
@pytest.mark.parametrize("pitch_degrees,gyro_bias", [(-.35, .0015), (.35, .0015)],
                         ids=["opposite_signs", "same_signs"])
def test_combined_projection_and_gyro_bias_with_two_visible_boundaries(speed_mps, pitch_degrees, gyro_bias):
    # Same original absolute fan bounds AND relative gyro-improvement criterion;
    # no new tighter numeric threshold is introduced for the combined case.
    score = _quality(_heldout_series(speed_mps, 10, 11, (-1.2, 1.2), pitch_degrees, gyro_bias))
    assert score["lateral_rmse"] <= FAN_LATERAL_RMSE_M, score
    assert score["yaw_rmse"] <= FAN_YAW_RMSE_RAD, score
    assert score["lateral_rmse"] <= GYRO_ERROR_RATIO * score["raw_lateral_rmse"], score
    assert score["yaw_rmse"] <= GYRO_ERROR_RATIO * score["raw_yaw_rmse"], score
