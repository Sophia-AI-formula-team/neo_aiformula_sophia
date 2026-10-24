"""Independent, pre-registered motion/vision acceptance scenarios.

Scope: the real frozen-map localizer, with analytically generated line scans
and a separate 100 Hz wheel/gyro integrator.  This is NOT an image/ROS/runtime
or teach-lap acceptance test.  The known anchor is a unit-test oracle, not a
future map supplied to the online mapper.  Synthetic truth is used only to
generate observations and score results, never as a match argument.

Noise-model assumptions and quality bounds below were fixed before executing
the candidate.  They are not fitted to trial25 or measured sensor covariance.
Keep failures: do not raise these bounds or change inputs to make a repair pass.
Development note: the initial q_lateral=.1 assumption produced 26 passes and
two single-edge fan yaw failures. The original source SHA256
8688d7716b5731fbf1ba3808e4e8e197e8b2d50d3e83077fbb13282e5870e113 and
motion_factor_scenarios_01.xml are preserved in local diagnostic evidence.
A separately authorized, one-candidate .05 comparison passed the unchanged
28 cases but slightly worsened fan lateral error. The planner then adopted
.05 as an explicit engineering assumption. Only this assumption changed here;
these are now DEVELOPMENT cases, not independent held-out acceptance.
The 2000 ms unit budget avoids host scheduling assertions; production 40/80 ms
budgets and all geometric/innovation gates are deliberately NOT changed here.
"""

import math
from functools import lru_cache

import numpy as np
import pytest

from lane_mapping_lya_reference.mask_localization import DEFAULT_CONFIG, LaneMapLocalizer


MOTION_MODEL = {
    "motion_visual_frame_sigma_m": 0.05,
    "motion_lateral_noise_m_sqrt_s": 0.05,
    "motion_yaw_noise_rad_sqrt_s": 0.002,
    "motion_initial_lateral_sigma_m": 0.05,
    "motion_initial_yaw_sigma_rad": 0.02,
    "motion_visual_reference_period_s": 0.1,
}
MIN_ACCEPTED_FRACTION = 0.95
CLEAN_LATERAL_RMSE_M = 0.03
CLEAN_YAW_RMSE_RAD = 0.005
FAN_LATERAL_RMSE_M = 0.10
FAN_YAW_RMSE_RAD = 0.01
GYRO_ERROR_RATIO = 0.80
RATE_FINAL_LATERAL_DIFFERENCE_M = 0.05
RATE_FINAL_YAW_DIFFERENCE_RAD = 0.005
START_NS = 2_000_000_000
MOTION_PERIOD_NS = 10_000_000
SPEED_MPS = 2.0
SYNTHETIC_CAMERA_HEIGHT_M = 0.65


def _anchor():
    # A deliberately independent straight frozen map, covering all test paths.
    x = np.arange(-2.0, 50.0001, 0.1)
    return np.vstack([np.column_stack((x, np.full(len(x), y))) for y in (-1.2, 1.2)])


def _scan(sides=(-1.2, 1.2)):
    # Each side alone has enough spatial extent and points for the unchanged
    # min_scan=30, min_inliers=24 and visual rank=2 requirements.
    x = np.arange(0.6, 6.0001, 0.12)
    return np.vstack([np.column_stack((x, np.full(len(x), y))) for y in sides])


def _tilted_camera_projection(points, pitch_rad):
    """Independent pinhole rays from pitched camera, intersected as level.

    The real synthetic camera looks along (cos(pitch), 0, sin(pitch)).
    A flat-ground point has displacement (x, y, -height) from its centre.
    Normalize that displacement into camera coordinates, then intersect its
    ray using the deliberately wrong *level* calibration.  No production
    projection, GroundLookup, pose transform or measured terminal tilt is used.
    Both signs and zero are tested.  This is an ideal ray-level stress test,
    not pixel rendering, calibration fitting, or a claim about actual roll.
    """
    c, s = math.cos(pitch_rad), math.sin(pitch_rad)
    height = SYNTHETIC_CAMERA_HEIGHT_M
    forward = c * points[:, 0] - s * height
    left = points[:, 1]
    down = s * points[:, 0] + c * height
    assert np.all(forward > 0) and np.all(down > 0)
    image_left = left / forward
    image_down = down / forward
    reconstructed_forward = height / image_down
    return np.column_stack((reconstructed_forward, image_left * reconstructed_forward))


def _raw_motion(duration_s, gyro_bias_rad_s):
    """100 Hz midpoint wheel/gyro integration independent of matcher helpers."""
    count = int(round(duration_s * 100))
    result = np.zeros((count + 1, 3), dtype=np.float64)
    delta_yaw = gyro_bias_rad_s * 0.01
    for k in range(1, count + 1):
        previous = result[k - 1]
        mid_yaw = previous[2] + delta_yaw / 2
        result[k, :2] = previous[:2] + SPEED_MPS * 0.01 * np.array(
            [math.cos(mid_yaw), math.sin(mid_yaw)])
        result[k, 2] = previous[2] + delta_yaw
    return result


def _localizer(**config):
    return LaneMapLocalizer(_anchor(), dict(MOTION_MODEL, runtime_budget_ms=2000.0, **config))


def _match(localizer, scan, prior, tick, *, commit=True):
    stamp = START_NS + int(tick) * MOTION_PERIOD_NS
    return localizer.match(scan, prior, stamp, prior_stamp_ns=stamp,
                           motion_source_stamp_ns=stamp, commit=commit)


@lru_cache(maxsize=24)
def _series(rate_hz, duration_s, sides, pitch_degrees=0.0, gyro_bias_rad_s=0.0):
    """Deterministic physical-time series; cache immutable scores, not a solver.

    The cache only avoids rerunning a scenario shared by its quality and rate
    assertions. It is test-side memoization, not an estimator input/cache.
    """
    assert rate_hz in (10, 20)
    raw = _raw_motion(duration_s, gyro_bias_rad_s)
    localizer = _localizer()
    clean = _scan(sides)
    rows = []
    for tick in range(0, len(raw), 100 // rate_hz):
        time_s = tick * 0.01
        # First acquire/track clean geometry for 1 s. Ramp the perturbation
        # over the next 0.5 s: this tests sustained bias, not a sudden outlier.
        ramp = min(1.0, max(0.0, (time_s - 1.0) / 0.5))
        pitch = math.radians(pitch_degrees) * ramp
        observed = _tilted_camera_projection(clean, pitch)
        result = _match(localizer, observed, raw[tick], tick)
        if result.accepted:
            forward = np.array([math.cos(raw[tick, 2]), math.sin(raw[tick, 2])])
            assert abs(float((result.pose_xyyaw[:2] - raw[tick, :2]) @ forward)) <= 1e-8
            assert result.metrics["observable_rank"] == 2
            assert not result.metrics["longitudinal_observable"]
            assert result.metrics["motion_factor_enabled"]
            assert np.all(np.isfinite(result.pose_xyyaw))
        truth = (SPEED_MPS * time_s, 0.0, 0.0)
        rows.append((tick, result.accepted, result.reason, tuple(result.pose_xyyaw),
                     tuple(raw[tick]), truth))
    return tuple(rows)


def _scores(rows):
    accepted = [row for row in rows if row[1]]
    assert accepted, "No accepted geometry: cannot report an empty-subset quality pass"
    pose = np.asarray([row[3] for row in accepted])
    raw = np.asarray([row[4] for row in accepted])
    truth = np.asarray([row[5] for row in accepted])
    return {
        "accepted_fraction": len(accepted) / len(rows),
        "lateral_rmse": float(np.sqrt(np.mean((pose[:, 1] - truth[:, 1]) ** 2))),
        "yaw_rmse": float(np.sqrt(np.mean((pose[:, 2] - truth[:, 2]) ** 2))),
        "raw_lateral_rmse": float(np.sqrt(np.mean((raw[:, 1] - truth[:, 1]) ** 2))),
        "raw_yaw_rmse": float(np.sqrt(np.mean((raw[:, 2] - truth[:, 2]) ** 2))),
        "last_accepted_tick": accepted[-1][0],
        "rejections": tuple((r[0], r[2]) for r in rows if not r[1]),
    }


def test_motion_assumptions_and_original_gate_values_are_explicit():
    for key, value in MOTION_MODEL.items():
        assert DEFAULT_CONFIG[key] == value
    assert DEFAULT_CONFIG["min_scan_points"] == 30
    assert DEFAULT_CONFIG["min_inliers"] == 24
    assert DEFAULT_CONFIG["min_overlap_ratio"] == 0.55
    assert DEFAULT_CONFIG["max_total_translation_m"] == 1.0
    assert DEFAULT_CONFIG["max_total_yaw_rad"] == 0.25
    assert DEFAULT_CONFIG["runtime_budget_ms"] == 40.0


@pytest.mark.parametrize("pitch_degrees", [-0.5, 0.0, 0.5])
def test_synthetic_projection_has_independent_fan_geometry(pitch_degrees):
    clean = _scan()
    pitch = math.radians(pitch_degrees)
    projected = _tilted_camera_projection(clean, pitch)
    # Elimination of the ray parameter gives this independently derived line:
    # y_projected = y_true * (cos(pitch) - x_projected*sin(pitch)/height).
    expected = clean[:, 1] * (math.cos(pitch)
                             - projected[:, 0] * math.sin(pitch) / SYNTHETIC_CAMERA_HEIGHT_M)
    np.testing.assert_allclose(projected[:, 1], expected, rtol=0, atol=1e-14)
    if pitch_degrees == 0:
        np.testing.assert_allclose(projected, clean, rtol=0, atol=1e-14)
    else:
        left = projected[clean[:, 1] > 0]
        right = projected[clean[:, 1] < 0]
        left_slope = (left[-1, 1] - left[0, 1]) / (left[-1, 0] - left[0, 0])
        right_slope = (right[-1, 1] - right[0, 1]) / (right[-1, 0] - right[0, 0])
        assert left_slope == pytest.approx(-right_slope, abs=1e-14)
        assert np.sign(left_slope) == -np.sign(pitch)


@pytest.mark.parametrize("sides", [(-1.2,), (1.2,), (-1.2, 1.2)], ids=["right", "left", "both"])
@pytest.mark.parametrize("rate_hz", [10, 20])
def test_clean_single_and_double_boundaries_remain_accurate(rate_hz, sides):
    rows = _series(rate_hz, 3, sides)
    score = _scores(rows)
    assert score["accepted_fraction"] >= MIN_ACCEPTED_FRACTION, score
    assert score["last_accepted_tick"] == rows[-1][0], score
    assert score["lateral_rmse"] <= CLEAN_LATERAL_RMSE_M, score
    assert score["yaw_rmse"] <= CLEAN_YAW_RMSE_RAD, score


@pytest.mark.parametrize("sides", [(-1.2,), (1.2,), (-1.2, 1.2)], ids=["right", "left", "both"])
@pytest.mark.parametrize("pitch_degrees", [-0.5, 0.5])
def test_fan_bias_with_correct_motion_cannot_create_large_drift(sides, pitch_degrees):
    rows = _series(10, 8, sides, pitch_degrees=pitch_degrees)
    score = _scores(rows)
    assert score["accepted_fraction"] >= MIN_ACCEPTED_FRACTION, score
    assert score["last_accepted_tick"] == rows[-1][0], score
    assert score["lateral_rmse"] <= FAN_LATERAL_RMSE_M, score
    assert score["yaw_rmse"] <= FAN_YAW_RMSE_RAD, score


@pytest.mark.parametrize("gyro_bias", [-0.002, -0.001, 0.001, 0.002])
def test_true_gyro_constant_bias_still_benefits_from_accurate_lanes(gyro_bias):
    rows = _series(10, 20, (-1.2, 1.2), gyro_bias_rad_s=gyro_bias)
    score = _scores(rows)
    assert score["accepted_fraction"] >= MIN_ACCEPTED_FRACTION, score
    assert score["last_accepted_tick"] == rows[-1][0], score
    assert score["lateral_rmse"] <= GYRO_ERROR_RATIO * score["raw_lateral_rmse"], score
    assert score["yaw_rmse"] <= GYRO_ERROR_RATIO * score["raw_yaw_rmse"], score


@pytest.mark.parametrize("pitch_degrees,gyro_bias,sides", [
    (0.5, 0.0, (1.2,)),
    (0.0, 0.002, (-1.2, 1.2)),
], ids=["left_fan", "real_gyro_bias"])
def test_mask_rate_changes_do_not_double_the_information_per_second(pitch_degrees, gyro_bias, sides):
    # Identical underlying 100 Hz motion stream and scan geometry at common
    # source times. 20 Hz adds real intervening frames, not replayed exposures.
    slow = _series(10, 8, sides, pitch_degrees, gyro_bias)
    fast = _series(20, 8, sides, pitch_degrees, gyro_bias)
    fast_by_tick = {row[0]: row for row in fast}
    for row in slow:
        other = fast_by_tick[row[0]]
        assert row[1:3] == other[1:3], (row, other)
        np.testing.assert_array_equal(row[4], other[4])
        if row[1]:
            assert abs(row[3][1] - other[3][1]) <= RATE_FINAL_LATERAL_DIFFERENCE_M
            assert abs(row[3][2] - other[3][2]) <= RATE_FINAL_YAW_DIFFERENCE_RAD
    assert _scores(slow)["accepted_fraction"] >= MIN_ACCEPTED_FRACTION
    assert _scores(fast)["accepted_fraction"] >= MIN_ACCEPTED_FRACTION


def _committed_observation(localizer):
    state = localizer.capture_tracking_state()
    points, votes = localizer.refined_points_and_votes()
    return (state.correction_se2, state.last_accepted_stamp_ns, state.motion_information,
            localizer.refinement_metrics(), points.tobytes(), votes.tobytes())


@pytest.mark.parametrize("rejected_kind", ["empty", "missing_motion", "future_motion", "discarded"])
def test_rejected_intermediate_frame_does_not_shorten_next_motion_interval(rejected_kind):
    localizer, direct = _localizer(), _localizer()
    scan = _scan()
    initial = [0.0, 0.15, 0.0]
    for instance in (localizer, direct):
        assert _match(instance, scan, initial, 0).accepted
    before = _committed_observation(localizer)
    next_prior = [0.2, 0.15, 0.0]
    if rejected_kind == "empty":
        rejected = _match(localizer, np.empty((0, 2)), next_prior, 10)
        assert not rejected.accepted
    elif rejected_kind == "missing_motion":
        rejected = localizer.match(scan, next_prior, START_NS + 10 * MOTION_PERIOD_NS)
        assert rejected.reason == "missing_motion_source_stamp"
    elif rejected_kind == "future_motion":
        stamp = START_NS + 10 * MOTION_PERIOD_NS
        rejected = localizer.match(scan, next_prior, stamp, prior_stamp_ns=stamp,
                                   motion_source_stamp_ns=stamp + 1)
        assert rejected.reason == "future_or_invalid_motion_source_stamp"
    else:
        proposal = _match(localizer, scan, next_prior, 10, commit=False)
        assert proposal.accepted
        localizer.discard_pending()
        assert not localizer.commit(proposal)
    assert _committed_observation(localizer) == before
    # Counterfactual A -> C covers the whole 0.2 s, not only B -> C's 0.1 s.
    result = _match(localizer, scan, [0.4, 0.15, 0.0], 20)
    reference = _match(direct, scan, [0.4, 0.15, 0.0], 20)
    assert result.accepted and reference.accepted
    assert result.metrics["motion_interval_s"] == pytest.approx(0.2, abs=1e-14)
    np.testing.assert_array_equal(result.pose_xyyaw, reference.pose_xyyaw)
    assert _committed_observation(localizer) == _committed_observation(direct)


@pytest.mark.parametrize("handoff_kind", ["candidate", "repeat"])
def test_pending_proposal_and_later_live_commit_do_not_leak_through_handoff(handoff_kind):
    live = _localizer()
    scan = _scan()
    assert _match(live, scan, [0.0, 0.15, 0.0], 0).accepted
    state_a = live.capture_tracking_state()
    direct = LaneMapLocalizer.from_tracking_state(_anchor(), state_a)
    pending = _match(live, scan, [0.2, 0.17, 0.001], 10, commit=False)
    assert pending.accepted
    captured = live.capture_tracking_state()
    assert captured.motion_information == state_a.motion_information
    if handoff_kind == "candidate":
        child = live.fork_with_anchor(_anchor())
    else:
        child = LaneMapLocalizer.from_tracking_state(_anchor(), captured)
    assert child.refinement_metrics()["refinement_retained_frames"] == 0
    # Now mutate the live owner by legitimately committing B. The child must
    # still consume A -> C. This models an immutable background handoff, not
    # authorization to use a stale result in the runtime engine.
    assert live.commit(pending)
    assert live.capture_tracking_state().motion_information != captured.motion_information
    assert child.capture_tracking_state().motion_information == state_a.motion_information
    result = _match(child, scan, [0.4, 0.15, 0.0], 20)
    reference = _match(direct, scan, [0.4, 0.15, 0.0], 20)
    assert result.accepted and reference.accepted
    assert result.metrics["motion_interval_s"] == pytest.approx(0.2, abs=1e-14)
    np.testing.assert_array_equal(result.pose_xyyaw, reference.pose_xyyaw)
    assert _committed_observation(child) == _committed_observation(direct)
