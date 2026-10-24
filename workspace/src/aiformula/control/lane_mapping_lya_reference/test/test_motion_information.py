"""Motion information units/causality; generous unit budgets are not runtime proof."""

from contextlib import contextmanager
from dataclasses import FrozenInstanceError
from pathlib import Path
import math
import json
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[2] / "lane_mapping_fixed_gnss"))

from lane_mapping_lya_reference.mask_localization import (
    DEFAULT_CONFIG, CommittedLocalizationState, LaneMapLocalizer,
    MotionInformationState, _configuration, _motion_prediction,
)
from lane_mapping_fixed_gnss.mapping import TeachMapEngine


def points():
    x = np.linspace(1., 7., 121)
    return np.vstack([np.column_stack((x, np.full(len(x), y))) for y in (-1., 1.)])


def localizer(**config):
    return LaneMapLocalizer(points(), dict(runtime_budget_ms=2000., **config))


def match(model, source=100000000, mask=None, prior=(0., .1, .01), **kwargs):
    return model.match(points(), prior, source + 10000000 if mask is None else mask,
                       prior_stamp_ns=source, motion_source_stamp_ns=source, **kwargs)


def state(source=100000000, prior=(0., 0., 0.), covariance=((.0025, 0.), (0., .0004))):
    return MotionInformationState(source, prior, covariance)


@pytest.mark.parametrize("name", [
    "motion_visual_frame_sigma_m", "motion_lateral_noise_m_sqrt_s",
    "motion_yaw_noise_rad_sqrt_s", "motion_initial_lateral_sigma_m", "motion_initial_yaw_sigma_rad",
])
@pytest.mark.parametrize("value", [1e200, 1e-200, 1e-155])
def test_new_noise_parameters_reject_square_overflow_underflow_or_inverse_overflow(name, value):
    with pytest.raises(ValueError, match="unrepresentable variance/information"):
        _configuration({name: value})


@pytest.mark.parametrize("value", [5e-324, 1e-310])
def test_visual_period_rejects_unrepresentable_reciprocal(value):
    with pytest.raises(ValueError, match="unrepresentable variance/information"):
        _configuration({"motion_visual_reference_period_s": value})


def test_motion_engineering_defaults_change_only_the_reviewed_lateral_density():
    config = _configuration(None)
    assert config["motion_lateral_noise_m_sqrt_s"] == .05
    assert config["motion_yaw_noise_rad_sqrt_s"] == .002
    assert config["motion_visual_frame_sigma_m"] == .05
    assert config["motion_initial_lateral_sigma_m"] == .05
    assert config["motion_initial_yaw_sigma_rad"] == .02
    assert config["motion_visual_reference_period_s"] == .1


def test_state_owns_immutable_values():
    prior = np.zeros(3)
    covariance = np.diag([.0025, .0004])
    value = state(prior=prior, covariance=covariance)
    prior[:] = 7.
    covariance[:] = 8.
    assert value.raw_prior_xyyaw == (0., 0., 0.)
    assert value.covariance == ((.0025, 0.), (0., .0004))
    with pytest.raises(FrozenInstanceError):
        value.source_stamp_ns = 7
    with pytest.raises(TypeError):
        value.covariance[0][0] = 7.


@pytest.mark.parametrize("stamp", [0, -1, True, 1.5, 2**63, float("nan")])
def test_state_rejects_invalid_source(stamp):
    with pytest.raises(ValueError):
        state(source=stamp)


@pytest.mark.parametrize("covariance", [
    ((0., 0.), (0., 1.)), ((1., 2.), (2., 1.)), ((1., .1), (0., 1.)),
    ((1., 0.), (0., float("nan"))), ((1., 0.),),
])
def test_state_rejects_invalid_covariance(covariance):
    with pytest.raises(ValueError):
        state(covariance=covariance)


def test_committed_container_requires_real_past_owner():
    with pytest.raises(ValueError):
        CommittedLocalizationState({}, (0., 0., 0.), None, 100000000, state())
    with pytest.raises(ValueError):
        CommittedLocalizationState({}, (0., 0., 0.), 90000000, 100000000, state())
    with pytest.raises(ValueError):
        CommittedLocalizationState({}, (0., 0., 0.), 100000000, 100000000, {})


@pytest.mark.parametrize("heading,correction,delta", [
    (0., 0., (2., 0.)), (.3, -.1, (1.8, .5)), (-.7, .2, (-1., .4)),
])
def test_prediction_physical_two_mode_transition_and_continuous_noise(heading, correction, delta):
    previous = state(prior=(2., 3., .1), covariance=((.003, .0002), (.0002, .0005)))
    raw = np.array([2.+delta[0], 3.+delta[1], heading])
    actual, dt = _motion_prediction(previous, raw, correction, 500000000, DEFAULT_CONFIG)
    c, s = math.cos(correction), math.sin(correction)
    dx, dy = c*delta[0]-s*delta[1], s*delta[0]+c*delta[1]
    lever = math.sin(heading)*dy + math.cos(heading)*dx
    transition = np.array([[math.cos(heading-.1), lever], [0., 1.]])
    qlat, qyaw = .05**2, .002**2
    process = np.array([[qlat*.4 + lever**2*qyaw*.4/3, lever*qyaw*.4/2],
                        [lever*qyaw*.4/2, qyaw*.4]])
    expected = transition @ np.asarray(previous.covariance) @ transition.T + process
    np.testing.assert_allclose(actual, expected, rtol=1e-14, atol=1e-17)
    assert dt == pytest.approx(.4)
    assert np.linalg.eigvalsh(actual).min() > 0


@pytest.mark.parametrize("increment", [1, 100000000, 1500000000])
def test_source_interval_is_not_frame_count_or_silently_clipped(increment):
    previous = state()
    actual, dt = _motion_prediction(previous, np.zeros(3), 0., 100000000+increment, DEFAULT_CONFIG)
    expected = np.asarray(previous.covariance) + np.diag([.05**2, .002**2])*(increment*1e-9)
    np.testing.assert_allclose(actual, expected, rtol=1e-14, atol=1e-17)
    assert dt == increment*1e-9


def test_cold_motion_acquisition_is_exact_original_visual_solve_then_initial_information():
    legacy, motion = localizer(), localizer()
    original = legacy.match(points(), [0., .1, .01], 110000000, prior_stamp_ns=100000000)
    result = match(motion)
    assert original.accepted and result.accepted
    np.testing.assert_array_equal(original.pose_xyyaw, result.pose_xyyaw)
    assert result.metrics["motion_factor_initialization"] is True
    assert "motion_visual_information_scale" not in result.metrics
    value = motion.capture_tracking_state().motion_information
    assert value.source_stamp_ns == 100000000
    assert value.raw_prior_xyyaw == (0., .1, .01)
    np.testing.assert_allclose(value.covariance, np.diag([.05**2, .02**2]), atol=0, rtol=0)
    assert legacy.capture_tracking_state().motion_information is None


def test_proposal_and_discard_never_publish_motion_information():
    model = localizer()
    first = match(model, commit=False)
    assert first.accepted
    assert model.capture_tracking_state().motion_information is None
    model.discard_pending()
    assert not model.commit(first)
    committed = match(model, source=200000000)
    assert committed.accepted
    before = model.capture_tracking_state().motion_information
    proposal = match(model, source=300000000, prior=(.2, .11, .012), commit=False)
    assert proposal.accepted, proposal.reason
    assert model.capture_tracking_state().motion_information is before
    model.discard_pending()
    assert not model.commit(proposal)
    assert model.capture_tracking_state().motion_information is before
    following = match(model, source=400000000, prior=(.4, .11, .012))
    assert following.accepted, following.reason
    assert following.metrics["motion_interval_s"] == pytest.approx(.2)
    assert model.capture_tracking_state().motion_information.source_stamp_ns == 400000000


@pytest.mark.parametrize("bad,reason", [
    (None, "missing_motion_source_stamp"),
    (100000000, "nonincreasing_motion_source_stamp"),
    (90000000, "nonincreasing_motion_source_stamp"),
    (220000000, "future_or_invalid_motion_source_stamp"),
    (True, "future_or_invalid_motion_source_stamp"),
    (0, "future_or_invalid_motion_source_stamp"),
    (150000000.5, "future_or_invalid_motion_source_stamp"),
    (float("nan"), "future_or_invalid_motion_source_stamp"),
    ("150000000", "future_or_invalid_motion_source_stamp"),
    (2**80, "future_or_invalid_motion_source_stamp"),
    (object(), "future_or_invalid_motion_source_stamp"),
])
def test_rejected_source_never_changes_committed_mean_covariance_or_refinement(bad, reason):
    model = localizer()
    assert match(model).accepted
    before = model.capture_tracking_state()
    refinements = model.refinement_metrics()
    rejected = model.match(points(), [0., .1, .01], 210000000, motion_source_stamp_ns=bad)
    assert not rejected.accepted and rejected.reason == reason
    json.dumps(rejected.metrics, allow_nan=False)
    if reason == "future_or_invalid_motion_source_stamp":
        assert rejected.metrics["motion_source_stamp_ns"] is None
        assert rejected.metrics["motion_source_stamp_invalid"]
    after = model.capture_tracking_state()
    assert after.motion_information is before.motion_information
    assert after.correction_se2 == before.correction_se2
    assert after.last_accepted_stamp_ns == before.last_accepted_stamp_ns
    assert model.refinement_metrics() == refinements
    assert model._pending_motion_information is None
    assert not model.commit(rejected)


def test_motion_stamp_must_be_the_same_paired_source_as_prior():
    model = localizer()
    result = model.match(points(), [0., 0., 0.], 200000000,
                         prior_stamp_ns=190000000, motion_source_stamp_ns=180000000)
    assert not result.accepted and result.reason == "future_or_invalid_motion_source_stamp"
    assert model.capture_tracking_state().motion_information is None


def test_rejected_visual_does_not_advance_motion_source_or_shorten_next_interval():
    model = localizer()
    assert match(model).accepted
    before = model.capture_tracking_state().motion_information
    rejected = model.match(points()+[0., 20.], [0., .1, .01], 210000000,
                           prior_stamp_ns=200000000, motion_source_stamp_ns=200000000)
    assert not rejected.accepted
    assert model.capture_tracking_state().motion_information is before
    following = match(model, source=300000000)
    assert following.accepted, following.reason
    assert following.metrics["motion_interval_s"] == pytest.approx(.2)


def test_motion_prior_cannot_rescue_visual_rank_gate(monkeypatch):
    model = localizer()
    assert match(model).accepted
    before = model.capture_tracking_state().motion_information
    original = model._correspondences

    def zero_heading_leverage(*args, **kwargs):
        world, rotated, selected, normals, residual = original(*args, **kwargs)
        return world, np.zeros_like(rotated), selected, normals, residual

    monkeypatch.setattr(model, "_correspondences", zero_heading_leverage)
    result = match(model, source=200000000)
    assert not result.accepted and result.reason == "unobservable_geometry"
    assert result.metrics["observable_rank"] == 1
    assert model.capture_tracking_state().motion_information is before


def test_fork_and_handoff_keep_only_committed_information_and_time():
    model = localizer()
    assert match(model).accepted
    saved = model.capture_tracking_state()
    proposal = match(model, source=200000000, commit=False)
    assert proposal.accepted
    fork = model.fork_with_anchor(points())
    handoff = LaneMapLocalizer.from_tracking_state(points(), saved)
    for child in (fork, handoff):
        assert child.capture_tracking_state().motion_information is saved.motion_information
        assert child._pending_motion_information is None
        assert child.refinement_metrics()["refinement_retained_frames"] == 0
        assert not child.commit(proposal)
        result = match(child, source=300000000)
        assert result.accepted and result.metrics["initial_acquisition"] is False
        assert result.metrics["motion_interval_s"] == pytest.approx(.2)
    assert model.capture_tracking_state().motion_information is saved.motion_information
    model.reset()
    assert model.capture_tracking_state().motion_information is None
    assert model._pending_motion_information is None


def test_physical_solution_and_covariance_do_not_depend_on_yaw_solver_scale():
    models = [localizer(rotation_scale_m=value) for value in (2., 4., 6.)]
    results = []
    for model in models:
        assert match(model, prior=(0., 0., 0.)).accepted
        result = match(model, source=200000000, prior=(.2, .04, .008))
        assert result.accepted, result.reason
        results.append(result)
    for model, result in zip(models[1:], results[1:]):
        np.testing.assert_allclose(result.pose_xyyaw, results[0].pose_xyyaw, rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(model.capture_tracking_state().motion_information.covariance,
                                   models[0].capture_tracking_state().motion_information.covariance,
                                   rtol=1e-12, atol=1e-14)


@pytest.mark.parametrize("interval", [.025, .05, .1, .5, 4.])
def test_one_image_never_gains_multiple_frames_of_information_after_gap(interval):
    model = localizer()
    assert match(model, prior=(0., 0., 0.)).accepted
    result = match(model, source=100000000+round(interval*1e9), prior=(0., 0., 0.))
    assert result.accepted, result.reason
    metrics = result.metrics
    assert metrics["motion_interval_s"] == pytest.approx(interval)
    assert metrics["motion_visual_interval_s"] == pytest.approx(min(interval, .1))
    assert metrics["motion_visual_information_scale"] == pytest.approx(
        min(interval/.1, 1.)/(metrics["scan_feature_weight_sum"]*.05**2))
    np.testing.assert_allclose(metrics["motion_predicted_covariance"],
        np.diag([.05**2 + .05**2*interval, .02**2 + .002**2*interval]), rtol=1e-14)


def test_engine_bootstrap_motion_mode_commits_only_under_original_guard():
    model = TeachMapEngine(config={"max_processing_ms": 2000.}, matcher_config={"runtime_budget_ms": 2000.})
    scan = points()
    sensitivity = np.full(len(scan), .05)

    @contextmanager
    def denied():
        yield False

    rejected = model.process(scan, sensitivity, [0., 0., 0.], 110000000,
                             commit_guard=denied, motion_source_stamp_ns=100000000)
    assert rejected["reason"] == "commit_guard_rejected"
    assert model._last_motion_source_stamp is None
    assert len(model._trace) == 0
    result = model.process(scan, sensitivity, [0., 0., 0.], 210000000,
                           motion_source_stamp_ns=200000000)
    assert result["accepted"]
    assert model._last_motion_source_stamp == 200000000
    before = model.points_and_votes()
    missing = model.process(scan, sensitivity, [0., 0., 0.], 310000000)
    assert missing["reason"] == "missing_motion_source_stamp"
    assert model._last_motion_source_stamp == 200000000
    assert len(model._trace) == 1
    for old, new in zip(before, model.points_and_votes()):
        np.testing.assert_array_equal(old, new)


def test_real_engine_forwards_paired_source_and_guard_preserves_information_into_repeat():
    engine = TeachMapEngine(config={"max_processing_ms": 2000.},
                            matcher_config={"runtime_budget_ms": 2000.})
    scan = points()
    sensitivity = np.full(len(scan), .05)
    for frame in range(1, 7):
        result = engine.process(scan, sensitivity, [0., 0., 0.], frame*100000000,
                                motion_source_stamp_ns=frame*100000000-20000000)
        assert result["accepted"], result
    active = engine._matcher
    before = active.capture_tracking_state().motion_information
    assert before.source_stamp_ns == 580000000
    assert active.last_accepted_stamp_ns == 600000000
    map_before = engine.points_and_votes()
    trace_before = np.asarray(engine._trace).copy()

    @contextmanager
    def denied():
        yield False

    rejected = engine.process(scan, sensitivity, [0., .01, 0.], 700000000,
                              motion_source_stamp_ns=680000000, commit_guard=denied)
    assert rejected["reason"] == "commit_guard_rejected"
    assert active.capture_tracking_state().motion_information is before
    assert active._pending_motion_information is None
    assert engine._last_motion_source_stamp == 580000000
    for old, new in zip(map_before, engine.points_and_votes()):
        np.testing.assert_array_equal(old, new)
    np.testing.assert_array_equal(trace_before, engine._trace)
    accepted = engine.process(scan, sensitivity, [0., .01, 0.], 800000000,
                              motion_source_stamp_ns=780000000)
    assert accepted["accepted"], accepted
    after = active.capture_tracking_state().motion_information
    assert after.source_stamp_ns == 780000000
    assert after.raw_prior_xyyaw == (0., .01, 0.)
    repeat = engine.repeat_localizer()
    assert repeat.capture_tracking_state().motion_information is after
    assert repeat.last_accepted_stamp_ns == 800000000
    assert repeat._pending_motion_information is None
    assert repeat.refinement_metrics()["refinement_retained_frames"] == 0


@pytest.mark.parametrize("bad", [float("nan"), True, "100000000", 2**80, object(), 0])
def test_engine_malformed_motion_source_never_publishes_map_or_source(bad):
    engine = TeachMapEngine(config={"max_processing_ms": 2000.})
    scan = points()
    rejected = engine.process(scan, np.full(len(scan), .05), [0., 0., 0.], 200000000,
                              motion_source_stamp_ns=bad)
    assert not rejected["accepted"]
    assert rejected["reason"] == "invalid_or_nonincreasing_motion_source_stamp"
    # The established engine API deliberately returns an owned NumPy pose.
    # Normalize only that documented field, never arbitrary invalid inputs.
    json.dumps(dict(rejected, pose_xyyaw=rejected["pose_xyyaw"].tolist()), allow_nan=False)
    assert engine._last_motion_source_stamp is None
    assert not engine._trace and not engine._map.candidate_votes
