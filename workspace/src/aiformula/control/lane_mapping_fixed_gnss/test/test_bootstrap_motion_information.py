"""Same-session raw-bootstrap prior: no forged visual acceptance or relaxed gate.

Unit-only 2s computation budgets remove host load as a geometry-test variable;
these are not runtime evidence. All numerical/statistical production defaults
remain unchanged. Captured-prefix runtime verification is a separate artifact.
"""
from contextlib import contextmanager
from dataclasses import replace
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[2] / "lane_mapping_lya_reference"))

from lane_mapping_fixed_gnss import mapping
from lane_mapping_lya_reference.mask_localization import (
    LaneMapLocalizer, MotionInformationState, _motion_prediction)


def lines(start=1., end=7.):
    x = np.linspace(start, end, 121)
    return np.vstack([np.column_stack((x, np.full(len(x), y))) for y in (-1., 1.)])


def seed(source=1000000000, prior=(0., 0., 0.)):
    return MotionInformationState(source, prior, ((.05**2, 0.), (0., .02**2)))


def model():
    return LaneMapLocalizer(lines(), {"runtime_budget_ms": 2000.})


def engine(**kwargs):
    return mapping.TeachMapEngine(dict(max_processing_ms=2000., **kwargs),
                                  {"runtime_budget_ms": 2000.})


def step(target, source, prior=(0.,0.,0.), points=None, **kwargs):
    points = lines() if points is None else points
    return target.process(points, np.full(len(points), .05), prior, source+10000000,
                          motion_source_stamp_ns=source, **kwargs)


def boot(target):
    for k in range(5):
        row = step(target, 1000000000+k*100000000)
        assert row["accepted"] and row["reason"] == "bootstrap_odometry"
    assert target._matcher is None
    return target._bootstrap_motion_information


def snapshot(target):
    points, votes = target.points_and_votes()
    return (points.tobytes(), votes.tobytes(), np.asarray(target._trace).tobytes(),
            target._map.last_stamp_ns, target._last_motion_source_stamp,
            target._bootstrap_motion_information, target._matcher)


@contextmanager
def deny():
    yield False


def test_plain_fresh_localizer_keeps_original_visual_only_semantics():
    plain, with_stamp = model(), model()
    old = plain.match(lines(), [0.,.05,.01], 1100000000, commit=False)
    new = with_stamp.match(lines(), [0.,.05,.01], 1100000000, commit=False,
                           motion_source_stamp_ns=1090000000)
    assert old.accepted and new.accepted
    np.testing.assert_array_equal(old.pose_xyyaw, new.pose_xyyaw)
    assert new.metrics["motion_factor_initialization"]
    assert not new.metrics["bootstrap_motion_prior_used"]
    assert "motion_visual_information_scale" not in new.metrics


def test_explicit_prior_regularizes_first_match_without_forging_visual_stamp():
    target = model()
    before = seed()
    value = target.match(lines(), [0.,.05,.01], 1110000000, commit=False,
                         motion_source_stamp_ns=1100000000, bootstrap_motion_information=before)
    assert value.accepted, value.reason
    assert value.metrics["initial_acquisition"]
    assert value.metrics["bootstrap_motion_prior_used"]
    assert not value.metrics["motion_factor_initialization"]
    assert value.metrics["motion_visual_interval_s"] == .1
    assert target.last_accepted_stamp_ns is None
    assert target.capture_tracking_state().motion_information is None
    assert target.commit(value)
    assert target.last_accepted_stamp_ns == 1110000000
    assert target.capture_tracking_state().motion_information.source_stamp_ns == 1100000000


@pytest.mark.parametrize("bad", [{}, object(), True])
def test_wrong_seed_type_rejected_without_commit(bad):
    target = model()
    value = target.match(lines(), [0.,0.,0.], 1110000000, commit=False,
                         motion_source_stamp_ns=1100000000, bootstrap_motion_information=bad)
    assert value.reason == "invalid_bootstrap_motion_information"
    assert target._pending_result is None and target._motion_information is None


@pytest.mark.parametrize("source", [1100000000, 1200000000])
def test_seed_source_must_be_strictly_past_current_motion(source):
    target = model()
    value = target.match(lines(), [0.,0.,0.], 1110000000, commit=False,
        motion_source_stamp_ns=1100000000, bootstrap_motion_information=seed(source))
    assert value.reason == "nonincreasing_motion_source_stamp"
    assert target._motion_information is None and target.last_accepted_stamp_ns is None


def test_seed_requires_explicit_current_motion_and_cannot_replace_committed_state():
    target = model()
    missing = target.match(lines(), [0.,0.,0.], 1110000000,
                           bootstrap_motion_information=seed())
    assert missing.reason == "invalid_bootstrap_motion_information"
    accepted = target.match(lines(), [0.,0.,0.], 1210000000,
        motion_source_stamp_ns=1200000000, bootstrap_motion_information=seed())
    assert accepted.accepted
    before = target.capture_tracking_state()
    again = target.match(lines(), [0.,0.,0.], 1310000000,
        motion_source_stamp_ns=1300000000, bootstrap_motion_information=seed())
    assert again.reason == "invalid_bootstrap_motion_information"
    assert target.capture_tracking_state().motion_information is before.motion_information
    np.testing.assert_array_equal(target.correction_se2, before.correction_se2)


def test_seed_is_only_pending_and_discard_does_not_leak_to_fork_or_handoff():
    target = model()
    value = target.match(lines(), [0.,.03,.003], 1110000000, commit=False,
        motion_source_stamp_ns=1100000000, bootstrap_motion_information=seed())
    assert value.accepted
    fork = target.fork_with_anchor(lines())
    handed = LaneMapLocalizer.from_tracking_state(lines(), target.capture_tracking_state())
    for other in (fork, handed):
        assert other.last_accepted_stamp_ns is None and other._motion_information is None
    target.discard_pending()
    assert not target.commit(value)
    assert target._motion_information is None


def test_visual_rank_cannot_be_rescued_by_bootstrap_prior():
    target = LaneMapLocalizer(lines(2.,4.), {"runtime_budget_ms":2000.})
    value = target.match(lines(2.,4.), [0.,0.,0.], 1110000000, commit=False,
        motion_source_stamp_ns=1100000000, bootstrap_motion_information=seed())
    assert not value.accepted
    assert value.reason == "unobservable_geometry" and value.metrics["observable_rank"] == 1
    assert target._motion_information is None and target.last_accepted_stamp_ns is None


def test_bootstrap_covariance_is_propagated_only_after_real_map_commits():
    target = engine()
    first = step(target, 1000000000)
    assert first["accepted"]
    assert first["metrics"]["bootstrap_motion_proposal"]["initialization"]
    assert first["metrics"]["bootstrap_motion_proposal"]["interval_s"] is None
    original = target._bootstrap_motion_information
    np.testing.assert_array_equal(original.covariance, np.diag([.05**2,.02**2]))
    prior = np.array([.2,.02,.1])
    second = step(target, 1200000000, prior)
    assert second["accepted"]
    telemetry = second["metrics"]["bootstrap_motion_proposal"]
    assert not telemetry["initialization"] and telemetry["interval_s"] == .2
    assert telemetry["previous_source_stamp_ns"] == 1000000000
    assert telemetry["source_stamp_ns"] == 1200000000
    # Literal independent F/Q from current raw-heading and prior displacement.
    lever = math.cos(.1)*.2+math.sin(.1)*.02
    f = np.array([[math.cos(.1),lever],[0.,1.]])
    q = np.array([[.05**2*.2+lever**2*.002**2*.2/3.,lever*.002**2*.2/2.],
                  [lever*.002**2*.2/2.,.002**2*.2]])
    expected = f @ np.diag([.05**2,.02**2]) @ f.T+q
    np.testing.assert_allclose(target._bootstrap_motion_information.covariance,expected,rtol=1e-14,atol=1e-18)
    assert target._bootstrap_motion_information.raw_prior_xyyaw == (
        prior[0], prior[1], math.atan2(math.sin(prior[2]), math.cos(prior[2])))
    assert original.source_stamp_ns == 1000000000


@pytest.mark.parametrize("kind", ["empty", "guard", "capacity", "deadline"])
def test_rejected_bootstrap_does_not_advance_origin_covariance_or_map(kind, monkeypatch):
    target = engine()
    assert step(target,1000000000)["accepted"]
    before = snapshot(target)
    kwargs = {}
    if kind == "empty":
        kwargs["points"] = np.empty((0,2))
    elif kind == "guard":
        kwargs["commit_guard"] = deny
    elif kind == "capacity":
        monkeypatch.setattr(target,"_capacity_allows",lambda *args: False)
    else:
        monkeypatch.setattr(target,"_deadline",lambda *args: True)
    result = step(target,1100000000, (.1,0.,0.),**kwargs)
    assert not result["accepted"]
    assert snapshot(target) == before


def test_empty_first_frame_is_not_a_bootstrap_origin():
    target = engine()
    assert not step(target,1000000000, points=np.empty((0,2)))["accepted"]
    assert target._bootstrap_motion_information is None
    assert step(target,1100000000, (.1,0.,0.))["accepted"]
    assert target._bootstrap_motion_information.source_stamp_ns == 1100000000
    assert target._bootstrap_motion_information.raw_prior_xyyaw == (.1,0.,0.)


@pytest.mark.parametrize("source", [None,1000000000,900000000])
def test_motion_mode_never_drops_or_replays_source_during_bootstrap(source):
    target = engine()
    assert step(target,1000000000)["accepted"]
    before = snapshot(target)
    result = target.process(lines(), np.full(len(lines()),.05), [0.,0.,0.], 1200000000,
                            motion_source_stamp_ns=source)
    assert not result["accepted"]
    assert snapshot(target) == before


def test_committed_legacy_map_cannot_acquire_an_invented_midrun_motion_origin():
    target = engine()
    assert target.process(lines(),np.full(len(lines()),.05),[0.,0.,0.],1010000000)["accepted"]
    before = snapshot(target)
    result = step(target,1100000000)
    assert result["reason"] == "missing_bootstrap_motion_information"
    assert snapshot(target) == before


def test_first_reference_uses_past_bootstrap_then_active_and_repeat_keep_posterior():
    target = engine(keyframe_refresh_s=.1)
    prior = boot(target)
    first = step(target,1500000000)
    assert first["accepted"] and first["metrics"]["bootstrap_motion_prior_used"]
    assert first["metrics"]["bootstrap_motion_source_stamp_ns"] == 1400000000
    assert first["metrics"]["initial_acquisition"]
    assert target._bootstrap_motion_information is prior
    active = target._matcher
    committed = active.capture_tracking_state()
    assert committed.motion_information is not prior
    second = step(target,1700000000)
    assert second["accepted"] and not second["metrics"]["bootstrap_motion_prior_used"]
    assert not second["metrics"]["initial_acquisition"]
    assert target._matcher is not active
    assert active.capture_tracking_state().motion_information is committed.motion_information
    active_state = target._matcher.capture_tracking_state()
    target.freeze()
    repeat = target.repeat_localizer()
    assert repeat.capture_tracking_state().motion_information is active_state.motion_information
    assert repeat.last_accepted_stamp_ns == active_state.last_accepted_stamp_ns
    np.testing.assert_array_equal(repeat.correction_se2,active_state.correction_se2)


@pytest.mark.parametrize("kind", ["guard", "deadline", "commit", "prepare_deadline"])
def test_first_candidate_abort_keeps_bootstrap_map_and_cannot_delayed_commit(kind, monkeypatch):
    target = engine()
    old = boot(target)
    before = snapshot(target)
    kwargs = {}
    if kind == "guard":
        kwargs["commit_guard"] = deny
    elif kind == "deadline":
        monkeypatch.setattr(target,"_deadline",lambda *args: True)
    elif kind == "commit":
        monkeypatch.setattr(LaneMapLocalizer,"commit",lambda *args: False)
    else:
        elapsed = [0.]
        target._clock = lambda: elapsed[0]
        prepare = target._prepare_matcher
        def slow(*args):
            result = prepare(*args)
            elapsed[0] = 3.
            return result
        monkeypatch.setattr(target,"_prepare_matcher",slow)
    result = step(target,1500000000,**kwargs)
    assert not result["accepted"]
    assert snapshot(target) == before
    assert target._bootstrap_motion_information is old
    assert target._candidate.matcher._pending_result is None
    assert target._candidate.matcher._motion_information is None
    assert target._candidate.matcher.last_accepted_stamp_ns is None


def test_candidate_seed_identity_is_frozen_and_invalidation_rebuilds_candidate():
    target = engine()
    prior = boot(target)
    a = target._prepare_matcher(np.zeros(3),1510000000)
    assert a.bootstrap_motion_owner is prior
    # Same immutable numeric value but different ownership must not reuse the
    # cached candidate. Real engine changes only happen after a guarded commit.
    target._bootstrap_motion_information = replace(prior)
    b = target._prepare_matcher(np.zeros(3),1520000000)
    assert b is not a and b.matcher is not a.matcher
    assert b.bootstrap_motion_owner is target._bootstrap_motion_information
    assert a.bootstrap_motion_owner is prior


def test_preparation_rejects_seed_newer_than_committed_map_frontier():
    target = engine()
    old = boot(target)
    target._bootstrap_motion_information = replace(old, source_stamp_ns=target._map.last_stamp_ns+1)
    with pytest.raises(ValueError, match="uncommitted or future"):
        target._prepare_matcher(np.zeros(3), 1520000000)


@pytest.mark.parametrize("bias", [-.0015,.0015])
def test_seeded_motion_prior_still_corrects_true_gyro_bias_not_only_acceptance(bias):
    target = LaneMapLocalizer(lines(-.5,11.), {"runtime_budget_ms":2000.})
    reference = seed()
    raw_error, estimated_error = [], []
    for k in range(1,21):
        t = k*.1
        prior = np.array([2./bias*math.sin(bias*t),2./bias*(1.-math.cos(bias*t)),bias*t])
        source = 1000000000+k*100000000
        result = target.match(lines(), prior, source+10000000, motion_source_stamp_ns=source,
                              bootstrap_motion_information=reference if k == 1 else None)
        assert result.accepted, result.reason
        raw_error.append(prior[[1,2]])
        estimated_error.append(result.pose_xyyaw[[1,2]])
    raw_rmse = np.sqrt(np.mean(np.asarray(raw_error)**2,axis=0))
    corrected_rmse = np.sqrt(np.mean(np.asarray(estimated_error)**2,axis=0))
    # Predeclared direction/quality check, not a measured hardware-noise claim.
    assert np.all(corrected_rmse < .8*raw_rmse), (raw_rmse,corrected_rmse)
