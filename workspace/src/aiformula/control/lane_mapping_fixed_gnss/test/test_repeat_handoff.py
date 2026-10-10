"""Continuous-session teach/repeat handoff; no simulated driving acceptance.

The drift fixture uses real matching, consensus, commit and unchanged gates.
Worker tests isolate bundle I/O only; existing test_bundle covers real disk
build/load with an explicitly owned analytic frozen-map fixture.
"""

from contextlib import nullcontext
from dataclasses import replace
from pathlib import Path
import sys
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[2] / "lane_mapping_lya_reference"))

from lane_mapping_fixed_gnss.bundle import geometry_sha256
from lane_mapping_fixed_gnss.mapping import TeachMapEngine
from lane_mapping_fixed_gnss import worker as worker_module
from lane_mapping_lya_reference.mask_localization import LaneMapLocalizer
from lane_mapping_lya_reference import mask_localization as localization


def scan():
    x = np.linspace(1., 7., 241)
    return np.vstack([np.column_stack((x, np.full(len(x), side))) for side in (-1., 1.)])


@pytest.fixture
def taught():
    """A stationary scene with gradually drifting raw lateral odometry.

    All 17 frames are admitted by the real engine. The first five establish a
    consensus; the next twelve accumulate a .6 m committed correction in .05 m
    steps, below the unchanged per-frame tracking gate.
    """
    engine = TeachMapEngine()
    points = scan()
    sensitivity = np.full(len(points), .05)
    for index in range(1, 18):
        prior = [0., max(0, index - 5) * .05, 0.]
        result = engine.process(points, sensitivity, prior, index * 100000000)
        assert result["accepted"], result
    np.testing.assert_allclose(engine._matcher.correction_se2, [0., -.6, 0.], atol=1e-12)
    return engine, points, sensitivity


def test_accumulated_correction_survives_repeat_without_reopening_initial_gate(taught):
    engine, points, _ = taught
    snapshot = engine.freeze()
    raw = [0., .6, 0.]
    # Regression oracle: the previous implementation discarded a valid .6 m
    # correction and asked the .45 m first-acquisition gate to recover it anew.
    reset = LaneMapLocalizer(snapshot.consensus_xy)
    failed = reset.match(points, raw, 1800000000, prior_stamp_ns=1800000000, commit=False)
    assert not failed.accepted and failed.reason == "initial_innovation_gate"
    assert failed.metrics["inlier_ratio"] == 1.
    assert failed.metrics["rmse_after_m"] < 1e-12

    repeat = engine.repeat_localizer()
    result = repeat.match(points, raw, 1800000000, prior_stamp_ns=1800000000, commit=False)
    assert result.accepted, result
    assert result.metrics["initial_acquisition"] is False
    assert result.metrics["innovation_translation_m"] < 1e-12
    assert result.metrics["total_translation_at_vehicle_m"] == pytest.approx(.6)
    np.testing.assert_allclose(result.pose_xyyaw, snapshot.trace[-1], atol=1e-12)
    assert repeat.commit(result)
    assert repeat.committed_frames == 1


@pytest.mark.parametrize("delta", [.121, .20])
def test_repeat_keeps_original_tracking_innovation_gate(taught, delta):
    engine, points, _ = taught
    repeat = engine.repeat_localizer()
    before = repeat.correction_se2.copy()
    result = repeat.match(points, [0., .6 + delta, 0.], 1800000000,
                          prior_stamp_ns=1800000000, commit=False)
    assert not result.accepted and result.reason == "tracking_innovation_gate"
    assert result.metrics["initial_acquisition"] is False
    assert repeat.config["tracking_max_translation_m"] == .12
    np.testing.assert_array_equal(repeat.correction_se2, before)
    assert repeat.committed_frames == 0


def test_handoff_inherits_last_seen_and_accepted_not_pending_candidate_or_refinement(taught):
    engine, points, sensitivity = taught
    # The last keyframe refresh intentionally resets anchor-specific votes.
    # Mature this actual active matcher with real committed observations first.
    for index in range(18, 21):
        result = engine.process(points, sensitivity, [0., .6, 0.], index * 100000000)
        assert result["accepted"], result
    active = engine._matcher
    committed = active.correction_se2.copy()
    candidate = engine._prepare_matcher(np.array([0., .6, 0.]), 2100000000)
    assert candidate is not None and candidate.matcher is not active
    candidate_pending = candidate.matcher.match(points, [0., .65, 0.], 2100000000,
                                                prior_stamp_ns=2100000000, commit=False)
    active_pending = active.match(points, [0., .7, 0.], 2200000000,
                                  prior_stamp_ns=2200000000, commit=False)
    assert candidate_pending.accepted and active_pending.accepted
    assert len(active.refined_points_and_votes()[0]) > 0
    before = geometry_sha256(engine.freeze())
    repeat = engine.repeat_localizer()
    assert repeat is not active and repeat is not candidate.matcher
    np.testing.assert_array_equal(repeat.correction_se2, committed)
    assert repeat.last_accepted_stamp_ns == active.last_accepted_stamp_ns == 2000000000
    assert repeat.last_seen_stamp_ns == active.last_seen_stamp_ns == 2200000000
    assert repeat.last_seen_stamp_ns != candidate.matcher.last_seen_stamp_ns
    assert repeat._pending_result is None and repeat._pending_data is None
    assert not repeat.commit(active_pending) and not repeat.commit(candidate_pending)
    assert repeat.committed_frames == 0
    assert len(repeat.refined_points_and_votes()[0]) == 0
    assert repeat.refinement_metrics()["refinement_retained_frames"] == 0
    # Preparing a child does not consume or otherwise mutate its source state.
    assert active._pending_result is active_pending
    assert candidate.matcher._pending_result is candidate_pending
    assert geometry_sha256(engine.freeze()) == before
    replay = repeat.match(points, [0., .6, 0.], 2200000000,
                          prior_stamp_ns=2200000000, commit=False)
    assert not replay.accepted and replay.reason == "nonincreasing_mask_stamp"


def test_repeat_refinement_and_multiple_children_cannot_mutate_frozen_teach_state(taught):
    engine, points, sensitivity = taught
    snapshot = engine.freeze()
    before = geometry_sha256(snapshot)
    active = engine._matcher
    correction = active.correction_se2.copy()
    commits = active.committed_frames
    first, second = engine.repeat_localizer(), engine.repeat_localizer()
    assert first is not second
    assert not np.shares_memory(first.anchor_points, second.anchor_points)
    assert not np.shares_memory(first.anchor_points, snapshot.consensus_xy)
    for index in range(18, 22):
        result = first.match(points, [0., .62, 0.], index * 100000000)
        assert result.accepted, result
    assert len(first.refined_points_and_votes()[0]) > 0
    assert len(second.refined_points_and_votes()[0]) == 0
    np.testing.assert_array_equal(second.correction_se2, correction)
    np.testing.assert_array_equal(active.correction_se2, correction)
    assert active.committed_frames == commits
    assert geometry_sha256(snapshot) == before
    with pytest.raises(ValueError, match="frozen"):
        engine.process(points, sensitivity, [0., .62, 0.], 2200000000)
    assert geometry_sha256(snapshot) == before


def test_bootstrap_without_active_keeps_fresh_acquisition_semantics():
    engine = TeachMapEngine()
    points = scan()
    for index in range(1, 6):
        assert engine.process(points, np.full(len(points), .05), [0., 0., 0.],
                              index * 100000000)["accepted"]
    assert engine._matcher is None
    repeat = engine.repeat_localizer()
    np.testing.assert_array_equal(repeat.correction_se2, np.zeros(3))
    assert repeat.last_seen_stamp_ns is None and repeat.last_accepted_stamp_ns is None
    result = repeat.match(points, [0., .2, 0.], 600000000, prior_stamp_ns=600000000)
    assert result.accepted and result.metrics["initial_acquisition"] is True


def test_capture_frozen_repeat_state_does_not_build_full_map_index(taught, monkeypatch):
    engine, points, _ = taught
    snapshot = engine.freeze()
    original_index = localization._FrozenIndex

    def forbidden(*args, **kwargs):
        raise AssertionError("capturing active repeat state must not construct an index")

    with monkeypatch.context() as capture_patch:
        capture_patch.setattr(localization, "_FrozenIndex", forbidden)
        state = engine.capture_repeat_state()
    assert engine.freeze() is snapshot
    # A later background operation constructs the index from immutable inputs.
    built = Mock(wraps=original_index)
    monkeypatch.setattr(localization, "_FrozenIndex", built)
    repeat = LaneMapLocalizer.from_tracking_state(snapshot.consensus_xy, state)
    assert built.call_count == 1
    assert repeat.matches_anchor_source(snapshot.consensus_xy)
    result = repeat.match(points, [0., .6, 0.], 1800000000)
    assert result.accepted and result.metrics["initial_acquisition"] is False


def test_bootstrap_capture_only_constructs_empty_index(monkeypatch):
    engine = TeachMapEngine()
    points = scan()
    assert engine.process(points, np.full(len(points), .05), [0., 0., 0.], 100000000)["accepted"]
    assert engine._matcher is None
    engine.freeze()
    original_index = localization._FrozenIndex
    anchor_sizes = []

    def record(anchor, config):
        anchor_sizes.append(len(anchor))
        return original_index(anchor, config)

    monkeypatch.setattr(localization, "_FrozenIndex", record)
    state = engine.capture_repeat_state()
    assert anchor_sizes == [0]
    assert state.correction_se2 == (0., 0., 0.)
    assert state.last_accepted_stamp_ns is None and state.last_seen_stamp_ns is None


@pytest.fixture
def bundle_worker(taught, monkeypatch):
    engine, points, _ = taught
    worker = worker_module.MapWorker(lambda _: nullcontext(True))
    # These synchronous branch tests need no background work. The original
    # threaded bundle regression remains covered by test_bundle.py.
    worker.close()
    assert not worker.thread.is_alive()
    worker.engine = engine
    worker.snapshot = engine.freeze()
    worker.calibration = {"frame_id": "camera"}
    path = Path("synthetic-current-session/bundle.json")
    loaded = ({"ready": True}, {"closed": True},
              {"snapshot_geometry_sha256": geometry_sha256(worker.snapshot),
               "source_frontier_ns": worker.snapshot.source_frontier_ns},
              worker.snapshot.consensus_xy.copy(), dict(worker.calibration))
    build = Mock(return_value=path)
    load = Mock(return_value=loaded)
    handoff = Mock(wraps=engine.repeat_localizer)
    monkeypatch.setattr(worker_module, "build_bundle", build)
    monkeypatch.setattr(worker_module, "load_bundle", load)
    monkeypatch.setattr(engine, "repeat_localizer", handoff)
    return worker, points, loaded, build, load, handoff


def test_worker_uses_validated_owned_snapshot_and_real_committed_handoff(bundle_worker):
    worker, points, _, build, load, handoff = bundle_worker
    result = worker._compute({"kind": "bundle", "root": "unused-test-output"})
    assert result["kind"] == "bundle"
    assert build.call_args.args[1] is worker.snapshot
    load.assert_called_once_with(build.return_value)
    handoff.assert_called_once_with()
    np.testing.assert_allclose(worker.matcher.correction_se2, [0., -.6, 0.], atol=1e-12)
    assert worker.matcher.last_accepted_stamp_ns == 1700000000
    matched = worker.matcher.match(points, [0., .6, 0.], 1800000000,
                                   prior_stamp_ns=1800000000, commit=False)
    assert matched.accepted and not matched.metrics["initial_acquisition"]


@pytest.mark.parametrize("fault", ["changed", "reordered", "missing", "nan", "tiny_change"])
def test_worker_rejects_nonidentical_loaded_anchor_before_seed(bundle_worker, fault):
    worker, _, loaded, _, _, handoff = bundle_worker
    anchor = loaded[3]
    if fault == "changed":
        anchor[0, 0] += .1
    elif fault == "reordered":
        anchor[:] = anchor[::-1].copy()
    elif fault == "missing":
        anchor.resize((len(anchor) - 1, 2), refcheck=False)
    elif fault == "nan":
        anchor[0, 0] = np.nan
    else:
        anchor[0, 0] = np.nextafter(anchor[0, 0], np.inf)
    prior_matcher = worker.engine.repeat_localizer()
    handoff.reset_mock()
    worker.matcher = prior_matcher
    with pytest.raises(ValueError, match="loaded bundle does not match"):
        worker._compute({"kind": "bundle", "root": "unused-test-output"})
    handoff.assert_not_called()
    assert worker.matcher is prior_matcher


@pytest.mark.parametrize("field", ["snapshot_geometry_sha256", "source_frontier_ns"])
def test_worker_rejects_same_anchor_with_different_frozen_evidence(bundle_worker, field):
    worker, _, loaded, _, _, handoff = bundle_worker
    loaded[2][field] = "foreign" if field.endswith("sha256") else worker.snapshot.source_frontier_ns + 1
    with pytest.raises(ValueError, match="loaded bundle does not match"):
        worker._compute({"kind": "bundle", "root": "unused-test-output"})
    handoff.assert_not_called()
    assert worker.matcher is None


def test_worker_rejects_identical_geometry_owned_by_another_session_before_build(bundle_worker):
    worker, _, _, build, load, handoff = bundle_worker
    # Geometry equality alone is not session identity. This detached snapshot
    # has no authority to borrow the live engine's committed correction.
    worker.snapshot = replace(worker.snapshot)
    with pytest.raises(ValueError, match="not this continuous teach session"):
        worker._compute({"kind": "bundle", "root": "unused-test-output"})
    build.assert_not_called()
    load.assert_not_called()
    handoff.assert_not_called()
    assert worker.matcher is None


def test_worker_requires_explicit_freeze_before_bundle(bundle_worker):
    worker, _, _, build, load, handoff = bundle_worker
    worker.snapshot = None
    with pytest.raises(ValueError, match="must be frozen"):
        worker._compute({"kind": "bundle", "root": "unused-test-output"})
    build.assert_not_called()
    load.assert_not_called()
    handoff.assert_not_called()
