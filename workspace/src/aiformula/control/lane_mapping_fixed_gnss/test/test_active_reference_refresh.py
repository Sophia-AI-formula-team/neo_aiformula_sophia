"""Coverage-triggered reference scheduling, not looser localization gates.

Real localizers produce proposals; injected rejections explicitly revoke them.
The fake engine clock tests the unchanged 80ms admission boundary without sleeps.
Only the two boundary-geometry cases use explicit 2s unit budgets: they test
ownership/18-array identity, not runtime acceptance. No simulator or ROS needed.
"""
from collections.abc import Mapping
from contextlib import contextmanager
from dataclasses import fields, is_dataclass, replace
from pathlib import Path
import sys
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[2] / "lane_mapping_lya_reference"))

from lane_mapping_fixed_gnss import mapping
from lane_mapping_lya_reference.mapping_core import GroundLookup, transform_local_points
from lane_mapping_lya_reference.paint_geometry import extract_boundary_observation


REFERENCE_FIELDS = (
    "points_xy", "normals", "support_segments", "segment_owner", "votes",
    "reliable_votes", "record_ids", "directed_edges", "edge_frame_votes",
    "edge_reliable_votes", "edge_last_stamp_ns", "full_in_degree", "full_out_degree",
    "certified_model_segments", "certified_model_owner", "certified_model_frame_votes",
    "certified_model_reliable_votes", "certified_model_last_stamp_ns",
)
NEXT_STAMP = 1600000000


def value(item):
    if isinstance(item, np.ndarray):
        return item.dtype.str, item.shape, item.tobytes()
    if is_dataclass(item):
        return tuple((field.name, value(getattr(item, field.name))) for field in fields(item))
    if isinstance(item, Mapping):
        return tuple((key, value(child)) for key, child in sorted(item.items()))
    if isinstance(item, (list, tuple)):
        return tuple(map(value, item))
    if isinstance(item, set):
        return tuple(sorted(item))
    return item


def committed_matcher(matcher):
    return value((matcher.correction_se2, matcher.last_accepted_stamp_ns,
                  matcher._motion_information, matcher.committed_frames,
                  matcher.refinement_metrics(), matcher.refined_points_and_votes(),
                  matcher._refinement.candidate_votes, matcher._refined_votes))


def committed_engine(engine):
    grid, boundary = engine._map, engine._boundary_map
    boundary_state = None if boundary is None else (
        boundary.source_frontier_ns, boundary._generation, boundary._next_record_id,
        boundary._segment_count, boundary._confirmed_count, boundary._records,
        boundary._edges, boundary._cell_records, boundary._domains,
        boundary._domain_interval_count, boundary._domain_expiry, boundary.snapshot())
    return value((grid.last_stamp_ns, grid._geometry_generation, grid.candidate_votes,
                  grid.candidate_reliable_votes, grid.confirmed_keys, grid._cell_means,
                  grid.capacity_rejected_cells, grid.confirmation_capacity_rejections,
                  boundary_state, engine._trace, engine._anchor_frontier,
                  engine._keyframe_stamp, engine._keyframe_pose,
                  engine._last_motion_source_stamp, engine._bootstrap_motion_information,
                  engine._past_direction_support, engine._past_direction_support_stamp_ns,
                  engine.counts["accepted"]))


class Clock:
    def __init__(self):
        self.now = 0.

    def __call__(self):
        return self.now


@pytest.fixture(scope="module")
def projected_boundary():
    mask = np.zeros((100, 160), dtype=np.uint8)
    mask[27:33, 20:145] = 255
    mask[67:73, 20:145] = 255
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = 2.
    camera = np.array([[25., 0., 0.], [0., 25., 50.], [0., 0., 1.]])
    lookup = GroundLookup(160, 100, camera, transform, .5)
    points, sensitivity, _ = lookup.project_mask_with_sensitivity(mask, 127)
    observation = extract_boundary_observation(mask, lookup, [0., 0., 1.], 1000000000)
    return points, sensitivity, observation


class Harness:
    def __init__(self, monkeypatch, projected=None):
        self.patch, self.clock = monkeypatch, Clock()
        if projected is None:
            x = np.linspace(1., 7., 121)
            self.points = np.vstack([np.column_stack((x, np.full(len(x), y))) for y in (-1., 1.)])
            self.sensitivity, self.observation = np.full(len(self.points), .05), None
            self.engine = mapping.TeachMapEngine(clock=self.clock)
        else:
            self.points, self.sensitivity, self.observation = projected
            self.engine = mapping.TeachMapEngine({"max_processing_ms": 2000.},
                                                {"runtime_budget_ms": 2000.}, clock=self.clock)
        for index in range(6):
            result = self.process(1000000000+index*100000000)
            assert result["accepted"], result
        self.active = self.engine._matcher
        assert self.active is not None
        assert self.engine._anchor_frontier == 1400000000
        assert self.engine._map.last_stamp_ns == 1500000000
        assert self.engine._keyframe_stamp == 1500000000
        self.before, self.active_before = committed_engine(self.engine), committed_matcher(self.active)
        self.seed_before = self.active.capture_tracking_state()
        self.map_votes = dict(self.engine._map.candidate_votes)
        self.events, self.candidates, self.captures = [], [], []
        self.active_reason, self.candidate_reason = "insufficient_overlap", None
        self.active_metrics, self.candidate_metrics = {}, {}
        self.active_seconds = self.prepare_seconds = self.candidate_seconds = 0.
        self.return_no_candidate = False
        self.candidate_commit_ok = True
        self.real_prepare = self.engine._prepare_matcher
        self.real_capture = self.active.capture_tracking_state

        def capture():
            self.events.append("capture")
            state = self.real_capture()
            self.captures.append(state)
            return state

        self.capture = Mock(side_effect=capture)
        monkeypatch.setattr(self.active, "capture_tracking_state", self.capture)
        self.active_match = self.instrument_match(self.active, "active")
        self.active_commit = Mock(wraps=self.active.commit)
        monkeypatch.setattr(self.active, "commit", self.active_commit)

        def prepare(prior, stamp, *, tracking_state=None):
            self.events.append("prepare")
            if self.return_no_candidate:
                return None
            prepared = self.real_prepare(prior, stamp, tracking_state=tracking_state)
            self.clock.now += self.prepare_seconds
            if prepared is not None:
                matcher = prepared.matcher
                record = {"prepared": prepared, "matcher": matcher, "seed": tracking_state,
                          "inherited": matcher.capture_tracking_state()}
                record["match"] = self.instrument_match(matcher, "candidate")
                original_commit = matcher.commit

                def commit(result):
                    self.events.append("candidate_commit")
                    assert committed_engine(self.engine) == self.before
                    return original_commit(result) if self.candidate_commit_ok else False

                record["commit"] = Mock(side_effect=commit)
                monkeypatch.setattr(matcher, "commit", record["commit"])
                self.candidates.append(record)
            return prepared

        self.prepare = Mock(side_effect=prepare)
        monkeypatch.setattr(self.engine, "_prepare_matcher", self.prepare)
        original_update = self.engine._map.update

        def update(*args, **kwargs):
            self.events.append("map_update")
            assert self.engine._matcher is self.active
            return original_update(*args, **kwargs)

        self.update = Mock(side_effect=update)
        monkeypatch.setattr(self.engine._map, "update", self.update)

    def process(self, stamp=NEXT_STAMP, guard=None):
        boundary = None if self.observation is None else replace(self.observation, source_stamp_ns=stamp)
        return self.engine.process(self.points, self.sensitivity, [0., 0., 0.], stamp,
            motion_source_stamp_ns=stamp-10000000, boundary_observation=boundary, commit_guard=guard)

    def instrument_match(self, matcher, name):
        original = matcher.match

        def run(*args, **kwargs):
            self.events.append(name+"_match")
            assert kwargs["commit"] is False
            assert kwargs["prior_stamp_ns"] == kwargs["motion_source_stamp_ns"] < args[2]
            before = committed_matcher(matcher)
            result = original(*args, **kwargs)
            assert committed_matcher(matcher) == before
            self.clock.now += getattr(self, name+"_seconds")
            reason = getattr(self, name+"_reason")
            if reason is not None:
                matcher.discard_pending()
                metrics = dict(result.metrics)
                for key, item in getattr(self, name+"_metrics").items():
                    if item == "omit":
                        metrics.pop(key, None)
                    else:
                        metrics[key] = item
                return replace(result, accepted=False, reason=reason, metrics=metrics)
            assert result.accepted, result
            return result

        spy = Mock(side_effect=run)
        self.patch.setattr(matcher, "match", spy)
        return spy

    def assert_aborted(self, result):
        assert not result["accepted"]
        assert self.engine._matcher is self.active
        assert committed_engine(self.engine) == self.before
        assert committed_matcher(self.active) == self.active_before
        self.update.assert_not_called()
        self.active_commit.assert_not_called()
        for matcher in [self.active]+[item["matcher"] for item in self.candidates]:
            assert matcher._pending_result is None
            assert matcher._pending_data is None
            assert matcher._pending_motion_information is None

    def assert_one_vote(self, result):
        self.update.assert_called_once()
        keys = self.engine._map.points_to_unique_keys(transform_local_points(self.points, *result["pose_xyyaw"]))
        expected = dict(self.map_votes)
        for key in keys:
            expected[int(key)] = expected.get(int(key), 0)+1
        assert self.engine._map.candidate_votes == expected
        assert len(self.engine._trace) == 7


@pytest.fixture
def harness(monkeypatch):
    return Harness(monkeypatch)


@pytest.mark.parametrize("reason", ["empty_anchor", "insufficient_anchor_line_support",
                                   "insufficient_overlap", "insufficient_overlap_after_raw_prior_anchor"])
def test_coverage_refresh_uses_pre_attempt_state_once_before_period(harness, reason):
    h = harness
    h.active_reason = reason

    @contextmanager
    def guard():
        h.events.append("guard")
        assert committed_engine(h.engine) == h.before
        assert committed_matcher(h.active) == h.active_before
        yield True
        assert h.engine._matcher is h.candidates[0]["matcher"]

    result = h.process(guard=guard)
    assert result["accepted"], result
    fresh = h.candidates[0]
    assert fresh["seed"] is h.captures[0]
    assert value(fresh["seed"]) == value(h.seed_before)
    assert value(fresh["inherited"]) == value(h.seed_before)
    assert fresh["inherited"].last_seen_stamp_ns == 1500000000
    assert h.active.last_seen_stamp_ns == NEXT_STAMP
    assert h.active.last_accepted_stamp_ns == 1500000000
    assert h.active._motion_information is h.seed_before.motion_information
    assert h.engine._matcher is fresh["matcher"]
    assert fresh["matcher"].last_accepted_stamp_ns == NEXT_STAMP
    assert fresh["matcher"]._motion_information.source_stamp_ns == NEXT_STAMP-10000000
    assert h.engine._anchor_frontier == 1500000000 < NEXT_STAMP
    assert h.engine._keyframe_stamp == NEXT_STAMP
    assert h.events == ["capture", "active_match", "prepare", "candidate_match",
                        "guard", "candidate_commit", "map_update"]
    attempts = result["metrics"]["reference_attempts"]
    assert [(item["reference"], item["frontier_ns"], item["accepted"]) for item in attempts] == [
        ("active", 1400000000, False), ("candidate", 1500000000, True)]
    assert result["metrics"]["selected_reference"] == "candidate"
    assert result["matched_against_frontier_ns"] == 1500000000
    h.active_commit.assert_not_called()
    fresh["commit"].assert_called_once()
    h.assert_one_vote(result)


@pytest.mark.parametrize("reason", ["tracking_innovation_gate", "total_correction_gate",
    "residual_gate", "residual_worsened", "unobservable_geometry", "insufficient_final_inliers",
    "invalid_input", "nonincreasing_mask_stamp", "future_or_invalid_prior_stamp",
    "nonincreasing_motion_source_stamp", "runtime_budget_exceeded"])
def test_noncoverage_failure_does_not_request_more_references(harness, reason):
    harness.active_reason = reason
    result = harness.process()
    harness.assert_aborted(result)
    harness.prepare.assert_not_called()
    assert len(result["metrics"]["reference_attempts"]) == 1


@pytest.mark.parametrize("metrics", [
    {"budget_exceeded": True}, {"budget_exceeded": "omit"},
    {"elapsed_ms": 40.000001}, {"elapsed_ms": -1.}, {"elapsed_ms": np.inf},
    {"elapsed_ms": np.nan}, {"elapsed_ms": "omit"},
])
def test_overlap_reason_cannot_hide_a_failed_or_missing_match_budget(harness, metrics):
    harness.active_metrics = metrics
    result = harness.process()
    harness.assert_aborted(result)
    harness.prepare.assert_not_called()


@pytest.mark.parametrize("milliseconds", [0., 40.])
def test_exact_nonnegative_match_budget_endpoints_remain_admissible(harness, milliseconds):
    harness.active_metrics = {"elapsed_ms": milliseconds, "budget_exceeded": False}
    result = harness.process()
    assert result["accepted"], result
    harness.prepare.assert_called_once()


@pytest.mark.parametrize("frontier", [None, 1400000000, NEXT_STAMP, NEXT_STAMP+1])
def test_no_newer_strictly_past_reference_means_no_refresh(harness, monkeypatch, frontier):
    # Adversarial frontier metadata: never fabricate map votes to arrange this.
    monkeypatch.setattr(harness.engine._map, "last_stamp_ns", frontier)
    harness.before = committed_engine(harness.engine)
    result = harness.process()
    harness.assert_aborted(result)
    harness.capture.assert_not_called()
    harness.prepare.assert_not_called()


@pytest.mark.parametrize("stage", ["before_prepare", "after_prepare", "after_candidate", "guard"])
def test_engine_budget_cannot_publish_fresh_reference_or_votes(harness, stage):
    h = harness
    if stage == "before_prepare":
        h.active_seconds = .081
    elif stage == "after_prepare":
        h.prepare_seconds = .081
    elif stage == "after_candidate":
        h.candidate_seconds = .081

    @contextmanager
    def guard():
        if stage == "guard":
            h.clock.now += .081
        yield True

    result = h.process(guard=guard)
    h.assert_aborted(result)
    assert result["reason"] == "processing_deadline"
    if stage == "before_prepare":
        h.prepare.assert_not_called()
    elif stage == "after_prepare":
        h.candidates[0]["match"].assert_not_called()
    assert len(result["metrics"]["reference_attempts"]) <= 2


@pytest.mark.parametrize("failure", ["candidate_rejection", "candidate_budget", "guard", "commit", "no_anchor"])
def test_fresh_failure_has_no_third_attempt_or_state_publication(harness, failure):
    h = harness
    if failure == "candidate_rejection":
        h.candidate_reason = "insufficient_overlap"
    elif failure == "candidate_budget":
        h.candidate_reason = "runtime_budget_exceeded"
        h.candidate_metrics = {"elapsed_ms": 40.001, "budget_exceeded": True}
    elif failure == "commit":
        h.candidate_commit_ok = False
    elif failure == "no_anchor":
        h.return_no_candidate = True

    @contextmanager
    def guard():
        yield failure != "guard"

    result = h.process(guard=guard)
    h.assert_aborted(result)
    h.active_match.assert_called_once()
    h.prepare.assert_called_once()
    assert len(result["metrics"]["reference_attempts"]) == (1 if failure == "no_anchor" else 2)
    if failure != "no_anchor":
        h.candidates[0]["match"].assert_called_once()


def test_periodic_candidate_then_active_does_not_add_a_third_fresh_attempt(harness):
    h = harness
    h.candidate_reason = "insufficient_overlap"
    result = h.process(2600000000)
    h.assert_aborted(result)
    h.prepare.assert_called_once()
    h.active_match.assert_called_once()
    h.candidates[0]["match"].assert_called_once()
    assert [item["reference"] for item in result["metrics"]["reference_attempts"]] == ["candidate", "active"]
    assert h.candidates[0]["seed"] is None


def test_duplicate_mask_after_failed_two_attempts_does_not_reopen_either_matcher(harness):
    h = harness
    h.candidate_reason = "insufficient_overlap"
    h.assert_aborted(h.process())
    calls = list(h.events)
    duplicate = h.process()
    assert duplicate["reason"] == "nonincreasing_mask_stamp"
    assert h.events == calls
    h.assert_aborted(duplicate)


def test_override_rebuilds_inactive_candidate_from_before_current_seen_fence(harness):
    h = harness
    # A retained inactive candidate has already seen this source stamp. Its
    # fence must not be rolled back or reused by an explicit saved-state build.
    old = h.real_prepare(np.zeros(3), NEXT_STAMP)
    result = old.matcher.match(h.points, [0., 0., 0.], NEXT_STAMP, commit=False,
        prior_stamp_ns=NEXT_STAMP-10000000, motion_source_stamp_ns=NEXT_STAMP-10000000,
        scan_support_xy=h.points, scan_sensitivity=h.sensitivity)
    assert result.accepted
    old.matcher.discard_pending()
    assert old.matcher.last_seen_stamp_ns == NEXT_STAMP
    h.events.clear()
    h.captures.clear()
    result = h.process()
    assert result["accepted"], result
    assert h.candidates[0]["matcher"] is not old.matcher
    assert old.matcher.last_seen_stamp_ns == NEXT_STAMP
    assert h.candidates[0]["inherited"].last_seen_stamp_ns == 1500000000
    h.assert_one_vote(result)


@pytest.mark.parametrize("deny", [False, True])
def test_boundary_refresh_keeps_eighteen_field_past_identity_and_hidden_votes(
        monkeypatch, projected_boundary, deny):
    h = Harness(monkeypatch, projected_boundary)
    past = h.engine._boundary_map.snapshot_near(np.zeros(2), 12., 3000)
    past_bytes = tuple(value(getattr(past, name)) for name in REFERENCE_FIELDS)
    generation = h.engine._boundary_map._generation

    @contextmanager
    def guard():
        assert committed_engine(h.engine) == h.before
        candidate_reference = h.candidates[0]["matcher"]._index.reference
        assert candidate_reference.source_frontier_ns == 1500000000 < NEXT_STAMP
        assert tuple(value(getattr(candidate_reference, name)) for name in REFERENCE_FIELDS) == past_bytes
        assert h.candidates[0]["matcher"].matches_anchor_source(past)
        yield not deny

    result = h.process(guard=guard)
    assert len(REFERENCE_FIELDS) == 18
    assert tuple(value(getattr(past, name)) for name in REFERENCE_FIELDS) == past_bytes
    if deny:
        h.assert_aborted(result)
        assert h.engine._boundary_map._generation == generation
    else:
        assert result["accepted"], result
        h.assert_one_vote(result)
        assert h.engine._boundary_map._generation == generation+1
        assert h.engine._boundary_map.source_frontier_ns == NEXT_STAMP
        assert h.engine._matcher._index.reference.source_frontier_ns == 1500000000
        assert h.engine._matcher._index.reference.boundary_version == 3
