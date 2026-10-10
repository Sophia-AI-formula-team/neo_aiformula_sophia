"""Keyframe refresh transactions using real matching, commit and map votes.

These are isolated synthetic scan tests, not a simulator lap or vehicle result.
Controlled match outcomes and a monotonic test clock inject failure boundaries;
production correction, proposal ownership, consensus and geometry stay real.
"""

import copy
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
import sys
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[2] / "lane_mapping_lya_reference"))

from lane_mapping_fixed_gnss import mapping
from lane_mapping_lya_reference.mapping_core import transform_local_points
from lane_mapping_lya_reference.mask_localization import LaneMapLocalizer


def scan():
    x = np.linspace(1., 7., 241)
    return np.vstack((np.column_stack((x, np.ones(len(x)))),
                      np.column_stack((x, -np.ones(len(x))))))


class Clock:
    def __init__(self):
        self.now = 0.

    def __call__(self):
        return self.now


def map_state(grid):
    return {"candidate_votes": dict(grid.candidate_votes),
            "candidate_reliable_votes": dict(grid.candidate_reliable_votes),
            "confirmed_keys": set(grid.confirmed_keys),
            "measured_means": dict(grid._cell_means),
            "geometry_generation": grid._geometry_generation,
            "last_stamp_ns": grid.last_stamp_ns}


def committed_state(engine):
    active = engine._matcher
    return {"active": active, "anchor_frontier": engine._anchor_frontier,
            "keyframe_stamp": engine._keyframe_stamp,
            "keyframe_pose": None if engine._keyframe_pose is None else engine._keyframe_pose.copy(),
            "map": map_state(engine._map), "trace": np.asarray(engine._trace).copy(),
            "correction": None if active is None else active.correction_se2.copy(),
            "accepted_stamp": None if active is None else active.last_accepted_stamp_ns,
            "committed_frames": None if active is None else active.committed_frames,
            "refinement": None if active is None else active.refined_points_and_votes()}


def assert_unchanged(engine, before):
    """Diagnostic last_seen/rejection counters may advance; committed state may not."""
    assert engine._matcher is before["active"]
    assert engine._anchor_frontier == before["anchor_frontier"]
    assert engine._keyframe_stamp == before["keyframe_stamp"]
    np.testing.assert_array_equal(engine._keyframe_pose, before["keyframe_pose"])
    assert map_state(engine._map) == before["map"]
    np.testing.assert_array_equal(engine._trace, before["trace"])
    if engine._matcher is not None:
        active = engine._matcher
        np.testing.assert_array_equal(active.correction_se2, before["correction"])
        assert active.last_accepted_stamp_ns == before["accepted_stamp"]
        assert active.committed_frames == before["committed_frames"]
        points, votes = active.refined_points_and_votes()
        np.testing.assert_array_equal(points, before["refinement"][0])
        np.testing.assert_array_equal(votes, before["refinement"][1])


class Harness:
    """Five real consensus frames, one real initial match, then a due refresh."""
    def __init__(self, monkeypatch):
        self.monkeypatch = monkeypatch
        self.clock = Clock()
        self.engine = mapping.TeachMapEngine({"keyframe_refresh_s": .25}, clock=self.clock)
        self.points = scan()
        self.sensitivity = np.full(len(self.points), .05)
        for index in range(1, 7):
            row = self.engine.process(self.points, self.sensitivity, [0., 0., 0.], index * 100000000)
            assert row["accepted"], row
        self.active = self.engine._matcher
        assert isinstance(self.active, LaneMapLocalizer)
        assert self.active.last_accepted_stamp_ns == 600000000
        assert self.engine._anchor_frontier == 500000000
        self.before = committed_state(self.engine)
        self.expected_map = copy.deepcopy(self.engine._map)
        self.calls = []
        self.candidates = []
        self.candidate_reason = None
        self.active_reason = None
        self.prepare_seconds = self.candidate_seconds = self.active_seconds = 0.
        self.candidate_commit_ok = True
        self.active_commit_ok = True
        self.on_candidate_commit = self.on_map_update = None
        self.original_fork = self.active.fork_with_anchor
        self.active_match = self.instrument_match(self.active, "active")
        self.active_commit = self.instrument_commit(self.active, "active")
        self.fork = Mock(side_effect=self.prepare)
        monkeypatch.setattr(self.active, "fork_with_anchor", self.fork)
        original_update = self.engine._map.update

        def update(*args, **kwargs):
            self.calls.append("map_update")
            if self.on_map_update is not None:
                self.on_map_update()
            return original_update(*args, **kwargs)

        self.update = Mock(side_effect=update)
        monkeypatch.setattr(self.engine._map, "update", self.update)

    def instrument_match(self, matcher, name):
        original = matcher.match

        def match(*args, **kwargs):
            self.calls.append(name + "_match")
            assert kwargs["commit"] is False
            assert kwargs["prior_stamp_ns"] <= args[2]
            result = original(*args, **kwargs)
            self.clock.now += getattr(self, name + "_seconds")
            reason = getattr(self, name + "_reason")
            if reason is not None:
                # Preserve the real match() rejection protocol: rejected results
                # have no pending proposal. A post-hoc accepted=False replacement
                # alone would fabricate an impossible retained-success state.
                matcher.discard_pending()
                return replace(result, accepted=False, reason=reason,
                               metrics=dict(result.metrics, injected_rejection=reason))
            assert result.accepted, result
            return result

        spy = Mock(side_effect=match)
        self.monkeypatch.setattr(matcher, "match", spy)
        return spy

    def instrument_commit(self, matcher, name):
        original = matcher.commit

        def commit(result):
            self.calls.append(name + "_commit")
            if name == "candidate" and self.on_candidate_commit is not None:
                self.on_candidate_commit()
            if not getattr(self, name + "_commit_ok"):
                return False
            return original(result)

        spy = Mock(side_effect=commit)
        self.monkeypatch.setattr(matcher, "commit", spy)
        return spy

    def prepare(self, anchor, anchor_support_xy=None, anchor_support_stamp_ns=None):
        self.calls.append("prepare")
        candidate = self.original_fork(anchor, anchor_support_xy, anchor_support_stamp_ns)
        self.clock.now += self.prepare_seconds
        record = {"matcher": candidate, "anchor": np.asarray(anchor).copy(),
                  "anchor_support": None if anchor_support_xy is None else np.asarray(anchor_support_xy).copy(),
                  "anchor_support_stamp_ns": anchor_support_stamp_ns,
                  "inherited_correction": candidate.correction_se2.copy(),
                  "inherited_accepted": candidate.last_accepted_stamp_ns,
                  "inherited_seen": candidate.last_seen_stamp_ns}
        record["match"] = self.instrument_match(candidate, "candidate")
        record["commit"] = self.instrument_commit(candidate, "candidate")
        self.candidates.append(record)
        return candidate

    def process(self, *, stamp=900000000, points=None, guard=None):
        points = self.points if points is None else points
        return self.engine.process(points, np.full(len(points), .05), [0., 0., 0.], stamp,
                                   commit_guard=guard)

    def assert_single_map_vote(self, row, points=None):
        points = self.points if points is None else points
        world = transform_local_points(points, *row["pose_xyyaw"])
        keys = self.expected_map.points_to_unique_keys(world)
        # The copied pre-transaction map now explicitly owns measured geometry.
        # Keep the independent key/vote expectation and every atomicity check;
        # supply actual current measurements instead of falling back to centers.
        geometry = self.expected_map.prepare_geometry(world, row["stamp_ns"])
        np.testing.assert_array_equal(geometry.keys, keys)
        self.expected_map.update(keys, keys, stamp_ns=row["stamp_ns"], geometry=geometry)
        assert map_state(self.engine._map) == map_state(self.expected_map)
        assert len(self.engine._trace) == len(self.before["trace"]) + 1
        self.update.assert_called_once()

    def assert_aborted(self, row):
        assert not row["accepted"], row
        assert row["reason"] != "bootstrap_odometry"
        assert_unchanged(self.engine, self.before)
        self.update.assert_not_called()
        for record in self.candidates:
            assert record["matcher"]._pending_result is None
            assert record["matcher"]._pending_data is None
        assert self.active._pending_result is None
        assert self.active._pending_data is None


@pytest.fixture
def harness(monkeypatch):
    return Harness(monkeypatch)


def test_candidate_promotes_only_after_guard_commit_and_map_update(harness):
    h = harness
    h.on_candidate_commit = lambda: assert_unchanged(h.engine, h.before)
    h.on_map_update = lambda: assert_unchanged(h.engine, h.before)

    @contextmanager
    def guard():
        h.calls.append("guard_enter")
        assert_unchanged(h.engine, h.before)
        yield True
        h.calls.append("guard_exit")
        assert h.engine._matcher is h.candidates[0]["matcher"]
        assert len(h.engine._trace) == len(h.before["trace"]) + 1

    extended = np.vstack((h.points, [[15., 4.]]))
    row = h.process(points=extended, guard=guard)
    assert row["accepted"], row
    candidate = h.candidates[0]
    assert h.engine._matcher is candidate["matcher"]
    assert h.engine._anchor_frontier == 600000000 < row["stamp_ns"]
    assert row["matched_against_frontier_ns"] == 600000000
    assert h.engine._keyframe_stamp == row["stamp_ns"]
    assert h.engine._map.last_stamp_ns == row["stamp_ns"]
    assert candidate["inherited_accepted"] == 600000000
    assert candidate["anchor_support_stamp_ns"] == 600000000 < row["stamp_ns"]
    assert candidate["matcher"].anchor_direction_source_stamp_ns == candidate["anchor_support_stamp_ns"]
    np.testing.assert_array_equal(candidate["inherited_correction"], h.before["correction"])
    assert not np.any(np.all(candidate["anchor"] == [15., 4.], axis=1))
    assert not np.any(np.all(candidate["anchor_support"] == [15., 4.], axis=1))
    candidate["match"].assert_called_once()
    candidate["commit"].assert_called_once()
    h.active_match.assert_not_called()
    h.active_commit.assert_not_called()
    assert h.calls == ["prepare", "candidate_match", "guard_enter", "candidate_commit", "map_update", "guard_exit"]
    h.assert_single_map_vote(row, extended)


@pytest.mark.parametrize("reason", ["empty_anchor", "insufficient_anchor_line_support",
    "insufficient_overlap", "insufficient_overlap_after_raw_prior_anchor", "unobservable_geometry",
    "insufficient_final_inliers", "residual_gate", "residual_worsened", "tracking_innovation_gate",
    "total_correction_gate"])
def test_candidate_geometry_rejection_uses_only_previous_actual_active_once(harness, reason):
    h = harness
    h.candidate_reason = reason
    row = h.process()
    assert row["accepted"], row
    assert h.engine._matcher is h.active
    assert h.engine._anchor_frontier == h.before["anchor_frontier"]
    assert h.engine._keyframe_stamp == h.before["keyframe_stamp"]
    assert row["matched_against_frontier_ns"] == h.before["anchor_frontier"] < 600000000
    h.candidates[0]["match"].assert_called_once()
    h.candidates[0]["commit"].assert_not_called()
    h.active_match.assert_called_once()
    h.active_commit.assert_called_once()
    assert h.active.last_accepted_stamp_ns == 900000000
    assert h.active.committed_frames == h.before["committed_frames"] + 1
    attempts = row["metrics"]["reference_attempts"]
    assert [(item["reference"], item["frontier_ns"], item["accepted"]) for item in attempts] == [
        ("candidate", 600000000, False), ("active", 500000000, True)]
    assert attempts[0]["reason"] == reason
    assert attempts[0]["metrics"]["injected_rejection"] == reason
    assert row["metrics"]["selected_reference"] == "active"
    assert row["metrics"]["selected_anchor_frontier_ns"] == 500000000
    h.assert_single_map_vote(row)
    assert h.candidates[0]["matcher"]._pending_result is None


def test_both_geometry_attempts_fail_without_active_map_trace_or_correction_mutation(harness):
    h = harness
    h.candidate_reason, h.active_reason = "unobservable_geometry", "insufficient_overlap"
    row = h.process()
    h.assert_aborted(row)
    assert "insufficient_overlap" in row["reason"]
    h.candidates[0]["match"].assert_called_once()
    h.active_match.assert_called_once()
    h.candidates[0]["commit"].assert_not_called()
    h.active_commit.assert_not_called()
    attempts = row["metrics"]["reference_attempts"]
    assert [item["reason"] for item in attempts] == ["unobservable_geometry", "insufficient_overlap"]
    assert [item["frontier_ns"] for item in attempts] == [600000000, 500000000]


@pytest.mark.parametrize("reason", ["invalid_input", "nonincreasing_mask_stamp",
                                  "future_or_invalid_prior_stamp", "runtime_budget_exceeded",
                                  "insufficient_scan_points"])
def test_invalid_stale_budget_or_scan_failure_cannot_try_another_anchor(harness, reason):
    h = harness
    h.candidate_reason = reason
    row = h.process()
    h.assert_aborted(row)
    assert reason in row["reason"]
    h.active_match.assert_not_called()
    h.active_commit.assert_not_called()
    h.candidates[0]["commit"].assert_not_called()


def test_preparation_budget_is_charged_before_any_match_or_promotion(harness):
    h = harness
    h.prepare_seconds = .081
    row = h.process()
    h.assert_aborted(row)
    assert row["reason"] == "processing_deadline"
    assert row["metrics"]["pre_match_engine_ms"] == pytest.approx(81.)
    assert not row["metrics"].get("reference_attempts")
    assert row["matched_against_frontier_ns"] is None
    h.candidates[0]["match"].assert_not_called()
    h.active_match.assert_not_called()


def test_candidate_consuming_remaining_budget_cannot_try_active(harness):
    h = harness
    h.candidate_reason = "unobservable_geometry"
    h.prepare_seconds, h.candidate_seconds = .04, .041
    row = h.process()
    h.assert_aborted(row)
    assert row["reason"] == "processing_deadline"
    h.active_match.assert_not_called()


def test_two_attempts_share_one_aggregate_budget_and_cannot_commit_late_active_result(harness):
    h = harness
    h.candidate_reason = "unobservable_geometry"
    h.prepare_seconds, h.candidate_seconds, h.active_seconds = .03, .025, .03
    row = h.process()
    h.assert_aborted(row)
    assert row["reason"] == "processing_deadline"
    h.candidates[0]["match"].assert_called_once()
    h.active_match.assert_called_once()
    h.active_commit.assert_not_called()
    h.candidates[0]["commit"].assert_not_called()


@pytest.mark.parametrize("elapsed,accepted", [(.080, True), (.080001, False)])
def test_whole_process_budget_boundary_does_not_gain_a_second_frame_allowance(harness, elapsed, accepted):
    h = harness
    h.prepare_seconds, h.candidate_seconds = .04, elapsed - .04
    row = h.process()
    assert row["accepted"] is accepted
    if accepted:
        h.candidates[0]["commit"].assert_called_once()
        h.assert_single_map_vote(row)
    else:
        h.assert_aborted(row)
        assert row["reason"] == "processing_deadline"
        h.candidates[0]["commit"].assert_not_called()
    h.active_match.assert_not_called()


def test_rejected_guard_does_not_replace_active_or_commit_a_prepared_match(harness):
    h = harness

    @contextmanager
    def guard():
        assert_unchanged(h.engine, h.before)
        yield False
        assert_unchanged(h.engine, h.before)

    row = h.process(guard=guard)
    h.assert_aborted(row)
    assert row["reason"] == "commit_guard_rejected"
    h.candidates[0]["commit"].assert_not_called()
    h.active_match.assert_not_called()


def test_rejected_guard_revokes_previous_active_proposal_after_candidate_fails(harness):
    h = harness
    h.candidate_reason = "unobservable_geometry"

    @contextmanager
    def guard():
        assert_unchanged(h.engine, h.before)
        yield False

    row = h.process(guard=guard)
    h.assert_aborted(row)
    assert row["reason"] == "commit_guard_rejected"
    h.active_match.assert_called_once()
    h.active_commit.assert_not_called()
    h.candidates[0]["commit"].assert_not_called()


def test_time_spent_waiting_for_guard_is_checked_before_commit(harness):
    h = harness

    @contextmanager
    def guard():
        h.clock.now += .081
        yield True

    row = h.process(guard=guard)
    h.assert_aborted(row)
    assert row["reason"] == "processing_deadline"
    h.candidates[0]["commit"].assert_not_called()


def test_map_capacity_failure_cannot_promote_candidate_or_commit_localization(harness, monkeypatch):
    h = harness
    monkeypatch.setattr(h.engine, "_capacity_allows", lambda keys, reliable: False)
    row = h.process()
    h.assert_aborted(row)
    assert row["reason"] == "map_capacity"
    h.candidates[0]["commit"].assert_not_called()
    h.active_match.assert_not_called()


def test_candidate_commit_failure_is_not_a_reason_to_retry_another_anchor(harness):
    h = harness
    h.candidate_commit_ok = False
    row = h.process()
    h.assert_aborted(row)
    assert row["reason"] == "matcher_commit_rejected"
    h.candidates[0]["commit"].assert_called_once()
    h.active_match.assert_not_called()
    h.active_commit.assert_not_called()


def test_previous_active_commit_failure_keeps_old_committed_state_and_map(harness):
    h = harness
    h.candidate_reason = "unobservable_geometry"
    h.active_commit_ok = False
    row = h.process()
    h.assert_aborted(row)
    assert row["reason"] == "matcher_commit_rejected"
    h.candidates[0]["commit"].assert_not_called()
    h.active_commit.assert_called_once()


def test_cached_candidate_is_rebuilt_after_old_active_accepts_new_tracking_state(harness):
    h = harness
    h.candidate_reason = "unobservable_geometry"
    first = h.process()
    assert first["accepted"], first
    assert h.engine._matcher is h.active
    correction = h.active.correction_se2.copy()
    h.candidate_reason = None
    second = h.process(stamp=1200000000)
    assert second["accepted"], second
    assert len(h.candidates) == 2
    assert h.candidates[1]["inherited_accepted"] == 900000000
    np.testing.assert_array_equal(h.candidates[1]["inherited_correction"], correction)
    assert h.engine._matcher is h.candidates[1]["matcher"]
    assert h.engine._anchor_frontier == 900000000
    assert second["matched_against_frontier_ns"] == 900000000


def test_candidate_retained_after_preparation_timeout_is_not_active_and_can_reuse_same_past_index(harness):
    h = harness
    h.prepare_seconds = .081
    first = h.process()
    h.assert_aborted(first)
    assert h.engine._matcher is h.active
    h.prepare_seconds = 0.
    second = h.process(stamp=1200000000)
    assert second["accepted"], second
    assert len(h.candidates) == 1
    assert h.engine._matcher is h.candidates[0]["matcher"]
    assert second["matched_against_frontier_ns"] == 600000000
    h.candidates[0]["match"].assert_called_once()
    h.candidates[0]["commit"].assert_called_once()
    h.assert_single_map_vote(second)


@pytest.mark.parametrize("changed_prior", [[0., .24, 0.], [0., .06, .07]], ids=["translation", "yaw"])
def test_fork_inherits_tracking_limits_correction_and_time_but_not_pending_or_refinement(changed_prior):
    points = scan()
    active = LaneMapLocalizer(points)
    for stamp in (100000000, 200000000, 300000000):
        result = active.match(points, [0., .06, 0.], stamp, prior_stamp_ns=stamp)
        assert result.accepted, result
    assert len(active.refined_points_and_votes()[0]) > 0
    pending = active.match(points, [0., .06, 0.], 400000000, prior_stamp_ns=400000000, commit=False)
    assert pending.accepted
    child = active.fork_with_anchor(points.copy())
    assert child is not active
    assert child.config == active.config
    np.testing.assert_array_equal(child.correction_se2, active.correction_se2)
    assert child.last_accepted_stamp_ns == 300000000
    assert child.last_seen_stamp_ns == 400000000
    assert child._pending_result is None and child._pending_data is None
    assert len(child.refined_points_and_votes()[0]) == 0
    assert child.commit(pending) is False
    row = child.match(points, changed_prior, 500000000, prior_stamp_ns=500000000, commit=False)
    assert not row.accepted and row.reason == "tracking_innovation_gate"
    assert row.metrics["initial_acquisition"] is False
    # This would pass if a keyframe refresh silently reopened initial acquisition.
    fresh = LaneMapLocalizer(points).match(points, changed_prior, 500000000,
                                           prior_stamp_ns=500000000, commit=False)
    assert fresh.accepted, fresh
    active.discard_pending()
    assert active.commit(pending) is False
    assert active.last_accepted_stamp_ns == 300000000


@pytest.mark.parametrize("changed_prior", [[0., .24, 0.], [0., .04, .07]], ids=["translation", "yaw"])
def test_real_keyframe_refresh_cannot_reopen_initial_tracking_gate(changed_prior):
    clock = Clock()
    engine = mapping.TeachMapEngine({"keyframe_refresh_s": .25}, clock=clock)
    points = scan()
    sensitivity = np.full(len(points), .05)
    for index in range(1, 6):
        assert engine.process(points, sensitivity, [0., 0., 0.], index * 100000000)["accepted"]
    assert engine.process(points, sensitivity, [0., .04, 0.], 600000000)["accepted"]
    before = committed_state(engine)
    row = engine.process(points, sensitivity, changed_prior, 900000000)
    assert not row["accepted"], row
    assert "tracking_innovation_gate" in row["reason"], row
    assert_unchanged(engine, before)


def test_no_previous_active_means_rejected_first_candidate_cannot_fall_back_to_odometry(monkeypatch):
    clock = Clock()
    engine = mapping.TeachMapEngine(clock=clock)
    points = scan()
    sensitivity = np.full(len(points), .05)
    for index in range(1, 6):
        assert engine.process(points, sensitivity, [0., 0., 0.], index * 100000000)["accepted"]
    before = committed_state(engine)
    real_class = mapping.LaneMapLocalizer
    created = []

    def factory(anchor, config=None, anchor_support_xy=None, anchor_support_stamp_ns=None):
        candidate = real_class(anchor, config, anchor_support_xy, anchor_support_stamp_ns)
        original = candidate.match

        def reject(*args, **kwargs):
            result = original(*args, **kwargs)
            candidate.discard_pending()
            return replace(result, accepted=False, reason="unobservable_geometry")

        monkeypatch.setattr(candidate, "match", Mock(side_effect=reject))
        monkeypatch.setattr(candidate, "commit", Mock(wraps=candidate.commit))
        created.append(candidate)
        return candidate

    monkeypatch.setattr(mapping, "LaneMapLocalizer", factory)
    row = engine.process(points, sensitivity, [0., 0., 0.], 600000000)
    assert not row["accepted"] and row["reason"] != "bootstrap_odometry"
    assert_unchanged(engine, before)
    assert engine._matcher is None
    assert len(created) == 1
    assert created[0].anchor_direction_source_stamp_ns == 500000000 < row["stamp_ns"]
    created[0].match.assert_called_once()
    created[0].commit.assert_not_called()
    assert created[0]._pending_result is None


def test_first_candidate_guard_rejection_logs_attempted_frontier_without_claiming_active():
    engine = mapping.TeachMapEngine(clock=Clock())
    points = scan()
    sensitivity = np.full(len(points), .05)
    for index in range(1, 6):
        assert engine.process(points, sensitivity, [0., 0., 0.], index * 100000000)["accepted"]
    before = committed_state(engine)

    @contextmanager
    def guard():
        yield False

    row = engine.process(points, sensitivity, [0., 0., 0.], 600000000, commit_guard=guard)
    assert row["reason"] == "commit_guard_rejected"
    assert row["matched_against_frontier_ns"] == 500000000 < row["stamp_ns"]
    assert row["active_anchor_frontier_ns"] is None
    assert row["metrics"]["selected_anchor_frontier_ns"] == 500000000
    attempts = row["metrics"]["reference_attempts"]
    assert [(item["reference"], item["frontier_ns"], item["accepted"]) for item in attempts] == [
        ("candidate", 500000000, True)]
    assert_unchanged(engine, before)
    assert engine._candidate.matcher._pending_result is None


@pytest.mark.parametrize("failure", ["commit", "capacity"])
def test_uncommitted_candidate_logs_attempted_reference_separately_from_existing_active(harness, monkeypatch, failure):
    h = harness
    if failure == "commit":
        h.candidate_commit_ok = False
        expected_reason = "matcher_commit_rejected"
    else:
        monkeypatch.setattr(h.engine, "_capacity_allows", lambda keys, reliable: False)
        expected_reason = "map_capacity"
    row = h.process()
    h.assert_aborted(row)
    assert row["reason"] == expected_reason
    assert row["matched_against_frontier_ns"] == 600000000 < row["stamp_ns"]
    assert row["active_anchor_frontier_ns"] == h.before["anchor_frontier"] == 500000000
    assert row["metrics"]["selected_reference"] == "candidate"
    assert row["metrics"]["selected_anchor_frontier_ns"] == 600000000
    attempts = row["metrics"]["reference_attempts"]
    assert [(item["reference"], item["frontier_ns"], item["accepted"]) for item in attempts] == [
        ("candidate", 600000000, True)]
    h.active_match.assert_not_called()


def test_deadline_after_candidate_rejection_logs_last_attempt_not_untried_active(harness):
    h = harness
    h.candidate_reason = "unobservable_geometry"
    h.prepare_seconds, h.candidate_seconds = .04, .041
    row = h.process()
    h.assert_aborted(row)
    assert row["reason"] == "processing_deadline"
    assert row["matched_against_frontier_ns"] == 600000000 < row["stamp_ns"]
    assert row["active_anchor_frontier_ns"] == 500000000
    assert "selected_anchor_frontier_ns" not in row["metrics"]
    attempts = row["metrics"]["reference_attempts"]
    assert [(item["reference"], item["frontier_ns"], item["accepted"]) for item in attempts] == [
        ("candidate", 600000000, False)]
    h.active_match.assert_not_called()


@pytest.mark.parametrize("failure", ["empty_projection", "preparation_deadline", "nonincreasing_stamp"])
def test_rejection_before_matching_does_not_claim_active_was_matched(harness, failure):
    h = harness
    if failure == "empty_projection":
        row = h.process(points=np.empty((0, 2)))
        expected_reason = "empty_reliable_projection"
    elif failure == "preparation_deadline":
        h.prepare_seconds = .081
        row = h.process()
        expected_reason = "processing_deadline"
    else:
        row = h.process(stamp=600000000)
        expected_reason = "nonincreasing_mask_stamp"
    h.assert_aborted(row)
    assert row["reason"] == expected_reason
    assert row["matched_against_frontier_ns"] is None
    assert row["active_anchor_frontier_ns"] == 500000000
    assert not row["metrics"].get("reference_attempts")
    h.active_match.assert_not_called()
    for candidate in h.candidates:
        candidate["match"].assert_not_called()
