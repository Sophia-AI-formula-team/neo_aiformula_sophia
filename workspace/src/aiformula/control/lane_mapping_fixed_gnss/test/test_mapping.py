"""No ROS: teach matching must use old map and commit under a generation fence."""

from contextlib import contextmanager
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[2] / "lane_mapping_lya_reference"))

from lane_mapping_fixed_gnss import mapping


def scan():
    x = np.linspace(1., 7., 241)
    return np.vstack((np.column_stack((x, np.ones(len(x)))),
                      np.column_stack((x, -np.ones(len(x))))))


class FakeMatcher:
    created = []
    accept = True

    def __init__(self, anchor, config=None):
        self.anchor = anchor.copy()
        self.commits = 0
        self.created.append(self)

    def match(self, points, prior, stamp, prior_stamp_ns=None, commit=True):
        assert commit is False
        return SimpleNamespace(accepted=self.accept, reason="no_overlap" if not self.accept else "matched",
                               pose_xyyaw=np.asarray(prior).copy(), metrics={})

    def commit(self, result):
        self.commits += 1
        return True


@pytest.fixture
def fake(monkeypatch):
    FakeMatcher.created, FakeMatcher.accept = [], True
    monkeypatch.setattr(mapping, "LaneMapLocalizer", FakeMatcher)
    return FakeMatcher


def bootstrap(engine):
    points = scan()
    for i in range(5):
        row = engine.process(points, np.full(len(points), .05), [0., 0., 0.], (i + 1) * 100000000)
        assert row["accepted"]
        assert row["reason"] == "bootstrap_odometry"
    return points


def test_first_match_is_against_only_previous_established_consensus(fake):
    engine = mapping.TeachMapEngine()
    points = bootstrap(engine)
    past, _ = engine.points_and_votes()
    extended = np.vstack((points, [[15., 4.]]))
    row = engine.process(extended, np.full(len(extended), .05), [0., 0., 0.], 600000000)
    assert row["accepted"] and row["matched_against_frontier_ns"] == 500000000
    assert len(fake.created) == 1
    assert np.max(fake.created[0].anchor[:, 0]) <= np.max(past[:, 0])
    assert not np.any(np.all(fake.created[0].anchor == [15., 4.], axis=1))
    assert fake.created[0].commits == 1


def test_rejected_matching_never_fuses_or_appends_trace(fake):
    engine = mapping.TeachMapEngine()
    points = bootstrap(engine)
    before, votes = engine.points_and_votes()
    fake.accept = False
    row = engine.process(points, np.full(len(points), .05), [0., 0., 0.], 600000000)
    assert not row["accepted"] and row["reason"] == "match_no_overlap"
    after, after_votes = engine.points_and_votes()
    assert np.array_equal(before, after) and np.array_equal(votes, after_votes)
    assert row["trace_points"] == 5 and fake.created[0].commits == 0


def test_guard_rejection_has_no_map_trace_or_matcher_commit(fake):
    engine = mapping.TeachMapEngine()
    points = bootstrap(engine)
    before, votes = engine.points_and_votes()
    @contextmanager
    def guard():
        yield False
    row = engine.process(points, np.full(len(points), .05), [0., 0., 0.], 600000000, commit_guard=guard)
    assert row["reason"] == "commit_guard_rejected"
    assert row["trace_points"] == 5 and fake.created[0].commits == 0
    after, after_votes = engine.points_and_votes()
    assert np.array_equal(before, after) and np.array_equal(votes, after_votes)


def test_guard_covers_persistent_mutation(fake):
    engine = mapping.TeachMapEngine()
    points = bootstrap(engine)
    events = []
    @contextmanager
    def guard():
        events.append(("enter", len(engine._trace)))
        yield True
        events.append(("exit", len(engine._trace)))
    row = engine.process(points, np.full(len(points), .05), [0., 0., 0.], 600000000, commit_guard=guard)
    assert row["accepted"]
    assert events == [("enter", 5), ("exit", 6)]


def test_freeze_is_immutable_and_later_process_forbidden(fake):
    engine = mapping.TeachMapEngine()
    points = bootstrap(engine)
    snapshot = engine.freeze()
    with pytest.raises(ValueError):
        snapshot.trace[0, 0] = 100
    with pytest.raises(ValueError):
        snapshot.consensus_xy.setflags(write=True)
    exposed, _ = engine.consensus_points_and_votes()
    exposed[:] = 9
    assert not np.all(engine.freeze().consensus_xy == 9)
    with pytest.raises(ValueError, match="frozen"):
        engine.process(points, np.full(len(points), .05), [0., 0., 0.], 600000000)
    independent = engine.repeat_localizer()
    independent.anchor[:] = 4
    assert not np.all(snapshot.consensus_xy == 4)


def test_bootstrap_cannot_silently_continue_without_anchor():
    engine = mapping.TeachMapEngine({"bootstrap_max_frames": 2})
    points = np.array([[2., 1.]])
    assert engine.process(points, [.05], [0., 0., 0.], 100000000)["accepted"]
    assert engine.process(points, [.05], [0., 0., 0.], 200000000)["accepted"]
    assert engine.process(points, [.05], [0., 0., 0.], 300000000)["reason"] == "bootstrap_anchor_not_established"
    with pytest.raises(ValueError, match="invalid"):
        engine.freeze()


def test_map_capacity_is_checked_before_mutating():
    engine = mapping.TeachMapEngine({"max_candidate_cells": 1})
    row = engine.process([[2., 1.], [3., 1.]], [.05, .05], [0., 0., 0.], 100000000)
    assert row["reason"] == "map_capacity"
    assert row["trace_points"] == 0 and not engine._map.candidate_votes


def test_nonincreasing_source_latches_invalid():
    engine = mapping.TeachMapEngine()
    engine.process([[2., 1.]], [.05], [0., 0., 0.], 100000000)
    assert engine.process([[2., 1.]], [.05], [0., 0., 0.], 100000000)["reason"] == "nonincreasing_mask_stamp"


def test_processing_deadline_prevents_bootstrap_geometry_commit():
    ticks = iter([0., 1., 1.])
    engine = mapping.TeachMapEngine(clock=lambda: next(ticks))
    row = engine.process([[2., 1.]], [.05], [0., 0., 0.], 100000000)
    assert row["reason"] == "processing_deadline" and row["trace_points"] == 0
    assert not engine._map.candidate_votes


def test_expensive_keyframe_is_reused_after_deadline_rejection(fake):
    engine = mapping.TeachMapEngine()
    points = bootstrap(engine)
    ticks = iter([0., 1., 1.])
    engine._clock = lambda: next(ticks)
    row = engine.process(points, np.full(len(points), .05), [0., 0., 0.], 600000000)
    assert row["reason"] == "processing_deadline"
    assert len(fake.created) == 1 and fake.created[0].commits == 0
    engine._clock = lambda: 0.
    # Even after the periodic refresh interval, unchanged geometry does not
    # trigger another expensive index; the valid future frame can recover.
    row = engine.process(points, np.full(len(points), .05), [0., 0., 0.], 1800000000)
    assert row["accepted"]
    assert len(fake.created) == 1 and fake.created[0].commits == 1


def test_actual_matcher_recovers_lateral_offset_against_old_map():
    engine = mapping.TeachMapEngine()
    points = bootstrap(engine)
    row = engine.process(points, np.full(len(points), .05), [0., .04, 0.], 600000000)
    assert row["accepted"], row
    assert abs(row["pose_xyyaw"][1]) < .015
    assert row["matched_against_frontier_ns"] == 500000000


def test_extreme_projected_coordinates_are_rejected_before_grid_conversion():
    engine = mapping.TeachMapEngine()
    with pytest.raises(ValueError, match="invalid"):
        engine.process([[1e100, 0.]], [.05], [0., 0., 0.], 100000000)
    assert not engine._map.candidate_votes


def test_duration_limit_latches_before_fusion():
    engine = mapping.TeachMapEngine({"max_duration_s": .1})
    engine.process([[2., 1.]], [.05], [0., 0., 0.], 100000000)
    row = engine.process([[2., 1.]], [.05], [0., 0., 0.], 300000000)
    assert row["reason"] == "teach_duration_capacity" and row["trace_points"] == 1


@pytest.mark.parametrize("setting", [{"max_duration_s": float("inf")}, {"foo": 1},
                                      {"bootstrap_max_frames": 1.2}, {"reliable_sensitivity": 1.}])
def test_invalid_settings_fail(setting):
    with pytest.raises(ValueError):
        mapping.TeachMapEngine(setting)
