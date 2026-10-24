"""Revoke real boundary proposals when preparation/admission raises.

These failures are injected BEFORE the first persistent commit. They do not
claim exception rollback after one of the map/matcher stores has committed.
The generous unit budgets isolate ownership, not runtime deadline acceptance.
"""
from contextlib import contextmanager
from dataclasses import fields, is_dataclass, replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[2] / "lane_mapping_lya_reference"))

from lane_mapping_fixed_gnss.mapping import TeachMapEngine
from lane_mapping_lya_reference.boundary_localization import BoundaryLaneLocalizer
from lane_mapping_lya_reference.mapping_core import GroundLookup
from lane_mapping_lya_reference.paint_geometry import extract_boundary_observation


class InjectedBeforeCommitError(RuntimeError):
    pass


def _value(value):
    """Capture exact numeric contents without retaining mutable array views."""
    if isinstance(value, np.ndarray):
        return (value.dtype.str, value.shape, value.tobytes())
    if is_dataclass(value):
        return tuple((field.name, _value(getattr(value, field.name)))
                     for field in fields(value))
    if isinstance(value, dict):
        return tuple((key, _value(item)) for key, item in sorted(value.items()))
    if isinstance(value, (tuple, list)):
        return tuple(_value(item) for item in value)
    if isinstance(value, set):
        return tuple(sorted(value))
    return value


@pytest.fixture
def projected_paint():
    # Two actual filled strips, extracted with the production projector and
    # contour/model geometry. No hand-forged accepted MatchResult or normals.
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


def _step(engine, projected, stamp, **kwargs):
    points, sensitivity, observation = projected
    return engine.process(points, sensitivity, [0., 0., 0.], stamp,
                          motion_source_stamp_ns=stamp - 10000000,
                          boundary_observation=replace(observation, source_stamp_ns=stamp),
                          **kwargs)


def _matcher_committed_state(matcher):
    # last_seen and current-image memoization may advance on rejected work;
    # committed correction, motion information and refinement may not.
    return _value((matcher.correction_se2, matcher.last_accepted_stamp_ns,
                   matcher.committed_frames, matcher._motion_information,
                   matcher.refinement_metrics(), matcher.refined_points_and_votes(),
                   matcher._refinement.candidate_votes,
                   matcher._refinement.confirmed_keys, matcher._refined_votes))


def _engine_committed_state(engine):
    filled, boundary = engine._map, engine._boundary_map
    return _value({
        "filled": (filled.last_stamp_ns, filled._geometry_generation,
                   filled.candidate_votes, filled.candidate_reliable_votes,
                   filled.confirmed_keys, filled._cell_means,
                   filled.capacity_rejected_cells, filled.confirmation_capacity_rejections),
        # Include unconfirmed nodes/edges, not only the visible snapshot. A
        # rejected observation must not gain a hidden vote towards confirmation.
        "boundary": (boundary.source_frontier_ns, boundary._generation,
                     boundary._next_record_id, boundary._segment_count,
                     boundary._confirmed_count, boundary._records, boundary._edges,
                     boundary.snapshot()),
        "trace": engine._trace,
        "anchor_frontier": engine._anchor_frontier,
        "keyframe": (engine._keyframe_stamp, engine._keyframe_pose),
        "motion_frontier": engine._last_motion_source_stamp,
        "bootstrap_motion": engine._bootstrap_motion_information,
        "direction": (engine._past_direction_support_stamp_ns, engine._past_direction_support),
        "accepted": engine.counts["accepted"],
    })


@pytest.mark.parametrize("phase", ["first_candidate", "active", "refresh_candidate"])
@pytest.mark.parametrize("failure", ["guard_enter", "prepare_geometry"])
def test_precommit_exception_revokes_boundary_proposal_without_publishing_evidence(
        projected_paint, monkeypatch, phase, failure):
    engine = TeachMapEngine({"max_processing_ms": 2000.}, {"runtime_budget_ms": 2000.})
    for step in range(5):
        result = _step(engine, projected_paint, 1000000000 + step * 100000000)
        assert result["accepted"], result
    assert engine._boundary_map.confirmed_count >= 100
    assert engine._matcher is None
    if phase != "first_candidate":
        result = _step(engine, projected_paint, 1500000000)
        assert result["accepted"], result
        assert isinstance(engine._matcher, BoundaryLaneLocalizer)
    stamp = 2600000000 if phase == "refresh_candidate" else 1600000000
    active_before = engine._matcher
    active_state_before = (None if active_before is None else
                           _matcher_committed_state(active_before))
    boundary_before = engine._boundary_map
    state_before = _engine_committed_state(engine)
    proposals = []
    real_match = BoundaryLaneLocalizer.match

    def observe_real_match(matcher, *args, **kwargs):
        committed_before = _matcher_committed_state(matcher)
        result = real_match(matcher, *args, **kwargs)
        assert result.accepted, result.metrics
        assert matcher._pending_result is result
        assert matcher._pending_data is not None
        assert matcher._pending_motion_information is not None
        proposals.append((matcher, result, committed_before))
        return result

    @contextmanager
    def guarded_admission():
        assert len(proposals) == 1, "the failure must occur after a real accepted fit"
        if failure == "guard_enter":
            raise InjectedBeforeCommitError("guard entry before commit")
        pytest.fail("a failed geometry preparation must not reach admission")
        yield True  # Make this a context manager; the body is never admitted.

    def fail_geometry_preparation(*args, **kwargs):
        assert len(proposals) == 1, "the failure must occur after a real accepted fit"
        raise InjectedBeforeCommitError("geometry preparation before commit")

    monkeypatch.setattr(BoundaryLaneLocalizer, "match", observe_real_match)
    if failure == "prepare_geometry":
        monkeypatch.setattr(engine._map, "prepare_geometry", fail_geometry_preparation)
    with pytest.raises(InjectedBeforeCommitError, match="before commit"):
        _step(engine, projected_paint, stamp, commit_guard=guarded_admission)

    assert len(proposals) == 1
    selected, proposal, selected_before = proposals[0]
    assert (selected is active_before) == (phase == "active")
    assert engine._matcher is active_before
    assert engine._boundary_map is boundary_before
    assert _engine_committed_state(engine) == state_before
    assert _matcher_committed_state(selected) == selected_before
    if active_before is not None:
        assert _matcher_committed_state(active_before) == active_state_before

    # Check delayed submission FIRST: it must be unable to acquire a stale
    # proposal, not merely appear clean when the next match resets pending.
    assert not selected.commit(proposal)
    assert selected._pending_result is None
    assert selected._pending_data is None
    assert selected._pending_motion_information is None
    assert _matcher_committed_state(selected) == selected_before
    assert _engine_committed_state(engine) == state_before
