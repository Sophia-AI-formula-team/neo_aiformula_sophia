"""Physical paint observations, guarded ownership, and unchanged pose gates."""
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[2] / "lane_mapping_lya_reference"))

from lane_mapping_fixed_gnss.mapping import TeachMapEngine
from lane_mapping_lya_reference.mapping_core import GroundLookup
from lane_mapping_lya_reference.paint_geometry import extract_boundary_observation
from lane_mapping_lya_reference.boundary_localization import BoundaryLaneLocalizer


def observation(stamp):
    mask = np.zeros((100, 160), dtype=np.uint8)
    mask[27:33, 20:145] = 255
    mask[67:73, 20:145] = 255
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = 2.
    camera = np.array([[25.,0.,0.],[0.,25.,50.],[0.,0.,1.]])
    lookup = GroundLookup(160, 100, camera, transform, .5)
    points, sensitivity, _ = lookup.project_mask_with_sensitivity(mask, 127)
    boundary = extract_boundary_observation(mask, lookup, [0.,0.,1.], stamp)
    return points, sensitivity, boundary


def engine():
    # Deterministic geometry/transaction unit budgets, NOT runtime evidence.
    return TeachMapEngine({"max_processing_ms":2000.}, {"runtime_budget_ms":2000.})


def step(target, stamp, prior=(0.,0.,0.), **kwargs):
    points, sensitivity, boundary = observation(stamp)
    return target.process(points, sensitivity, prior, stamp,
                          motion_source_stamp_ns=stamp-10000000,
                          boundary_observation=boundary, **kwargs)


def bootstrap(target):
    for k in range(5):
        result = step(target, 1000000000+k*100000000)
        assert result["accepted"], result
    return target._boundary_map.snapshot()


@contextmanager
def denied():
    yield False


def test_bootstrap_boundaries_have_same_causal_frontier_without_invented_visual_success():
    target = engine()
    reference = bootstrap(target)
    assert len(reference.points_xy) >= 100
    assert np.all(reference.votes == 5)
    assert reference.source_frontier_ns == target._map.last_stamp_ns
    assert target._matcher is None
    result = step(target, 1500000000)
    assert result["accepted"], result
    assert isinstance(target._matcher, BoundaryLaneLocalizer)
    assert result["matched_against_frontier_ns"] < result["stamp_ns"]
    assert result["metrics"]["bootstrap_motion_prior_used"]
    assert np.linalg.norm(result["pose_xyyaw"]) < .01


@pytest.mark.parametrize("phase", ["empty", "bootstrap", "tracking"])
def test_guard_rejection_never_publishes_boundary_votes_or_frontier(phase):
    target = engine()
    if phase != "empty":
        bootstrap(target)
    if phase == "tracking":
        assert step(target,1500000000)["accepted"]
    old = target._boundary_map
    reference = None if old is None else old.snapshot()
    trace = np.asarray(target._trace).copy()
    map_stamp = target._map.last_stamp_ns
    result = step(target,1600000000,commit_guard=denied)
    assert not result["accepted"]
    assert result["reason"] == "commit_guard_rejected"
    assert target._map.last_stamp_ns == map_stamp
    np.testing.assert_array_equal(target._trace,trace)
    assert target._boundary_map is old
    if old is not None:
        current = old.snapshot()
        assert current.source_frontier_ns == reference.source_frontier_ns
        for name in ("points_xy","normals","votes","reliable_votes","record_ids",
                     "support_segments","segment_owner"):
            np.testing.assert_array_equal(getattr(current,name),getattr(reference,name))


def test_mode_cannot_disappear_or_be_enabled_midrun():
    target = engine()
    bootstrap(target)
    points, sensitivity, obs = observation(1600000000)
    before = target._map.last_stamp_ns
    missing = target.process(points,sensitivity,[0.,0.,0.],1600000000,
                             motion_source_stamp_ns=1590000000)
    assert missing["reason"] == "changed_feature_representation"
    assert target._map.last_stamp_ns == before
    legacy = engine()
    assert legacy.process(points,sensitivity,[0.,0.,0.],1600000000)["accepted"]
    changed = step(legacy,1700000000)
    assert changed["reason"] == "changed_feature_representation"


def test_wrong_boundary_source_cannot_enter_bootstrap():
    target = engine()
    points,sensitivity,obs = observation(1000000000)
    result = target.process(points,sensitivity,[0.,0.,0.],1100000000,
        boundary_observation=obs, motion_source_stamp_ns=1090000000)
    assert result["reason"] == "invalid_boundary_observation"
    assert target._boundary_map is None and not target._trace


def test_repeat_handoff_keeps_boundary_identity_and_committed_motion():
    target = engine()
    bootstrap(target)
    assert step(target,1500000000)["accepted"]
    snapshot = target.freeze()
    repeat = target.repeat_localizer()
    assert isinstance(repeat,BoundaryLaneLocalizer)
    assert repeat.matches_anchor_source(snapshot.boundary_reference)
    assert not repeat.matches_anchor_source(snapshot.consensus_xy)
    assert repeat.capture_tracking_state().motion_information is target._matcher._motion_information
    altered = replace(snapshot.boundary_reference,
                      reliable_votes=snapshot.boundary_reference.reliable_votes-1,
                      edge_reliable_votes=snapshot.boundary_reference.edge_reliable_votes-1)
    assert not repeat.matches_anchor_source(altered)
    points,sensitivity,obs = observation(1600000000)
    missing = repeat.match(points[sensitivity <= .1],[0.,0.,0.],1600000000,
        scan_support_xy=points, motion_source_stamp_ns=1590000000)
    assert missing.reason == "missing_boundary_observation"
    assert repeat.last_accepted_stamp_ns == 1500000000
