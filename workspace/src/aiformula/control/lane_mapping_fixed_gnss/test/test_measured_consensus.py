"""R9a: preserve measured positions without changing confirmation evidence.

These bounded geometry/transaction tests are not runtime latency or lap proof.
"""
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[2] / "lane_mapping_lya_reference"))

from lane_mapping_fixed_gnss import mapping
from lane_mapping_lya_reference.mapping_core import PreparedCellMeans, SparseConsensusMap
from lane_mapping_lya_reference.mask_localization import LaneMapLocalizer, _voxel_means


def grid(votes=2, capacity=20, ttl=0.):
    return SparseConsensusMap(.1, votes, 1, capacity, capacity, ttl, measured_geometry=True)


def update(value, points, stamp, reliable=True):
    prepared = value.prepare_geometry(np.asarray(points, dtype=float).reshape(-1, 2), stamp)
    return value.update(prepared.keys, prepared.keys if reliable else [], stamp, geometry=prepared)


def state(value):
    points, votes = value.confirmed_points_and_votes()
    return (dict(value.candidate_votes), dict(value.candidate_reliable_votes),
            set(value.confirmed_keys), dict(value._cell_means), value.last_stamp_ns,
            value._geometry_generation, points.tobytes(), votes.tobytes())


def test_old_key_only_map_keeps_integer_centers_and_api():
    value = SparseConsensusMap(.1, 2)
    keys = value.points_to_unique_keys([[.031, .019]])
    value.update(keys, stamp_ns=1)
    value.update(keys, stamp_ns=2)
    points, votes = value.confirmed_points_and_votes()
    np.testing.assert_array_equal(points, [[0., 0.]])
    np.testing.assert_array_equal(votes, [2])
    assert not value.measured_geometry and not value._cell_means
    with pytest.raises(ValueError):
        value.prepare_geometry(np.asarray([[0., 0.]]), 3)


def test_pixels_average_within_frame_then_frames_have_equal_weight():
    value = grid()
    update(value, [[.01, .02], [.03, .04]], 1)
    update(value, np.tile([[-.02, -.01]], (101, 1)), 2)
    points, votes = value.confirmed_points_and_votes()
    np.testing.assert_allclose(points, [[0., .01]], rtol=0, atol=3e-17)
    np.testing.assert_array_equal(votes, [2])
    assert len(value._cell_means) == len(value.confirmed_keys) == 1
    update(value, [[.03, .04]], 3)
    np.testing.assert_allclose(value.confirmed_points_and_votes()[0], [[.01, .02]], atol=3e-17)


def test_reliable_and_total_votes_are_unchanged_and_geometry_waits_for_confirmation():
    value = grid(votes=3)
    for stamp in (1, 2):
        update(value, [[.031, .021]], stamp, reliable=False)
        assert not len(value.confirmed_points_and_votes()[0])
    update(value, [[.01, .02]], 3)
    assert next(iter(value.candidate_votes.values())) == 3
    assert next(iter(value.candidate_reliable_votes.values())) == 1
    np.testing.assert_allclose(value.confirmed_points_and_votes()[0], [[.024, .020666666666666667]])


def test_preparation_is_immutable_and_does_not_vote_or_expire():
    value = grid(ttl=1.)
    update(value, [[.03, .02]], 1)
    before = state(value)
    source = np.asarray([[.02, .03]])
    prepared = value.prepare_geometry(source, 2_000_000_001)
    source[:] = 999.
    assert state(value) == before
    for array in (prepared.keys, prepared.means, prepared.updated_means):
        with pytest.raises(ValueError):
            array.flags.writeable = True
    np.testing.assert_allclose(prepared.means, [[.02, .03]])
    np.testing.assert_allclose(prepared.updated_means, [[.02, .03]])


@pytest.mark.parametrize("change", ["owner", "generation", "stamp", "keys"])
def test_prepared_token_mismatch_is_rejected_before_any_mutation(change):
    value = grid()
    prepared = value.prepare_geometry(np.asarray([[.02, .03]]), 1)
    before = state(value)
    if change == "owner":
        prepared = replace(prepared, owner=grid())
    elif change == "generation":
        prepared = replace(prepared, base_generation=1)
    elif change == "stamp":
        prepared = replace(prepared, source_stamp_ns=2)
    keys = value.points_to_unique_keys([[.2, 0.]]) if change == "keys" else prepared.keys
    with pytest.raises(ValueError):
        value.update(keys, keys, stamp_ns=1, geometry=prepared)
    assert state(value) == before


@pytest.mark.parametrize("intervention", ["update", "clear", "expire"])
def test_every_geometry_mutation_invalidates_old_token_even_at_same_stamp(intervention):
    value = grid()
    prepared = value.prepare_geometry(np.asarray([[.02, .03]]), 1)
    if intervention == "update":
        value.update(prepared.keys, prepared.keys, 1, geometry=prepared)
    elif intervention == "clear":
        value.clear()
    else:
        value.expire_candidates(1)
    before = state(value)
    with pytest.raises(ValueError):
        value.update(prepared.keys, prepared.keys, 1, geometry=prepared)
    assert state(value) == before


def test_measured_mode_cannot_fall_back_to_unmeasured_keys():
    value = grid()
    keys = value.points_to_unique_keys([[.02, .03]])
    before = state(value)
    with pytest.raises(ValueError):
        value.update(keys, keys, 1)
    assert state(value) == before


def test_ttl_at_exact_cutoff_deletes_old_mean_and_restarts_frame_count():
    value = grid(ttl=1.)
    update(value, [[.04, .04]], 1)
    update(value, [[-.02, -.02]], 1_000_000_001)
    assert value.expired_candidate_cells == 1
    assert next(iter(value.candidate_votes.values())) == 1
    assert not value.confirmed_keys
    update(value, [[0., 0.]], 1_000_000_002)
    np.testing.assert_allclose(value.confirmed_points_and_votes()[0], [[-.01, -.01]])


def test_expiry_removes_geometry_but_never_removes_confirmed_means():
    value = grid(ttl=1.)
    update(value, [[.02, .03], [.21, .22]], 1)
    update(value, [[.02, .03]], 2)
    value.expire_candidates(2_000_000_002)
    assert len(value._cell_means) == len(value.candidate_votes) == len(value.confirmed_keys) == 1
    value.clear()
    assert not value._cell_means and not value.candidate_votes


def test_capacity_rejected_cells_never_retain_geometry():
    value = grid(votes=1, capacity=1)
    update(value, [[.01, .02], [.21, .22]], 1)
    assert len(value._cell_means) == len(value.candidate_votes) == 1
    assert value.capacity_rejected_cells == 1


@pytest.mark.parametrize("points", [
    [[np.nan, 0.]], [[np.inf, 0.]], [[1e308, 0.]], [[-1e308, 0.]],
    [[1., 2., 3.]], np.zeros((200001, 2)),
])
def test_nonfinite_overflow_shape_or_capacity_rejected_before_mutation(points):
    value = grid()
    before = state(value)
    with pytest.raises(ValueError):
        value.prepare_geometry(np.asarray(points), 1)
    assert state(value) == before


@pytest.mark.parametrize("capacity", [0, 200001])
def test_measured_storage_must_have_explicit_hard_capacity(capacity):
    with pytest.raises(ValueError):
        grid(capacity=capacity)


def test_prepared_means_keep_original_key_at_rounding_boundary():
    value = grid(votes=1)
    # The validated cell owns the closed boundary even if rounding this derived
    # mean again would choose its neighbour. No second quantization is allowed.
    raw = value.prepare_geometry(np.asarray([[.049, 0.]]), 1)
    prepared = replace(raw, means=np.asarray([[.05, 0.]]), updated_means=np.asarray([[.05, 0.]]))
    assert not np.array_equal(value.points_to_unique_keys(prepared.means), prepared.keys)
    value.update(prepared.keys, prepared.keys, 1, geometry=prepared)
    np.testing.assert_array_equal(value.confirmed_points_and_votes()[0], [[.05, 0.]])
    with pytest.raises(ValueError):
        replace(prepared, means=np.asarray([[.051, 0.]]))


def scan(offset=.01):
    x = np.arange(65) * .1 + 1. + offset
    return np.vstack((np.column_stack((x, np.full(len(x), 1. + offset))),
                      np.column_stack((x, np.full(len(x), -1. + offset)))))


def engine(config=None):
    return mapping.TeachMapEngine(dict(max_processing_ms=2000., **(config or {})),
                                  matcher_config={"runtime_budget_ms": 2000.})


def bootstrap(value):
    points = scan()
    for i in range(5):
        row = value.process(points, np.full(len(points), .05), [0., 0., 0.], (i+1)*100000000)
        assert row["accepted"] and row["reason"] == "bootstrap_odometry"
    return points


def test_engine_prepares_only_past_measured_anchor_and_one_mean_per_cell(monkeypatch):
    value = engine()
    points = bootstrap(value)
    past, votes = value.points_and_votes()
    np.testing.assert_array_equal(votes, np.full(len(points), 5))
    assert len(past) == len(points) == 130
    assert np.any(np.abs(past / .1 - np.rint(past / .1)) > .01)
    captured = []
    original = mapping.LaneMapLocalizer
    class RecordingMatcher(original):
        def __init__(self, anchor, *args, **kwargs):
            captured.append(anchor.copy())
            super().__init__(anchor, *args, **kwargs)
    monkeypatch.setattr(mapping, "LaneMapLocalizer", RecordingMatcher)
    row = value.process(scan(.02), np.full(len(points), .05), [0., 0., 0.], 600000000)
    assert row["accepted"] and row["matched_against_frontier_ns"] == 500000000
    # Selection may sort the cell representatives; it cannot incorporate the
    # current shifted observations into its own reference.
    assert {tuple(p) for p in captured[0]} == {tuple(p) for p in past}
    assert not np.array_equal(value.points_and_votes()[0], past)


def test_bootstrap_100_uses_confirmed_cells_not_post_voxel_geometry():
    value = engine()
    # 104 distinct confirmation cells, but many measured points merge under
    # the unchanged 0.08 m localization voxel; never duplicate them to reach100.
    x = np.arange(52) * .1
    x += np.where(np.arange(52) % 2 == 0, .049, -.049)
    points = np.vstack((np.column_stack((x, np.ones(len(x)))),
                        np.column_stack((x, -np.ones(len(x))))))
    for i in range(5):
        assert value.process(points, np.full(len(points), .05), [0., 0., 0.], (i+1)*100000000)["accepted"]
    assert len(value._map.confirmed_keys) == 104
    prepared = value._prepare_matcher(np.zeros(3), 600000000)
    assert prepared is not None
    assert len(prepared.matcher._anchor) < 100
    assert len(prepared.matcher._anchor) == len(_voxel_means(value.points_and_votes()[0], .08))


@pytest.mark.parametrize("failure", ["match", "guard", "deadline", "commit", "capacity"])
def test_failed_engine_transaction_never_changes_means_votes_or_trace(monkeypatch, failure):
    value = engine()
    points = bootstrap(value)
    before, trace = state(value._map), np.asarray(value._trace).tobytes()
    if failure == "match":
        monkeypatch.setattr(LaneMapLocalizer, "match", lambda *a, **k: SimpleNamespace(
            accepted=False, reason="residual_gate", metrics={}))
    elif failure == "commit":
        monkeypatch.setattr(LaneMapLocalizer, "commit", lambda *a: False)
    elif failure == "capacity":
        monkeypatch.setattr(value, "_capacity_allows", lambda *a: False)
    @contextmanager
    def guard():
        if failure == "deadline":
            value._clock = lambda: 100000000000.
        yield failure != "guard"
    row = value.process(scan(.02), np.full(len(points), .05), [0., 0., 0.],
                        600000000, commit_guard=guard)
    assert not row["accepted"]
    assert state(value._map) == before
    assert np.asarray(value._trace).tobytes() == trace


def test_frozen_snapshot_and_repeat_keep_exact_measured_geometry():
    value = engine()
    points = bootstrap(value)
    assert value.process(points, np.full(len(points), .05), [0., 0., 0.], 600000000)["accepted"]
    snapshot = value.freeze()
    assert snapshot.stats["consensus_coordinate_semantics"] == "measured_frame_equal_mean"
    assert value.freeze() is snapshot
    with pytest.raises(ValueError):
        snapshot.consensus_xy.flags.writeable = True
    before = snapshot.consensus_xy.tobytes()
    copy, _ = value.points_and_votes()
    copy[:] = 999.
    matcher = value.repeat_localizer()
    assert matcher.matches_anchor_source(snapshot.consensus_xy)
    assert matcher.capture_tracking_state() == value.capture_repeat_state()
    assert snapshot.consensus_xy.tobytes() == before
    with pytest.raises(ValueError):
        value.process(points, np.full(len(points), .05), [0., 0., 0.], 700000000)
