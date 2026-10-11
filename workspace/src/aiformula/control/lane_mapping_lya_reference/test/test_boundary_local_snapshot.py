"""Local snapshots must equal full snapshot/filter/stable-sort/subset bytes.

All records and edges are created through real prepare/commit transactions.
Private indexes are inspected, never populated or modified by these tests.
No runtime budget, image extractor, native extension or pose solver is involved.
"""
from dataclasses import replace

import numpy as np
import pytest

from lane_mapping_lya_reference.paint_geometry import BoundaryConsensus, BoundaryObservation


REFERENCE_ARRAYS = (
    'points_xy', 'normals', 'support_segments', 'segment_owner', 'votes',
    'reliable_votes', 'record_ids', 'directed_edges', 'edge_frame_votes',
    'edge_reliable_votes', 'edge_last_stamp_ns', 'full_in_degree', 'full_out_degree',
)


def observation(points=(), stamp=1, order=None, *, normals=None, closed=False,
                reliable=True):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    n = len(points)
    normals = (np.tile([0., 1.], (n, 1)) if normals is None
               else np.asarray(normals, dtype=np.float64).reshape(n, 2))
    segments = np.stack((points + [-.002, 0.], points + [.002, 0.]), axis=1)
    previous, following = np.full(n, -1, dtype=np.int64), np.full(n, -1, dtype=np.int64)
    contour_ids = np.arange(1, n + 1, dtype=np.int64)
    if order is not None:
        order = list(order)
        contour_ids[order] = 0
        for first, second in zip(order[:-1], order[1:]):
            following[first], previous[second] = second, first
        if closed and order:
            following[order[-1]], previous[order[0]] = order[0], order[-1]
    true = np.ones(n, dtype=bool)
    white = np.column_stack((np.arange(n, dtype=np.int64), np.zeros(n, dtype=np.int64)))
    black = white + np.array([0, 1], dtype=np.int64)
    return BoundaryObservation(
        points=points, normals=normals, segments=segments, sensitivity=np.full(n, .02),
        metric_valid=true, stable=true, normal_valid=true, valid=true,
        reliable=true & reliable, source_stamp_ns=stamp, pixel_segments=segments,
        white_pixels=white, black_pixels=black, previous=previous, next=following,
        contour_ids=contour_ids, reason_codes=np.zeros(n, dtype=np.int64),
        normal_support_count=np.full(n, 6, dtype=np.int64), normal_radius=np.full(n, .45))


def assert_cell_index(consensus):
    expected = {}
    for record_id, record in consensus._records.items():
        expected.setdefault(record.cell, {})[record_id] = None
    assert set(consensus._cell_records) == set(expected)
    for cell, members in expected.items():
        assert isinstance(consensus._cell_records[cell], dict)
        assert list(consensus._cell_records[cell]) == list(members)
        assert all(value is None for value in consensus._cell_records[cell].values())
    assert sum(map(len, consensus._cell_records.values())) == len(consensus._records)


def commit(consensus, obs):
    token = consensus.prepare(obs)
    assert consensus.commit(token)
    assert_cell_index(consensus)
    return token


def confirmed(points, order=None, *, closed=False, normals=None, config=None):
    consensus = BoundaryConsensus(config)
    for stamp in range(1, 6):
        commit(consensus, observation(points, stamp, order, closed=closed, normals=normals))
    return consensus


def full_oracle(consensus, center, radius, maximum_points):
    full = consensus.snapshot()
    distance = np.sum((full.points_xy - np.asarray(center, dtype=np.float64)) ** 2, axis=1)
    available = np.flatnonzero(distance <= radius ** 2)
    selected = available[np.argsort(distance[available], kind='stable')[:maximum_points]]
    return full.subset(selected)


def assert_reference_bytes(actual, expected):
    assert len(REFERENCE_ARRAYS) == 13
    assert actual.source_frontier_ns == expected.source_frontier_ns
    assert actual.representation == expected.representation
    assert actual.topology_representation == expected.topology_representation
    for name in REFERENCE_ARRAYS:
        got, wanted = getattr(actual, name), getattr(expected, name)
        assert isinstance(got, np.ndarray), name
        assert got.dtype == wanted.dtype, name
        assert got.shape == wanted.shape, name
        assert got.tobytes() == wanted.tobytes(), name
        assert not got.flags.writeable, name


def check_near(consensus, monkeypatch, center=(0., 0.), radius=100., maximum_points=100):
    expected = full_oracle(consensus, center, radius, maximum_points)
    before = (consensus._generation, consensus.last_stamp_ns,
              tuple((key, tuple(value.items())) for key, value in consensus._cell_records.items()))
    def forbidden_full_snapshot(*args, **kwargs):
        raise AssertionError('snapshot_near must not build a full snapshot')
    with monkeypatch.context() as patch:
        patch.setattr(BoundaryConsensus, 'snapshot', forbidden_full_snapshot)
        actual = consensus.snapshot_near(center, radius, maximum_points)
    assert_reference_bytes(actual, expected)
    assert before == (consensus._generation, consensus.last_stamp_ns,
                      tuple((key, tuple(value.items())) for key, value in consensus._cell_records.items()))
    assert_cell_index(consensus)
    return actual


@pytest.mark.parametrize('kind', ['new', 'committed_empty', 'all_invalid', 'unconfirmed'])
def test_empty_or_no_confirmed_records_preserves_exact_frontier(kind, monkeypatch):
    consensus = BoundaryConsensus()
    if kind == 'committed_empty':
        commit(consensus, observation(stamp=7))
    elif kind == 'all_invalid':
        obs = observation([[.2, .1], [-.2, .1]], stamp=7)
        false = np.zeros(2, dtype=bool)
        commit(consensus, replace(obs, normal_valid=false, valid=false, stable=false, reliable=false))
    elif kind == 'unconfirmed':
        commit(consensus, observation([[.2, .1]], stamp=7))
    actual = check_near(consensus, monkeypatch)
    assert actual.points_xy.shape == (0, 2)
    assert actual.source_frontier_ns == (None if kind == 'new' else 7)


def test_single_confirmed_closed_record_and_query_outside_all_cells(monkeypatch):
    consensus = confirmed([[.01, .01]], order=[0], closed=True)
    result = check_near(consensus, monkeypatch, center=[.01, .01], radius=.01, maximum_points=1)
    assert result.points_xy.shape == (1, 2)
    assert result.support_segments.shape == (1, 2, 2)
    assert result.directed_edges.shape == (0, 2)
    empty = check_near(consensus, monkeypatch, center=[100., -100.], radius=.1)
    assert len(empty.points_xy) == 0


@pytest.mark.parametrize('maximum_points', [1, 2, 4, 10])
def test_equal_distance_ties_follow_global_record_order_stably(maximum_points, monkeypatch):
    consensus = confirmed([[1., 0.], [0., 1.], [-1., 0.], [0., -1.]])
    result = check_near(consensus, monkeypatch, radius=1., maximum_points=maximum_points)
    expected_ids = consensus.snapshot().record_ids[:maximum_points]
    np.testing.assert_array_equal(result.record_ids, expected_ids)


def test_radius_boundary_is_inclusive_without_epsilon_or_cell_center_substitution(monkeypatch):
    inner = np.nextafter(1., 0.)
    outer = np.nextafter(1., np.inf)
    consensus = confirmed([[1., 0.], [-inner, 0.], [0., outer]])
    result = check_near(consensus, monkeypatch, radius=1.)
    assert len(result.points_xy) == 2
    assert any(np.array_equal(point, [1., 0.]) for point in result.points_xy)
    assert not any(np.array_equal(point, [0., outer]) for point in result.points_xy)


@pytest.mark.parametrize('maximum_points', [1, 3, 99])
def test_distance_order_and_capacity_cut_are_exact_even_without_truncation(maximum_points, monkeypatch):
    consensus = confirmed([[-2., 0.], [-1., 0.], [.25, 0.], [1., 0.], [3., 0.]])
    result = check_near(consensus, monkeypatch, radius=3., maximum_points=maximum_points)
    expected_points = np.array([[.25, 0.], [-1., 0.], [1., 0.], [-2., 0.], [3., 0.]])
    np.testing.assert_array_equal(result.points_xy, expected_points[:maximum_points])


def test_cell_fringe_and_negative_coordinates_use_committed_means(monkeypatch):
    consensus = confirmed([[-.151, .021], [-.049, .021], [.051, .021], [.149, .021],
                           [-19.951, -30.049], [20.049, 30.049]])
    for center, radius in (([-.049, .021], .002), ([0., .021], .05),
                           ([.05, .021], .002), ([-20., -30.], .07), ([20., 30.], .07)):
        check_near(consensus, monkeypatch, center=center, radius=radius)


def test_local_segment_uses_global_confirmed_neighbours_not_only_local_ids(monkeypatch):
    consensus = confirmed([[0., 0.], [.2, 0.], [.4, 0.]], order=[0, 1, 2])
    result = check_near(consensus, monkeypatch, center=[.2, 0.], radius=.01, maximum_points=1)
    assert len(result.points_xy) == 1
    assert len(result.support_segments) == 1
    np.testing.assert_array_equal(result.segment_owner, [0])
    assert result.directed_edges.shape == (0, 2)
    np.testing.assert_array_equal(result.full_in_degree, [1])
    np.testing.assert_array_equal(result.full_out_degree, [1])


def test_unconfirmed_global_neighbour_cannot_supply_local_support(monkeypatch):
    consensus = BoundaryConsensus()
    for stamp in range(1, 5):
        commit(consensus, observation([[0., 0.]], stamp, order=[0], closed=True))
    commit(consensus, observation([[0., 0.], [.2, 0.], [.4, 0.]], 5, order=[0, 1, 2]))
    result = check_near(consensus, monkeypatch, center=[0., 0.], radius=.01)
    assert len(result.points_xy) == 1
    assert len(result.support_segments) == 0
    assert result.directed_edges.shape == (0, 2)


def test_cropped_confirmed_branch_keeps_full_degrees(monkeypatch):
    consensus = BoundaryConsensus()
    points = [[0., 0.], [.2, 0.], [.4, 0.]]
    for stamp in range(1, 11):
        commit(consensus, observation(points, stamp, order=[0, 1] if stamp <= 5 else [0, 2]))
    full = consensus.snapshot()
    np.testing.assert_array_equal(full.full_out_degree, [2, 0, 0])
    result = check_near(consensus, monkeypatch, center=[0., 0.], radius=.25)
    np.testing.assert_array_equal(result.directed_edges, [[0, 1]])
    np.testing.assert_array_equal(result.full_out_degree, [2, 0])
    assert result.full_out_degree[0] > np.count_nonzero(result.directed_edges[:, 0] == 0)


def test_opposite_directions_in_same_cell_remain_distinct_index_members(monkeypatch):
    consensus = confirmed([[.01, .01], [.02, .02]], normals=[[0., 1.], [0., -1.]])
    assert len(consensus._cell_records) == 1
    assert len(next(iter(consensus._cell_records.values()))) == 2
    result = check_near(consensus, monkeypatch, center=[.01, .01], radius=.1)
    assert len(result.record_ids) == 2
    assert set(result.normals[:, 1]) == {-1., 1.}


def test_same_cell_reentry_preserves_distinct_ids_and_segment_order(monkeypatch):
    consensus = confirmed([[.01, .01], [.12, .01], [.02, .02]], order=[0, 1, 2])
    assert len(consensus._records) == 3
    assert sorted(map(len, consensus._cell_records.values())) == [1, 2]
    check_near(consensus, monkeypatch, center=[.01, .01], radius=.2, maximum_points=3)


def test_discarded_wrong_owner_and_stale_tokens_never_mutate_index(monkeypatch):
    consensus = confirmed([[0., 0.], [.2, 0.], [.4, 0.]], order=[0, 1, 2])
    before = consensus.snapshot()
    discarded = consensus.prepare(observation([[4., 5.]], 6))
    assert_cell_index(consensus)
    assert_reference_bytes(consensus.snapshot(), before)
    other = BoundaryConsensus()
    assert not other.commit(discarded)
    assert_cell_index(other)
    assert not other._cell_records
    check_near(consensus, monkeypatch)
    commit(consensus, observation([[0., 0.], [.2, 0.], [.4, 0.]], 7, order=[0, 1, 2]))
    after = consensus.snapshot()
    assert not consensus.commit(discarded)
    assert_cell_index(consensus)
    assert_reference_bytes(consensus.snapshot(), after)
    check_near(consensus, monkeypatch)


def test_capacity_rejection_does_not_leave_candidate_index_entries(monkeypatch):
    consensus = confirmed([[0., 0.]], order=[0], closed=True,
                          config={'max_candidate_cells': 1})
    before = consensus.snapshot()
    with pytest.raises(ValueError, match='capacity'):
        consensus.prepare(observation([[1., 1.]], 6))
    assert_cell_index(consensus)
    assert_reference_bytes(consensus.snapshot(), before)
    check_near(consensus, monkeypatch)


def test_ttl_equality_retains_candidate_later_expiry_reinsertion_uses_new_id(monkeypatch):
    consensus = confirmed([[0., 0.]], order=[0], closed=True,
                          config={'candidate_ttl_s': 1.})
    commit(consensus, observation([[2., 0.]], 10))
    old_id = next(key for key, record in consensus._records.items() if record.cell == (20, 0))
    commit(consensus, observation(stamp=1_000_000_010))
    assert old_id in consensus._records  # equality must NOT expire
    check_near(consensus, monkeypatch, center=[2., 0.], radius=.1)
    token = consensus.prepare(observation(stamp=1_000_000_011))
    assert old_id in consensus._records
    assert_cell_index(consensus)
    assert consensus.commit(token)
    assert_cell_index(consensus)
    assert old_id not in consensus._records and (20, 0) not in consensus._cell_records
    assert consensus.confirmed_count == 1  # confirmed node is not candidate-TTL data
    for stamp in range(1_000_000_012, 1_000_000_017):
        commit(consensus, observation([[2., 0.]], stamp))
    new_id = next(key for key, record in consensus._records.items() if record.cell == (20, 0))
    assert new_id > old_id
    result = check_near(consensus, monkeypatch, center=[2., 0.], radius=.1)
    np.testing.assert_array_equal(result.record_ids, [new_id])


def test_multiple_updates_preserve_frame_means_latest_segments_and_old_snapshot(monkeypatch):
    points = np.column_stack((np.arange(-6, 7)*.2, np.zeros(13)))
    consensus = confirmed(points, order=range(13))
    previous = check_near(consensus, monkeypatch, center=[.1, 0.], radius=.85, maximum_points=6)
    old_bytes = {name: getattr(previous, name).tobytes() for name in REFERENCE_ARRAYS}
    for stamp, offset in ((6, .01), (7, -.02), (8, .03)):
        commit(consensus, observation(points + [offset, offset*.5], stamp, order=range(13)))
        check_near(consensus, monkeypatch, center=[-.12, .005], radius=.62, maximum_points=4)
    assert previous.source_frontier_ns == 5
    assert all(getattr(previous, name).tobytes() == value for name, value in old_bytes.items())


@pytest.mark.parametrize('radius', [1.e155, np.finfo(np.float64).max])
def test_finite_radius_with_unrepresentable_square_is_rejected(radius):
    consensus = BoundaryConsensus()
    with pytest.raises(ValueError, match='radius square overflow'):
        consensus.snapshot_near([0., 0.], radius, 100)
    assert consensus.source_frontier_ns is None
    assert not consensus._records and not consensus._cell_records
