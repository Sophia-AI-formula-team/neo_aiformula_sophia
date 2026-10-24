"""R16 identity continuity: real public transactions, never synthetic votes.

The model-valid faces below are explicit finite observations, not extractor or
vehicle-accuracy fixtures.  Every old record and directed edge is obtained by
prepare/commit; private state is only inspected for identity/transaction proof.
No pose solve, runtime deadline, image truth, or lowered consensus gate is used.
"""
from dataclasses import replace

import numpy as np
import pytest

from lane_mapping_lya_reference.paint_geometry import (
    BoundaryConsensus, BoundaryObservation, _finite_tangent_overlap,
)


DT = 100_000_000
Y = .05001
A = (.14, Y)
B = (.04, Y)
B_CROSS = (.04, .049)


def observation(points, stamp, *, links=(), normals=None, segments=None, reliable=False):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    count = len(points)
    normals = (np.tile([0., 1.], (count, 1)) if normals is None else
               np.asarray(normals, dtype=np.float64).reshape(count, 2))
    if segments is None:
        # Finite faces, not infinite fitted lines or a hull across the record.
        segments = np.stack((points + [.045, 0.], points - [.045, 0.]), axis=1)
    segments = np.asarray(segments, dtype=np.float64).reshape(count, 2, 2)
    previous = np.full(count, -1, dtype=np.int64)
    following = previous.copy()
    for first, second in links:
        assert following[first] == -1 and previous[second] == -1
        following[first], previous[second] = second, first
    contour_ids = np.full(count, -1, dtype=np.int64)
    contour = 0
    for start in list(np.flatnonzero(previous < 0)) + list(range(count)):
        current = int(start)
        if contour_ids[current] >= 0:
            continue
        while current >= 0 and contour_ids[current] < 0:
            contour_ids[current] = contour
            current = int(following[current])
        contour += 1
    reliable = np.broadcast_to(np.asarray(reliable, dtype=bool), (count,)).copy()
    true = np.ones(count, dtype=bool)
    white = np.column_stack((np.arange(count, dtype=np.int64), np.zeros(count, dtype=np.int64)))
    return BoundaryObservation(
        points=points, normals=normals, segments=segments,
        sensitivity=np.where(reliable, .02, .2), metric_valid=true,
        stable=true, normal_valid=true, valid=true, reliable=reliable,
        source_stamp_ns=stamp, pixel_segments=segments, white_pixels=white,
        black_pixels=white + [0, 1], previous=previous, next=following,
        contour_ids=contour_ids, reason_codes=np.zeros(count, dtype=np.int64),
        normal_support_count=np.full(count, 6, dtype=np.int64), normal_radius=np.full(count, .45))


def index_members(consensus):
    return {cell: tuple(members) for cell, members in consensus._cell_records.items()}


def assert_index(consensus):
    seen = []
    for cell, members in consensus._cell_records.items():
        assert members and all(value is None for value in members.values())
        for record_id in members:
            record = consensus._records[record_id]
            assert record.cell == cell
            assert tuple(np.rint(np.asarray(record.point) /
                                 consensus.config['grid_resolution_m']).astype(np.int64)) == cell
            seen.append(record_id)
    assert len(seen) == len(set(seen)) == len(consensus._records)
    assert set(seen) == set(consensus._records)


def fingerprint(consensus):
    return (
        consensus._generation, consensus.source_frontier_ns, consensus._next_record_id,
        consensus._segment_count, consensus.confirmed_count,
        tuple((key, rec.cell, rec.point, rec.normal, rec.votes, rec.reliable_votes,
               rec.last_stamp_ns, rec.segments.tobytes(), rec.neighbours.tobytes())
              for key, rec in consensus._records.items()),
        tuple((key, edge.votes, edge.reliable_votes, edge.last_stamp_ns)
              for key, edge in consensus._edges.items()),
        tuple((cell, tuple(members.items())) for cell, members in consensus._cell_records.items()),
    )


def commit(consensus, obs):
    token = consensus.prepare(obs)
    assert consensus.commit(token)
    assert_index(consensus)
    return token


def record_at(consensus, point):
    candidates = [record for record in consensus._records.values()
                  if np.array_equal(record.point, point)]
    assert len(candidates) == 1
    return candidates[0].record_id


def history(points=(A, B), *, links=((0, 1),), frames=5, normals=None,
            segments=None, reliable=False, config=None):
    consensus = BoundaryConsensus(config)
    for frame in range(1, frames + 1):
        commit(consensus, observation(points, frame * DT, links=links, normals=normals,
                                      segments=segments, reliable=reliable))
    return consensus, [record_at(consensus, point) for point in points]


def crossed(stamp=6 * DT, **kwargs):
    return observation((A, B_CROSS), stamp, links=((0, 1),), reliable=True, **kwargs)


def test_cross_cell_reuses_past_identity_without_retroactive_reliable_votes():
    consensus, (a, b) = history()
    assert consensus.confirmed_count == 0
    before = fingerprint(consensus)
    token = consensus.prepare(crossed())
    assert fingerprint(consensus) == before
    records = {record.record_id: record for record in token.updates}
    assert set(records) == {a, b}
    assert token.stats['adjacent_cell_associations'] == 1
    assert records[b].votes == 6 and records[b].reliable_votes == 1
    assert records[b].cell == (0, 0)  # FUSED mean, not the old or current face cell.
    expected = np.asarray(B) + (np.asarray(B_CROSS) - B) / 6
    np.testing.assert_array_equal(records[b].point, expected)
    assert [(edge.source_id, edge.target_id, edge.votes, edge.reliable_votes)
            for edge in token.edge_updates] == [(a, b, 6, 1)]
    assert consensus._edges[(a, b)].votes == 5
    assert consensus._edges[(a, b)].reliable_votes == 0
    assert consensus.commit(token)
    assert_index(consensus)
    assert b not in consensus._cell_records.get((0, 1), {})
    assert b in consensus._cell_records[(0, 0)]
    result = consensus.snapshot()
    assert set(result.record_ids) == {a, b}
    np.testing.assert_array_equal(result.edge_frame_votes, [6])
    np.testing.assert_array_equal(result.edge_reliable_votes, [1])


def test_current_face_crosses_cell_but_fused_mean_need_not_migrate():
    old_b = (.04, .051)
    consensus, (a, b) = history((A, old_b))
    commit(consensus, crossed())
    assert consensus._records[b].votes == 6
    assert consensus._records[b].cell == (0, 1)
    assert b in consensus._cell_records[(0, 1)]
    assert (0, 0) not in consensus._cell_records


@pytest.mark.parametrize('transform', [(0., 0., 0.), (0., 0., np.pi / 2),
                                        (-.3, -.2, np.pi), (.2, -.4, -np.pi / 2)])
def test_grid_symmetries_and_negative_cells_do_not_break_identity(transform):
    consensus = BoundaryConsensus()
    initial = observation((A, B), DT, links=((0, 1),)).transformed(transform)
    for frame in range(1, 6):
        commit(consensus, replace(initial, source_stamp_ns=frame * DT))
    ids = set(consensus._records)
    commit(consensus, crossed().transformed(transform))
    assert set(consensus._records) == ids
    assert all(record.votes == 6 for record in consensus._records.values())
    assert all(edge.votes == 6 for edge in consensus._edges.values())


@pytest.mark.parametrize('case', ['no_edge', 'four_votes', 'reverse_edge', 'current_gap', 'wrong_normal'])
def test_missing_usable_directed_witness_never_borrows_old_votes(case):
    links = () if case == 'no_edge' else (((1, 0),) if case == 'reverse_edge' else ((0, 1),))
    frames = 4 if case == 'four_votes' else 5
    consensus, (a, b) = history(links=links, frames=frames)
    obs = observation((A, B_CROSS), 6 * DT,
                      links=() if case == 'current_gap' else ((0, 1),), reliable=True,
                      normals=((0., 1.), (0., -1.)) if case == 'wrong_normal' else None)
    token = commit(consensus, obs)
    assert consensus._records[b].votes == frames
    new = [record for record in token.updates if record.record_id != a]
    assert len(new) == 1 and new[0].record_id != b
    assert new[0].votes == new[0].reliable_votes == 1


def test_incoming_seed_uses_matching_historical_direction():
    consensus, (a, b) = history(links=((1, 0),))
    commit(consensus, observation((A, B_CROSS), 6 * DT, links=((1, 0),), reliable=True))
    assert consensus._records[b].votes == 6
    assert consensus._edges[(b, a)].votes == 6
    assert (a, b) not in consensus._edges


@pytest.mark.parametrize('kind', ['no_overlap', 'touch_only', 'sub_ulp_positive'])
def test_raw_finite_overlap_is_strict_and_not_a_model_point_distance(kind):
    old_segments = np.array([[[.095, Y], [.185, Y]], [[0., Y], [.04, Y]]])
    consensus, (a, b) = history(segments=old_segments)
    begin = .05 if kind == 'no_overlap' else (.04 if kind == 'touch_only' else np.nextafter(.04, 0.))
    new_segments = np.array([[[.095, Y], [.185, Y]], [[begin, .049], [.08, .049]]])
    commit(consensus, crossed(segments=new_segments))
    assert consensus._records[b].votes == (6 if kind == 'sub_ulp_positive' else 5)
    assert len(consensus._records) == (2 if kind == 'sub_ulp_positive' else 3)


def test_disjoint_raw_intervals_are_not_replaced_by_their_hull():
    # Public BoundaryObservation permits disconnected finite segment arrays;
    # the identity matcher must not silently infer the missing interval.
    points = (A, (.035, Y), (.005, Y))
    segments = np.array([[[.095, Y], [.185, Y]], [[.03, Y], [.04, Y]], [[0., Y], [.01, Y]]])
    consensus = BoundaryConsensus()
    for frame in range(1, 6):
        commit(consensus, observation(points, frame * DT, links=((0, 1), (1, 2)), segments=segments))
    a = record_at(consensus, A)
    b = next(record.record_id for record in consensus._records.values() if record.record_id != a)
    current_segments = np.array([[[.095, Y], [.185, Y]], [[.015, .049], [.025, .049]]])
    commit(consensus, observation((A, (.02, .049)), 6 * DT, links=((0, 1),),
                                  segments=current_segments, reliable=True))
    assert consensus._records[b].votes == 5
    assert len(consensus._records) == 3


@pytest.mark.parametrize('angle', [0., .7, -1.4])
def test_two_tangent_charts_must_overlap_for_the_same_raw_face_pair(angle):
    # Pair 0 overlaps ONLY the first chart, pair 1 ONLY the second. Separate
    # existential tests would wrongly combine two unrelated finite supports.
    first = np.array([[[0., 0.], [1., 0.]]])
    second = np.array([[[.2, 2.], [.4, 2.]], [[1.2, -1.], [1.4, -1.]]])
    normals = np.array([[0., 1.], [-.5, np.sqrt(.75)]])
    rotation = np.array([[np.cos(angle), -np.sin(angle)],
                         [np.sin(angle), np.cos(angle)]])
    first = first @ rotation.T + [2., -3.]
    second = second @ rotation.T + [2., -3.]
    normals = normals @ rotation.T
    pairwise = []
    for normal in normals:
        tangent = np.array([normal[1], -normal[0]])
        left = first @ tangent
        right = second @ tangent
        pairwise.append((np.max(left, axis=1)[:, None] > np.min(right, axis=1)) &
                        (np.max(right, axis=1) > np.min(left, axis=1)[:, None]))
    np.testing.assert_array_equal(pairwise[0], [[True, False]])
    np.testing.assert_array_equal(pairwise[1], [[False, True]])
    assert not np.any(pairwise[0] & pairwise[1])
    assert not _finite_tangent_overlap(first, second, *normals)
    assert not _finite_tangent_overlap(second, first, *normals)
    positive = np.array([[[.3, 0.], [.4, 0.]]]) @ rotation.T + [2., -3.]
    assert _finite_tangent_overlap(first, np.concatenate((second, positive)), *normals)


def test_two_cells_away_is_not_recovered_by_large_finite_support():
    consensus, (a, b) = history()
    commit(consensus, observation((A, (.04, -.051)), 6 * DT, links=((0, 1),), reliable=True))
    assert consensus._records[b].votes == 5
    assert len(consensus._records) == 3


def test_original_same_cell_match_has_priority_over_closer_topological_candidate():
    old_b, adjacent = (.04, .07), (.04, .049)
    consensus, (a, b, other) = history((A, old_b, adjacent), links=((0, 2),))
    commit(consensus, observation((A, (.04, .051)), 6 * DT, links=((0, 1),), reliable=True))
    assert consensus._records[b].votes == 6
    assert consensus._records[other].votes == 5


def test_legacy_same_cell_distance_only_hazard_is_explicit_not_fixed_by_r16():
    # Same-cell matching still ignores finite overlap: this compatibility test
    # documents its old false-identity risk, NOT a claimed desirable association.
    consensus, (a, b) = history()
    segments = np.array([[[.095, Y], [.185, Y]], [[10., Y], [11., Y]]])
    commit(consensus, observation((A, B), 6 * DT, links=((0, 1),), segments=segments))
    assert consensus._records[b].votes == 6


@pytest.mark.parametrize('kind', ['distance_tie', 'multiple_claimants'])
def test_same_cell_ambiguity_does_not_unlock_cross_cell_fallback(kind):
    if kind == 'distance_tie':
        points = (A, B, (-.01, .01), (.01, .01))
        current = (A, (0., .049))
    else:
        points = (A, B, (.02, .02))
        current = (A, (.019, .049), (.021, .049))
    consensus, ids = history(points)
    commit(consensus, observation(current, 6 * DT, links=((0, 1),), reliable=True))
    assert all(consensus._records[record_id].votes == 5 for record_id in ids[1:])
    assert len(consensus._records) == len(ids) + len(current) - 1


@pytest.mark.parametrize('claimants', [1, 2])
def test_neighbor_record_claimed_in_original_same_cell_pass_is_not_available(claimants):
    consensus, (a, b) = history()
    points = (A, B_CROSS, B) if claimants == 1 else (A, B_CROSS, (.039, Y), (.041, Y))
    token = commit(consensus, observation(points, 6 * DT, links=((0, 1),), reliable=True))
    crossed_record = next(record for record in token.updates
                          if np.array_equal(record.point, B_CROSS))
    assert crossed_record.record_id != b and crossed_record.votes == 1
    assert consensus._records[b].votes == (6 if claimants == 1 else 5)


def test_exact_same_cell_nearest_tie_reserves_both_old_candidates_against_cross_claim():
    # Binary-exact +/- x distances make the original same-cell tie genuine.
    points = (A, (.03125, Y), (-.03125, Y))
    consensus, (a, b, c) = history(points)
    token = commit(consensus, observation((A, (.03125, .049), (0., Y)), 6 * DT,
                                          links=((0, 1),), reliable=True))
    assert consensus._records[b].votes == consensus._records[c].votes == 5
    assert token.stats['adjacent_cell_associations'] == 0
    assert len(consensus._records) == 5
    assert all(record.votes == 1 for record in token.updates if record.record_id != a)


def test_competing_viable_neighbor_candidates_reject_instead_of_nearest_tie_break():
    points = (A, B, (.04, .14))
    consensus, (a, b, c) = history(points)
    for frame in range(6, 11):
        commit(consensus, observation(points, frame * DT, links=((0, 2),)))
    commit(consensus, crossed(stamp=11 * DT))
    assert consensus._records[b].votes == consensus._records[c].votes == 10
    assert len(consensus._records) == 4


def test_ambiguous_cross_group_reserves_all_viable_candidates_against_another_group():
    # Group U has two genuine viable histories B/C. Group V can use B alone.
    # Discarding U's ambiguous proposal must not let V steal a contested B.
    points = (A, (.03125, Y), (-.03125, Y), (-.14, Y))
    consensus, (a, b, c, d) = history(points)
    for frame in range(6, 11):
        commit(consensus, observation(points, frame * DT, links=((0, 2),)))
    for frame in range(11, 16):
        commit(consensus, observation(points, frame * DT, links=((3, 1),)))
    token = commit(consensus, observation((A, (.01, .049), points[3], (.02, .049)),
                                          16 * DT, links=((0, 1), (2, 3)), reliable=True))
    assert consensus._records[b].votes == consensus._records[c].votes == 15
    assert consensus._records[a].votes == consensus._records[d].votes == 16
    assert token.stats['adjacent_cell_associations'] == 0
    assert len(consensus._records) == 6


def test_all_independently_matched_seeds_must_agree_on_old_identity():
    points = (A, B, (-.04, Y), (-.14, Y))
    consensus, (a, b, c, d) = history(points, links=((0, 1), (2, 3)))
    segments = np.array([[[.095, Y], [.185, Y]], [[-.045, .049], [.045, .049]],
                         [[-.185, Y], [-.095, Y]]])
    commit(consensus, observation((A, (0., .049), points[3]), 6 * DT,
                                  links=((0, 1), (1, 2)), segments=segments, reliable=True))
    assert consensus._records[b].votes == consensus._records[c].votes == 5
    assert consensus._records[a].votes == consensus._records[d].votes == 6
    assert len(consensus._records) == 5


def test_two_current_groups_cannot_both_claim_one_cross_cell_identity():
    points = (A, B, (-.14, Y))
    consensus, (a, b, c) = history(points)
    for frame in range(6, 11):
        commit(consensus, observation(points, frame * DT, links=((2, 1),)))
    token = commit(consensus, observation((A, B_CROSS, points[2], (.045, .049)), 11 * DT,
                                          links=((0, 1), (2, 3)), reliable=True))
    assert consensus._records[b].votes == 10
    new = [record for record in token.updates if record.record_id not in {a, b, c}]
    assert len(new) == 2 and all(record.votes == 1 for record in new)


def test_new_associations_cannot_propagate_or_seed_each_other():
    points = (A, B, (-.06, Y))
    consensus, (a, b, c) = history(points, links=((0, 1), (1, 2)))
    commit(consensus, observation((A, B_CROSS, (-.06, .049)), 6 * DT,
                                  links=((0, 1), (1, 2)), reliable=True))
    assert consensus._records[b].votes == 6
    assert consensus._records[c].votes == 5
    assert len(consensus._records) == 4


def test_seedless_closed_cycle_cannot_bootstrap_cross_cell_votes():
    points = ((-.06, Y), (.04, Y), (.14, Y))
    consensus, ids = history(points, links=((0, 1), (1, 2), (2, 0)))
    moved = np.asarray(points) + [0., -.00101]
    commit(consensus, observation(moved, 6 * DT, links=((0, 1), (1, 2), (2, 0)), reliable=True))
    assert all(consensus._records[record_id].votes == 5 for record_id in ids)
    assert len(consensus._records) == 2 * len(ids)


def test_normal_incompatible_historical_branch_is_retained_not_erased():
    points = (A, B, (.04, .09))
    normals = ((0., 1.), (0., 1.), (1., 0.))
    consensus, (a, b, cap) = history(points, normals=normals, reliable=True)
    for frame in range(6, 11):
        commit(consensus, observation(points, frame * DT, links=((0, 2),),
                                      normals=normals, reliable=True))
    old_branch = consensus._edges[(a, cap)]
    commit(consensus, crossed(stamp=11 * DT))
    assert consensus._records[b].votes == 11
    assert consensus._edges[(a, b)].votes == 6
    assert consensus._edges[(a, cap)] == old_branch
    reference = consensus.snapshot()
    index = int(np.flatnonzero(reference.record_ids == a)[0])
    assert reference.full_out_degree[index] == 2


@pytest.mark.parametrize('extra_ns, expected_reuse', [(0, True), (1, False)])
def test_historical_edge_ttl_equality_and_expiry_are_not_reset_by_record_updates(extra_ns, expected_reuse):
    consensus, (a, b) = history(config={'candidate_ttl_s': 1.})
    # Confirm the NODES without observing an edge.  Its 5/0 evidence stays at .5s.
    commit(consensus, observation((A, B), 6 * DT, reliable=True))
    assert consensus._edges[(a, b)].last_stamp_ns == 5 * DT
    stamp = 15 * DT + extra_ns
    token = commit(consensus, crossed(stamp=stamp))
    assert consensus._records[b].votes == (7 if expected_reuse else 6)
    if expected_reuse:
        assert consensus._edges[(a, b)].votes == 6
    else:
        assert (a, b) not in consensus._edges
        assert len(token.edge_updates) == 1 and token.edge_updates[0].votes == 1


def test_expired_candidate_record_cannot_be_resurrected_by_its_old_edge():
    consensus, (a, b) = history(config={'candidate_ttl_s': 1.})
    commit(consensus, observation((A,), 6 * DT, reliable=True))
    commit(consensus, crossed(stamp=15 * DT + 1))
    assert b not in consensus._records
    assert consensus._records[a].votes == 7
    assert (a, b) not in consensus._edges


def test_discard_foreign_stale_and_duplicate_tokens_cannot_migrate_index():
    consensus, (a, b) = history()
    before = fingerprint(consensus)
    discarded = consensus.prepare(crossed())
    assert fingerprint(consensus) == before
    other = BoundaryConsensus()
    assert not other.commit(discarded) and not other._cell_records
    # Commit a different later frame, invalidating the earlier migration token.
    commit(consensus, observation((A, B), 7 * DT, links=((0, 1),)))
    after = fingerprint(consensus)
    assert not consensus.commit(discarded)
    assert fingerprint(consensus) == after
    migrated = commit(consensus, crossed(stamp=8 * DT))
    after = fingerprint(consensus)
    assert not consensus.commit(migrated)
    assert fingerprint(consensus) == after
    assert consensus._records[b].cell == (0, 0)


def test_capacity_failure_during_otherwise_valid_migration_is_fully_pure():
    consensus, (a, b) = history(config={'max_candidate_cells': 2})
    before = fingerprint(consensus)
    with pytest.raises(ValueError, match='capacity'):
        consensus.prepare(observation((A, B_CROSS, (4., 4.)), 6 * DT,
                                      links=((0, 1),), reliable=True))
    assert fingerprint(consensus) == before
    assert_index(consensus)


def test_each_reliable_current_frame_increments_once_without_replaying_support():
    consensus, (a, b) = history()
    first = commit(consensus, crossed())
    assert consensus._records[b].votes == 6 and consensus._records[b].reliable_votes == 1
    with pytest.raises(ValueError, match='stamp'):
        consensus.prepare(crossed())
    assert not consensus.commit(first)
    # Two adjacent faces of the same current run are still exactly ONE vote.
    obs = observation((A, (.039, .049), (.041, .049)), 7 * DT,
                      links=((0, 1), (1, 2)), reliable=True)
    commit(consensus, obs)
    assert consensus._records[b].votes == 7 and consensus._records[b].reliable_votes == 2
    assert consensus._edges[(a, b)].votes == 7
    assert consensus._edges[(a, b)].reliable_votes == 2
