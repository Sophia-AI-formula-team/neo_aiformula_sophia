"""R22 integration contracts for evidence-qualified continuation permissions.

Analytical raw/model observations isolate public prepare/commit/snapshot and
nearest-query behavior. They are not image accuracy, pose, runtime or driving
tests. All consensus tests retain the original five distinct frames plus one
reliable frame per continuation; explicit references isolate the wire contract.
"""
from dataclasses import fields, is_dataclass, replace
from collections.abc import Mapping

import numpy as np
import pytest

from lane_mapping_lya_reference.boundary_localization import BoundaryLaneLocalizer
from lane_mapping_lya_reference.paint_geometry import (
    BoundaryConsensus, CONTINUATION_FIELDS,
)
from test_boundary_certified_domains import reference, query, assert_bytes
from test_boundary_domain_consensus import DT, commit
from test_boundary_footprint_domains import fragments


def permissions(ref, spans=((.2, .4), (.6, .8)), primitive_ids=None):
    count = len(spans)
    return replace(ref,
        continuation_intervals=np.asarray(spans, dtype=np.float64).reshape(-1, 2),
        continuation_primitive=(np.zeros(count, dtype=np.int64) if primitive_ids is None
                                else np.asarray(primitive_ids, dtype=np.int64)),
        continuation_frame_votes=np.full(count, 5, dtype=np.int64),
        continuation_reliable_votes=np.ones(count, dtype=np.int64),
        continuation_last_stamp_ns=np.full(count, ref.source_frontier_ns, dtype=np.int64))


def branch_reference():
    # Owner 0's certificate at y=0 is globally nearer than the valid y=.3 line.
    # Its two backward graph edges create a real retained branch inventory.
    return reference(
        points=((0., 0.), (-1., 0.), (-1., .02), (-1., .3), (1., .3)),
        edges=((0, 1), (0, 2), (3, 4)),
        segments=(((0., 0.), (1., 0.)),), owners=(0,))


def frozen(value):
    """Exact read-only transaction fingerprint, including new history arrays."""
    if isinstance(value, np.ndarray):
        return ('array', value.dtype.str, value.shape, value.tobytes())
    if is_dataclass(value):
        return (type(value).__name__, tuple((field.name, frozen(getattr(value, field.name)))
                                          for field in fields(value)))
    if isinstance(value, Mapping):
        return tuple(sorted(((frozen(key), frozen(item)) for key, item in value.items()), key=repr))
    if isinstance(value, (tuple, list)):
        return tuple(map(frozen, value))
    if isinstance(value, set):
        return tuple(sorted(map(frozen, value), key=repr))
    return value


def state(consensus):
    return frozen(vars(consensus)), frozen(consensus.snapshot())


def continuation_observation(stamp, kind='direct', *, y=.01, quad_y=.01,
                             half_height=.004, trusted=True, quads=True,
                             gap=False, model_invalid_neighbor=False):
    """Two sampled versions of one ordered contour, with no spatial voting.

    P -> A -> C and P -> A -> B -> C are distinct observed hypotheses.
    Only A's actual owned rectangle is allowed to authorize its certificate.
    Different cells deliberately prevent fixture identity ties.
    """
    xs = (-.2, 0., .4) if kind == 'direct' else (-.2, 0., .2, .4)
    points = np.column_stack((xs, np.full(len(xs), y)))
    links = tuple(zip(range(len(xs)-1), range(1, len(xs))))
    if gap:
        links = tuple(pair for pair in links if pair[0] != 1)
    obs = fragments(stamp, points=points, links=links)
    trusted_flags = np.full(len(xs), trusted, dtype=bool)
    obs = replace(obs, position_reliable=trusted_flags,
                  position_segment_reliable=trusted_flags)
    if model_invalid_neighbor:
        flags = obs.position_valid.copy()
        flags[2] = False
        reliable = trusted_flags & flags
        obs = replace(obs, position_valid=flags, position_reliable=reliable,
                      position_segment_valid=flags, position_segment_reliable=reliable)
    if quads:
        corners = np.stack((points+[-.03, quad_y-y-half_height],
                            points+[.03, quad_y-y-half_height],
                            points+[.03, quad_y-y+half_height],
                            points+[-.03, quad_y-y+half_height]), axis=1)
        obs = replace(obs, position_footprints=corners)
    return obs


def direct_then_subdivided(*, direct=5, subdivided=5, trusted=True, config=None):
    consensus = BoundaryConsensus(config)
    for frame in range(1, direct+1):
        commit(consensus, continuation_observation(frame*DT))
    for frame in range(direct+1, direct+subdivided+1):
        commit(consensus, continuation_observation(frame*DT, 'subdivided', trusted=trusted))
    return consensus


def owner_id(consensus):
    matches = [record.record_id for record in consensus._records.values()
               if record.point[0] == 0.]
    assert len(matches) == 1
    return matches[0]


def owner_permissions(consensus, record_id=None):
    ref = consensus.snapshot()
    record_id = owner_id(consensus) if record_id is None else record_id
    slot = np.flatnonzero(ref.record_ids == record_id)
    assert len(slot) == 1
    primitives = np.flatnonzero(ref.certified_model_owner == slot[0])
    if ref.continuation_intervals is None:
        return ref, np.empty((0, 2))
    return ref, ref.continuation_intervals[np.isin(ref.continuation_primitive, primitives)]


def test_permission_changes_only_admission_not_nearest_geometry_or_order():
    legacy = branch_reference()
    permitted = permissions(legacy)
    points = [[.3, .01], [.5, .01], [.7, .01], [1.1, .01]]
    old_matcher, old = query(legacy, points)
    new_matcher, new = query(permitted, points)
    for name in ('starts', 'ends', 'delta', 'length_squared', 'normals', 'primitive_nodes'):
        assert_bytes(getattr(old_matcher._index.graph, name), getattr(new_matcher._index.graph, name))
    for original, updated in zip(old[:3], new[:3]):
        assert_bytes(original, updated)
    assert old[3].tolist() == [False]*4
    assert new[3].tolist() == [True, False, True, False]
    assert new[0].tolist() == [0]*4


def test_permission_hole_does_not_expose_farther_valid_line():
    ref = permissions(branch_reference())
    _, result = query(ref, [[.5, .01]])
    assert result[0][0] == 0 and not result[3][0]
    assert result[1][0, 1] == 0.
    _, farther = query(ref.subset([3, 4]), [[.5, .01]])
    assert farther[3][0] and farther[1][0, 1] == .3


def test_permissions_cannot_override_original_signed_normal_gate():
    _, result = query(permissions(branch_reference()), [[.3, .01]], [[0., -1.]])
    assert result[0][0] == -1 and not result[3][0]


def test_permission_does_not_expand_original_matching_distance():
    _, result = query(permissions(branch_reference()), [[.3, -.66]])
    assert result[0][0] == -1 and not result[3][0]


def test_empty_permissions_keep_legacy_branch_rejection_exact():
    ref = branch_reference()
    points = [[.3, .01], [.5, .01], [-.5, .005]]
    _, original = query(ref, points)
    _, explicit = query(permissions(ref, ()), points)
    for first, second in zip(original, explicit):
        assert_bytes(first, second)
    assert ref.boundary_version == 3 and ref.continuation_representation is None


def test_legacy_unambiguous_none_keeps_original_closed_geometry():
    ref = reference()
    _, before = query(ref, [[0., .01], [1.1, .01]])
    _, after = query(permissions(ref, ()), [[0., .01], [1.1, .01]])
    assert before[3].tolist() == [True, False]
    for first, second in zip(before, after):
        assert_bytes(first, second)


def test_subset_keeps_global_branch_and_remaps_certificate_permission_not_graph_slot():
    ref = branch_reference()
    ref = replace(ref,
        certified_model_segments=np.array([[[2., .5], [3., .5]], [[0., 0.], [1., 0.]]]),
        certified_model_owner=np.array([4, 0], dtype=np.int64),
        certified_model_frame_votes=np.array([5, 5], dtype=np.int64),
        certified_model_reliable_votes=np.array([1, 1], dtype=np.int64),
        certified_model_last_stamp_ns=np.full(2, ref.source_frontier_ns, dtype=np.int64))
    ref = permissions(ref, ((.2, .4), (.6, .8)), (1, 1))
    cropped = ref.subset([0, 3])
    assert cropped.full_out_degree.tolist() == [2, 1]
    assert cropped.directed_edges.shape == (0, 2)
    assert cropped.certified_model_owner.tolist() == [0]
    assert cropped.continuation_primitive.tolist() == [0, 0]
    assert_bytes(cropped.continuation_intervals, ref.continuation_intervals)
    _, result = query(cropped, [[.3, .01], [.5, .01]])
    assert result[3].tolist() == [True, False]
    without_owner = ref.subset([4, 3])
    assert not len(without_owner.continuation_intervals)
    assert without_owner.certified_model_owner.tolist() == [0]


@pytest.mark.parametrize('name', CONTINUATION_FIELDS)
def test_every_permission_array_is_immutable_and_matches_source_identity(name):
    ref = permissions(branch_reference())
    values = getattr(ref, name)
    with pytest.raises(ValueError):
        values.setflags(write=True)
    matcher = BoundaryLaneLocalizer(ref)
    assert matcher.matches_anchor_source(ref)
    changed = values.copy()
    if name == 'continuation_intervals':
        changed[0, 0] += .01
    elif name == 'continuation_primitive':
        changed = changed[:1]
        replacement = permissions(ref, ((.2, .4),), changed)
        assert not matcher.matches_anchor_source(replacement)
        return
    elif name == 'continuation_frame_votes':
        changed[0] += 1
    elif name == 'continuation_reliable_votes':
        changed[0] += 1
    else:
        changed[0] -= 1
    assert not matcher.matches_anchor_source(replace(ref, **{name: changed}))


@pytest.mark.parametrize('name', CONTINUATION_FIELDS)
def test_partial_permission_payload_is_not_legacy(name):
    with pytest.raises(ValueError):
        replace(permissions(branch_reference()), **{name: None})


@pytest.mark.parametrize('name,value', [
    ('continuation_intervals', [[-.1, .4], [.6, .8]]),
    ('continuation_intervals', [[.2, .4], [.4, .8]]),
    ('continuation_intervals', [[.2, .4], [.6, 1.1]]),
    ('continuation_primitive', [1, 1]),
    ('continuation_last_stamp_ns', [1000000001, 1000000000]),
    ('continuation_frame_votes', [11, 5]),
    ('continuation_reliable_votes', [0, 1]),
])
def test_invalid_or_future_reference_evidence_rejected(name, value):
    with pytest.raises(ValueError):
        replace(permissions(branch_reference()), **{name: np.asarray(value)})


def test_public_commit_requires_five_plus_one_for_each_observed_hypothesis():
    consensus = direct_then_subdivided()
    identity = owner_id(consensus)
    history = consensus._continuations[identity]
    assert len(history) == 2
    assert all(len(frames) == 5 for frames in history.values())
    assert all(sum(bool(np.any(frame.reliable)) for frame in frames) >= 1 for frames in history.values())
    ref, spans = owner_permissions(consensus)
    assert len(spans) > 0
    slot = int(np.flatnonzero(ref.record_ids == identity)[0])
    assert ref.full_out_degree[slot] == 2  # Kept, not erased to authorize support.
    assert np.all(ref.continuation_frame_votes == 5)
    assert np.all(ref.continuation_reliable_votes == 1)
    assert np.all(ref.continuation_last_stamp_ns <= 10*DT)


def test_underobserved_new_hypothesis_vetoes_existing_confirmed_branch():
    consensus = direct_then_subdivided()
    assert len(owner_permissions(consensus)[1])
    # A third actual successor in a new cell; keep original links and every face.
    for index in range(1, 5):
        obs = continuation_observation((10+index)*DT, 'subdivided')
        shift = np.array([[0., 0.], [0., 0.], [.1, 0.], [0., 0.]])
        obs = replace(obs, points=obs.points+shift, segments=obs.segments+shift[:, None],
                      position_estimates=obs.model_points+shift,
                      position_segments=obs.position_segments+shift[:, None],
                      position_footprints=obs.position_footprints+shift[:, None])
        commit(consensus, obs)
        assert not len(owner_permissions(consensus)[1])
    assert len(consensus._continuations[owner_id(consensus)]) == 3


def test_fifth_new_vote_and_reliable_vote_cannot_be_borrowed_before_commit():
    consensus = direct_then_subdivided(subdivided=4)
    identity = owner_id(consensus)
    before = state(consensus)
    pending = consensus.prepare(continuation_observation(10*DT, 'subdivided'))
    assert state(consensus) == before
    assert sorted(map(len, consensus._continuations[identity].values())) == [4, 5]
    assert consensus.source_frontier_ns == 9*DT
    assert consensus.commit(pending)
    assert len(owner_permissions(consensus)[1])
    assert consensus.snapshot().source_frontier_ns == 10*DT
    after = state(consensus)
    assert not consensus.commit(pending)
    assert state(consensus) == after


def test_one_hypothesis_reliability_cannot_be_borrowed_from_another():
    consensus = direct_then_subdivided(trusted=False)
    # New intermediary was untrusted; independently establish its node/edges
    # with one reliable frame. Only then can all-hypothesis geometry qualify.
    owner = owner_id(consensus)
    histories = consensus._continuations[owner]
    assert len(histories) == 2
    assert sorted(sum(bool(np.any(frame.reliable)) for frame in frames)
                  for frames in histories.values()) == [0, 5]
    assert not len(owner_permissions(consensus)[1])
    commit(consensus, continuation_observation(11*DT, 'subdivided'))
    assert len(owner_permissions(consensus)[1])


@pytest.mark.parametrize('change', ['gap', 'invalid_neighbor', 'legacy_missing'])
def test_observed_unknown_or_broken_continuation_veto_is_retained(change):
    consensus = direct_then_subdivided()
    assert len(owner_permissions(consensus)[1])
    options = {'gap': {'gap': True}, 'invalid_neighbor': {'model_invalid_neighbor': True},
               'legacy_missing': {'quads': False}}[change]
    commit(consensus, continuation_observation(11*DT, 'subdivided', **options))
    assert not len(owner_permissions(consensus)[1])
    history = consensus._continuations[owner_id(consensus)]
    newest = [frame for frames in history.values() for frame in frames if frame.stamp_ns == 11*DT]
    assert len(newest) == 1 and not len(newest[0].quads)


def test_discarded_alternative_and_stale_token_cannot_change_history_or_permission():
    consensus = direct_then_subdivided()
    before = state(consensus)
    stale = consensus.prepare(continuation_observation(11*DT, gap=True))
    assert state(consensus) == before
    chosen = consensus.prepare(continuation_observation(11*DT, 'subdivided'))
    assert state(consensus) == before
    assert consensus.commit(chosen)
    after = state(consensus)
    assert len(owner_permissions(consensus)[1])
    assert not consensus.commit(stale)
    assert state(consensus) == after
    assert not any(-1 in key for key in consensus._continuations[owner_id(consensus)])


def test_foreign_owner_token_and_nonincreasing_stamp_are_atomic():
    first, second = direct_then_subdivided(), direct_then_subdivided()
    before = state(second)
    token = first.prepare(continuation_observation(11*DT))
    assert not second.commit(token)
    with pytest.raises(ValueError, match='stamp'):
        second.prepare(continuation_observation(10*DT))
    assert state(second) == before


def test_permission_reclips_unchanged_old_world_quads_after_fused_mean_moves():
    consensus = direct_then_subdivided()
    ref0, spans0 = owner_permissions(consensus)
    assert len(spans0)
    identity = owner_id(consensus)
    history_before = frozen(consensus._continuations[identity])
    # Same rounded cell, but new fused line above the OLD quads' top y=.014.
    # One observation cannot grant support under the unchanged direct history.
    commit(consensus, continuation_observation(11*DT, 'subdivided', y=.049, quad_y=.049))
    ref1, spans1 = owner_permissions(consensus)
    old_slot = int(np.flatnonzero(ref0.record_ids == identity)[0])
    new_slot = int(np.flatnonzero(ref1.record_ids == identity)[0])
    assert ref1.points_xy[new_slot, 1] > ref0.points_xy[old_slot, 1]
    # For the old 10 observations, the 11th mean is .013545..., within .014.
    # A second equal committed observation moves it outside, without moving
    # the historical witnesses. This is independent of query-time decisions.
    commit(consensus, continuation_observation(12*DT, 'subdivided', y=.049, quad_y=.049))
    ref2, spans2 = owner_permissions(consensus)
    slot = int(np.flatnonzero(ref2.record_ids == identity)[0])
    assert ref2.points_xy[slot, 1] > .014
    assert not len(spans2)
    direct_key = next(key for key, frames in consensus._continuations[identity].items()
                      if all(frame.stamp_ns <= 5*DT for frame in frames))
    direct_frames = consensus._continuations[identity][direct_key]
    assert all(np.max(frame.quads[:, :, 1]) == .014 for frame in direct_frames)
    assert history_before != frozen(consensus._continuations[identity])


def test_new_history_capacity_counts_hypotheses_empty_frames_and_every_owned_quad():
    consensus = BoundaryConsensus()
    token = consensus.prepare(continuation_observation(DT))
    histories = dict(token.continuation_updates)
    # P and C are missing one neighbour: their empty frames still cost metadata.
    # A owns one qualified quad: 1 hypothesis + 1 frame + 1 quad.
    assert token.continuation_storage_count == 2 + 3 + 2
    measured = sum(1 + sum(1+len(frame.quads) for frame in frames)
                   for hypotheses in histories.values() for frames in hypotheses.values())
    assert measured == token.continuation_storage_count
    assert consensus._continuation_storage_count == 0 and not consensus._continuations
    assert consensus.commit(token)
    assert consensus._continuation_storage_count == measured


def test_capacity_failure_leaves_all_original_and_new_state_unchanged():
    legacy = BoundaryConsensus()
    legacy_token = legacy.prepare(continuation_observation(DT, quads=False))
    # Exact existing contribution, before any continuation sidecar is included.
    legacy_units = max(legacy_token.segment_count, len(legacy_token.edge_updates)) + legacy_token.domain_interval_count
    bounded = BoundaryConsensus({'max_support_segments': legacy_units})
    before = state(bounded)
    with pytest.raises(ValueError, match='continuation capacity'):
        bounded.prepare(continuation_observation(DT))
    assert state(bounded) == before

    # A failed prepare must not apply expiry or debit the new cached bounds.
    # At 14 units the three-node first frame fits exactly after exact export
    # checking; the four-node alternative exceeds the existing storage cap.
    expiring = BoundaryConsensus({'max_support_segments': 14, 'candidate_ttl_s': .05})
    commit(expiring, continuation_observation(DT))
    old_ids = set(expiring._records)
    before_expiry = state(expiring)
    assert expiring._permission_bound_count > 0
    with pytest.raises(ValueError, match='capacity'):
        expiring.prepare(continuation_observation(3*DT, 'subdivided'))
    assert state(expiring) == before_expiry
    replacement = expiring.prepare(continuation_observation(3*DT))
    assert set(replacement.expired_ids) == old_ids
    assert state(expiring) == before_expiry
    assert expiring.commit(replacement)
    assert not old_ids.intersection(expiring._records)
    assert not old_ids.intersection(expiring._permission_bounds)
    assert expiring._permission_bound_count == sum(expiring._permission_bounds.values())


def test_legacy_observation_path_creates_no_continuation_sidecar_or_new_votes():
    consensus = BoundaryConsensus()
    for frame in range(1, 6):
        token = commit(consensus, continuation_observation(frame*DT, quads=False))
        assert token.continuation_updates == () and token.continuation_storage_count == 0
        assert token.permission_bound_updates == () and token.permission_bound_count == 0
    ref = consensus.snapshot()
    assert ref.boundary_version == 3
    assert not consensus._continuations and consensus._continuation_storage_count == 0
    assert not consensus._permission_bounds and consensus._permission_bound_count == 0
    assert all(getattr(ref, name) is None for name in CONTINUATION_FIELDS)


def test_snapshot_and_local_snapshot_do_not_mutate_retained_witnesses():
    consensus = direct_then_subdivided()
    before = state(consensus)
    entire = consensus.snapshot()
    local = consensus.snapshot_near([0., .01], .1, 100)
    assert state(consensus) == before
    assert local.full_out_degree.tolist() == [2]
    assert not len(local.directed_edges)
    assert len(local.continuation_intervals)
    assert local.source_frontier_ns == entire.source_frontier_ns
    for name in CONTINUATION_FIELDS:
        with pytest.raises(ValueError):
            getattr(local, name).setflags(write=True)


@pytest.mark.parametrize('capacity', [59, 60])
def test_permission_export_capacity_is_reserved_before_commit_without_overreservation(capacity):
    """Actual reviewed 4 raw/4 edge + 4 domains + 51 history + 1 permission.

    The old implementation admitted frame ten with cap 59 and only failed on
    its already-committed snapshot. Capacity 60 is exactly sufficient for the
    SAME analytical observations; a loose blanket reserve must not reject it.
    """
    consensus = direct_then_subdivided(subdivided=4,
        config={'max_support_segments': capacity})
    before = state(consensus)
    assert consensus.source_frontier_ns == 9*DT
    assert consensus._generation == 9
    final = continuation_observation(10*DT, 'subdivided')
    if capacity == 59:
        with pytest.raises(ValueError, match='capacity'):
            consensus.prepare(final)
        assert state(consensus) == before
        assert consensus.source_frontier_ns == 9*DT
        assert consensus._generation == 9
        return
    prepared = consensus.prepare(final)
    assert state(consensus) == before
    assert consensus.commit(prepared)
    reference_after = consensus.snapshot()
    assert consensus.source_frontier_ns == 10*DT
    assert consensus._generation == 10
    assert consensus._segment_count == len(consensus._edges) == 4
    assert consensus._domain_interval_count == 4
    assert consensus._continuation_storage_count == 51
    assert len(reference_after.continuation_intervals) == 1
    assert (max(consensus._segment_count, len(consensus._edges))
            + consensus._domain_interval_count + consensus._continuation_storage_count
            + len(reference_after.continuation_intervals)) == 60

