"""R17 public consensus transactions with actual directed model-next evidence.

These are explicit finite model-observation units, not camera accuracy,
localization, runtime, or driving tests. Original 5+1 thresholds are retained.
Private state is read only for transaction/immutable-chart verification.
"""
from dataclasses import replace
from fractions import Fraction

import numpy as np
import pytest

from lane_mapping_lya_reference.paint_geometry import (
    BoundaryConsensus, BoundaryObservation, CERTIFIED_MODEL_FIELDS,
)


DT = 100_000_000
BASE = np.array([[-.03125, .01], [0., .01], [.03125, .01]])


def observation(stamp, points=BASE, *, links=None, reliable=True,
                model_valid=None, model_j=None, normals=None, raw_points=None,
                raw_segments=None):
    """Construct original raw faces plus explicitly distinct model geometry.

    Missing/invalid model-next endpoints never gain an invented shortcut.
    All positions are analytical test inputs; no future map or pose is used.
    """
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    n = len(points)
    raw = points.copy() if raw_points is None else np.asarray(raw_points, dtype=np.float64)
    directions = (np.tile((0., 1.), (n, 1)) if normals is None else
                  np.asarray(normals, dtype=np.float64).reshape(n, 2))
    faces = (np.stack((raw - [.002, 0.], raw + [.002, 0.]), axis=1)
             if raw_segments is None else np.asarray(raw_segments, dtype=np.float64))
    previous = np.full(n, -1, dtype=np.int64)
    following = previous.copy()
    pairs = list(zip(range(n-1), range(1, n))) if links is None else links
    for first, second in pairs:
        assert following[first] == previous[second] == -1
        following[first], previous[second] = second, first
    contour = np.full(n, -1, dtype=np.int64)
    label = 0
    for seed in list(np.flatnonzero(previous < 0)) + list(range(n)):
        current = int(seed)
        if contour[current] >= 0:
            continue
        while current >= 0 and contour[current] < 0:
            contour[current] = label
            current = int(following[current])
        label += 1
    true = np.ones(n, dtype=bool)
    valid = true.copy() if model_valid is None else np.asarray(model_valid, dtype=bool)
    trusted = np.broadcast_to(np.asarray(reliable, dtype=bool), (n,)).copy()
    sensitivity = np.full(n, .02) if model_j is None else np.asarray(model_j, dtype=np.float64)
    pixels = np.column_stack((np.arange(n, dtype=np.int64), np.zeros(n, dtype=np.int64)))
    return BoundaryObservation(
        points=raw, normals=directions, segments=faces, sensitivity=np.full(n, .02),
        metric_valid=true, stable=true, normal_valid=true, valid=true, reliable=true,
        source_stamp_ns=stamp, pixel_segments=faces, white_pixels=pixels,
        black_pixels=pixels + [0, 1], previous=previous, next=following,
        contour_ids=contour, reason_codes=np.zeros(n, dtype=np.int64),
        normal_support_count=np.full(n, 6, dtype=np.int64), normal_radius=np.full(n, .45),
        position_estimates=points, position_valid=valid,
        position_sensitivity=sensitivity, position_reliable=trusted & valid,
        position_normals=directions,
    )


def commit(consensus, obs):
    prepared = consensus.prepare(obs)
    assert consensus.commit(prepared)
    return prepared


def history(*, frames=5, config=None, **options):
    consensus = BoundaryConsensus(config)
    for frame in range(1, frames+1):
        commit(consensus, observation(frame*DT, **options))
    return consensus


def fingerprint(consensus):
    reference = consensus.snapshot()
    names = ('points_xy', 'normals', 'record_ids', 'votes', 'reliable_votes',
             'directed_edges', 'edge_frame_votes', 'edge_last_stamp_ns') + CERTIFIED_MODEL_FIELDS
    return (consensus._generation, consensus.source_frontier_ns, consensus._next_record_id,
            consensus._domain_interval_count, tuple(consensus._domains.items()),
            tuple((name, getattr(reference, name).tobytes()) for name in names))


def covered(reference, x):
    segments = reference.certified_model_segments
    return any(min(segment[:, 0]) <= x <= max(segment[:, 0]) for segment in segments)


def assert_owner_evidence(reference):
    owners = reference.certified_model_owner
    assert np.all(reference.certified_model_frame_votes >= 5)
    assert np.all(reference.certified_model_reliable_votes >= 1)
    assert np.all(reference.certified_model_frame_votes <= reference.votes[owners])
    assert np.all(reference.certified_model_reliable_votes <= reference.reliable_votes[owners])
    assert np.all(reference.certified_model_last_stamp_ns <= reference.source_frontier_ns)


def test_empty_snapshot_has_explicit_immutable_v3_arrays():
    reference = BoundaryConsensus().snapshot()
    assert reference.boundary_version == 3
    assert reference.certified_model_representation == 'confirmed_creation_chart_domains_v1'
    assert reference.certified_model_segments.shape == (0, 2, 2)
    for name in CERTIFIED_MODEL_FIELDS:
        array = getattr(reference, name)
        assert not array.flags.writeable
        with pytest.raises(ValueError):
            array.setflags(write=True)


def test_fifth_frame_does_not_exist_in_reference_until_successful_commit():
    consensus = history(frames=4)
    before = fingerprint(consensus)
    fifth = consensus.prepare(observation(5*DT))
    assert fingerprint(consensus) == before
    assert not len(consensus.snapshot().certified_model_segments)
    assert consensus.commit(fifth)
    reference = consensus.snapshot()
    assert covered(reference, .02)
    assert_owner_evidence(reference)
    assert np.all(reference.certified_model_last_stamp_ns == 5*DT)


def test_prepared_future_domain_does_not_self_support_current_reference():
    consensus = history()
    before = fingerprint(consensus)
    shifted = BASE + [.01, 0.]
    prepared = consensus.prepare(observation(6*DT, points=shifted))
    assert fingerprint(consensus) == before
    assert consensus.snapshot().source_frontier_ns == 5*DT
    assert not covered(consensus.snapshot(), .035)
    assert all(domain.source_stamp_ns == 6*DT for _, domain in prepared.domain_updates)
    assert all(domain.source_stamp_ns == 5*DT for domain in consensus._domains.values())


def test_node_confirmation_cannot_substitute_for_pair_reliability():
    consensus = history(reliable=[True, False, True])
    assert consensus.confirmed_count == 1
    assert consensus.snapshot().reliable_votes.tolist() == [5]
    assert not len(consensus.snapshot().certified_model_segments)
    commit(consensus, observation(6*DT))
    reference = consensus.snapshot()
    assert covered(reference, .02)
    assert_owner_evidence(reference)
    assert np.all(reference.certified_model_reliable_votes == 1)


def test_reliable_pair_subdomain_does_not_certify_other_half():
    consensus = history(reliable=[True, True, False])
    reference = consensus.snapshot()
    assert covered(reference, -.02)
    assert not covered(reference, .02)
    assert_owner_evidence(reference)


def test_many_same_frame_internal_links_do_not_multiply_votes():
    points = np.column_stack((np.linspace(-.04, .04, 21), np.full(21, .01)))
    consensus = BoundaryConsensus()
    prepared = commit(consensus, observation(DT, points=points))
    assert len(prepared.updates) == 1
    domain = next(iter(consensus._domains.values()))
    assert domain.intervals
    assert all(piece.frame_votes == piece.reliable_votes == 1 for piece in domain.intervals)
    assert not len(consensus.snapshot().certified_model_segments)


def test_raw_face_hull_without_actual_next_never_becomes_model_domain():
    raw = np.stack((BASE - [.4, 0.], BASE + [.4, 0.]), axis=1)
    consensus = history(links=(), raw_segments=raw)
    assert consensus.confirmed_count == 3
    assert all(not state.intervals for state in consensus._domains.values())
    assert not len(consensus.snapshot().certified_model_segments)


def test_cross_record_next_confirms_incident_support_without_extra_graph_votes():
    points = BASE + [[-.1, 0.], [0., 0.], [.1, 0.]]
    reference = history(points=points).snapshot()
    assert len(reference.points_xy) == 3
    assert len(reference.directed_edges) == 2
    assert np.all(reference.edge_frame_votes == 5)
    assert len(reference.certified_model_segments) == 3
    assert np.all(reference.certified_model_frame_votes == 5)
    assert np.all(reference.certified_model_reliable_votes == 1)
    assert_owner_evidence(reference)


def test_domain_uses_model_next_positions_not_raw_centers_or_raw_face_endpoints():
    raw_points = BASE + [2., 3.]
    raw_segments = np.stack((raw_points - [.3, 0.], raw_points + [.3, 0.]), axis=1)
    reference = history(raw_points=raw_points, raw_segments=raw_segments).snapshot()
    assert covered(reference, .02)
    assert np.max(np.abs(reference.certified_model_segments[:, :, 0])) <= .03125
    np.testing.assert_array_equal(reference.certified_model_segments[:, :, 1],
                                  np.full(reference.certified_model_segments.shape[:2], .01))


@pytest.mark.parametrize('invalid', ['position', 'sensitivity'])
def test_invalid_model_or_j_cut_never_invents_shortcut(invalid):
    options = ({'model_valid': [True, False, True]} if invalid == 'position' else
               {'model_j': [.02, .6, .02]})
    consensus = history(**options)
    assert consensus.confirmed_count == 2
    assert not len(consensus.snapshot().directed_edges)
    assert not len(consensus.snapshot().certified_model_segments)


def test_opposite_signed_endpoint_is_not_merged_into_one_domain():
    reference = history(normals=[[0., 1.], [0., -1.], [0., 1.]]).snapshot()
    assert len(reference.points_xy) == 3
    assert not len(reference.certified_model_segments)


def test_two_temporal_same_record_pieces_keep_real_unobserved_gap():
    left = np.array([[-.04, .01], [-.025, .01]])
    right = np.array([[.025, .01], [.04, .01]])
    consensus = history(points=left)
    for frame in range(6, 11):
        commit(consensus, observation(frame*DT, points=right))
    reference = consensus.snapshot()
    assert len(reference.points_xy) == 1
    assert covered(reference, -.03) and covered(reference, .03)
    assert not covered(reference, 0.)
    assert_owner_evidence(reference)


def test_stale_and_foreign_tokens_cannot_publish_domain_updates():
    consensus = history()
    stale = consensus.prepare(observation(6*DT))
    later = consensus.prepare(observation(7*DT))
    assert consensus.commit(later)
    before = fingerprint(consensus)
    assert not consensus.commit(stale)
    foreign = BoundaryConsensus().prepare(observation(8*DT))
    assert not consensus.commit(foreign)
    assert fingerprint(consensus) == before


def test_duplicate_source_stamp_rejected_without_any_domain_mutation():
    consensus = history()
    before = fingerprint(consensus)
    with pytest.raises(ValueError, match='stamp'):
        consensus.prepare(observation(5*DT))
    assert fingerprint(consensus) == before


def test_capacity_rejection_leaves_nodes_domains_and_frontier_untouched():
    # Three original faces alone fit; adding their real positive finite-domain
    # piece exceeds the combined budget. Do not truncate the new evidence.
    consensus = BoundaryConsensus({'max_support_segments': 3})
    before = fingerprint(consensus)
    with pytest.raises(ValueError, match='finite-domain capacity'):
        consensus.prepare(observation(DT))
    assert fingerprint(consensus) == before
    assert not consensus._records and not consensus._domains


def test_expired_candidate_owner_removes_its_domain_atomically():
    consensus = history(frames=1, config={'candidate_ttl_s': .1})
    before = fingerprint(consensus)
    token = consensus.prepare(observation(4*DT, points=np.empty((0, 2))))
    assert token.expired_ids and fingerprint(consensus) == before
    assert consensus.commit(token)
    assert not consensus._domains and consensus._domain_interval_count == 0


def test_confirmed_domain_survives_empty_frame_without_new_last_observed_stamp():
    consensus = history(config={'candidate_ttl_s': .1})
    reference = consensus.snapshot()
    commit(consensus, observation(20*DT, points=np.empty((0, 2))))
    after = consensus.snapshot()
    assert after.source_frontier_ns == 20*DT
    for name in CERTIFIED_MODEL_FIELDS:
        np.testing.assert_array_equal(getattr(after, name), getattr(reference, name))


def test_creation_chart_stays_fixed_when_committed_fused_mean_moves():
    consensus = history()
    owner = next(iter(consensus._domains))
    old = consensus._domains[owner]
    old_mean = consensus.snapshot().points_xy.copy()
    commit(consensus, observation(6*DT, points=BASE + [.01, 0.]))
    current = consensus._domains[owner]
    reference = consensus.snapshot()
    assert current.origin == old.origin and current.tangent == old.tangent
    assert reference.points_xy[0, 0] > old_mean[0, 0]
    assert covered(reference, -.02) and covered(reference, .02)
    assert not covered(reference, .035)  # New extension has only one vote.
    for segment in reference.certified_model_segments:
        for point in segment:
            coordinate = sum((Fraction(float(p))-Fraction(o))*Fraction(t)
                             for p, o, t in zip(point, old.origin, old.tangent))
            assert any(Fraction(piece.lower) <= coordinate <= Fraction(piece.upper)
                       for piece in old.intervals)


def test_empty_creation_chart_is_not_replaced_by_later_internal_geometry():
    consensus = history(frames=1, points=[[0., .01]])
    owner = next(iter(consensus._domains))
    original = consensus._domains[owner]
    assert not original.intervals
    commit(consensus, observation(2*DT, points=BASE + [.005, 0.]))
    current = consensus._domains[owner]
    assert current.origin == original.origin and current.tangent == original.tangent
    assert current.intervals and all(piece.frame_votes == 1 for piece in current.intervals)


def test_frozen_reference_does_not_track_later_domain_or_node_updates():
    consensus = history()
    reference = consensus.snapshot()
    saved = {name: getattr(reference, name).tobytes() for name in CERTIFIED_MODEL_FIELDS}
    commit(consensus, observation(6*DT, points=BASE + [.01, 0.]))
    assert reference.source_frontier_ns == 5*DT
    for name, content in saved.items():
        assert getattr(reference, name).tobytes() == content
        with pytest.raises(ValueError):
            getattr(reference, name).setflags(write=True)


def test_subset_preserves_certificates_and_remaps_only_owners():
    points = np.vstack((BASE, BASE + [.2, 0.]))
    consensus = history(points=points, links=((0, 1), (1, 2), (3, 4), (4, 5)))
    reference = consensus.snapshot()
    assert len(reference.points_xy) == 2
    subset = reference.subset([1])
    keep = reference.certified_model_owner == 1
    np.testing.assert_array_equal(subset.certified_model_owner, np.zeros(np.count_nonzero(keep), dtype=np.int64))
    for name in CERTIFIED_MODEL_FIELDS:
        if name != 'certified_model_owner':
            np.testing.assert_array_equal(getattr(subset, name), getattr(reference, name)[keep])
    assert subset.source_frontier_ns == reference.source_frontier_ns


def test_snapshot_near_matches_full_snapshot_certificate_subset_exactly():
    points = np.vstack((BASE, BASE + [.2, 0.]))
    consensus = history(points=points, links=((0, 1), (1, 2), (3, 4), (4, 5)))
    full = consensus.snapshot()
    near = consensus.snapshot_near((.2, .01), .05, 10)
    expected = full.subset([1])
    for name in CERTIFIED_MODEL_FIELDS:
        np.testing.assert_array_equal(getattr(near, name), getattr(expected, name))
    np.testing.assert_array_equal(near.record_ids, expected.record_ids)


@pytest.mark.parametrize('field', ['certified_model_last_stamp_ns',
                                  'certified_model_frame_votes',
                                  'certified_model_reliable_votes'])
def test_frozen_reference_rejects_future_or_owner_exceeding_evidence(field):
    reference = history().snapshot()
    wrong = np.full(len(reference.certified_model_owner),
                    6*DT if field.endswith('stamp_ns') else 6, dtype=np.int64)
    with pytest.raises(ValueError):
        replace(reference, **{field: wrong})


def extended_confirmed_owner(config=None):
    values={'candidate_ttl_s': .1}
    if config:
        values.update(config)
    consensus=history(config=values)
    commit(consensus,observation(6*DT,points=BASE+[.01,0.]))
    owner=next(iter(consensus._domains))
    assert any(piece.frame_votes<5 for piece in consensus._domains[owner].intervals)
    return consensus,owner


def assert_expiry_index(consensus):
    expected={}
    for owner,domain in consensus._domains.items():
        pending=[piece.last_stamp_ns for piece in domain.intervals
                 if piece.frame_votes<5 or piece.reliable_votes<1]
        if pending:
            expected[owner]=min(pending)
    assert consensus._domain_expiry==expected
    assert consensus._domain_interval_count==sum(len(domain.intervals) for domain in consensus._domains.values())


def test_unseen_confirmed_owner_pending_domain_expiry_is_transactional():
    consensus,owner=extended_confirmed_owner()
    before=fingerprint(consensus)
    old_index=dict(consensus._domain_expiry)
    pending=consensus.prepare(observation(8*DT,points=np.empty((0,2))))
    assert fingerprint(consensus)==before and consensus._domain_expiry==old_index
    assert pending.domain_updates and owner not in pending.expired_ids
    # Discarding this token cannot erase even one old interval.
    assert any(piece.frame_votes<5 for piece in consensus._domains[owner].intervals)
    assert consensus.commit(pending)
    assert owner in consensus._records and consensus.confirmed_count==1
    assert all(piece.frame_votes>=5 and piece.reliable_votes>=1
               for piece in consensus._domains[owner].intervals)
    assert owner not in consensus._domain_expiry
    assert_expiry_index(consensus)


def test_pending_domain_ttl_equality_retains_then_one_nanosecond_expires():
    consensus,owner=extended_confirmed_owner()
    old_domain=consensus._domains[owner]
    assert consensus._domain_expiry[owner]==6*DT
    commit(consensus,observation(7*DT,points=np.empty((0,2))))
    assert consensus._domains[owner] is old_domain
    assert_expiry_index(consensus)
    commit(consensus,observation(7*DT+1,points=np.empty((0,2))))
    assert len(consensus._domains[owner].intervals)<len(old_domain.intervals)
    assert_expiry_index(consensus)


def test_stale_expiry_token_does_not_change_frontier_index_or_domain():
    consensus,owner=extended_confirmed_owner()
    expiry=consensus.prepare(observation(8*DT,points=np.empty((0,2))))
    earlier=consensus.prepare(observation(7*DT,points=np.empty((0,2))))
    assert consensus.commit(earlier)
    before=fingerprint(consensus);index=dict(consensus._domain_expiry)
    assert not consensus.commit(expiry)
    assert fingerprint(consensus)==before and consensus._domain_expiry==index
    assert owner in consensus._domain_expiry


def test_expiry_reclaims_combined_capacity_for_different_current_owner():
    consensus,owner=extended_confirmed_owner({'max_support_segments':8})
    count=consensus._domain_interval_count
    before=fingerprint(consensus)
    token=consensus.prepare(observation(8*DT,points=[[.2,.01]]))
    assert fingerprint(consensus)==before
    assert token.domain_interval_count<count
    assert consensus.commit(token)
    assert owner in consensus._records and len(consensus._records)==2
    assert max(consensus._segment_count,len(consensus._edges))+consensus._domain_interval_count<=8
    assert_expiry_index(consensus)


def test_zero_ttl_retains_unseen_owner_pending_domain_and_expiry_index():
    consensus,owner=extended_confirmed_owner({'candidate_ttl_s':0.})
    domain=consensus._domains[owner]
    index=dict(consensus._domain_expiry)
    commit(consensus,observation(1000*DT,points=np.empty((0,2))))
    assert consensus._domains[owner] is domain
    assert consensus._domain_expiry==index
    assert_expiry_index(consensus)


def test_expired_owner_removes_its_expiry_index_entry_as_well_as_domain():
    consensus=history(frames=1,config={'candidate_ttl_s':.1})
    owner=next(iter(consensus._domains))
    assert owner in consensus._domain_expiry
    commit(consensus,observation(4*DT,points=np.empty((0,2))))
    assert owner not in consensus._domains and owner not in consensus._domain_expiry
    assert_expiry_index(consensus)
