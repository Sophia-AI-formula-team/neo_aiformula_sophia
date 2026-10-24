"""Direct-incident finite-position evidence, not inferred graph connectivity.

Public prepare/commit inputs retain the original five-frame/one-reliable rules.
Analytical model observations isolate consensus; they are not camera, pose,
timing, or driving evidence. Private state is inspected, never overwritten.
The shallow-angle example deliberately documents the fitted-model limitation
shared by the old internal path, rather than asserting physical ground truth.
"""
from fractions import Fraction
import math

import numpy as np
import pytest

from lane_mapping_lya_reference.paint_geometry import BoundaryConsensus
from test_boundary_domain_consensus import observation


DT = 100_000_000
PAIR = np.array([[0., .01], [.2, .01]])


def commit(consensus, obs):
    token = consensus.prepare(obs)
    assert consensus.commit(token)
    return token


def history(frames=5, *, consensus=None, points=PAIR, **options):
    consensus = BoundaryConsensus() if consensus is None else consensus
    for frame in range(1, frames + 1):
        commit(consensus, observation(frame * DT, points=points, **options))
    return consensus


def owner_at(consensus, point):
    matches = [record_id for record_id, record in consensus._records.items()
               if np.array_equal(record.point, point)]
    assert len(matches) == 1
    return matches[0]


def parameter(point, state):
    return sum((Fraction(float(value)) - Fraction(origin)) * Fraction(tangent)
               for value, origin, tangent in zip(point, state.origin, state.tangent))


def domain_evidence(consensus, owner, point):
    state = consensus._domains[owner]
    value = parameter(point, state)
    return [piece for piece in state.intervals
            if Fraction(piece.lower) <= value <= Fraction(piece.upper)]


def certificates(reference, owner):
    slots = np.flatnonzero(reference.record_ids == owner)
    if not len(slots):
        return np.empty((0, 2, 2))
    assert len(slots) == 1
    return reference.certified_model_segments[reference.certified_model_owner == slots[0]]


def fingerprint(consensus):
    reference = consensus.snapshot()
    arrays = tuple((name, value.dtype.str, value.shape, value.tobytes())
                   for name, value in reference.__dict__.items() if isinstance(value, np.ndarray))
    records = tuple((key, value.cell, value.point, value.normal, value.votes,
                     value.reliable_votes, value.last_stamp_ns, value.segments.tobytes(),
                     value.neighbours.tobytes()) for key, value in consensus._records.items())
    return (consensus._generation, consensus.source_frontier_ns, consensus._next_record_id,
            records, tuple(consensus._edges.items()), tuple(consensus._domains.items()),
            tuple(consensus._domain_expiry.items()), consensus._domain_interval_count,
            tuple((cell, tuple(ids)) for cell, ids in consensus._cell_records.items()), arrays)


def test_original_thresholds_and_only_actual_fifth_commit_confirm_both_owners():
    consensus = history(frames=4, reliable=False)
    assert consensus.config['minimum_frame_votes'] == 5
    assert consensus.config['minimum_reliable_votes'] == 1
    before = fingerprint(consensus)
    fifth = consensus.prepare(observation(5 * DT, points=PAIR))
    assert fingerprint(consensus) == before
    assert not len(consensus.snapshot().certified_model_segments)
    assert consensus.commit(fifth)
    reference = consensus.snapshot()
    assert len(reference.record_ids) == 2
    assert len(reference.directed_edges) == 1
    np.testing.assert_array_equal(reference.edge_frame_votes, [5])
    np.testing.assert_array_equal(reference.edge_reliable_votes, [1])
    for point in PAIR:
        owner = owner_at(consensus, point)
        evidence = domain_evidence(consensus, owner, [.1, .01])
        assert evidence and all((p.frame_votes, p.reliable_votes) == (5, 1) for p in evidence)
        assert len(certificates(reference, owner))
    assert np.all(reference.certified_model_last_stamp_ns == 5 * DT)


def test_rotating_neighbour_ids_certify_position_without_confirming_any_graph_edge():
    consensus = BoundaryConsensus()
    neighbour_ids = []
    for frame, x in enumerate((.2, .35, .5, .65, .8), 1):
        commit(consensus, observation(frame * DT, points=[[0., .01], [x, .01]]))
        neighbour_ids.append(owner_at(consensus, [x, .01]))
    owner = owner_at(consensus, [0., .01])
    assert len(set(neighbour_ids)) == 5
    assert len(consensus._edges) == 5
    assert {(edge.source_id, edge.target_id) for edge in consensus._edges.values()} == {
        (owner, other) for other in neighbour_ids}
    assert all(edge.votes == edge.reliable_votes == 1 for edge in consensus._edges.values())
    reference = consensus.snapshot()
    np.testing.assert_array_equal(reference.record_ids, [owner])
    assert not len(reference.directed_edges)
    np.testing.assert_array_equal(reference.full_in_degree, [0])
    np.testing.assert_array_equal(reference.full_out_degree, [0])
    assert len(certificates(reference, owner))
    evidence = domain_evidence(consensus, owner, [.1, .01])
    assert evidence and all((p.frame_votes, p.reliable_votes) == (5, 1) for p in evidence)
    # Five observations of .1 do not give five observations of a later extension.
    far = domain_evidence(consensus, owner, [.7, .01])
    assert far and all(p.frame_votes == 1 for p in far)


def test_direct_incidence_never_propagates_through_the_next_owner():
    points = np.array([[0., .01], [.2, .01], [.4, .01]])
    consensus = history(points=points)
    first, middle, last = (owner_at(consensus, point) for point in points)
    assert domain_evidence(consensus, first, [.1, .01])
    assert not domain_evidence(consensus, first, [.3, .01])
    assert domain_evidence(consensus, middle, [.1, .01])
    assert domain_evidence(consensus, middle, [.3, .01])
    assert not domain_evidence(consensus, last, [.1, .01])
    assert domain_evidence(consensus, last, [.3, .01])
    reference = consensus.snapshot()
    assert len(reference.directed_edges) == 2
    np.testing.assert_array_equal(reference.edge_frame_votes, [5, 5])
    np.testing.assert_array_equal(reference.edge_reliable_votes, [5, 5])


def test_no_next_links_cannot_turn_nearby_or_large_raw_faces_into_a_domain():
    raw_faces = np.array([[[-.5, .01], [.5, .01]], [[-.3, .01], [.7, .01]]])
    consensus = history(links=(), raw_segments=raw_faces)
    assert consensus.confirmed_count == 2
    assert not consensus._edges
    assert all(not state.intervals for state in consensus._domains.values())
    assert not len(consensus.snapshot().certified_model_segments)


@pytest.mark.parametrize('invalid', ['position', 'model_j', 'raw_j'])
def test_invalid_middle_face_does_not_create_incident_shortcut(invalid):
    from dataclasses import replace
    consensus = BoundaryConsensus()
    points = [[0., .01], [.1, .01], [.2, .01]]
    for frame in range(1, 6):
        options = {'model_valid': [True, False, True]} if invalid == 'position' else {}
        if invalid == 'model_j':
            options['model_j'] = [.02, .6, .02]
        obs = observation(frame * DT, points=points, **options)
        if invalid == 'raw_j':
            obs = replace(obs, sensitivity=np.array([.02, .6, .02]))
        commit(consensus, obs)
    assert consensus.confirmed_count == 2
    assert not consensus._edges
    assert all(not state.intervals for state in consensus._domains.values())


@pytest.mark.parametrize('unreliable_end', [0, 1])
@pytest.mark.parametrize('cause', ['reliable_flag', 'j_above_reliable'])
def test_one_reliable_endpoint_never_grants_a_reliable_segment_vote(unreliable_end, cause):
    options = {}
    if cause == 'reliable_flag':
        flags = [True, True]
        flags[unreliable_end] = False
        options['reliable'] = flags
    else:
        sensitivity = [.02, .02]
        sensitivity[unreliable_end] = np.nextafter(.1, np.inf)
        options['model_j'] = sensitivity
    consensus = history(**options)
    assert all(edge.votes == 5 and edge.reliable_votes == 0 for edge in consensus._edges.values())
    for point in PAIR:
        evidence = domain_evidence(consensus, owner_at(consensus, point), [.1, .01])
        assert evidence and all((p.frame_votes, p.reliable_votes) == (5, 0) for p in evidence)
    assert not len(consensus.snapshot().certified_model_segments)
    commit(consensus, observation(6 * DT, points=PAIR))
    reference = consensus.snapshot()
    assert len(reference.certified_model_segments)
    assert np.all(reference.certified_model_reliable_votes == 1)
    np.testing.assert_array_equal(reference.edge_frame_votes, [6])
    np.testing.assert_array_equal(reference.edge_reliable_votes, [1])


def test_opposite_normals_keep_original_graph_votes_but_supply_no_incident_domain():
    consensus = history(normals=[[0., 1.], [0., -1.]])
    assert len(consensus._edges) == 1
    edge = next(iter(consensus._edges.values()))
    assert edge.votes == edge.reliable_votes == 5
    assert all(not state.intervals for state in consensus._domains.values())
    assert not len(consensus.snapshot().certified_model_segments)


def normal(degrees):
    radians = math.radians(degrees)
    return np.array([-math.sin(radians), math.cos(radians)])


def test_mutual_normal_gate_is_required_even_when_both_fit_the_creation_chart():
    consensus = history(frames=1, links=())
    directions = np.array([normal(-20.), normal(20.)])
    cosine = math.cos(consensus.config['normal_max_angle_rad'])
    assert all(np.dot(value, [0., 1.]) > cosine for value in directions)
    assert np.dot(directions[0], directions[1]) < cosine
    for frame in range(2, 7):
        commit(consensus, observation(frame * DT, points=PAIR, normals=directions))
    assert len(consensus._records) == 2
    assert all(edge.votes == 5 for edge in consensus._edges.values())
    assert all(not state.intervals for state in consensus._domains.values())


def test_compatible_current_pair_cannot_replace_the_immutable_creation_chart():
    consensus = BoundaryConsensus()
    for frame, angle in enumerate((0., 15., 29., 44.), 1):
        commit(consensus, observation(frame * DT, points=PAIR,
            links=() if frame < 4 else [(0, 1)], normals=[normal(angle)] * 2))
    assert len(consensus._records) == 2  # All original identities really survived.
    assert len(consensus._edges) == 1
    assert all(not state.intervals for state in consensus._domains.values())
    for state in consensus._domains.values():
        assert state.tangent == (1., -0.)


def test_same_frame_reciprocal_links_are_one_position_vote_not_two():
    consensus = BoundaryConsensus()
    commit(consensus, observation(DT, points=PAIR, links=[(0, 1), (1, 0)]))
    assert len(consensus._edges) == 2
    assert all(edge.votes == edge.reliable_votes == 1 for edge in consensus._edges.values())
    for point in PAIR:
        evidence = domain_evidence(consensus, owner_at(consensus, point), [.1, .01])
        assert evidence and all((p.frame_votes, p.reliable_votes) == (1, 1) for p in evidence)
    assert not len(consensus.snapshot().certified_model_segments)


def test_real_temporal_gap_between_incident_spans_remains_uncovered():
    consensus = history(points=[[-.2, .01], [-.1, .01]])
    for frame in range(6, 11):
        commit(consensus, observation(frame * DT, points=[[.1, .01], [.2, .01]]))
    reference = consensus.snapshot()
    assert len(reference.directed_edges) == 2
    assert len(reference.certified_model_segments)
    for segment in reference.certified_model_segments:
        assert not min(segment[:, 0]) <= 0. <= max(segment[:, 0])


def test_confirmed_branches_still_have_their_original_two_edge_histories():
    consensus = history(points=[[0., .01], [.2, .04]])
    for frame in range(6, 11):
        commit(consensus, observation(frame * DT, points=[[0., .01], [.4, -.05]]))
    owner = owner_at(consensus, [0., .01])
    reference = consensus.snapshot()
    slot = int(np.flatnonzero(reference.record_ids == owner)[0])
    assert len(reference.directed_edges) == 2
    np.testing.assert_array_equal(reference.edge_frame_votes, [5, 5])
    np.testing.assert_array_equal(reference.edge_reliable_votes, [5, 5])
    assert reference.full_out_degree[slot] == 2
    # Keep branch geometry for the existing global-nearest blocker; do not
    # delete it and thereby let a farther unrelated primitive win.
    assert len(certificates(reference, owner))


def test_duplicate_stale_foreign_and_unused_tokens_cannot_publish_support():
    consensus = history(frames=4)
    before = fingerprint(consensus)
    unused = consensus.prepare(observation(5 * DT, points=PAIR))
    later = consensus.prepare(observation(6 * DT, points=PAIR))
    assert fingerprint(consensus) == before
    assert consensus.commit(later)
    committed = fingerprint(consensus)
    assert not consensus.commit(unused)
    assert not consensus.commit(later)
    foreign = BoundaryConsensus().prepare(observation(7 * DT, points=PAIR))
    assert not consensus.commit(foreign)
    with pytest.raises(ValueError, match='stamp'):
        consensus.prepare(observation(6 * DT, points=PAIR))
    assert fingerprint(consensus) == committed
    assert consensus.source_frontier_ns == 6 * DT
    assert np.all(consensus.snapshot().certified_model_last_stamp_ns <= 6 * DT)


def test_incident_domain_capacity_failure_is_atomic_not_silent_truncation():
    # Two raw faces and one graph edge fit; two genuine incident domains do
    # not fit the original collective capacity of three support items.
    consensus = BoundaryConsensus({'max_support_segments': 3})
    before = fingerprint(consensus)
    with pytest.raises(ValueError, match='finite-domain capacity'):
        consensus.prepare(observation(DT, points=PAIR))
    assert fingerprint(consensus) == before


def test_pending_incident_evidence_expires_at_original_ttl_without_extra_votes():
    consensus = BoundaryConsensus({'candidate_ttl_s': .1})
    commit(consensus, observation(DT, points=PAIR))
    before = fingerprint(consensus)
    expired = consensus.prepare(observation(3 * DT + 1, points=np.empty((0, 2))))
    assert fingerprint(consensus) == before
    assert len(expired.expired_ids) == 2
    assert consensus.commit(expired)
    assert not consensus._records and not consensus._edges and not consensus._domains
    assert consensus._domain_interval_count == 0


@pytest.mark.parametrize('internal', [True, False], ids=['old_same_owner_path', 'direct_incident_path'])
def test_shallow_rotating_observations_are_a_finite_fit_not_a_truth_guarantee(internal):
    consensus = BoundaryConsensus()
    observed_normals = []
    for frame, angle in enumerate((10., -10., 10., -10., 10.), 1):
        length = .04 if internal else .2 + (frame - 1) * .15
        radians = math.radians(angle)
        endpoint = length * np.array([math.cos(radians), math.sin(radians)])
        direction = normal(angle)
        observed_normals.append(direction)
        commit(consensus, observation(frame * DT, points=[[0., 0.], endpoint],
                                      normals=[direction, direction]))
    reference = consensus.snapshot()
    assert len(reference.record_ids) == 1
    assert len(reference.certified_model_segments)
    assert not len(reference.directed_edges)
    if internal:
        assert len(consensus._records) == 1 and not consensus._edges
    else:
        assert len(consensus._records) == 6 and len(consensus._edges) == 5
        assert all(edge.votes == 1 for edge in consensus._edges.values())
    point = np.mean(reference.certified_model_segments[0], axis=0)
    normal_offsets = np.abs(np.asarray(observed_normals) @ point)
    # These analytical observations all pass the unchanged signed-angle gate,
    # yet their fused model need not lie on ANY individual source line. This
    # explicitly documents a shared modelling limitation, not an acceptance
    # tolerance, a claim of safety, or permission to invent graph votes.
    assert np.all(normal_offsets > 1.e-6)
    assert np.all(reference.certified_model_frame_votes >= 5)
    assert np.all(reference.certified_model_reliable_votes >= 1)
