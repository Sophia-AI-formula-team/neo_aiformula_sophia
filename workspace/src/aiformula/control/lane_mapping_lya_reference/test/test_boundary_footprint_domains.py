"""Per-face finite-model evidence has original temporal/topology semantics.

These analytical observations isolate transactions, not camera accuracy or
runtime acceptance. Whole-footprint geometry has separate Fraction tests.
"""
from dataclasses import replace

import numpy as np
import pytest

from lane_mapping_lya_reference.paint_geometry import BoundaryConsensus
from test_boundary_domain_consensus import observation, commit, fingerprint, covered, DT


def fragments(stamp, *, points=((0., .01),), links=(), valid=None, j=None, reliable=None):
    obs = observation(stamp, points=points, links=links)
    count = len(obs.points)
    return replace(obs,
        position_segments=np.stack((obs.model_points-[.03, 0.], obs.model_points+[.03, 0.]), axis=1),
        position_segment_valid=np.ones(count, dtype=bool) if valid is None else np.asarray(valid, dtype=bool),
        position_segment_sensitivity=np.full(count, .03) if j is None else np.asarray(j, dtype=np.float64),
        position_segment_reliable=np.ones(count, dtype=bool) if reliable is None else np.asarray(reliable, dtype=bool))


def test_isolated_face_needs_five_distinct_frames_and_never_creates_a_graph_edge():
    consensus = BoundaryConsensus()
    for frame in range(1, 5):
        commit(consensus, fragments(frame*DT))
        assert not len(consensus.snapshot().certified_model_segments)
        assert all(piece.frame_votes == frame and piece.reliable_votes == 1
                   for state in consensus._domains.values() for piece in state.intervals)
    before = fingerprint(consensus)
    token = consensus.prepare(fragments(5*DT))
    assert fingerprint(consensus) == before
    assert consensus.commit(token)
    ref = consensus.snapshot()
    assert len(ref.record_ids) == 1 and covered(ref, 0.) and covered(ref, .02)
    assert not covered(ref, .04)
    assert not len(ref.directed_edges) and not consensus._edges
    assert np.all(ref.certified_model_frame_votes == 5)
    assert np.all(ref.certified_model_reliable_votes == 1)  # Existing per-counter threshold saturation.
    assert np.all(ref.full_in_degree == 0) and np.all(ref.full_out_degree == 0)
    after = fingerprint(consensus)
    assert not consensus.commit(token)
    with pytest.raises(ValueError, match='stamp'):
        consensus.prepare(fragments(5*DT))
    assert fingerprint(consensus) == after


@pytest.mark.parametrize('case', ['flag', 'whole_j', 'unstable_j', 'invalid'])
def test_root_reliability_cannot_certify_an_unreliable_or_invalid_whole_fragment(case):
    options = {'flag': {'reliable': [False]}, 'whole_j': {'j': [.2]},
               'unstable_j': {'j': [.6]}, 'invalid': {'valid': [False], 'reliable': [False]}}[case]
    consensus = BoundaryConsensus()
    for frame in range(1, 6):
        obs = fragments(frame*DT, **options)
        assert obs.model_reliable[0] and obs.model_sensitivity[0] < .1
        commit(consensus, obs)
    assert consensus.confirmed_count == 1
    assert not len(consensus.snapshot().certified_model_segments)
    if case in ('flag', 'whole_j'):
        assert all(piece.reliable_votes == 0 for domain in consensus._domains.values() for piece in domain.intervals)
    else:
        assert all(not domain.intervals for domain in consensus._domains.values())


def test_own_internal_and_incident_overlaps_still_count_only_once_per_frame():
    consensus = BoundaryConsensus()
    for frame in range(1, 6):
        commit(consensus, fragments(frame*DT, points=((0., .01), (.02, .01)), links=((0, 1),)))
    assert len(consensus._records) == 1
    ref = consensus.snapshot()
    assert covered(ref, .01) and not covered(ref, .06)
    assert np.all(ref.certified_model_frame_votes == 5)
    assert np.all(ref.certified_model_reliable_votes == 1)  # Existing per-counter threshold saturation.
    assert all(piece.frame_votes == 5 for domain in consensus._domains.values() for piece in domain.intervals)


def test_black_gap_between_independent_faces_stays_empty():
    consensus = BoundaryConsensus()
    for frame in range(1, 6):
        commit(consensus, fragments(frame*DT, points=((-1., .01), (1., .01))))
    ref = consensus.snapshot()
    assert covered(ref, -1.) and covered(ref, 1.)
    assert not any(covered(ref, float(x)) for x in np.linspace(-.9, .9, 19))
    assert not consensus._edges and not len(ref.directed_edges)


def test_original_combined_capacity_failure_is_atomic():
    consensus = BoundaryConsensus({'max_support_segments': 1})
    before = fingerprint(consensus)
    with pytest.raises(ValueError, match='finite-domain capacity'):
        consensus.prepare(fragments(DT))
    assert fingerprint(consensus) == before


def test_legacy_no_fragment_observation_does_not_gain_a_raw_face_span():
    consensus = BoundaryConsensus()
    for frame in range(1, 6):
        commit(consensus, observation(frame*DT, points=[[0., .01]], links=()))
    assert consensus.confirmed_count == 1
    assert not len(consensus.snapshot().certified_model_segments)


def test_fragment_buffers_are_immutable_and_transform_with_position_geometry():
    obs = fragments(DT)
    for name in ('position_segments', 'position_segment_valid', 'position_segment_sensitivity', 'position_segment_reliable'):
        with pytest.raises(ValueError):
            getattr(obs, name).setflags(write=True)
    transformed = obs.transformed([1., 2., np.pi/2.])
    np.testing.assert_allclose(transformed.position_segments[0], [[.99, 1.97], [.99, 2.03]], atol=1.e-15)
    for name in ('position_segment_valid', 'position_segment_sensitivity', 'position_segment_reliable'):
        assert getattr(obs, name).tobytes() == getattr(transformed, name).tobytes()


@pytest.mark.parametrize('change', [
    {'position_segments': None}, {'position_estimates': None},
    {'position_segments': np.array([[[np.nan, 0.], [0., 0.]]])},
    {'position_segments': np.zeros((1, 2, 2))},
    {'position_segment_sensitivity': np.array([-1.])},
    {'position_segment_sensitivity': np.array([np.inf])},
    {'position_segment_valid': np.array([False])},
])
def test_malformed_fragment_evidence_is_rejected_not_silently_coerced(change):
    with pytest.raises(ValueError):
        replace(fragments(DT), **change)
