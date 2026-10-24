"""Closed actual source seams; physical holes and insufficient votes remain."""
from fractions import Fraction as F
import math
from types import SimpleNamespace
from dataclasses import replace
from unittest.mock import patch

import numpy as np
import pytest

from lane_mapping_lya_reference import curve_consensus_compact as compact
from lane_mapping_lya_reference import curve_geometry_builder as builder
from lane_mapping_lya_reference.curve_closed_support import close_supported_seams, _projection, _covers
from lane_mapping_lya_reference.finite_support import DomainState, Interval
from lane_mapping_lya_reference.paint_geometry import BoundaryConsensus
from test_curve_consensus_compact import _linear_packets, COSINE


def example(intervals=((0., 1.), (1., 2.)), frames=5, reliable=True):
    state = compact.create_curve_state(7, 1, (-1, -1), (0., 0.), (1., 0.),
                                       normal_cosine=COSINE, max_support_records=200000)
    for stamp in range(1, frames+1):
        packet, _ = _linear_packets(stamp, intervals=intervals, reliable=reliable)
        state = compact.advance_curve(state, packet, available_before_ns=stamp)
    # DomainState is a public immutable already-validated finite-domain record.
    domain = DomainState((0., 0.), (1., 0.), (Interval(-.1, 2.1, 5, 1, max(frames, 1)),), max(frames, 1))
    local = []
    for piece in state.pieces:
        for interval in domain.intervals:
            value = builder._finite_piece(state, piece, interval, COSINE)
            if value is not None: local.append(value)
    return state, domain, tuple(local)


def test_numeric_seam_closes_with_five_original_shared_root_paths():
    state, domain, local = example()
    assert len(local) == 2
    original = tuple((item[0], item[-1]) for item in local)
    result, count = close_supported_seams(state, domain, local, COSINE)
    assert count == 1
    assert result[0][0][1] == result[1][0][0]
    assert result[0][-1] == result[1][-1]
    assert result[0][-1][0] <= 1. <= result[0][-1][1]
    assert all(item[2:5] == before[2:5] for item, before in zip(result, local))
    assert tuple((item[0], item[-1]) for item in local) == original
    # Complete nominal parameter support is exact before inward conversion.
    a, b = result[0][-1]
    assert F(a) >= _projection(result[0][0][0], state)
    assert F(b) <= _projection(result[-1][0][1], state)


def test_disconnected_original_contour_gap_is_not_closed():
    state, domain, local = example(((0., .9), (1.1, 2.)))
    result, count = close_supported_seams(state, domain, local, COSINE)
    assert count == 0
    assert result[0][0][1] != result[1][0][0]
    assert result[0][-1][1] < result[1][-1][0]


def test_original_five_vote_domain_hole_is_not_closed():
    state, _, local = example()
    domain = DomainState((0., 0.), (1., 0.),
        (Interval(-.1, .99, 5, 1, 5), Interval(1.01, 2.1, 5, 1, 5)), 5)
    result, count = close_supported_seams(state, domain, local, COSINE)
    assert count == 0
    assert result[0][0][1] != result[1][0][0]


def test_four_frames_never_create_a_confirmed_seam():
    state, domain, local = example(frames=4)
    assert not local and state.status == 'unconfirmed'
    with pytest.raises(ValueError, match='confirmed'):
        close_supported_seams(state, domain, local, COSINE)


def test_no_reliable_original_frame_never_creates_a_confirmed_seam():
    state, domain, local = example(reliable=False)
    assert not local and state.status == 'unconfirmed'
    with pytest.raises(ValueError): close_supported_seams(state, domain, local, COSINE)


def test_factories_and_creation_chart_are_required():
    state, domain, local = example()
    with pytest.raises(ValueError): close_supported_seams(SimpleNamespace(), domain, local, COSINE)
    different = DomainState((1., 0.), (1., 0.), domain.intervals, 5)
    with pytest.raises(ValueError): close_supported_seams(state, different, local, COSINE)


def test_flag_is_explicit_and_does_not_enable_engine_or_merge_signatures():
    assert not BoundaryConsensus()._curve_closed_support
    assert not BoundaryConsensus(curve_geometry=True)._curve_closed_support
    assert BoundaryConsensus(curve_geometry=True, curve_closed_support=True)._curve_closed_support
    with pytest.raises(ValueError): BoundaryConsensus(curve_closed_support=True)
    with pytest.raises(ValueError): BoundaryConsensus(curve_geometry=True, curve_closed_support=1)
    state, domain, _ = example()
    normal = builder._export_key(state, domain, True, True, COSINE, retained=False)
    candidate = builder._export_key(state, domain, True, True, COSINE, retained=False, closed_support=True)
    assert normal == candidate  # Every competing signature keeps its whole box.


def test_coordinate_coincidence_with_different_face_ids_is_not_connectivity():
    normal = np.array([[0., 1.], [0., 1.]])
    rows = ((F(0), F(1), (0, b'a'), (1, b'b'), True, normal),
            (F(1), F(2), (2, b'b'), (3, b'c'), True, normal))
    assert not _covers(rows, F(.5), F(1.5))
    shared = (rows[0], rows[1][:2]+((1, b'b'),)+rows[1][3:])
    assert _covers(shared, F(.5), F(1.5))
    unreliable = (shared[0], shared[1][:4]+(False,)+shared[1][5:])
    assert not _covers(unreliable, F(.5), F(1.5), reliable=True)


def test_duplicate_source_stamp_and_changed_signed_direction_fail_closed():
    state, domain, local = example()
    duplicate = replace(state, frames=state.frames+(state.frames[-1],))
    with pytest.raises(ValueError, match='duplicate'):
        close_supported_seams(duplicate, domain, local, COSINE)
    with patch('lane_mapping_lya_reference.curve_closed_support.arithmetic._dot', return_value=-1.):
        result, count = close_supported_seams(state, domain, local, COSINE)
    assert count == 0
    assert result[0][0][1] != result[1][0][0]


def test_actual_prepare_is_transactional_and_keeps_original_configuration():
    from test_curve_geometry_builder import observation, DT
    mapper = BoundaryConsensus(curve_geometry=True, curve_storage='streamed', curve_closed_support=True)
    baseline = BoundaryConsensus(curve_geometry=True, curve_storage='streamed')
    assert mapper.config == baseline.config
    for sequence in range(1, 6):
        obs = observation(sequence*DT)
        previous = mapper.source_frontier_ns
        old_geometry = mapper._curve_reference
        token = mapper.prepare(obs)
        assert mapper.source_frontier_ns == previous
        assert mapper._curve_reference is old_geometry
        assert mapper.commit(token)
        assert baseline.commit(baseline.prepare(obs))
    assert tuple(mapper._curve_states) == tuple(baseline._curve_states)
    assert mapper._curve_inventory['total_support_records'] <= mapper.config['max_support_segments']
    assert mapper._curve_inventory['geometry_model'] == 'closed_original_source_support_experiment_r34'
    for key in mapper._curve_states:
        a, b = mapper._curve_states[key], baseline._curve_states[key]
        assert tuple(frame.packet.stamp_ns for frame in a.frames) == tuple(frame.packet.stamp_ns for frame in b.frames)
        assert all(x.packet.gather_original('model_roots').tobytes() == y.packet.gather_original('model_roots').tobytes()
                   for x, y in zip(a.frames, b.frames))
