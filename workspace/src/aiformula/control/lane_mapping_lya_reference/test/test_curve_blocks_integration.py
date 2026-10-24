"""Dense and retained-block full prepare/commit geometry and provenance parity."""
from dataclasses import fields, is_dataclass
import gc
import struct
import weakref

import numpy as np
import pytest

from lane_mapping_lya_reference import curve_evidence as evidence
from lane_mapping_lya_reference import curve_evidence_blocks as blocks
from lane_mapping_lya_reference import curve_consensus_compact as compact
from lane_mapping_lya_reference import curve_geometry_builder as builder
from lane_mapping_lya_reference.paint_geometry import BoundaryConsensus
from test_curve_geometry_builder import observation, reference_bits, DT, BASE


def exact(value):
    if isinstance(value, np.ndarray):
        return value.dtype.str, value.shape, value.tobytes()
    if type(value) is float:
        return struct.pack('d', value)
    if is_dataclass(value):
        return tuple((f.name, exact(getattr(value, f.name))) for f in fields(value))
    if type(value) is tuple:
        return tuple(exact(item) for item in value)
    return value


def semantic_states(states):
    result = []
    for key, state in states.items():
        frames = []
        for frame in state.frames:
            packet = frame.packet
            if blocks.is_block_packet(packet):
                own = packet.own_original
                unowned = packet.lease.unowned.gather_original
            else:
                source = packet.source
                own_mask = ((source.endpoint_owners == packet.owner_id)
                            & (source.valid | source.quad_valid))
                unowned_mask = source.quad_valid & (source.endpoint_owners < 0)
                own = lambda name: getattr(source, name)[own_mask]
                unowned = lambda name: getattr(source, name)[unowned_mask]
            frames.append((packet.stamp_ns, exact(packet.roles), exact(packet.face_indices),
                tuple(exact(packet.gather_original(name)) for name, _, _ in evidence._FIELDS),
                tuple(exact(own(name)) for name, _, _ in evidence._FIELDS),
                tuple(exact(unowned(name)) for name, _, _ in evidence._FIELDS),
                frame.status, exact(frame.pieces)))
        result.append((key, exact(state.origin), exact(state.tangent), state.source_frontier_ns,
                       state.status, state.reason, exact(state.pieces), tuple(frames)))
    return tuple(result)


@pytest.mark.parametrize('case', ['normal', 'singleton', 'fold', 'unowned', 'changing'])
def test_full_prepare_commit_semantics_and_capacity_charge(case):
    dense = BoundaryConsensus(curve_geometry=True)
    retained = BoundaryConsensus(curve_geometry=True, curve_storage='blocks')
    for index in range(1, 8):
        points = BASE[:1] if case == 'singleton' else BASE
        if case == 'fold':
            points = np.array([[0., .01], [.03125, .01], [0., .01]])
        j = np.ones(len(points)) if case == 'unowned' or (case == 'changing' and index == 1) else None
        obs = observation(index * DT, points, model_j=j, reliable=index == 1 or index > 5)
        old, new = dense.prepare(obs), retained.prepare(obs)
        assert reference_bits(old.curve_reference) == reference_bits(new.curve_reference)
        assert semantic_states(old.curve_states) == semantic_states(new.curve_states)
        assert len(old.curve_orphan_sources) == len(new.curve_orphan_sources)
        for original_orphan, retained_orphan in zip(old.curve_orphan_sources, new.curve_orphan_sources):
            source, capsule = original_orphan.source, retained_orphan.source
            assert (source.stamp_ns, original_orphan.has_reliable) == (capsule.stamp_ns, retained_orphan.has_reliable)
            selected = source.quad_valid & (source.endpoint_owners < 0)
            for name, _, _ in evidence._FIELDS:
                assert exact(getattr(source, name)[selected]) == exact(capsule.gather_original(name))
        for metric in ('source_tables', 'source_faces', 'packets', 'pair_occurrences',
                       'support_records', 'scalar_entries', 'total_support_records',
                       'base_support_records', 'export_support_records', 'blocker_boxes',
                       'unowned_valid_quads', 'standalone_unowned_boxes'):
            assert old.curve_inventory[metric] == new.curve_inventory[metric], metric
        assert new.curve_inventory['raw_payload_bytes'] <= old.curve_inventory['raw_payload_bytes']
        assert dense.commit(old) and retained.commit(new)
        assert reference_bits(dense.snapshot()) == reference_bits(retained.snapshot())


def test_rejected_or_foreign_token_does_not_publish_block_state():
    mapper = BoundaryConsensus(curve_geometry=True, curve_storage='blocks')
    other = BoundaryConsensus(curve_geometry=True, curve_storage='blocks')
    first = mapper.prepare(observation(DT))
    foreign = other.prepare(observation(DT))
    before = (mapper._generation, mapper.source_frontier_ns, id(mapper._curve_states),
              reference_bits(mapper.snapshot()))
    assert not mapper.commit(foreign)
    assert (mapper._generation, mapper.source_frontier_ns, id(mapper._curve_states),
            reference_bits(mapper.snapshot())) == before
    assert mapper.commit(first)
    committed = (mapper._generation, mapper.source_frontier_ns, id(mapper._curve_states))
    assert not mapper.commit(first)
    assert (mapper._generation, mapper.source_frontier_ns, id(mapper._curve_states)) == committed


def test_retained_token_does_not_pin_dense_source_tables(monkeypatch):
    refs = []
    original = blocks.project_frame
    def project(source, packets):
        refs.append(weakref.ref(source))
        refs.extend(weakref.ref(getattr(source, name)) for name, _, _ in evidence._FIELDS)
        return original(source, packets)
    monkeypatch.setattr(blocks, 'project_frame', project)
    mapper = BoundaryConsensus(curve_geometry=True, curve_storage='blocks')
    token = mapper.prepare(observation(DT))
    assert len(refs) == 11
    gc.collect()
    assert all(ref() is None for ref in refs)
    assert mapper.commit(token)
    assert all(blocks.is_block_packet(frame.packet)
               for state in mapper._curve_states.values() for frame in state.frames)


def test_cannot_switch_packet_representation_inside_existing_state():
    from test_curve_evidence_blocks import fixture
    source, packets = fixture()
    state = compact.create_curve_state(10, 1, packets[0].signature, (0., 0.), (1., 0.),
                                      normal_cosine=.9, max_support_records=200000)
    state = compact.advance_curve(state, packets[0], available_before_ns=source.stamp_ns)
    following, next_packets = fixture(stamp=101)
    projected, _ = blocks.project_frame(following, next_packets)
    with pytest.raises(ValueError, match='cannot mix'):
        compact.advance_curve(state, projected[0], available_before_ns=101)


def test_block_mode_capacity_rejects_same_frame_as_dense():
    costs = []
    for mode in ('dense', 'blocks'):
        mapper = BoundaryConsensus(curve_geometry=True, curve_storage=mode)
        costs.append(mapper.prepare(observation(DT)).curve_inventory['total_support_records'])
    assert costs[0] == costs[1]
    for mode in ('dense', 'blocks'):
        mapper = BoundaryConsensus({'max_support_segments': costs[0]-1},
                                   curve_geometry=True, curve_storage=mode)
        with pytest.raises(ValueError, match='capacity'):
            mapper.prepare(observation(DT))
        assert mapper.source_frontier_ns is None and mapper._generation == 0


def test_v4_cannot_accidentally_enable_block_storage():
    with pytest.raises(ValueError, match='explicit v5'):
        BoundaryConsensus(curve_storage='blocks')
