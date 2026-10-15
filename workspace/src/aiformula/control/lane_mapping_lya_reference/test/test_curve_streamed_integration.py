"""Sequential overlay storage experiment; no runtime or driving acceptance."""
from dataclasses import replace
import gc
import weakref

import numpy as np
import pytest

from lane_mapping_lya_reference import curve_consensus as arithmetic
from lane_mapping_lya_reference import curve_geometry_builder as builder
from lane_mapping_lya_reference import curve_retained_state as retained
from lane_mapping_lya_reference.paint_geometry import BoundaryConsensus
from test_curve_geometry_builder import observation, reference_bits, DT, BASE
from test_curve_blocks_integration import semantic_states


def overlay_charge(states):
    return sum(8 + len(piece.contributors) + len(piece.reliable_contributors)
               for state in states.values() for piece in state.pieces)


@pytest.mark.parametrize("case", ["normal", "singleton", "fold", "unowned", "changing"])
def test_streamed_export_and_future_evidence_match_materialized(case):
    full = BoundaryConsensus(curve_geometry=True, curve_storage="blocks")
    streamed = BoundaryConsensus(curve_geometry=True, curve_storage="streamed")
    for index in range(1, 10):
        points = BASE[:1] if case == "singleton" else BASE
        if case == "fold":
            points = np.array([[0., .01], [.03125, .01], [0., .01]])
        sensitivity = np.ones(len(points)) if case == "unowned" or (case == "changing" and index == 1) else None
        obs = observation(index * DT, points, model_j=sensitivity,
                          reliable=index == 1 or index > 6)
        old, new = full.prepare(obs), streamed.prepare(obs)
        assert reference_bits(old.curve_reference) == reference_bits(new.curve_reference)
        reconstructed = {key: retained.materialize(state) for key, state in new.curve_states.items()}
        assert semantic_states(old.curve_states) == semantic_states(reconstructed)
        removed = overlay_charge(old.curve_states)
        peak = max((20 + max(0, len({x for frame in state.frames for piece in frame.pieces
                                   for x in (piece.lower, piece.upper)})-1)
                    * (8 + 2*len(state.frames)) for state in old.curve_states.values()), default=0)
        assert new.curve_inventory["support_records"] == old.curve_inventory["support_records"] - removed
        assert new.curve_inventory["total_support_records"] == old.curve_inventory["total_support_records"] - removed + peak
        assert new.curve_inventory["overlay_scratch_bound_records"] == peak
        assert new.curve_inventory["reconstructed_overlay_logical_records"] == removed
        for field in ("source_tables", "source_faces", "retained_complete_faces", "packets",
                      "pair_occurrences", "states", "retained_frames", "affine_records",
                      "base_support_records", "export_support_records", "nominal_segments",
                      "blocker_boxes", "unowned_valid_quads", "standalone_unowned_boxes"):
            assert old.curve_inventory[field] == new.curve_inventory[field], field
        assert full.commit(old) and streamed.commit(new)
        assert reference_bits(full.snapshot()) == reference_bits(streamed.snapshot())


def test_streamed_prepare_releases_each_overlay_and_retains_none(monkeypatch):
    mapper = BoundaryConsensus(curve_geometry=True, curve_storage="streamed")
    refs, simultaneous = [], []
    original = arithmetic._overlay
    def tracked(*args):
        assert not any(ref() is not None for ref in refs), "previous key overlay still retained"
        result = original(*args)
        simultaneous.append(len(result[0]))
        refs.extend(weakref.ref(piece) for piece in result[0])
        return result
    monkeypatch.setattr(arithmetic, "_overlay", tracked)
    for index in range(1, 8):
        token = mapper.prepare(observation(index * DT))
        gc.collect()
        assert not any(ref() is not None for ref in refs)
        assert all(type(state) is retained.RetainedCurveState and not hasattr(state, "pieces")
                   for state in token.curve_states.values())
        assert mapper.commit(token)
    assert max(simultaneous) > 0


def test_streamed_prepare_derives_each_key_once(monkeypatch):
    mapper = BoundaryConsensus(curve_geometry=True, curve_storage="streamed")
    calls = []
    original = arithmetic._overlay
    def tracked(state, frames):
        calls.append((state.owner_id, state.signed_side, state.signature))
        return original(state, frames)
    monkeypatch.setattr(arithmetic, "_overlay", tracked)
    for index in range(1, 7):
        calls.clear()
        token = mapper.prepare(observation(index * DT))
        assert sorted(calls) == sorted(token.curve_states)
        assert mapper.commit(token)


def test_original_capacity_bound_includes_live_materialization():
    mapper = BoundaryConsensus(curve_geometry=True, curve_storage="streamed")
    for index in range(1, 6):
        assert mapper.commit(mapper.prepare(observation(index * DT)))
    ordered = tuple(mapper._records[key] for key in sorted(mapper._records))
    total = mapper._curve_inventory["total_support_records"]
    assert mapper._curve_inventory["overlay_scratch_bound_records"] > 20
    config = dict(mapper.config, max_support_segments=total-1)
    before = reference_bits(mapper.snapshot())
    with pytest.raises(ValueError, match="combined v5 shared source/export/permission capacity"):
        builder.build_curve_geometry(ordered, mapper._domains, mapper._curve_states,
            mapper._edges, mapper._continuations, config, mapper.source_frontier_ns,
            base_support_records=mapper._curve_inventory["base_support_records"],
            orphan_sources=mapper._curve_orphan_sources, retained=True)
    assert reference_bits(mapper.snapshot()) == before
    with pytest.raises(ValueError, match="inconsistent boundary"):
        BoundaryConsensus({"max_support_segments": 200001}, curve_geometry=True, curve_storage="streamed")


def test_streamed_generation_and_full_reference_lifetime():
    mapper = BoundaryConsensus(curve_geometry=True, curve_storage="streamed")
    other = BoundaryConsensus(curve_geometry=True, curve_storage="streamed")
    first = mapper.prepare(observation(DT))
    foreign = other.prepare(observation(DT))
    assert not mapper.commit(foreign)
    assert mapper.commit(first)
    assert not mapper.commit(first)
    old_reference = weakref.ref(mapper._curve_reference)
    old_geometry = weakref.ref(mapper._curve_reference.curve_geometry)
    second = mapper.prepare(observation(2 * DT))
    assert old_reference() is mapper._curve_reference
    assert mapper.commit(second)
    del first
    gc.collect()
    assert old_reference() is None and old_geometry() is None


def test_retained_export_keeps_all_key_and_causality_validation():
    mapper = BoundaryConsensus(curve_geometry=True, curve_storage="streamed")
    assert mapper.commit(mapper.prepare(observation(DT)))
    states = dict(mapper._curve_states)
    key = next(iter(states))
    states[key] = replace(states[key], source_frontier_ns=2 * DT)
    with pytest.raises(ValueError, match="future curve source evidence"):
        builder.build_curve_geometry(tuple(mapper._records.values()), mapper._domains,
            states, mapper._edges, mapper._continuations, mapper.config, DT,
            base_support_records=0, retained=True)
    with pytest.raises(ValueError, match="requires explicit v5"):
        BoundaryConsensus(curve_storage="streamed")
    assert BoundaryConsensus()._curve_storage == "dense"


def test_late_unknown_charges_discarded_overlay_work(monkeypatch):
    mapper = BoundaryConsensus(curve_geometry=True, curve_storage="blocks")
    for index in range(1, 6):
        assert mapper.commit(mapper.prepare(observation(index * DT)))
    full = next(state for state in mapper._curve_states.values() if len(state.pieces) > 1)
    raw = retained.from_materialized(full)
    original = arithmetic._endpoint
    calls = []
    def late_failure(*args, **kwargs):
        calls.append(None)
        if len(calls) == 3:
            raise ArithmeticError("late numerical export failure")
        return original(*args, **kwargs)
    monkeypatch.setattr(arithmetic, "_endpoint", late_failure)
    reconstructed = retained.materialize(raw)
    assert reconstructed.status == "unknown" and not reconstructed.pieces
    assert len(calls) == 3  # One complete piece existed before being discarded.
    bound = builder._overlay_record_bound(raw)
    assert bound >= 20 + overlay_charge({0: full}) > 20
    calls.clear()
    result = builder._export_key(raw, mapper._domains[raw.owner_id], True, False,
                                 raw.normal_cosine, retained=True)
    assert result[6] == "unknown" and result[7] == bound and result[8] == 0
