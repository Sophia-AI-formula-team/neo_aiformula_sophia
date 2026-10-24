"""Artificial v5 producer transactions; not mask/runtime/vehicle acceptance.

The original observation factory is AST-extracted from its explicit sibling
file, never imported or discovered as a test module. Analytic roots/quads are
independent inputs; original 5/1 and semantic capacity are not changed.
"""
import ast
from dataclasses import replace
from fractions import Fraction
import hashlib
import math
from pathlib import Path
from types import MappingProxyType
import unittest

import numpy as np

from lane_mapping_lya_reference import curve_consensus_compact as curves
from lane_mapping_lya_reference import curve_evidence as evidence
from lane_mapping_lya_reference import curve_geometry_builder as builder
from lane_mapping_lya_reference.continuation_support import FIELDS as PERMISSION_FIELDS
from lane_mapping_lya_reference.finite_support import DomainState, Interval
from lane_mapping_lya_reference.paint_geometry import (
    BoundaryConsensus, BoundaryObservation, CERTIFIED_MODEL_FIELDS,
)


DT = 100_000_000
BASE = np.array([[-.03125, .01], [0., .01], [.03125, .01]], dtype=np.float64)
FIXTURE_PATH = Path(__file__).with_name("test_boundary_domain_consensus.py")
FIXTURE_SHA256 = "106003f068971bc339ddb6cf31ef342983054659de0f43853c1286187e360589"
if FIXTURE_PATH.resolve(strict=True).name.lower() != FIXTURE_PATH.name.lower():
    raise AssertionError("original sibling observation factory alias refused")
_fixture_bytes = FIXTURE_PATH.read_bytes()
if hashlib.sha256(_fixture_bytes).hexdigest() != FIXTURE_SHA256:
    raise AssertionError("original sibling observation factory source changed")
_fixture_tree = ast.parse(_fixture_bytes, filename=str(FIXTURE_PATH))
_factories = [node for node in _fixture_tree.body
              if isinstance(node, ast.FunctionDef) and node.name == "observation"]
if len(_factories) != 1:
    raise AssertionError("expected exactly one original observation factory")
_factory_namespace = {"np": np, "BASE": BASE, "BoundaryObservation": BoundaryObservation}
exec(compile(ast.Module(body=_factories, type_ignores=[]), str(FIXTURE_PATH), "exec"),
     _factory_namespace)
_original_observation = _factory_namespace["observation"]


def observation(stamp, points=BASE, *, links=None, model_j=None, reliable=True):
    original = _original_observation(stamp, points=points, links=links,
        model_j=model_j, reliable=reliable)
    count = len(original.points)
    roots = original.model_points
    # Independent analytical finite pixel footprints and own tangent pieces.
    quads = roots[:, None, :] + np.array(
        [[-.003, -.005], [.003, -.005], [.003, .005], [-.003, .005]])[None, :, :]
    return replace(original,
        position_segments=np.stack((roots-[.003, 0.], roots+[.003, 0.]), axis=1),
        position_segment_valid=np.ones(count, dtype=np.bool_),
        position_segment_sensitivity=np.full(count, .03),
        position_segment_reliable=original.position_reliable.copy(),
        position_footprints=quads)


def history(frames=5, *, config=None, points=BASE, **options):
    result = BoundaryConsensus(config, curve_geometry=True)
    for frame in range(1, frames+1):
        token = result.prepare(observation(frame*DT, points, **options))
        if not result.commit(token):
            raise AssertionError("independent fixture commit failed")
    return result


def raw(array):
    return array.dtype.str, array.shape, array.tobytes(order="C"), array.flags.writeable


def reference_bits(reference):
    fields = ("points_xy", "normals", "support_segments", "segment_owner", "votes",
              "reliable_votes", "record_ids", "directed_edges", "edge_frame_votes",
              "edge_reliable_votes", "edge_last_stamp_ns", "full_in_degree", "full_out_degree")
    result = [reference.boundary_version, reference.source_frontier_ns]
    for field in fields+CERTIFIED_MODEL_FIELDS+PERMISSION_FIELDS:
        value = getattr(reference, field)
        result.append((field, None if value is None else raw(value)))
    geometry = reference.curve_geometry
    if geometry is not None:
        for field in ("segments", "normals", "owner", "frame_votes", "reliable_votes",
                      "last_stamp_ns", "owner_status", "blocker_boxes", "blocker_owner"):
            result.append((field, raw(getattr(geometry, field))))
        result.append(("complete", geometry.complete))
    return tuple(result)


def fingerprint(consensus):
    records = tuple((key, value.cell, value.point, value.normal, value.votes,
                     value.reliable_votes, value.last_stamp_ns, raw(value.segments),
                     raw(value.neighbours)) for key, value in sorted(consensus._records.items()))
    curve_keys = tuple((key, id(value), value.source_frontier_ns, value.status,
                       tuple((id(frame.packet), id(frame.packet.source),
                              raw(frame.packet.roles), raw(frame.packet.face_indices))
                             for frame in value.frames))
                      for key, value in sorted(consensus._curve_states.items()))
    return (consensus._generation, consensus.source_frontier_ns, consensus._next_record_id,
            consensus._segment_count, consensus._domain_interval_count,
            tuple(sorted(consensus._domains.items())), records, curve_keys,
            tuple((id(frame.source), frame.source.stamp_ns, frame.has_reliable)
                  for frame in consensus._curve_orphan_sources),
            id(consensus._curve_reference), reference_bits(consensus.snapshot()))


def contains(boxes, point):
    value = np.asarray(point)
    return any(bool(np.all(box[0] <= value) and np.all(value <= box[1])) for box in boxes)


class CurveGeometryBuilderTests(unittest.TestCase):
    def test_01_empty_v5_is_explicit_and_legacy_default_is_unchanged(self):
        v5 = BoundaryConsensus(curve_geometry=True).snapshot()
        self.assertEqual(v5.boundary_version, 5)
        self.assertTrue(v5.curve_geometry.complete)
        self.assertEqual(v5.curve_geometry.segments.shape, (0, 2, 2))
        self.assertIsNone(v5.certified_model_segments)
        legacy = BoundaryConsensus()
        self.assertEqual(legacy.snapshot().boundary_version, 3)
        self.assertIsNone(legacy.snapshot().curve_geometry)
        for frame in range(1, 6):
            self.assertTrue(legacy.commit(legacy.prepare(observation(frame*DT))))
        self.assertGreater(len(legacy.snapshot().certified_model_segments), 0)
        self.assertIsNone(legacy.snapshot().curve_geometry)

    def test_02_prepare_does_not_publish_any_curve_source_or_frontier(self):
        consensus = history(4)
        before = fingerprint(consensus)
        fifth = consensus.prepare(observation(5*DT))
        self.assertEqual(fingerprint(consensus), before)
        self.assertEqual(consensus.source_frontier_ns, 4*DT)
        self.assertEqual(len(consensus.snapshot().curve_geometry.segments), 0)
        self.assertGreater(len(fifth.curve_reference.curve_geometry.segments), 0)
        self.assertEqual(fifth.curve_reference.source_frontier_ns, 5*DT)
        self.assertIsInstance(fifth.curve_states, type(MappingProxyType({})))

    def test_03_commit_publishes_cached_immutable_finite_curve_not_meanline(self):
        consensus = history()
        reference = consensus.snapshot()
        self.assertIs(reference, consensus.snapshot())
        self.assertIsNone(reference.certified_model_segments)
        self.assertEqual(len(reference.support_segments), 0)
        geometry = reference.curve_geometry
        self.assertGreater(len(geometry.segments), 0)
        self.assertTrue(geometry.complete)
        self.assertTrue(np.all(geometry.frame_votes == 5))
        self.assertTrue(np.all(geometry.reliable_votes == 1))
        for segment, normal, slot in zip(geometry.segments, geometry.normals, geometry.owner):
            delta = segment[1]-segment[0]
            self.assertLessEqual(abs(float(np.dot(delta/np.linalg.norm(delta), normal))),
                                 64*np.finfo(np.float64).eps)
            owner = int(reference.record_ids[slot])
            domain = consensus._domains[owner]
            for point in segment:
                exact = sum((Fraction(float(x))-Fraction(o))*Fraction(t)
                            for x, o, t in zip(point, domain.origin, domain.tangent))
                self.assertTrue(any(Fraction(piece.lower) <= exact <= Fraction(piece.upper)
                    for piece in domain.intervals if piece.frame_votes == 5 and piece.reliable_votes == 1))
        for field in ("segments", "normals", "owner", "blocker_boxes", "owner_status"):
            array = getattr(geometry, field)
            self.assertFalse(array.flags.writeable)
            with self.assertRaises(ValueError):
                array.setflags(write=True)

    def test_04_stale_foreign_and_incomplete_v5_tokens_do_not_commit(self):
        consensus = history()
        stale = consensus.prepare(observation(6*DT))
        later = consensus.prepare(observation(7*DT))
        self.assertTrue(consensus.commit(later))
        before = fingerprint(consensus)
        self.assertFalse(consensus.commit(stale))
        foreign = history().prepare(observation(8*DT))
        self.assertFalse(consensus.commit(foreign))
        valid = consensus.prepare(observation(9*DT))
        self.assertFalse(consensus.commit(replace(valid, curve_reference=None)))
        self.assertEqual(fingerprint(consensus), before)

    def test_05_combined_semantic_capacity_rejects_without_partial_publication(self):
        generous = BoundaryConsensus(curve_geometry=True)
        token = generous.prepare(observation(DT))
        required = token.curve_inventory["total_support_records"]
        self.assertGreater(required, 20)
        limited = BoundaryConsensus({"max_support_segments": required-1}, curve_geometry=True)
        before = fingerprint(limited)
        with self.assertRaisesRegex(ValueError, "capacity"):
            limited.prepare(observation(DT))
        self.assertEqual(fingerprint(limited), before)
        self.assertEqual(limited._records, {})
        self.assertEqual(len(limited._curve_states), 0)

    def test_06_pending_singleton_retains_full_source_and_reject_only_box(self):
        consensus = history(points=np.array([[0., .01]]), links=())
        geometry = consensus.snapshot().curve_geometry
        self.assertEqual(consensus.confirmed_count, 1)
        self.assertEqual(len(geometry.segments), 0)
        self.assertTrue(contains(geometry.blocker_boxes, (0., .01)))
        self.assertEqual(geometry.owner_status.tolist(), [1])
        state = next(iter(consensus._curve_states.values()))
        self.assertTrue(all(len(frame.packet.face_indices) == 0 for frame in state.frames))
        self.assertEqual(consensus._curve_inventory["source_faces"], 5)
        self.assertTrue(all(frame.packet.source.quad_valid.tolist() == [True] for frame in state.frames))

    def test_07_unknown_fold_retains_all_keys_source_and_blocks_instead_of_chord(self):
        points = np.array([[-.03, .01], [0., .01], [-.03, .01]])
        consensus = history(points=points)
        state = next(iter(consensus._curve_states.values()))
        self.assertEqual(state.status, "unknown")
        self.assertEqual(consensus._curve_inventory["source_faces"], 15)
        self.assertEqual(len(consensus.snapshot().curve_geometry.segments), 0)
        self.assertEqual(consensus.snapshot().curve_geometry.owner_status.tolist(), [0])
        self.assertTrue(contains(consensus.snapshot().curve_geometry.blocker_boxes, (-.015, .01)))
        self.assertIsNone(consensus.snapshot().certified_model_segments)

    def test_08_zero_owner_unowned_quad_is_retained_as_local_unknown_boxes(self):
        consensus = BoundaryConsensus(curve_geometry=True)
        token = consensus.prepare(observation(DT, model_j=[.6, .6, .6]))
        self.assertEqual(len(token.updates), 0)
        self.assertEqual(len(token.curve_orphan_sources), 1)
        geometry = token.curve_reference.curve_geometry
        self.assertTrue(geometry.complete)
        self.assertEqual(geometry.blocker_owner.tolist(), [-1, -1, -1])
        self.assertEqual(len(geometry.segments), 0)
        self.assertEqual(len(geometry.normals), 0)
        self.assertEqual(len(geometry.owner_status), 0)
        self.assertFalse(contains(geometry.blocker_boxes, (.015625, .01)))
        self.assertEqual(token.curve_inventory["unowned_valid_quads"], 3)
        self.assertEqual(token.curve_inventory["standalone_unowned_boxes"], 3)
        self.assertEqual(token.curve_inventory["source_faces"], 3)
        self.assertTrue(consensus.commit(token))
        source = consensus._curve_orphan_sources[0].source
        self.assertEqual(source.endpoint_owners.tolist(), [-1, -1, -1])
        self.assertEqual(source.valid.tolist(), [False, False, False])
        self.assertEqual(source.quad_valid.tolist(), [True, True, True])
        for box, quad in zip(geometry.blocker_boxes, source.quads):
            self.assertTrue(np.all(box[0] <= quad))
            self.assertTrue(np.all(quad <= box[1]))
            for first, second in zip(quad, np.roll(quad, -1, axis=0)):
                self.assertTrue(contains((box,), (first+second)/2.))
        self.assertTrue(consensus.snapshot_near((100., 100.), .01, 10).curve_geometry.complete)

    def test_09_orphan_original_five_plus_reliable_and_shared_source_inventory(self):
        consensus = BoundaryConsensus(curve_geometry=True)
        for frame in range(1, 8):
            token = consensus.prepare(observation(frame*DT, model_j=[.6]*3, reliable=frame == 1))
            self.assertTrue(consensus.commit(token))
        self.assertEqual([frame.source.stamp_ns for frame in consensus._curve_orphan_sources],
                         [DT, 3*DT, 4*DT, 5*DT, 6*DT, 7*DT])
        self.assertEqual(consensus._curve_inventory["source_tables"], 6)
        self.assertEqual(consensus._curve_inventory["source_faces"], 18)
        self.assertTrue(consensus.snapshot().curve_geometry.complete)
        self.assertEqual(consensus._curve_inventory["standalone_unowned_boxes"], 18)
        self.assertEqual(consensus.snapshot().curve_geometry.blocker_owner.tolist(), [-1]*18)
        # One table simultaneously reached by a real key and orphan ref is
        # not charged twice. This manual frame has one owned and one orphan face.
        source = evidence.SourceFrame(stamp_ns=DT, face_ids=np.arange(2, dtype=np.int64),
            previous_next=np.full((2, 2), -1, dtype=np.int64), contour_ids=np.arange(2, dtype=np.int64),
            endpoint_owners=np.array([0, -1], dtype=np.int64), model_roots=np.array([[0., .01], [1., .01]]),
            normals=np.array([[0., 1.], [0., 1.]]), quads=np.tile(np.array(
                [[[-.01, 0.], [.01, 0.], [.01, .02], [-.01, .02]]]), (2, 1, 1)),
            quad_valid=np.ones(2, dtype=np.bool_), reliable=np.array([True, False]),
            valid=np.array([True, False]))
        packet = evidence.PairPacket(source, 0, 1, (-1, -1),
                                     np.empty(0, dtype=np.uint8), np.empty((0, 2), dtype=np.int64))
        state = curves.create_curve_state(0, 1, (-1, -1), (0., .01), (1., -0.),
                                          normal_cosine=math.cos(math.pi/6.), max_support_records=200000)
        state = curves.advance_curve(state, packet, available_before_ns=DT)
        template = BoundaryConsensus(curve_geometry=True).prepare(observation(DT, [[0., .01]], links=()))
        built = builder.build_curve_geometry(template.updates, dict(template.domain_updates),
            {(0, 1, (-1, -1)): state}, {}, {}, template.owner.config, DT,
            base_support_records=0, orphan_sources=(builder.OrphanSourceFrame(source, True),))
        self.assertEqual(built.inventory["source_tables"], 1)
        self.assertEqual(built.inventory["source_faces"], 2)
        self.assertEqual(built.inventory["support_records"], curves.inventory((state,))["support_records"]+4)
        self.assertTrue(built.geometry.complete)
        self.assertEqual(built.inventory["standalone_unowned_boxes"], 1)
        self.assertEqual(built.geometry.blocker_owner.tolist().count(-1), 1)

    def test_10_partly_confirmed_curve_keeps_extension_competition_locally(self):
        consensus = history()
        token = consensus.prepare(observation(6*DT, BASE+[.01, 0.]))
        geometry = token.curve_reference.curve_geometry
        self.assertGreater(len(geometry.segments), 0)
        self.assertGreater(token.curve_inventory["pending_complement_boxes"], 0)
        self.assertTrue(contains(geometry.blocker_boxes, (.038, .01)))
        self.assertTrue(contains(geometry.blocker_boxes, (-.028, .01)))
        self.assertFalse(contains(geometry.blocker_boxes, (.017, .01)))
        self.assertTrue(any(np.min(segment[:, 0]) < .017 < np.max(segment[:, 0])
                            for segment in geometry.segments))
        self.assertEqual(consensus.source_frontier_ns, 5*DT)

    def test_11_original_confirmed_domain_hole_retains_source_competitor(self):
        consensus = history()
        owner = next(iter(consensus._domains))
        original = consensus._domains[owner]
        domain = DomainState(original.origin, original.tangent,
            (Interval(-.034, -.005, 5, 1, 5*DT), Interval(.005, .034, 5, 1, 5*DT)), 5*DT)
        built = builder.build_curve_geometry(tuple(consensus._records.values()), {owner: domain},
            consensus._curve_states, consensus._edges, consensus._continuations,
            consensus.config, 5*DT, base_support_records=0)
        self.assertGreater(len(built.geometry.segments), 0)
        self.assertTrue(contains(built.geometry.blocker_boxes, (0., .01)))
        self.assertFalse(contains(built.geometry.blocker_boxes, (.017, .01)))
        self.assertFalse(any(np.min(segment[:, 0]) < 0. < np.max(segment[:, 0])
                             for segment in built.geometry.segments))

    def test_12_same_owner_distinct_signatures_are_all_kept_as_competitors(self):
        consensus = history()
        state = next(iter(consensus._curve_states.values()))
        alternative = curves.create_curve_state(state.owner_id, 1, (-1, 99),
            state.origin, state.tangent, normal_cosine=state.normal_cosine, max_support_records=200000)
        for frame in state.frames:
            packet = evidence.PairPacket(frame.packet.source, state.owner_id, 1, (-1, 99),
                                         frame.packet.roles, frame.packet.face_indices)
            alternative = curves.advance_curve(alternative, packet,
                                                available_before_ns=packet.stamp_ns)
        states = dict(consensus._curve_states)
        states[(state.owner_id, 1, (-1, 99))] = alternative
        built = builder.build_curve_geometry(tuple(consensus._records.values()), consensus._domains,
            states, consensus._edges, consensus._continuations, consensus.config,
            5*DT, base_support_records=0)
        self.assertEqual(built.inventory["states"], 2)
        self.assertEqual(built.inventory["source_tables"], 5)
        self.assertEqual(built.inventory["competing_signature_keys"], 2)
        self.assertEqual(len(built.geometry.blocker_boxes), 2)
        self.assertTrue(contains(built.geometry.blocker_boxes, (.017, .01)))

    def test_13_snapshot_near_uses_source_envelope_and_never_truncates_competitors(self):
        points = np.vstack((BASE, BASE+[.2, 0.]))
        consensus = history(points=points, links=((0, 1), (1, 2), (3, 4), (4, 5)))
        reference = consensus.snapshot()
        near = consensus.snapshot_near((.235, .01), .001, 10)
        self.assertEqual(near.record_ids.tolist(), [1])
        self.assertGreater(len(near.curve_geometry.blocker_boxes), 0)
        before = fingerprint(consensus)
        with self.assertRaisesRegex(ValueError, "competitor owner capacity"):
            consensus.snapshot_near((.1, .01), .2, 1)
        self.assertEqual(fingerprint(consensus), before)
        self.assertEqual(len(reference.record_ids), 2)

    def test_14_candidate_expiry_removes_keys_atomically_and_frozen_export_stays(self):
        consensus = history(1, config={"candidate_ttl_s": .1})
        before = fingerprint(consensus)
        old_reference = consensus.snapshot()
        old_bits = reference_bits(old_reference)
        token = consensus.prepare(observation(4*DT, np.empty((0, 2))))
        self.assertTrue(token.expired_ids)
        self.assertEqual(fingerprint(consensus), before)
        self.assertEqual(len(token.curve_states), 0)
        self.assertTrue(consensus.commit(token))
        self.assertEqual(len(consensus._curve_states), 0)
        self.assertEqual(len(consensus._domains), 0)
        self.assertEqual(len(consensus.snapshot().record_ids), 0)
        self.assertEqual(reference_bits(old_reference), old_bits)

    def test_15_builder_future_wrong_domain_and_original_thresholds_reject(self):
        consensus = history()
        before = fingerprint(consensus)
        records = tuple(consensus._records.values())
        with self.assertRaisesRegex(ValueError, "future"):
            builder.build_curve_geometry(records, consensus._domains, consensus._curve_states,
                consensus._edges, consensus._continuations, consensus.config,
                4*DT, base_support_records=0)
        with self.assertRaises(ValueError):
            BoundaryConsensus({"minimum_frame_votes": 3}, curve_geometry=True)
        wrong_domains = dict(consensus._domains)
        wrong_domains[records[0].record_id] = object()
        built = builder.build_curve_geometry(records, wrong_domains, consensus._curve_states,
            consensus._edges, consensus._continuations, consensus.config, 5*DT, base_support_records=0)
        self.assertFalse(built.geometry.complete)
        self.assertEqual(len(built.geometry.segments), 0)
        self.assertEqual(fingerprint(consensus), before)

    def test_16_clean_frame_keeps_original_orphan_source_and_independent_boxes(self):
        consensus = BoundaryConsensus(curve_geometry=True)
        self.assertTrue(consensus.commit(consensus.prepare(observation(DT, model_j=[.6]*3))))
        orphan = consensus._curve_orphan_sources[0]
        source_bits = {name: raw(getattr(orphan.source, name)) for name, _, _ in evidence._FIELDS}
        original_boxes = raw(consensus.snapshot().curve_geometry.blocker_boxes)
        token = consensus.prepare(observation(2*DT))
        self.assertEqual(len(token.curve_orphan_sources), 1)
        self.assertIs(token.curve_orphan_sources[0], orphan)
        self.assertTrue(consensus.commit(token))
        self.assertIs(consensus._curve_orphan_sources[0].source, orphan.source)
        self.assertEqual({name: raw(getattr(orphan.source, name)) for name, _, _ in evidence._FIELDS},
                         source_bits)
        geometry = consensus.snapshot().curve_geometry
        standalone = geometry.blocker_boxes[geometry.blocker_owner == -1]
        self.assertEqual(raw(np.frombuffer(standalone.tobytes(), dtype=standalone.dtype)
                             .reshape(standalone.shape)), original_boxes)
        self.assertTrue(geometry.complete)
        self.assertEqual(consensus._curve_inventory["retained_orphan_sources"], 1)
        self.assertEqual(consensus._curve_inventory["source_tables"], 2)
        self.assertEqual(consensus._curve_inventory["standalone_unowned_boxes"], 3)

    def test_17_state_pinned_source_exports_unowned_quad_without_orphan_reference(self):
        template = BoundaryConsensus(curve_geometry=True).prepare(
            observation(DT, model_j=[.03, .6, .03]))
        self.assertTrue(template.curve_states)
        sources = {id(frame.packet.source): frame.packet.source
                   for state in template.curve_states.values() for frame in state.frames}
        self.assertEqual(len(sources), 1)
        source = next(iter(sources.values()))
        self.assertEqual(int(np.sum(source.quad_valid & (source.endpoint_owners < 0))), 1)
        built = builder.build_curve_geometry(template.updates, dict(template.domain_updates),
            template.curve_states, {}, {}, template.owner.config, DT,
            base_support_records=0, orphan_sources=())
        self.assertTrue(built.geometry.complete)
        self.assertEqual(built.inventory["source_tables"], 1)
        self.assertEqual(built.inventory["source_faces"], 3)
        self.assertEqual(built.inventory["retained_orphan_sources"], 0)
        self.assertEqual(built.inventory["unowned_valid_quads"], 1)
        self.assertEqual(built.inventory["standalone_unowned_boxes"], 1)
        standalone = built.geometry.blocker_boxes[built.geometry.blocker_owner == -1]
        self.assertEqual(len(standalone), 1)
        for point in source.quads[source.endpoint_owners < 0].reshape(-1, 2):
            self.assertTrue(contains(standalone, point))
        self.assertEqual(built.inventory["support_records"],
                         curves.inventory(template.curve_states.values())["support_records"])

    def test_18_unowned_quad_coordinate_limit_fails_closed_and_subnormals_enclose(self):
        template = BoundaryConsensus(curve_geometry=True).prepare(observation(DT, model_j=[.6]*3))
        original = template.curve_orphan_sources[0]
        quads = original.source.quads.copy()
        quads[1] = [[1.e7-.01, 0.], [1.e7, 0.], [1.e7, .01], [1.e7-.01, .01]]
        source = replace(original.source, quads=quads)
        built = builder.build_curve_geometry((), {}, {}, {}, {}, template.owner.config, DT,
            base_support_records=0, orphan_sources=(builder.OrphanSourceFrame(source, True),))
        self.assertFalse(built.geometry.complete)
        self.assertEqual(built.inventory["unowned_valid_quads"], 3)
        self.assertEqual(built.inventory["standalone_unowned_boxes"], 2)
        self.assertEqual(built.inventory["incomplete_reasons"],
                         {"unrepresentable finite unowned quad box": 1})
        self.assertEqual(built.geometry.blocker_owner.tolist(), [-1, -1])
        self.assertEqual(built.inventory["source_faces"], 3)
        tiny = float.fromhex("0x0.0000000000001p-1022")
        quad = np.array([[-tiny, 0.], [tiny, 0.], [tiny, tiny], [-tiny, tiny]])
        policy = np.geterr()
        with np.errstate(all="raise"):
            strict = np.geterr()
            box = builder._unowned_quad_box(quad)
            self.assertEqual(np.geterr(), strict)
        self.assertEqual(np.geterr(), policy)
        self.assertTrue(np.all(np.asarray(box[0]) <= quad))
        self.assertTrue(np.all(quad <= np.asarray(box[1])))

    def test_19_standalone_boxes_each_consume_original_global_export_capacity(self):
        template = BoundaryConsensus(curve_geometry=True).prepare(observation(DT, model_j=[.6]*3))
        options = dict(orphan_sources=template.curve_orphan_sources)
        built = builder.build_curve_geometry((), {}, {}, {}, {}, template.owner.config, DT,
            base_support_records=0, **options)
        self.assertEqual(built.inventory["export_support_records"], 1+3)
        self.assertEqual(built.support_records,
                         built.inventory["support_records"]+built.inventory["export_support_records"])
        original_capacity = template.owner.config["max_support_segments"]
        self.assertEqual(original_capacity, 200000)
        base = original_capacity-built.support_records
        exact = builder.build_curve_geometry((), {}, {}, {}, {}, template.owner.config, DT,
            base_support_records=base, **options)
        self.assertEqual(exact.support_records, original_capacity)
        # Without charging these three boxes, this prospective map would fit.
        with self.assertRaisesRegex(ValueError, "combined v5 shared source/export/permission capacity"):
            builder.build_curve_geometry((), {}, {}, {}, {}, template.owner.config, DT,
                base_support_records=base+3, **options)

