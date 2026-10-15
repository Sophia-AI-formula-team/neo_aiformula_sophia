"""Focused retained-state parity, lifecycle and original rejection contracts.

The ordinary compact updater is the oracle.  Fixtures are independent complete
two-face source events; no mapper, native module, bag, mask or Unity is loaded.
"""
from dataclasses import FrozenInstanceError
import gc
import math
import struct
import unittest
from unittest import mock
import weakref

import numpy as np

from lane_mapping_lya_reference import curve_consensus as arithmetic
from lane_mapping_lya_reference import curve_consensus_compact as compact
from lane_mapping_lya_reference import curve_evidence as evidence
from lane_mapping_lya_reference import curve_evidence_blocks as blocks
from lane_mapping_lya_reference import curve_retained_state as retained


COSINE = math.cos(math.pi / 6.)
HEADER_FIELDS = (
    'owner_id', 'signed_side', 'signature', 'origin', 'tangent',
    'normal_cosine', 'max_support_records', 'source_frontier_ns')
MATERIALIZED_FIELDS = HEADER_FIELDS + ('status', 'reason')
AFFINE_FIELDS = (
    'lower', 'upper', 'intercept', 'slope', 'occurrence', 'reliable',
    'lower_endpoint', 'upper_endpoint')
PIECE_FIELDS = (
    'lower', 'upper', 'intercept', 'slope', 'contributors',
    'reliable_contributors', 'source_stamp_ns', 'segment', 'world_derivative', 'normal')


def _bits(value):
    if isinstance(value, (float, np.floating)):
        return ('binary64', struct.pack('>d', float(value)))
    if type(value) is tuple:
        return ('tuple', tuple(_bits(item) for item in value))
    return (type(value).__name__, value)


def _state(capacity=200000):
    return compact.create_curve_state(
        7, 1, (-1, -1), (0., 0.), (1., 0.),
        normal_cosine=COSINE, max_support_records=capacity)


def _packet(stamp, *, reliable=True, duplicate=False, bad_direction=False,
            storage='dense', owner=7, side=1, signature=(-1, -1)):
    intercept = stamp * .0001
    roots = np.array([[0., intercept], [2., intercept + (2. if bad_direction else 0.)]],
                     dtype=np.float64)
    offsets = np.array([[-.0625, -.0625], [.0625, -.0625],
                        [.0625, .0625], [-.0625, .0625]], dtype=np.float64)
    source = evidence.SourceFrame(
        stamp_ns=stamp, face_ids=np.arange(2, dtype=np.int64),
        previous_next=np.array([[-1, 1], [0, -1]], dtype=np.int64),
        contour_ids=np.zeros(2, dtype=np.int64),
        endpoint_owners=np.full(2, owner, dtype=np.int64), model_roots=roots,
        normals=np.array([[0., 1.], [0., 1.]], dtype=np.float64),
        quads=roots[:, None, :] + offsets[None, :, :],
        quad_valid=np.ones(2, dtype=np.bool_),
        reliable=np.full(2, reliable, dtype=np.bool_), valid=np.ones(2, dtype=np.bool_))
    indices = np.array([[0, 1]] * (2 if duplicate else 1), dtype=np.int64)
    packet = evidence.PairPacket(source, owner, side, signature,
                                 np.ones(len(indices), dtype=np.uint8), indices)
    if storage == 'blocks':
        packet = blocks.project_frame(source, (packet,))[0][0]
    return packet


def _confirmed(storage='dense'):
    state = _state()
    for stamp in range(1, 6):
        state = compact.advance_curve(
            state, _packet(stamp, storage=storage), available_before_ns=stamp)
    return state


class RetainedCurveStateTests(unittest.TestCase):
    def assertEquivalent(self, actual, expected):
        self.assertIs(type(actual), compact.CurveState)
        for name in MATERIALIZED_FIELDS:
            self.assertEqual(_bits(getattr(actual, name)), _bits(getattr(expected, name)), name)
        self.assertEqual(len(actual.frames), len(expected.frames))
        for current, old in zip(actual.frames, expected.frames):
            self.assertIs(current.packet, old.packet)
            self.assertEqual(current.status, old.status)
            self.assertEqual(
                [tuple(_bits(getattr(piece, name)) for name in AFFINE_FIELDS)
                 for piece in current.pieces],
                [tuple(_bits(getattr(piece, name)) for name in AFFINE_FIELDS)
                 for piece in old.pieces])
        self.assertEqual(
            [tuple(_bits(getattr(piece, name)) for name in PIECE_FIELDS) for piece in actual.pieces],
            [tuple(_bits(getattr(piece, name)) for name in PIECE_FIELDS) for piece in expected.pieces])
        self.assertEqual(compact.inventory((actual,)), compact.inventory((expected,)))

    def advanceBoth(self, full, raw, packet, available=None):
        available = packet.stamp_ns if available is None else available
        expected = compact.advance_curve(full, packet, available_before_ns=available)
        actual = retained.advance(raw, packet, available_before_ns=available)
        self.assertEquivalent(actual, expected)
        self.assertEquivalent(retained.materialize(retained.from_materialized(actual)), actual)
        staged = retained.stage(raw, packet, available_before_ns=available)
        self.assertEquivalent(retained.materialize(staged), expected)
        return expected, staged

    def test_01_retention_shares_all_frames_and_affines_without_overlay_member(self):
        for storage in ('dense', 'blocks'):
            with self.subTest(storage=storage):
                full = _confirmed(storage)
                with mock.patch.object(arithmetic, '_overlay', side_effect=AssertionError('must not derive')):
                    raw = retained.from_materialized(full)
                self.assertIs(type(raw), retained.RetainedCurveState)
                self.assertIs(raw.frames, full.frames)
                self.assertIs(raw.origin, full.origin)
                self.assertIs(raw.tangent, full.tangent)
                self.assertFalse(hasattr(raw, 'pieces'))
                self.assertFalse(hasattr(raw, 'status'))
                self.assertFalse(hasattr(raw, 'reason'))
                self.assertEqual(set(vars(raw)), set(HEADER_FIELDS) | {'frames', '_seal'})
                for name in HEADER_FIELDS:
                    self.assertEqual(_bits(getattr(raw, name)), _bits(getattr(full, name)))
                self.assertTrue(all(a is b for a, b in zip(raw.frames, full.frames)))
                self.assertEquivalent(retained.materialize(raw), full)

    def test_02_overlay_objects_are_collectible_while_all_frame_objects_remain(self):
        for storage in ('dense', 'blocks'):
            with self.subTest(storage=storage):
                full = _confirmed(storage)
                self.assertTrue(full.pieces)
                overlay_refs = tuple(weakref.ref(piece) for piece in full.pieces)
                frame_refs = tuple(weakref.ref(frame) for frame in full.frames)
                raw = retained.from_materialized(full)
                del full
                gc.collect()
                self.assertTrue(all(reference() is None for reference in overlay_refs))
                self.assertTrue(all(reference() is frame
                                    for reference, frame in zip(frame_refs, raw.frames)))
                self.assertEqual(retained.materialize(raw).status, 'confirmed')

    def test_03_future_known_updates_match_original_dense_and_blocks(self):
        for storage in ('dense', 'blocks'):
            with self.subTest(storage=storage):
                full = _state()
                raw = retained.from_materialized(full)
                for stamp in range(1, 10):
                    full, raw = self.advanceBoth(full, raw, _packet(stamp, storage=storage))
                self.assertEqual(full.status, 'confirmed')

    def test_04_unknown_remains_until_original_retention_releases_it(self):
        for storage in ('dense', 'blocks'):
            with self.subTest(storage=storage):
                full = _state()
                raw = retained.from_materialized(full)
                for stamp in range(1, 7):
                    full, raw = self.advanceBoth(full, raw, _packet(
                        stamp, bad_direction=stamp == 1, storage=storage))
                    if stamp <= 5:
                        self.assertEqual(full.status, 'unknown')
                        self.assertEqual(full.reason, 'pair_direction')
                self.assertEqual([frame.packet.stamp_ns for frame in raw.frames], [2, 3, 4, 5, 6])
                self.assertEqual(full.status, 'confirmed')

    def test_05_latest_five_plus_old_reliable_and_replacement_match_original(self):
        for storage in ('dense', 'blocks'):
            with self.subTest(storage=storage):
                full = _state()
                raw = retained.from_materialized(full)
                for stamp in range(1, 9):
                    full, raw = self.advanceBoth(full, raw, _packet(
                        stamp, reliable=stamp in (1, 8), storage=storage))
                    if stamp == 7:
                        self.assertEqual([frame.packet.stamp_ns for frame in raw.frames],
                                         [1, 3, 4, 5, 6, 7])
                        self.assertEqual(len(raw.frames), 6)
                self.assertEqual([frame.packet.stamp_ns for frame in raw.frames], [4, 5, 6, 7, 8])

    def test_06_duplicate_occurrences_and_unknown_reason_are_not_erased(self):
        for storage in ('dense', 'blocks'):
            with self.subTest(storage=storage):
                full = _state()
                raw = retained.from_materialized(full)
                full, raw = self.advanceBoth(full, raw, _packet(1, duplicate=True, storage=storage))
                self.assertEqual(full.status, 'unknown')
                self.assertEqual(full.reason, 'overlap_or_nonunique_chart')
                self.assertEqual(raw.frames[0].packet.face_indices.tolist(), [[0, 1], [0, 1]])

    def test_07_duplicate_stale_and_future_stamps_leave_previous_unchanged(self):
        full = compact.advance_curve(_state(), _packet(2), available_before_ns=2)
        raw = retained.from_materialized(full)
        before = vars(raw).copy()
        for stamp, available in ((2, 2), (1, 2), (3, 2)):
            packet = _packet(stamp)
            with self.subTest(stamp=stamp), self.assertRaises(ValueError) as old_error:
                compact.advance_curve(full, packet, available_before_ns=available)
            with self.assertRaises(ValueError) as new_error:
                retained.advance(raw, packet, available_before_ns=available)
            self.assertEqual(str(new_error.exception), str(old_error.exception))
            self.assertTrue(all(getattr(raw, name) is value for name, value in before.items()))

    def test_08_owner_side_signature_and_mixed_storage_still_reject(self):
        full = compact.advance_curve(_state(), _packet(1), available_before_ns=1)
        raw = retained.from_materialized(full)
        for options in ({'owner': 9}, {'side': -1}, {'signature': (3, 9)}, {'storage': 'blocks'}):
            packet = _packet(2, **options)
            with self.subTest(options=options), self.assertRaises(ValueError) as old_error:
                compact.advance_curve(full, packet, available_before_ns=2)
            with self.assertRaises(ValueError) as new_error:
                retained.advance(raw, packet, available_before_ns=2)
            self.assertEqual(str(new_error.exception), str(old_error.exception))

    def test_09_original_capacity_gate_remains_in_shared_advance(self):
        full = _state(capacity=20)
        raw = retained.from_materialized(full)
        packet = _packet(1)
        with self.assertRaises(ValueError) as old_error:
            compact.advance_curve(full, packet, available_before_ns=1)
        with self.assertRaises(ValueError) as new_error:
            retained.advance(raw, packet, available_before_ns=1)
        self.assertEqual(str(new_error.exception), str(old_error.exception))
        self.assertIn('capacity', str(new_error.exception))
        self.assertEqual(raw.source_frontier_ns, 0)

    def test_10_materialize_recomputes_numerical_failure_without_mutating_history(self):
        full = _confirmed()
        raw = retained.from_materialized(full)
        before = vars(raw).copy()
        with mock.patch.object(arithmetic, '_endpoint', side_effect=ArithmeticError('changed environment')):
            rebuilt = retained.materialize(raw)
        self.assertEqual((rebuilt.status, rebuilt.reason, rebuilt.pieces),
                         ('unknown', 'numerical_export', ()))
        self.assertTrue(all(getattr(raw, name) is value for name, value in before.items()))
        self.assertFalse(hasattr(raw, 'status'))
        self.assertFalse(hasattr(raw, 'reason'))
        self.assertEquivalent(retained.materialize(raw), full)

    def test_11_materialize_invokes_one_current_overlay(self):
        full = _confirmed()
        raw = retained.from_materialized(full)
        original = arithmetic._overlay
        with mock.patch.object(arithmetic, '_overlay', wraps=original) as overlay:
            rebuilt = retained.materialize(raw)
        self.assertEqual(overlay.call_count, 1)
        self.assertIs(overlay.call_args[0][0], raw)
        self.assertIs(overlay.call_args[0][1], raw.frames)
        self.assertEquivalent(rebuilt, full)

    def test_12_advance_uses_retained_object_directly_and_only_one_overlay(self):
        raw = retained.from_materialized(_confirmed())
        self.assertFalse(hasattr(raw, 'pieces'))
        original_advance, original_overlay = compact._advance_prepared, arithmetic._overlay
        with mock.patch.object(compact, '_advance_prepared', wraps=original_advance) as updater:
            with mock.patch.object(arithmetic, '_overlay', wraps=original_overlay) as overlay:
                updated = retained.advance(raw, _packet(6), available_before_ns=6)
        self.assertEqual(updater.call_count, 1)
        self.assertIs(updater.call_args[0][0], raw)
        self.assertEqual(overlay.call_count, 1)
        self.assertEqual(updated.status, 'confirmed')
        self.assertIs(type(updated), compact.CurveState)

    def test_13_empty_raw_state_naturally_materializes_as_unconfirmed(self):
        raw = retained.from_materialized(_state())
        self.assertFalse(hasattr(raw, 'status'))
        self.assertFalse(hasattr(raw, 'reason'))
        rebuilt = retained.materialize(raw)
        self.assertEqual((rebuilt.status, rebuilt.reason, rebuilt.pieces), ('unconfirmed', '', ()))
        self.assertIs(rebuilt.frames, raw.frames)
        self.assertEqual(rebuilt.source_frontier_ns, 0)

    def test_14_wrong_type_subclasses_and_invalid_seals_reject(self):
        full = _confirmed()
        raw = retained.from_materialized(full)

        class CompactSubclass(compact.CurveState):
            pass

        class RetainedSubclass(retained.RetainedCurveState):
            pass

        for value in (None, raw, CompactSubclass(**vars(full))):
            with self.subTest(operation='from_materialized', kind=type(value).__name__), self.assertRaises(ValueError):
                retained.from_materialized(value)
        forged = object.__new__(retained.RetainedCurveState)
        for value in (None, full, forged, object.__new__(RetainedSubclass)):
            with self.subTest(operation='materialize', kind=type(value).__name__), self.assertRaises(ValueError):
                retained.materialize(value)
            with self.subTest(operation='advance', kind=type(value).__name__), self.assertRaises(ValueError):
                retained.advance(value, _packet(6), available_before_ns=6)
            with self.subTest(operation='stage', kind=type(value).__name__), self.assertRaises(ValueError):
                retained.stage(value, _packet(6), available_before_ns=6)
        fields = vars(raw).copy()
        fields['_seal'] = None
        with self.assertRaises(ValueError):
            retained.RetainedCurveState(**fields)
        with self.assertRaises(ValueError):
            RetainedSubclass(**vars(raw))

    def test_15_retained_is_frozen_and_not_accepted_as_a_materialized_inventory(self):
        raw = retained.from_materialized(_confirmed())
        with self.assertRaises(FrozenInstanceError):
            raw.source_frontier_ns = 999
        with self.assertRaises(ValueError):
            compact.inventory((raw,))
        with self.assertRaises(ValueError):
            compact.advance_curve(raw, _packet(6), available_before_ns=6)

    def test_16_stage_performs_zero_overlays_then_materialize_performs_exactly_one(self):
        for storage in ('dense', 'blocks'):
            with self.subTest(storage=storage):
                full = _confirmed(storage)
                raw = retained.from_materialized(full)
                packet = _packet(6, storage=storage)
                expected = compact.advance_curve(full, packet, available_before_ns=6)
                with mock.patch.object(arithmetic, '_overlay', side_effect=AssertionError('stage derived overlay')) as overlay:
                    staged = retained.stage(raw, packet, available_before_ns=6)
                self.assertEqual(overlay.call_count, 0)
                self.assertEqual(set(vars(staged)), set(HEADER_FIELDS) | {'frames', '_seal'})
                self.assertTrue(all(a is b for a, b in zip(staged.frames[:-1], raw.frames[1:])))
                original_overlay = arithmetic._overlay
                with mock.patch.object(arithmetic, '_overlay', wraps=original_overlay) as overlay:
                    rebuilt = retained.materialize(staged)
                self.assertEqual(overlay.call_count, 1)
                self.assertEquivalent(rebuilt, expected)

    def test_17_stage_preserves_current_type_time_key_and_storage_rejections(self):
        full = compact.advance_curve(_state(), _packet(2), available_before_ns=2)
        raw = retained.from_materialized(full)
        before = vars(raw).copy()

        class PacketSubclass(evidence.PairPacket):
            pass

        cases = [(_packet(stamp), available) for stamp, available in
                 ((2, 2), (1, 2), (3, 2), (3, True), (3, 1.5), (3, 0))]
        cases += [(_packet(3, **options), 3) for options in
                  ({'owner': 9}, {'side': -1}, {'signature': (3, 9)}, {'storage': 'blocks'})]
        cases += [(None, 3), (object.__new__(PacketSubclass), 3)]
        for index, (packet, available) in enumerate(cases):
            with self.subTest(case=index):
                with self.assertRaises(ValueError) as old_error:
                    compact.advance_curve(full, packet, available_before_ns=available)
                with mock.patch.object(arithmetic, '_overlay', side_effect=AssertionError('invalid stage derived overlay')):
                    with self.assertRaises(ValueError) as new_error:
                        retained.stage(raw, packet, available_before_ns=available)
                self.assertEqual(str(new_error.exception), str(old_error.exception))
                self.assertTrue(all(getattr(raw, name) is value for name, value in before.items()))

    def test_18_stage_rejects_raw_capacity_lower_bound_without_overlay(self):
        full = _state(capacity=20)
        raw = retained.from_materialized(full)
        packet = _packet(1)
        with self.assertRaises(ValueError) as old_error:
            compact.advance_curve(full, packet, available_before_ns=1)
        with mock.patch.object(arithmetic, '_overlay', side_effect=AssertionError('capacity stage derived overlay')):
            with self.assertRaises(ValueError) as new_error:
                retained.stage(raw, packet, available_before_ns=1)
        self.assertEqual(str(new_error.exception), str(old_error.exception))
        self.assertEqual((raw.source_frontier_ns, raw.frames), (0, ()))

    def test_19_materialize_rejects_complete_piece_capacity_before_returning(self):
        for storage in ('dense', 'blocks'):
            with self.subTest(storage=storage):
                full = _confirmed(storage)
                key = (full.owner_id, full.signed_side, full.signature)
                raw_charge = compact._inventory_parts({key: full}, ())['support_records']
                self.assertGreater(compact.inventory((full,))['support_records'], raw_charge)
                previous = _state(capacity=raw_charge)
                for frame in full.frames[:-1]:
                    previous = compact.advance_curve(previous, frame.packet,
                                                     available_before_ns=frame.packet.stamp_ns)
                raw = retained.from_materialized(previous)
                packet = full.frames[-1].packet
                staged = retained.stage(raw, packet, available_before_ns=packet.stamp_ns)
                self.assertEqual(compact._inventory_parts({key: staged}, ())['support_records'], raw_charge)
                with self.assertRaises(ValueError) as old_error:
                    compact.advance_curve(previous, packet, available_before_ns=packet.stamp_ns)
                with self.assertRaises(ValueError) as new_error:
                    retained.materialize(staged)
                self.assertEqual(str(new_error.exception), str(old_error.exception))
                self.assertEqual(str(new_error.exception), 'complete compact support capacity exceeded')
                self.assertEqual(raw.source_frontier_ns, 4)
                self.assertEqual(staged.source_frontier_ns, 5)


if __name__ == '__main__':
    unittest.main()
