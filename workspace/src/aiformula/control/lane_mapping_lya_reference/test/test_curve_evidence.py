"""Independent artificial source-table tests; PREPARED, not executed.

No mask, pose, ROS, native module or original curve implementation is imported.
Fixtures and record/scalar/payload expectations are literal independent data.
These tests do not authorize geometry integration or runtime/vehicle claims.
"""
from dataclasses import FrozenInstanceError
import sys
import unittest

import numpy as np

from lane_mapping_lya_reference import curve_evidence as subject


SOURCE_FIELDS = (
    'face_ids', 'previous_next', 'contour_ids', 'endpoint_owners',
    'model_roots', 'normals', 'quads', 'quad_valid', 'reliable', 'valid',
)
INTEGER_FIELDS = ('face_ids', 'previous_next', 'contour_ids', 'endpoint_owners')
FLOAT_FIELDS = ('model_roots', 'normals', 'quads')
BOOL_FIELDS = ('quad_valid', 'reliable', 'valid')


def _nan_bits(bits):
    return np.array([bits], dtype=np.uint64).view(np.float64)[0]


def _source_arguments(stamp=123):
    # Two literal original directed chains, plus an unowned invalid face.
    # Face4's invalid placeholders remain evidence, not deleted geometry.
    roots = np.array([[-0., 0.], [1., 0.], [10., -0.], [11., 0.],
                      [_nan_bits(0x7ff8000000000042), -np.inf]], dtype=np.float64)
    normals = np.array([[0., 1.], [0., 1.], [0., 1.], [0., 1.],
                        [np.inf, _nan_bits(0x7ff8000000000071)]], dtype=np.float64)
    quads = np.array([
        [[-.25, -.25], [.25, -.25], [.25, .25], [-.25, .25]],
        [[.75, -.25], [1.25, -.25], [1.25, .25], [.75, .25]],
        [[9.75, -.25], [10.25, -.25], [10.25, .25], [9.75, .25]],
        [[10.75, -.25], [11.25, -.25], [11.25, .25], [10.75, .25]],
        [[_nan_bits(0x7ff80000000000ab), -np.inf], [np.inf, -0.],
         [_nan_bits(0x7ff80000000000cd), 0.], [0., 0.]],
    ], dtype=np.float64)
    return dict(
        stamp_ns=stamp,
        face_ids=np.array([0, 1, 2, 3, 4], dtype=np.int64),
        previous_next=np.array([[-1, 1], [0, -1], [-1, 3], [2, -1],
                                [-1, -1]], dtype=np.int64),
        contour_ids=np.array([0, 0, 1, 1, 2], dtype=np.int64),
        endpoint_owners=np.array([7, 7, 7, 9, -1], dtype=np.int64),
        model_roots=roots, normals=normals, quads=quads,
        quad_valid=np.array([True, True, True, True, False], dtype=np.bool_),
        reliable=np.array([True, True, True, False, False], dtype=np.bool_),
        valid=np.array([True, True, True, True, False], dtype=np.bool_),
    )


def _empty_arguments(stamp=1):
    return dict(stamp_ns=stamp, face_ids=np.empty((0,), dtype=np.int64),
                previous_next=np.empty((0, 2), dtype=np.int64),
                contour_ids=np.empty((0,), dtype=np.int64),
                endpoint_owners=np.empty((0,), dtype=np.int64),
                model_roots=np.empty((0, 2), dtype=np.float64),
                normals=np.empty((0, 2), dtype=np.float64),
                quads=np.empty((0, 4, 2), dtype=np.float64),
                quad_valid=np.empty((0,), dtype=np.bool_),
                reliable=np.empty((0,), dtype=np.bool_),
                valid=np.empty((0,), dtype=np.bool_))


def _packet(source, owner=7, indices=((2, 3), (0, 1), (2, 3)),
            roles=(2, 1, 2), side=1, signature=(-1, 9)):
    return subject.PairPacket(source=source, owner_id=owner, signed_side=side,
                              signature=signature,
                              roles=np.array(roles, dtype=np.uint8),
                              face_indices=np.array(indices, dtype=np.int64).reshape(-1, 2))


def _raw(array):
    return array.dtype.str, array.shape, array.tobytes(order='C')


class CurveEvidenceIndependentTests(unittest.TestCase):
    def assertRawEqual(self, actual, expected):
        self.assertIs(type(actual), np.ndarray)
        self.assertEqual(_raw(actual), _raw(expected))

    def test_01_source_preserves_all_ten_original_fields(self):
        arguments = _source_arguments()
        expected = {name: _raw(arguments[name]) for name in SOURCE_FIELDS}
        table = subject.SourceFrame(**arguments)
        self.assertEqual(table.stamp_ns, 123)
        self.assertEqual(table.source_kind, 'actual_final_observation_directed_root_pairs')
        for name in SOURCE_FIELDS:
            with self.subTest(field=name):
                self.assertEqual(_raw(getattr(table, name)), expected[name])
                self.assertFalse(getattr(table, name).flags.writeable)

    def test_02_all_eleven_pair_fields_roundtrip_dtype_shape_and_bytes(self):
        arguments = _source_arguments()
        table = subject.SourceFrame(**arguments)
        packet = _packet(table)
        expected_indices = np.array([[2, 3], [0, 1], [2, 3]], dtype=np.int64)
        self.assertRawEqual(packet.roles, np.array([2, 1, 2], dtype=np.uint8))
        for name in SOURCE_FIELDS:
            with self.subTest(field=name):
                # Oracle uses the original caller data, not subject tables.
                self.assertRawEqual(packet.gather_original(name), arguments[name][expected_indices])

    def test_03_signed_zero_bits_survive_source_and_pair_gather(self):
        table = subject.SourceFrame(**_source_arguments())
        packet = _packet(table, indices=((0, 1),), roles=(1,))
        self.assertEqual(table.model_roots.view(np.uint64)[0, 0], 0x8000000000000000)
        self.assertEqual(table.model_roots.view(np.uint64)[0, 1], 0)
        self.assertEqual(packet.gather_original('model_roots').view(np.uint64)[0, 0, 0],
                         0x8000000000000000)

    def test_04_invalid_nan_payloads_are_retained_without_normalizing(self):
        arguments = _source_arguments()
        table = subject.SourceFrame(**arguments)
        for name in FLOAT_FIELDS:
            self.assertEqual(getattr(table, name)[4].tobytes(), arguments[name][4].tobytes())
        self.assertEqual(table.model_roots.view(np.uint64)[4, 0], 0x7ff8000000000042)
        self.assertEqual(table.normals.view(np.uint64)[4, 1], 0x7ff8000000000071)
        self.assertEqual(table.quads.view(np.uint64)[4, 0, 0], 0x7ff80000000000ab)

    def test_05_valid_own_quad_is_independent_of_nominal_validity(self):
        arguments = _source_arguments()
        arguments['quad_valid'][4] = True
        arguments['quads'][4] = arguments['quads'][0]
        table = subject.SourceFrame(**arguments)
        self.assertFalse(table.valid[4])
        self.assertEqual(table.endpoint_owners[4], -1)
        self.assertTrue(table.quad_valid[4])
        self.assertRawEqual(table.quads, arguments['quads'])
        self.assertEqual(subject.reachable_inventory([_packet(table)])['source_faces'], 5)

    def test_06_invalid_quad_bytes_survive_for_a_valid_nominal_face(self):
        arguments = _source_arguments()
        arguments['quad_valid'][0] = False
        arguments['quads'][0] = arguments['quads'][4]
        table = subject.SourceFrame(**arguments)
        packet = _packet(table, indices=((0, 1),), roles=(1,))
        self.assertTrue(packet.gather_original('valid')[0, 0])
        self.assertFalse(packet.gather_original('quad_valid')[0, 0])
        self.assertEqual(packet.gather_original('quads')[0, 0].tobytes(),
                         arguments['quads'][0].tobytes())

    def test_07_incident_both_owners_share_source_not_packet_arrays(self):
        table = subject.SourceFrame(**_source_arguments())
        left = _packet(table, indices=((2, 3),), roles=(2,))
        right = _packet(table, owner=9, indices=((2, 3),), roles=(2,), signature=(7, -1))
        self.assertIs(left.source, right.source)
        for name in ('roles', 'face_indices'):
            self.assertRawEqual(getattr(left, name), getattr(right, name))
            self.assertFalse(getattr(left, name).flags.writeable)
            self.assertFalse(getattr(right, name).flags.writeable)
            self.assertFalse(np.shares_memory(getattr(left, name), getattr(right, name)))
        self.assertRawEqual(left.gather_original('endpoint_owners'),
                            np.array([[7, 9]], dtype=np.int64))

    def test_08_occurrence_order_roles_and_multiplicity_are_not_deduplicated(self):
        packet = _packet(subject.SourceFrame(**_source_arguments()))
        self.assertEqual(packet.face_indices.tolist(), [[2, 3], [0, 1], [2, 3]])
        self.assertEqual(packet.roles.tolist(), [2, 1, 2])
        self.assertEqual(len(packet.roles), 3)
        self.assertEqual(packet.support_records, 16)
        self.assertEqual(packet.gather_original('face_ids').tolist(), [[2, 3], [0, 1], [2, 3]])

    def test_09_caller_source_array_mutation_is_isolated(self):
        arguments = _source_arguments()
        expected = {name: _raw(arguments[name]) for name in SOURCE_FIELDS}
        table = subject.SourceFrame(**arguments)
        for name in SOURCE_FIELDS:
            arguments[name][...] = 0
        for name in SOURCE_FIELDS:
            self.assertEqual(_raw(getattr(table, name)), expected[name])

    def test_10_caller_roles_and_indices_mutation_is_isolated(self):
        table = subject.SourceFrame(**_source_arguments())
        roles = np.array([2, 1, 2], dtype=np.uint8)
        indices = np.array([[2, 3], [0, 1], [2, 3]], dtype=np.int64)
        packet = subject.PairPacket(table, 7, 1, (-1, 9), roles, indices)
        roles[:] = 255
        indices[:] = -1
        self.assertEqual(packet.roles.tolist(), [2, 1, 2])
        self.assertEqual(packet.face_indices.tolist(), [[2, 3], [0, 1], [2, 3]])

    def test_11_source_strides_and_readonly_layout_keep_original_c_bytes(self):
        arguments = _source_arguments()
        original = {name: _raw(arguments[name]) for name in SOURCE_FIELDS}
        for name in SOURCE_FIELDS:
            value = arguments[name]
            backing = np.empty((2 * len(value),) + value.shape[1:], dtype=value.dtype)
            backing[::2] = value
            arguments[name] = backing[::2]
            arguments[name].flags.writeable = False
        table = subject.SourceFrame(**arguments)
        for name in SOURCE_FIELDS:
            self.assertEqual(_raw(getattr(table, name)), original[name])
            self.assertTrue(getattr(table, name).flags.c_contiguous)

    def test_12_packet_strides_readonly_and_frozen_fields(self):
        table = subject.SourceFrame(**_source_arguments())
        roles = np.array([2, 99, 1, 99, 2, 99], dtype=np.uint8)[::2]
        indices = np.array([[2, 3], [-1, -1], [0, 1], [-1, -1],
                            [2, 3], [-1, -1]], dtype=np.int64)[::2]
        roles.flags.writeable = False
        indices.flags.writeable = False
        packet = subject.PairPacket(table, 7, 1, (-1, 9), roles, indices)
        self.assertRawEqual(packet.roles, roles)
        self.assertRawEqual(packet.face_indices, indices)
        with self.assertRaises(FrozenInstanceError):
            packet.owner_id = 8
        with self.assertRaises(FrozenInstanceError):
            table.stamp_ns = 456
        for array in (packet.roles, packet.face_indices, table.model_roots):
            with self.assertRaises(ValueError):
                array.flags.writeable = True

    def test_13_empty_source_and_empty_packet_exact_inventory(self):
        table = subject.SourceFrame(**_empty_arguments())
        packet = _packet(table, indices=(), roles=())
        self.assertFalse(packet._has_reliable)
        self.assertEqual(subject.reachable_inventory([packet]), dict(
            source_tables=1, packets=1, source_faces=0, pair_occurrences=0,
            support_records=11, scalar_entries=43, payload_bytes=0))
        for name in SOURCE_FIELDS:
            value = packet.gather_original(name)
            self.assertEqual(value.shape[:2], (0, 2))
            self.assertEqual(value.dtype, getattr(table, name).dtype)

    def test_14_contiguous_face_id_inventory_is_required(self):
        for ids in ((0, 1, 2, 3, -1), (0, 1, 2, 3, 3),
                    (1, 0, 2, 3, 4), (0, 1, 2, 3, 5)):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                arguments = _source_arguments()
                arguments['face_ids'] = np.array(ids, dtype=np.int64)
                subject.SourceFrame(**arguments)

    def test_15_neighbor_bounds_and_reciprocal_links_are_required(self):
        for row, side, value in ((0, 0, -2), (0, 1, 5), (1, 0, -1), (1, 0, 2)):
            with self.subTest(change=(row, side, value)), self.assertRaises(ValueError):
                arguments = _source_arguments()
                arguments['previous_next'][row, side] = value
                subject.SourceFrame(**arguments)

    def test_16_contour_ids_must_be_nonnegative_and_reciprocal(self):
        for row, value in ((4, -1), (1, 6)):
            with self.subTest(change=(row, value)), self.assertRaises(ValueError):
                arguments = _source_arguments()
                arguments['contour_ids'][row] = value
                subject.SourceFrame(**arguments)

    def test_17_owner_and_nominal_validity_must_agree(self):
        for row, value in ((4, -2), (4, 7), (0, -1)):
            with self.subTest(change=(row, value)), self.assertRaises(ValueError):
                arguments = _source_arguments()
                arguments['endpoint_owners'][row] = value
                subject.SourceFrame(**arguments)
        arguments = _source_arguments()
        arguments['valid'][0] = False
        arguments['reliable'][0] = False
        with self.assertRaises(ValueError):
            subject.SourceFrame(**arguments)

    def test_18_reliable_invalid_nominal_source_is_rejected(self):
        arguments = _source_arguments()
        arguments['reliable'][4] = True
        with self.assertRaises(ValueError):
            subject.SourceFrame(**arguments)

    def test_19_each_source_dtype_is_exact_native_original_dtype(self):
        for name in SOURCE_FIELDS:
            arguments = _source_arguments()
            wrong = np.int32 if name in INTEGER_FIELDS else np.float32 if name in FLOAT_FIELDS else np.uint8
            arguments[name] = arguments[name].astype(wrong)
            with self.subTest(field=name), self.assertRaises(ValueError):
                subject.SourceFrame(**arguments)

    def test_20_each_source_shape_is_exact(self):
        for name in SOURCE_FIELDS:
            arguments = _source_arguments()
            arguments[name] = arguments[name].reshape(arguments[name].shape + (1,))
            with self.subTest(field=name), self.assertRaises(ValueError):
                subject.SourceFrame(**arguments)

    def test_21_foreign_endian_integer_and_float_fields_are_rejected(self):
        order = '>' if sys.byteorder == 'little' else '<'
        for name in INTEGER_FIELDS + FLOAT_FIELDS:
            arguments = _source_arguments()
            arguments[name] = arguments[name].astype(arguments[name].dtype.newbyteorder(order))
            with self.subTest(field=name), self.assertRaises(ValueError):
                subject.SourceFrame(**arguments)

    def test_22_subclasses_and_conversion_protocols_are_not_called(self):
        class ArraySubclass(np.ndarray):
            pass
        class Foreign:
            def __array__(self, *args, **kwargs):
                raise AssertionError('foreign conversion must not execute')
        for name in SOURCE_FIELDS:
            for bad in ('subclass', 'protocol', 'list'):
                arguments = _source_arguments()
                arguments[name] = (arguments[name].view(ArraySubclass) if bad == 'subclass'
                                   else Foreign() if bad == 'protocol' else arguments[name].tolist())
                with self.subTest(field=name, bad=bad), self.assertRaises(ValueError):
                    subject.SourceFrame(**arguments)

    def test_23_valid_geometry_nonfinite_unbounded_and_nonunit_reject(self):
        for name, value in (('model_roots', np.nan), ('model_roots', np.inf),
                            ('model_roots', 1.e7 + 1.), ('normals', np.nan),
                            ('normals', 2.), ('quads', np.inf), ('quads', 1.e7 + 1.)):
            arguments = _source_arguments()
            arguments[name][0] = value
            with self.subTest(field=name, value=value), self.assertRaises(ValueError):
                subject.SourceFrame(**arguments)
        arguments = _source_arguments()
        arguments['normals'][0] = (0., 0.)
        with self.assertRaises(ValueError):
            subject.SourceFrame(**arguments)

    def test_24_source_stamp_and_kind_contract(self):
        for stamp in (True, np.bool_(True), 0, -1, 1.5, 2**63):
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                subject.SourceFrame(**_source_arguments(stamp))
        arguments = _source_arguments()
        arguments['source_kind'] = 'declared_but_not_original'
        with self.assertRaises(ValueError):
            subject.SourceFrame(**arguments)
        table = subject.SourceFrame(**_source_arguments(np.int64(123)))
        self.assertIs(type(table.stamp_ns), int)

    def test_25_pair_role_and_both_endpoint_owners_are_required(self):
        table = subject.SourceFrame(**_source_arguments())
        cases = ((7, ((2, 3),), (1,)), (7, ((0, 1),), (2,)),
                 (8, ((2, 3),), (2,)), (9, ((0, 1),), (1,)),
                 (7, ((2, 3),), (0,)), (7, ((2, 3),), (3,)))
        for owner, indices, roles in cases:
            with self.subTest(owner=owner, indices=indices, roles=roles), self.assertRaises(ValueError):
                _packet(table, owner=owner, indices=indices, roles=roles)

    def test_26_pair_indices_must_be_original_directed_distinct_next(self):
        table = subject.SourceFrame(**_source_arguments())
        for indices in (((-1, 0),), ((0, 5),), ((0, 0),), ((1, 0),),
                        ((0, 2),), ((3, 4),)):
            with self.subTest(indices=indices), self.assertRaises(ValueError):
                _packet(table, indices=indices, roles=(1,))

    def test_27_packet_dtype_shape_endian_and_protocol_reject(self):
        table = subject.SourceFrame(**_source_arguments())
        roles = np.array([2], dtype=np.uint8)
        indices = np.array([[2, 3]], dtype=np.int64)
        order = '>' if sys.byteorder == 'little' else '<'
        bads = ((roles.astype(np.int64), indices), (roles[:, None], indices),
                (roles, indices.astype(np.int32)), (roles, indices.astype(np.float64)),
                (roles, indices.astype(indices.dtype.newbyteorder(order))),
                (roles, indices.reshape(2)), (roles, np.zeros((1, 3), dtype=np.int64)),
                (roles.tolist(), indices), (roles, indices.tolist()))
        for bad_roles, bad_indices in bads:
            with self.subTest(role_type=type(bad_roles).__name__, index_type=type(bad_indices).__name__), self.assertRaises(ValueError):
                subject.PairPacket(table, 7, 1, (-1, 9), bad_roles, bad_indices)

    def test_28_packet_key_bounds_side_signature_and_wrong_source_reject(self):
        table = subject.SourceFrame(**_source_arguments())
        for owner in (True, -1, 1.5, 2**63):
            with self.subTest(owner=owner), self.assertRaises(ValueError):
                _packet(table, owner=owner)
        for side in (True, 0, -2, 2, 1.5):
            with self.subTest(side=side), self.assertRaises(ValueError):
                _packet(table, side=side)
        for signature in ((-2, 9), (True, 9), (2**63, 9), (-1,), [-1, 9]):
            with self.subTest(signature=signature), self.assertRaises(ValueError):
                _packet(table, signature=signature)
        with self.assertRaises(ValueError):
            _packet(None)
        packet = _packet(table, owner=np.int64(7), side=np.int64(-1))
        self.assertEqual(packet.signed_side, -1)
        self.assertIs(type(packet.owner_id), int)

    def test_29_reliability_cache_is_original_pair_truth_not_owner_guess(self):
        table = subject.SourceFrame(**_source_arguments())
        self.assertTrue(_packet(table)._has_reliable)
        self.assertFalse(_packet(table, owner=9, indices=((2, 3),),
                                 roles=(2,), signature=(7, -1))._has_reliable)

    def test_30_inventory_charges_full_source_once_and_each_packet(self):
        table = subject.SourceFrame(**_source_arguments())
        left = _packet(table)
        right = _packet(table, owner=9, indices=((2, 3),), roles=(2,), signature=(7, -1))
        # Source5*20+28 scalars,5*139 payload. Packet3/1 occurrences:
        # support16/12, scalars24/18, payload51/17. No subject formula used.
        self.assertEqual(subject.reachable_inventory([left, right]), dict(
            source_tables=1, packets=2, source_faces=5, pair_occurrences=4,
            support_records=34, scalar_entries=170, payload_bytes=763))

    def test_31_repeated_reference_counts_one_physical_packet(self):
        packet = _packet(subject.SourceFrame(**_source_arguments()))
        self.assertEqual(subject.reachable_inventory([packet, packet]), dict(
            source_tables=1, packets=1, source_faces=5, pair_occurrences=3,
            support_records=22, scalar_entries=152, payload_bytes=746))

    def test_32_equal_byte_distinct_source_allocations_are_both_charged(self):
        first = subject.SourceFrame(**_source_arguments())
        second = subject.SourceFrame(**_source_arguments())
        self.assertIsNot(first, second)
        for name in SOURCE_FIELDS:
            self.assertRawEqual(getattr(first, name), getattr(second, name))
            self.assertFalse(np.shares_memory(getattr(first, name), getattr(second, name)))
        left = _packet(first, indices=((2, 3),), roles=(2,))
        right = _packet(second, owner=9, indices=((2, 3),), roles=(2,), signature=(7, -1))
        self.assertEqual(subject.reachable_inventory([left, right]), dict(
            source_tables=2, packets=2, source_faces=10, pair_occurrences=2,
            support_records=36, scalar_entries=292, payload_bytes=1424))

    def test_33_both_signatures_share_source_but_keep_packet_metadata(self):
        table = subject.SourceFrame(**_source_arguments())
        old = _packet(table, indices=((0, 1),), roles=(1,), signature=(-1, 9))
        new = _packet(table, indices=((0, 1),), roles=(1,), signature=(3, 9))
        self.assertNotEqual(old.signature, new.signature)
        self.assertIs(old.source, new.source)
        self.assertEqual(subject.reachable_inventory([old, new]), dict(
            source_tables=1, packets=2, source_faces=5, pair_occurrences=2,
            support_records=30, scalar_entries=164, payload_bytes=729))

    def test_34_stale_unknown_and_empty_owner_packets_pin_full_old_table(self):
        old_table = subject.SourceFrame(**_source_arguments(1))
        current_table = subject.SourceFrame(**_source_arguments(100))
        unknown = _packet(old_table, indices=((2, 3),), roles=(2,))
        empty = _packet(old_table, owner=55, indices=(), roles=(), signature=(-1, -1))
        current = _packet(current_table, owner=9, indices=((2, 3),), roles=(2,), signature=(7, -1))
        # Artificial outer state labels are not evidence-helper selection rules.
        all_staged = {'unknown_old_signature': (unknown,),
                      'empty_stale_owner': (empty,), 'current': (current,)}
        complete_packets = [packet for frames in all_staged.values() for packet in frames]
        self.assertEqual(subject.reachable_inventory(complete_packets), dict(
            source_tables=2, packets=3, source_faces=10, pair_occurrences=2,
            support_records=46, scalar_entries=307, payload_bytes=1424))
        self.assertEqual(unknown.stamp_ns, 1)
        self.assertEqual(empty.stamp_ns, 1)
        self.assertFalse(empty._has_reliable)

    def test_35_gather_is_transient_and_cannot_mutate_shared_source(self):
        packet = _packet(subject.SourceFrame(**_source_arguments()))
        before = _raw(packet.source.model_roots)
        gathered = packet.gather_original('model_roots')
        self.assertFalse(np.shares_memory(gathered, packet.source.model_roots))
        gathered[:] = 0.
        self.assertEqual(_raw(packet.source.model_roots), before)
        with self.assertRaises(ValueError):
            packet.gather_original('source_kind')

    def test_36_noncanonical_boolean_backing_is_not_silently_repacked(self):
        arguments = _source_arguments()
        arguments['valid'].view(np.uint8)[0] = 255
        arguments['reliable'].view(np.uint8)[0] = 2
        arguments['quad_valid'].view(np.uint8)[0] = 128
        table = subject.SourceFrame(**arguments)
        packet = _packet(table, indices=((0, 1),), roles=(1,))
        for name in BOOL_FIELDS:
            self.assertRawEqual(getattr(table, name), arguments[name])
            self.assertRawEqual(packet.gather_original(name), arguments[name][np.array([[0, 1]])])

    def test_37_capacity_limits_reject_oversized_shapes_before_allocating_full_fields(self):
        arguments = _source_arguments()
        arguments['face_ids'] = np.empty((200001,), dtype=np.int64)
        with self.assertRaisesRegex(ValueError, 'source face record capacity'):
            subject.SourceFrame(**arguments)
        table = subject.SourceFrame(**_source_arguments())
        with self.assertRaisesRegex(ValueError, 'pair occurrence capacity'):
            subject.PairPacket(table, 7, 1, (-1, 9), np.empty((0,), dtype=np.uint8),
                               np.empty((200001, 2), dtype=np.int64))
        self.assertEqual(subject.MAX_RECORDS, 200000)

    def test_38_empty_inventory_and_wrong_packet_types(self):
        self.assertEqual(subject.reachable_inventory([]), dict(
            source_tables=0, packets=0, source_faces=0, pair_occurrences=0,
            support_records=0, scalar_entries=0, payload_bytes=0))
        for wrong in (None, {}, object(), subject.SourceFrame(**_source_arguments())):
            with self.subTest(type=type(wrong).__name__), self.assertRaises(ValueError):
                subject.reachable_inventory([wrong])

    def test_39_legacy_normal_property_is_explicit_transient_gather(self):
        arguments = _source_arguments()
        packet = _packet(subject.SourceFrame(**arguments))
        first, second = packet.normals, packet.normals
        expected = arguments['normals'][np.array([[2, 3], [0, 1], [2, 3]])]
        self.assertRawEqual(first, expected)
        self.assertRawEqual(second, expected)
        self.assertFalse(np.shares_memory(first, packet.source.normals))
        self.assertFalse(np.shares_memory(first, second))
        before = _raw(packet.source.normals)
        first[:] = 0.
        self.assertEqual(_raw(packet.source.normals), before)


if __name__ == '__main__':
    unittest.main()
