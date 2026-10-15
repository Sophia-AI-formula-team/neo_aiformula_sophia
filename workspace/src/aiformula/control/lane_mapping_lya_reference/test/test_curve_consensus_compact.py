"""Prepared independent compact/v2 parity and artificial adapter tests.

The unchanged v2 finite arithmetic is the byte oracle, not a runtime oracle.
No production mapper, native module, mask, pose, GNSS or Unity is imported.
Capacity dimensions intentionally differ; geometry/source bytes must not.
"""
from fractions import Fraction
import math
import struct
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

from lane_mapping_lya_reference import curve_consensus as original
from lane_mapping_lya_reference import curve_consensus_compact as compact
from lane_mapping_lya_reference import curve_evidence as evidence
from lane_mapping_lya_reference import curve_observation_adapter as adapter


COSINE = math.cos(math.pi / 6.)
RAW_FIELDS = ('roles', 'face_ids', 'previous_next', 'contour_ids', 'endpoint_owners',
              'model_roots', 'normals', 'quads', 'quad_valid', 'reliable', 'valid')
AFFINE_FIELDS = ('lower', 'upper', 'intercept', 'slope', 'occurrence', 'reliable',
                 'lower_endpoint', 'upper_endpoint')
PIECE_FIELDS = ('lower', 'upper', 'intercept', 'slope', 'contributors',
                'reliable_contributors', 'source_stamp_ns', 'segment',
                'world_derivative', 'normal')
STATE_FIELDS = ('owner_id', 'signed_side', 'signature', 'origin', 'tangent',
                'normal_cosine', 'source_frontier_ns', 'status', 'reason')


def _bits(value):
    if isinstance(value, (float, np.floating)):
        return ('binary64', struct.pack('>d', float(value)))
    if isinstance(value, (tuple, list)):
        return (type(value).__name__, tuple(_bits(item) for item in value))
    return (type(value).__name__, value)


def _raw(value):
    return value.dtype.str, value.shape, value.tobytes(order='C')


def _legacy(source_arguments, indices, roles, owner, side, signature):
    # Entire oracle packet comes from independent caller arrays, not gathers.
    return original.FrameCurve(
        owner_id=owner, signed_side=side, signature=signature,
        stamp_ns=source_arguments['stamp_ns'], roles=roles,
        face_ids=source_arguments['face_ids'][indices],
        previous_next=source_arguments['previous_next'][indices],
        contour_ids=source_arguments['contour_ids'][indices],
        endpoint_owners=source_arguments['endpoint_owners'][indices],
        model_roots=source_arguments['model_roots'][indices],
        normals=source_arguments['normals'][indices], quads=source_arguments['quads'][indices],
        quad_valid=source_arguments['quad_valid'][indices],
        reliable=source_arguments['reliable'][indices], valid=source_arguments['valid'][indices])


def _linear_packets(stamp, intervals=((0., 2.),), *, owner=7, side=1,
                    signature=(-1, -1), slope=0., intercept=0., reliable=True,
                    reverse=False, forced_normal=None, world_origin=(0., 0.),
                    duplicate=False, invalid_quads=False):
    values, contours, links, pairs = [], [], [], []
    last_u, last_face, contour = None, None, -1
    for lower, upper in intervals:
        start, stop = (upper, lower) if reverse else (lower, upper)
        if last_face is not None and start == last_u:
            first = last_face
        else:
            contour += 1
            first = len(values)
            values.append(start); contours.append(contour); links.append([-1, -1])
        second = len(values)
        values.append(stop); contours.append(contour); links.append([first, -1])
        links[first][1] = second
        pairs.append((first, second))
        last_u, last_face = stop, second
    count = len(values)
    length = math.hypot(slope, 1.)
    normal = forced_normal if forced_normal is not None else (-slope / length, 1. / length)
    roots = np.array([(world_origin[0] + u, world_origin[1] + intercept + slope * u)
                      for u in values], dtype=np.float64).reshape(-1, 2)
    quads = np.array([[(float(x) + dx, float(y) + dy)
                       for dx, dy in ((-.0625, -.0625), (.0625, -.0625),
                                      (.0625, .0625), (-.0625, .0625))]
                      for x, y in roots], dtype=np.float64).reshape(-1, 4, 2)
    if invalid_quads:
        quads[...] = np.nan
    source_arguments = dict(
        stamp_ns=stamp, face_ids=np.arange(count, dtype=np.int64),
        previous_next=np.array(links, dtype=np.int64).reshape(-1, 2),
        contour_ids=np.array(contours, dtype=np.int64),
        endpoint_owners=np.full(count, owner, dtype=np.int64), model_roots=roots,
        normals=np.array([normal] * count, dtype=np.float64).reshape(-1, 2), quads=quads,
        quad_valid=np.full(count, not invalid_quads, dtype=np.bool_),
        reliable=np.full(count, reliable, dtype=np.bool_), valid=np.ones(count, dtype=np.bool_))
    if duplicate:
        pairs = pairs + pairs
    indices = np.array(pairs, dtype=np.int64).reshape(-1, 2)
    roles = np.ones(len(indices), dtype=np.uint8)
    source = evidence.SourceFrame(**source_arguments)
    packet = evidence.PairPacket(source, owner, side, signature, roles, indices)
    return packet, _legacy(source_arguments, indices, roles, owner, side, signature)


def _states(owner=7, side=1, signature=(-1, -1), origin=(0., 0.), capacity=200000):
    common = dict(owner_id=owner, signed_side=side, signature=signature, origin=origin,
                  tangent=(1., 0.), normal_cosine=COSINE)
    return (compact.create_curve_state(max_support_records=capacity, **common),
            original.create_curve_state(max_storage_units=capacity, **common))


def _adapter_fixture(stamp=10):
    points = np.array([[0., 0.], [1., 0.], [10., 0.], [11., 0.], [np.nan, np.nan]])
    normals = np.array([[0., 1.]] * 4 + [[np.nan, np.nan]])
    previous = np.array([-1, 0, -1, 2, -1], dtype=np.int64)
    following = np.array([1, -1, 3, -1, -1], dtype=np.int64)
    contours = np.array([0, 0, 1, 1, 2], dtype=np.int64)
    segments = np.array([[[i, -.125], [i, .125]] for i in range(5)], dtype=np.float64)
    quad = np.array([[-.25, -.25], [.25, -.25], [.25, .25], [-.25, .25]])
    quads = np.array([quad + (i, 0.) for i in range(5)], dtype=np.float64)
    valid = np.array([True, True, True, True, False])
    reliable = np.array([True, True, True, False, False])
    obs = SimpleNamespace(source_stamp_ns=stamp, points=points, segments=segments,
        previous=previous, next=following, contour_ids=contours,
        model_points=points, model_normals=normals, model_valid=valid,
        model_sensitivity=np.zeros(5), model_reliable=reliable,
        position_footprints=quads, position_segment_valid=np.ones(5, dtype=np.bool_),
        position_segment_sensitivity=np.zeros(5))
    # Actual source proof is represented by hand-written owner membership and
    # exact expected original face/neighbor bytes, never spatial matching.
    first = SimpleNamespace(record_id=7, last_stamp_ns=stamp, segments=segments[[0, 1, 2]].copy(),
                            neighbours=np.array([[-1, 7], [7, -1], [-1, 9]], dtype=np.int64))
    second = SimpleNamespace(record_id=9, last_stamp_ns=stamp, segments=segments[[3]].copy(),
                             neighbours=np.array([[7, -1]], dtype=np.int64))
    groups = (((0, 0), 0, [0, 1, 2]), ((1, 0), 0, [3]))
    prepared = SimpleNamespace(source_stamp_ns=stamp, updates=(first, second))
    config = dict(max_sensitivity=.5, reliable_sensitivity=.1, normal_max_angle_rad=math.pi / 6.)
    return obs, groups, prepared, config


class CompactCurveIndependentTests(unittest.TestCase):
    def assertEquivalent(self, actual, expected):
        for name in STATE_FIELDS:
            self.assertEqual(_bits(getattr(actual, name)), _bits(getattr(expected, name)), name)
        self.assertEqual(len(actual.frames), len(expected.frames))
        for current, old in zip(actual.frames, expected.frames):
            self.assertEqual(current.status, old.status)
            self.assertEqual(current.packet.stamp_ns, old.packet.stamp_ns)
            self.assertEqual(current.packet._has_reliable, old.packet._has_reliable)
            self.assertEqual(current.packet.source.source_kind, old.packet.source_kind)
            for name in RAW_FIELDS:
                value = current.packet.roles if name == 'roles' else current.packet.gather_original(name)
                self.assertEqual(_raw(value), _raw(getattr(old.packet, name)), name)
            self.assertEqual([tuple(_bits(getattr(value, name)) for name in AFFINE_FIELDS)
                              for value in current.pieces],
                             [tuple(_bits(getattr(value, name)) for name in AFFINE_FIELDS)
                              for value in old.pieces])
        self.assertEqual([tuple(_bits(getattr(value, name)) for name in PIECE_FIELDS)
                          for value in actual.pieces],
                         [tuple(_bits(getattr(value, name)) for name in PIECE_FIELDS)
                          for value in expected.pieces])

    def advanceBoth(self, states, packets, available=None):
        available = packets[0].stamp_ns if available is None else available
        current = compact.advance_curve(states[0], packets[0], available_before_ns=available)
        old = original.advance_curve(states[1], packets[1], available_before_ns=available)
        self.assertEquivalent(current, old)
        return current, old

    def test_01_empty_factory_metadata_matches_v2_not_capacity_dimension(self):
        states = _states()
        self.assertEquivalent(*states)
        self.assertEqual(states[0].max_support_records, 200000)
        self.assertEqual(compact.inventory((states[0],))['support_records'], 20)

    def test_02_ascending_five_frames_compare_all_geometry_bits_each_frame(self):
        states = _states()
        for stamp in range(1, 6):
            states = self.advanceBoth(states, _linear_packets(stamp, intercept=stamp * .0001))
            if stamp < 5:
                self.assertEqual(states[0].status, 'unconfirmed')
                self.assertEqual(states[0].pieces, ())
        self.assertEqual(states[0].status, 'confirmed')
        self.assertTrue(states[0].pieces)

    def test_03_decreasing_source_next_keeps_endpoint_index_and_chart(self):
        states = _states()
        for stamp in range(1, 6):
            states = self.advanceBoth(states, _linear_packets(stamp, reverse=True))
        self.assertEqual(states[0].origin, (0., 0.))
        self.assertEqual(states[0].tangent, (1., 0.))
        self.assertTrue(all(piece.lower_endpoint == 1 for frame in states[0].frames
                            for piece in frame.pieces))
        self.assertEqual(states[0].frames[0].packet.face_indices.tolist(), [[0, 1]])

    def test_04_disjoint_finite_pieces_gaps_are_not_filled(self):
        states = _states()
        for stamp in range(1, 6):
            states = self.advanceBoth(states, _linear_packets(stamp, intervals=((0., 1.), (2., 3.))))
        self.assertTrue(states[0].pieces)
        self.assertFalse(any(piece.lower < 1.5 < piece.upper for piece in states[0].pieces))

    def test_05_duplicate_occurrences_remain_original_unknown_overlap(self):
        states = self.advanceBoth(_states(), _linear_packets(1, duplicate=True))
        self.assertEqual(states[0].frames[0].packet.face_indices.tolist(), [[0, 1], [0, 1]])
        self.assertEqual(states[0].status, 'unknown')
        self.assertEqual(states[0].reason, 'overlap_or_nonunique_chart')
        self.assertEqual(compact.inventory((states[0],))['pair_occurrences'], 2)

    def test_06_pair_direction_unknown_is_not_filtered(self):
        states = self.advanceBoth(_states(), _linear_packets(1, slope=1., forced_normal=(0., 1.)))
        self.assertEqual(states[0].status, 'unknown')
        self.assertEqual(states[0].reason, 'pair_direction')
        self.assertEqual(len(states[0].frames), 1)
        self.assertEqual(states[0].pieces, ())

    def test_07_overlapping_distinct_faces_preserve_unknown(self):
        states = self.advanceBoth(_states(), _linear_packets(1, intervals=((0., 2.), (1., 3.))))
        self.assertEqual(states[0].status, 'unknown')
        self.assertEqual(states[0].reason, 'overlap_or_nonunique_chart')

    def test_08_zero_or_uncertain_chart_preserves_unknown(self):
        states = self.advanceBoth(_states(), _linear_packets(1, intervals=((0., 0.),)))
        self.assertEqual(states[0].reason, 'fold_or_zero_or_uncertain_chart')
        self.assertEqual(states[0].status, 'unknown')

    def test_09_old_unknown_stays_until_original_retention_drops_it(self):
        states = self.advanceBoth(_states(), _linear_packets(1, slope=1., forced_normal=(0., 1.)))
        for stamp in range(2, 7):
            states = self.advanceBoth(states, _linear_packets(stamp))
            if stamp <= 5:
                self.assertEqual(states[0].status, 'unknown')
        self.assertEqual([frame.packet.stamp_ns for frame in states[0].frames], [2, 3, 4, 5, 6])
        self.assertEqual(states[0].status, 'confirmed')

    def test_10_latest_five_plus_latest_reliable_keep_six_original_frames(self):
        states = _states()
        for stamp in range(1, 8):
            states = self.advanceBoth(states, _linear_packets(stamp, reliable=stamp == 1))
        self.assertEqual([frame.packet.stamp_ns for frame in states[0].frames], [1, 3, 4, 5, 6, 7])
        self.assertEqual(compact.inventory((states[0],))['source_tables'], 6)

    def test_11_new_reliable_witness_releases_original_oldest_reference(self):
        states = _states()
        for stamp in range(1, 9):
            states = self.advanceBoth(states, _linear_packets(stamp, reliable=stamp in (1, 8)))
        self.assertEqual([frame.packet.stamp_ns for frame in states[0].frames], [4, 5, 6, 7, 8])
        self.assertEqual(compact.inventory((states[0],))['source_tables'], 5)

    def test_12_large_origin_rounded_endpoints_equal_v2_and_inside_domain(self):
        origin = (1.e6, 1.e6)
        states = _states(origin=origin)
        for stamp in range(1, 6):
            states = self.advanceBoth(states, _linear_packets(stamp, world_origin=origin))
        self.assertTrue(states[0].pieces)
        for piece in states[0].pieces:
            for point in piece.segment:
                projection = Fraction(point[0]) - Fraction(origin[0])
                self.assertLessEqual(Fraction(piece.lower), projection)
                self.assertLessEqual(projection, Fraction(piece.upper))

    def test_13_duplicate_stale_and_future_updates_do_not_mutate_old_state(self):
        states = self.advanceBoth(_states(), _linear_packets(2))
        before = _bits(tuple(getattr(states[0], name) for name in STATE_FIELDS))
        for stamp, available in ((2, 2), (1, 2), (3, 2)):
            packets = _linear_packets(stamp)
            for implementation, state, packet in ((compact, states[0], packets[0]),
                                                   (original, states[1], packets[1])):
                with self.subTest(stamp=stamp, implementation=implementation.__name__), self.assertRaises(ValueError):
                    implementation.advance_curve(state, packet, available_before_ns=available)
        self.assertEqual(_bits(tuple(getattr(states[0], name) for name in STATE_FIELDS)), before)
        self.assertEquivalent(*states)

    def test_14_owner_side_or_signature_is_not_merged(self):
        for options in ({'owner': 9}, {'side': -1}, {'signature': (3, 9)}):
            states = _states()
            packets = _linear_packets(1, **options)
            for implementation, state, packet in ((compact, states[0], packets[0]),
                                                   (original, states[1], packets[1])):
                with self.subTest(options=options, implementation=implementation.__name__), self.assertRaises(ValueError):
                    implementation.advance_curve(state, packet, available_before_ns=1)
            self.assertEquivalent(*states)

    def test_15_capacity_rejection_is_atomic_in_each_declared_dimension(self):
        states = _states(capacity=20)
        packets = _linear_packets(1)
        before = _raw(packets[0].source.model_roots)
        for implementation, state, packet in ((compact, states[0], packets[0]),
                                               (original, states[1], packets[1])):
            with self.subTest(implementation=implementation.__name__), self.assertRaises(ValueError):
                implementation.advance_curve(state, packet, available_before_ns=1)
            self.assertEqual(state.frames, ())
            self.assertEqual(state.source_frontier_ns, 0)
        self.assertEqual(_raw(packets[0].source.model_roots), before)
        self.assertEquivalent(*states)

    def test_16_single_frame_inventory_metrics_are_independently_accounted(self):
        state, _ = self.advanceBoth(_states(), _linear_packets(1))
        value = compact.inventory((state,))
        expected = dict(source_tables=1, packets=1, source_faces=2, pair_occurrences=1,
                        support_records=41, scalar_entries=119, payload_bytes=295,
                        states=1, retained_frames=1, affine_records=1, exported_pieces=0,
                        contributor_records=0, raw_payload_bytes=295,
                        derived_canonical_payload_bytes=57, raw_plus_canonical_payload_bytes=352)
        self.assertEqual(value, expected)

    def test_17_dual_signatures_count_every_state_and_occurrence(self):
        packet, _ = _linear_packets(1, signature=(-1, 9))
        other = evidence.PairPacket(packet.source, 7, 1, (3, 9), packet.roles, packet.face_indices)
        first = compact.advance_curve(_states(signature=(-1, 9))[0], packet, available_before_ns=1)
        second = compact.advance_curve(_states(signature=(3, 9))[0], other, available_before_ns=1)
        value = compact.inventory((first, second))
        self.assertEqual(value['source_tables'], 1)
        self.assertEqual(value['packets'], 2)
        self.assertEqual(value['support_records'], 79)
        self.assertEqual(value['scalar_entries'], 170)
        self.assertEqual(value['raw_payload_bytes'], 312)
        self.assertEqual(value['raw_plus_canonical_payload_bytes'], 426)
        with self.assertRaises(ValueError):
            compact.inventory((first, first))

    def test_18_unknown_empty_and_stale_owner_inventory_has_no_status_filter(self):
        bad, _ = _linear_packets(1, slope=1., forced_normal=(0., 1.))
        unknown = compact.advance_curve(_states()[0], bad, available_before_ns=1)
        empty_packet = evidence.PairPacket(bad.source, 55, 1, (-1, -1),
            np.empty(0, dtype=np.uint8), np.empty((0, 2), dtype=np.int64))
        empty = compact.advance_curve(_states(owner=55)[0], empty_packet, available_before_ns=1)
        new, _ = _linear_packets(100, owner=9)
        current = compact.advance_curve(_states(owner=9)[0], new, available_before_ns=100)
        value = compact.inventory((unknown, empty, current))
        self.assertEqual(value['states'], 3)
        self.assertEqual(value['source_tables'], 2)
        self.assertEqual(value['source_faces'], 4)
        self.assertEqual(value['packets'], 3)
        self.assertEqual(value['pair_occurrences'], 2)
        self.assertEqual(value['affine_records'], 1)
        self.assertEqual(unknown.status, 'unknown')
        self.assertEqual(empty.status, 'unconfirmed')

    def test_19_raw_quads_and_chart_temporary_gathers_are_not_lost_or_retained(self):
        packets = _linear_packets(1, invalid_quads=True)
        states = self.advanceBoth(_states(), packets)
        self.assertIs(states[0].frames[0].packet, packets[0])
        self.assertFalse(hasattr(states[0].frames[0].packet, 'model_roots'))
        self.assertEqual(states[0].frames[0].packet.source.quads.tobytes(),
                         packets[0].source.quads.tobytes())
        self.assertFalse(packets[0].source.quad_valid.any())

    def test_20_actual_adapter_fields_match_independent_original_pair_schema(self):
        obs, groups, prepared, config = _adapter_fixture()
        packets, owners = adapter.prepare_packets(obs, groups, prepared, config)
        self.assertEqual(owners.tolist(), [7, 7, 7, 9, -1])
        self.assertEqual([packet.owner_id for packet in packets], [7, 9])
        self.assertIs(packets[0].source, packets[1].source)
        self.assertEqual(packets[0].signature, (-1, 9))
        self.assertEqual(packets[1].signature, (7, -1))
        source_arguments = dict(stamp_ns=10, face_ids=np.arange(5, dtype=np.int64),
            previous_next=np.column_stack((obs.previous, obs.next)), contour_ids=obs.contour_ids,
            endpoint_owners=np.array([7, 7, 7, 9, -1], dtype=np.int64),
            model_roots=obs.model_points, normals=obs.model_normals, quads=obs.position_footprints,
            quad_valid=np.ones(5, dtype=np.bool_), reliable=obs.model_reliable, valid=obs.model_valid)
        for packet, indices, roles in ((packets[0], np.array([[0, 1], [2, 3]], dtype=np.int64),
                                        np.array([1, 2], dtype=np.uint8)),
                                       (packets[1], np.array([[2, 3]], dtype=np.int64),
                                        np.array([2], dtype=np.uint8))):
            old = _legacy(source_arguments, indices, roles, packet.owner_id, 1, packet.signature)
            for name in RAW_FIELDS:
                actual = packet.roles if name == 'roles' else packet.gather_original(name)
                self.assertEqual(_raw(actual), _raw(getattr(old, name)), name)
        self.assertFalse(packets[0].source.valid[4])
        self.assertTrue(packets[0].source.quad_valid[4])
        self.assertEqual(evidence.reachable_inventory(packets)['source_faces'], 5)

    def test_21_adapter_rejects_changed_source_members_and_neighbor_bytes(self):
        obs, groups, prepared, config = _adapter_fixture()
        changed = ((groups[0][0], groups[0][1], [1, 0, 2]), groups[1])
        with self.assertRaises(ValueError):
            adapter.prepare_packets(obs, changed, prepared, config)
        prepared.updates[0].neighbours[0, 1] = 9
        with self.assertRaises(ValueError):
            adapter.prepare_packets(obs, groups, prepared, config)

    def test_22_factory_validation_and_global_inventory_are_not_auto_publication(self):
        for implementation, capacity_name in ((compact, 'max_support_records'),
                                               (original, 'max_storage_units')):
            arguments = dict(owner_id=7, signed_side=1, signature=(-1, -1),
                origin=(0., 0.), tangent=(1.01, 0.), normal_cosine=COSINE)
            arguments[capacity_name] = 200000
            with self.assertRaises(ValueError):
                implementation.create_curve_state(**arguments)
        self.assertEqual(compact.inventory(())['support_records'], 0)
        with self.assertRaises(ValueError):
            compact.inventory((object(),))

    def test_23_extreme_finite_span_matches_original_numerical_unknown(self):
        arguments = dict(stamp_ns=1, face_ids=np.array([0, 1], dtype=np.int64),
            previous_next=np.array([[-1, 1], [0, -1]], dtype=np.int64),
            contour_ids=np.array([0, 0], dtype=np.int64),
            endpoint_owners=np.array([7, 7], dtype=np.int64),
            model_roots=np.array([[0., 0.], [1.e-320, 1.]], dtype=np.float64),
            normals=np.array([[0., 1.], [0., 1.]], dtype=np.float64),
            quads=np.zeros((2, 4, 2), dtype=np.float64),
            quad_valid=np.zeros(2, dtype=np.bool_),
            reliable=np.ones(2, dtype=np.bool_), valid=np.ones(2, dtype=np.bool_))
        indices = np.array([[0, 1]], dtype=np.int64)
        roles = np.array([1], dtype=np.uint8)
        packet = evidence.PairPacket(evidence.SourceFrame(**arguments), 7, 1, (-1, -1),
                                     roles, indices)
        old = _legacy(arguments, indices, roles, 7, 1, (-1, -1))
        states = self.advanceBoth(_states(), (packet, old))
        # The complete old algorithm determines the FP-policy-specific reason;
        # neither representation invents a finite usable model from this span.
        self.assertEqual(states[0].status, 'unknown')
        self.assertEqual(states[0].pieces, ())


class OverlayReaderTests(unittest.TestCase):
    def test_changed_normal_property_keeps_original_second_read_failure(self):
        state, _ = _states()
        for stamp in range(1, 6):
            packet, _ = _linear_packets(stamp, intervals=((0., 1.), (1., 2.)))
            state = compact.advance_curve(state, packet, available_before_ns=stamp)
        self.assertEqual(state.status, 'confirmed')
        descriptor = evidence.PairPacket.normals
        reads = {}
        def changed(packet):
            reads[id(packet)] = reads.get(id(packet), 0) + 1
            if reads[id(packet)] == 2:
                raise ArithmeticError('second-read diagnostic sentinel')
            return descriptor.fget(packet)
        with mock.patch.object(evidence.PairPacket, 'normals', property(changed)):
            self.assertFalse(original._normal_readers_unchanged())
            self.assertEqual(original._overlay(state, state.frames),
                             ((), 'unknown', 'numerical_export'))
        self.assertEqual(max(reads.values()), 2)
        self.assertTrue(original._normal_readers_unchanged())

    def test_changed_reader_code_without_rebinding_disables_cache(self):
        function = evidence.PairPacket.gather_original
        old_code = function.__code__
        def changed(self, name):
            raise ArithmeticError('changed normal reader')
        try:
            function.__code__ = changed.__code__
            self.assertFalse(original._normal_readers_unchanged())
        finally:
            function.__code__ = old_code
        self.assertTrue(original._normal_readers_unchanged())


if __name__ == '__main__':
    unittest.main()
