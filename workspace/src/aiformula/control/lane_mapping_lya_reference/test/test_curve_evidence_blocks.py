"""Independent dense-source byte/lifetime and unchanged-arithmetic oracles."""
import gc
from dataclasses import fields, FrozenInstanceError
import math
import struct
import unittest
import weakref
import numpy as np
from lane_mapping_lya_reference import curve_evidence as original
from lane_mapping_lya_reference import curve_evidence_blocks as candidate
from lane_mapping_lya_reference import curve_consensus_compact as compact
from lane_mapping_lya_reference import curve_consensus as arithmetic


def fixture(stamp=100, *, noncanonical=False):
    count = 8
    links = np.full((count, 2), -1, np.int64)
    for first in range(6):
        links[first, 1] = first + 1
        links[first + 1, 0] = first
    owners = np.array([10, 10, 20, 20, -1, 30, 30, 10], np.int64)
    valid = owners >= 0
    roots = np.column_stack((np.arange(count, dtype=np.float64), np.zeros(count)))
    roots[1, 1] = -0.0
    roots.view(np.uint64)[4, 0] = np.uint64(0x7ff8000000000042)
    normals = np.tile([0., 1.], (count, 1))
    normals[4] = np.nan
    quads = np.stack([np.array([[i, 0.], [i+.1, 0.], [i+.1, .1], [i, .1]])
                      for i in range(count)])
    quad_valid, reliable = np.ones(count, np.bool_), valid.copy()
    if noncanonical:
        valid.view(np.uint8)[0] = 255
        reliable.view(np.uint8)[0] = 2
        quad_valid.view(np.uint8)[0] = 128
        quad_valid.view(np.uint8)[4] = 128
        valid.view(np.uint8)[2] = 128
        reliable.view(np.uint8)[2] = 255
        quad_valid.view(np.uint8)[2] = 2
    source = original.SourceFrame(stamp, np.arange(count, dtype=np.int64), links,
        np.array([0, 0, 0, 0, 0, 0, 0, 1], np.int64), owners, roots, normals,
        quads, quad_valid, reliable, valid)
    rows = (
        (10, (-1, 20), [1, 2, 2], [[0, 1], [1, 2], [1, 2]]),
        (20, (10, -1), [1, 2], [[2, 3], [1, 2]]),
        (30, (-1, -1), [1], [[5, 6]]),
    )
    packets = tuple(original.PairPacket(source, owner, 1, signature,
        np.array(roles, np.uint8), np.array(indices, np.int64).reshape(-1, 2))
        for owner, signature, roles, indices in rows)
    return source, packets


class RetainedBlocksTests(unittest.TestCase):
    def test_multiple_nonadjacent_unowned_faces_keep_every_complete_row(self):
        source, old = fixture(noncanonical=True)
        args = {name: getattr(source, name).copy() for name, _, _ in original._FIELDS}
        args['endpoint_owners'][7] = -1
        args['valid'][7] = False
        args['reliable'][7] = False
        args['quad_valid'].view(np.uint8)[7] = 255
        sparse = original.SourceFrame(stamp_ns=source.stamp_ns, **args)
        packet = original.PairPacket(sparse, old[0].owner_id, 1, old[0].signature,
                                     old[0].roles, old[0].face_indices)
        expected = {face: {name: getattr(sparse, name)[face:face+1].tobytes()
                           for name, _, _ in original._FIELDS} for face in (4, 7)}
        (retained,), capsule = candidate.project_frame(sparse, (packet,))
        del source, old, sparse, packet, args, capsule
        gc.collect()
        self.assertEqual([b.face_id for b in retained.lease.unowned.blocks], [4, 7])
        for block in retained.lease.unowned.blocks:
            for name, gold in expected[block.face_id].items():
                self.assertEqual(block.get(name).tobytes(), gold)

    def test_isolated_valid_root_with_invalid_quad_keeps_complete_placeholder(self):
        source, old = fixture()
        args = {name: getattr(source, name).copy() for name, _, _ in original._FIELDS}
        args['quad_valid'][7] = False
        args['quads'].view(np.uint64)[7, 0, 0] = np.uint64(0x7ff8000000000043)
        sparse = original.SourceFrame(stamp_ns=source.stamp_ns, **args)
        packet = original.PairPacket(sparse, old[0].owner_id, 1, old[0].signature,
                                     old[0].roles, old[0].face_indices)
        (retained,), _ = candidate.project_frame(sparse, (packet,))
        block = next(b for b in retained.lease.own_blocks if b.face_id == 7)
        for name, _, _ in original._FIELDS:
            self.assertEqual(block.get(name).tobytes(), getattr(sparse, name)[7:8].tobytes())
        self.assertNotIn(7, retained.face_indices)

    def test_noncanonical_opposite_incident_endpoint_keeps_raw_flags(self):
        source, old = fixture(noncanonical=True)
        new, _ = candidate.project_frame(source, old)
        opposite = next(b for b in new[0].lease.required_blocks if b.face_id == 2)
        self.assertNotIn(2, [b.face_id for b in new[0].lease.own_blocks])
        for name in ('valid', 'reliable', 'quad_valid'):
            with self.subTest(field=name):
                self.assertEqual(opposite.get(name).tobytes(),
                                 getattr(source, name)[2:3].tobytes())
                self.assertEqual(new[0].gather_original(name).tobytes(),
                                 getattr(source, name)[old[0].face_indices].tobytes())

    def test_noncanonical_bool_owned_row_and_pair_preserve_raw_bytes(self):
        source, old = fixture(noncanonical=True)
        new, _ = candidate.project_frame(source, old)
        block = next(b for b in new[0].lease.own_blocks if b.face_id == 0)
        for name in ('valid', 'reliable', 'quad_valid'):
            with self.subTest(field=name):
                # A scalar bool getitem would normalize the gold too. Use a
                # complete one-row ndarray slice, independently of candidate.
                self.assertEqual(block.get(name).tobytes(),
                                 getattr(source, name)[0:1].tobytes())
                self.assertEqual(new[0].gather_original(name).tobytes(),
                                 getattr(source, name)[old[0].face_indices].tobytes())

    def test_noncanonical_unowned_quad_flag_survives_state_only_reference(self):
        source, old = fixture(noncanonical=True)
        new, capsule = candidate.project_frame(source, old)
        expected = source.quad_valid[4:5].tobytes()
        retained = new[0]
        del source, old, new, capsule
        gc.collect()
        self.assertEqual(retained.lease.unowned.blocks[0].get('quad_valid').tobytes(), expected)

    def test_noncanonical_boolean_payload_decodes_after_dense_source_release(self):
        source, old = fixture(noncanonical=True)
        expected = {name: getattr(source, name)[old[0].face_indices].tobytes()
                    for name in ('valid', 'reliable', 'quad_valid')}
        refs = [weakref.ref(source)] + [weakref.ref(getattr(source, name))
                                        for name, _, _ in original._FIELDS]
        new, capsule = candidate.project_frame(source, old)
        retained = new[0]
        del source, old, new, capsule
        gc.collect()
        self.assertTrue(all(ref() is None for ref in refs))
        for name, gold in expected.items():
            with self.subTest(field=name):
                self.assertEqual(retained.gather_original(name).tobytes(), gold)

    def test_all_ten_original_pair_fields_exact_dtype_shape_bytes(self):
        source, old = fixture()
        new, capsule = candidate.project_frame(source, old)
        for gold, projected in zip(old, new):
            self.assertEqual((gold.owner_id, gold.signed_side, gold.signature,
                              gold.stamp_ns, gold._has_reliable),
                (projected.owner_id, projected.signed_side, projected.signature,
                 projected.stamp_ns, projected._has_reliable))
            for name, _, _ in original._FIELDS:
                expected = getattr(source, name)[gold.face_indices]
                actual = projected.gather_original(name)
                self.assertEqual(actual.dtype, expected.dtype)
                self.assertEqual(actual.shape, expected.shape)
                self.assertEqual(actual.tobytes(), expected.tobytes())
                self.assertFalse(actual.flags.writeable)
            self.assertEqual(projected.roles.tobytes(), gold.roles.tobytes())
            self.assertEqual(projected.face_indices.tobytes(), gold.face_indices.tobytes())
        self.assertEqual(len(capsule.blocks), 1)

    def test_isolated_own_face_not_in_any_pair_is_retained(self):
        source, old = fixture()
        new, _ = candidate.project_frame(source, old)
        self.assertEqual([block.face_id for block in new[0].lease.own_blocks], [0, 1, 7])
        for block in new[0].lease.own_blocks:
            for name, _, _ in original._FIELDS:
                self.assertEqual(block.get(name).tobytes(),
                                 getattr(source, name)[block.face_id:block.face_id+1].tobytes())

    def test_order_multiplicity_and_cross_owner_face_shared_not_deduplicated(self):
        source, old = fixture()
        new, _ = candidate.project_frame(source, old)
        self.assertEqual(new[0].face_indices.tolist(), [[0, 1], [1, 2], [1, 2]])
        self.assertEqual(new[0].roles.tolist(), [1, 2, 2])
        first = {b.face_id: b for b in new[0].lease.required_blocks}
        second = {b.face_id: b for b in new[1].lease.required_blocks}
        self.assertIs(first[1], second[1])
        self.assertIs(first[2], second[2])
        self.assertEqual([b.face_id for b in new[0].lease.required_blocks], [0, 1, 2, 7])

    def test_invalid_root_valid_unowned_quad_and_nan_payload_preserved(self):
        source, old = fixture()
        new, capsule = candidate.project_frame(source, old)
        self.assertEqual([b.face_id for b in capsule.blocks], [4])
        for packet in new:
            self.assertIs(packet.lease.unowned, capsule)
        block = capsule.blocks[0]
        for name, _, _ in original._FIELDS:
            self.assertEqual(block.get(name).tobytes(), getattr(source, name)[4:5].tobytes())
        self.assertEqual(block.get('model_roots').view(np.uint64)[0],
                         np.uint64(0x7ff8000000000042))

    def test_actual_source_arrays_and_object_release_while_leases_decode(self):
        source, old = fixture()
        expected = {name: getattr(source, name)[old[0].face_indices].tobytes()
                    for name, _, _ in original._FIELDS}
        refs = [weakref.ref(source)] + [weakref.ref(getattr(source, name))
                                        for name, _, _ in original._FIELDS]
        new, capsule = candidate.project_frame(source, old)
        retained = new[0]
        del source, old, new, capsule
        gc.collect()
        self.assertTrue(all(ref() is None for ref in refs))
        for name, _, _ in original._FIELDS:
            self.assertEqual(retained.gather_original(name).tobytes(), expected[name])

    def test_unused_other_owner_blocks_are_released_not_hidden_in_header(self):
        source, old = fixture()
        new, capsule = candidate.project_frame(source, old)
        other = next(b for b in new[2].lease.required_blocks if b.face_id == 5)
        ref = weakref.ref(other)
        retained = new[0]
        del source, old, new, capsule, other
        gc.collect()
        self.assertIsNone(ref())
        inv = candidate.inventory((retained,))
        self.assertEqual(inv['retained_complete_faces'], 5)  # own0/1/7 + incident2 + unowned4
        self.assertEqual(inv['face_payload_bytes'], 5 * 139)
        self.assertEqual(inv['legacy_source_packet_weighted_charge'], 9 + 10 + 2*3)
        self.assertFalse(inv['heap_or_runtime_acceptance'])

    def test_unowned_capsule_survives_without_external_orphan_history(self):
        source, old = fixture()
        new, capsule = candidate.project_frame(source, old)
        ref = weakref.ref(capsule)
        retained = new[0]
        del source, old, new, capsule
        gc.collect()
        self.assertIs(ref(), retained.lease.unowned)
        self.assertEqual(candidate.inventory((retained,))['retained_complete_faces'], 5)

    def test_empty_packet_still_keeps_own_and_unowned_rows(self):
        source, _ = fixture()
        old = original.PairPacket(source, 10, 1, (-1, -1),
            np.empty(0, np.uint8), np.empty((0, 2), np.int64))
        (new,), _ = candidate.project_frame(source, (old,))
        self.assertEqual([b.face_id for b in new.lease.own_blocks], [0, 1, 7])
        for name, _, tail in original._FIELDS:
            self.assertEqual(new.gather_original(name).shape, (0, 2) + tail)
        self.assertEqual(candidate.inventory((new,))['retained_complete_faces'], 4)

    def test_distinct_same_stamp_source_events_never_merge(self):
        first_source, first_old = fixture()
        second_source, second_old = fixture()
        first, _ = candidate.project_frame(first_source, first_old)
        second, _ = candidate.project_frame(second_source, second_old)
        self.assertIsNot(first[0].lease.frame, second[0].lease.frame)
        self.assertEqual(candidate.inventory((first[0], second[0]))['source_events'], 2)

    def test_other_source_packet_and_duplicate_key_rejected(self):
        first_source, first_old = fixture()
        _, second_old = fixture()
        with self.assertRaisesRegex(ValueError, 'original source event'):
            candidate.project_frame(first_source, second_old)
        with self.assertRaisesRegex(ValueError, 'duplicate key'):
            candidate.project_frame(first_source, (first_old[0], first_old[0]))

    def test_all_face_field_views_and_original_sources_remain_immutable(self):
        source, old = fixture()
        before = tuple(getattr(source, name).tobytes() for name, _, _ in original._FIELDS)
        new, _ = candidate.project_frame(source, old)
        for block in new[0].lease.required_blocks:
            self.assertEqual(len(block.payload), 139)
            for name, _, _ in original._FIELDS:
                view = block.get(name)
                self.assertFalse(view.flags.writeable)
                with self.assertRaises(ValueError):
                    view.setflags(write=True)
        self.assertEqual(before, tuple(getattr(source, name).tobytes()
                                      for name, _, _ in original._FIELDS))

    def test_zero_packets_retains_only_complete_unowned_capsule(self):
        source, _ = fixture()
        new, capsule = candidate.project_frame(source, ())
        self.assertEqual(new, ())
        inv = candidate.inventory((), (capsule,))
        self.assertEqual(inv['retained_complete_faces'], 1)
        self.assertEqual(inv['legacy_source_packet_weighted_charge'], 9)



def _line_packet(stamp, reliable):
    roots = np.array([[1., .25], [2., .25]], np.float64)
    corners = np.array([[-.0625, -.0625], [.0625, -.0625],
                        [.0625, .0625], [-.0625, .0625]], np.float64)
    source = original.SourceFrame(
        stamp, np.arange(2, dtype=np.int64), np.array([[-1, 1], [0, -1]], np.int64),
        np.zeros(2, np.int64), np.full(2, 7, np.int64), roots,
        np.array([[0., 1.], [0., 1.]], np.float64), roots[:, None, :] + corners,
        np.ones(2, np.bool_), np.full(2, reliable, np.bool_), np.ones(2, np.bool_))
    return original.PairPacket(source, 7, 1, (-1, -1),
        np.ones(1, np.uint8), np.array([[0, 1]], np.int64))


def _bits(value):
    if isinstance(value, (float, np.floating)):
        return ("binary64", struct.pack(">d", float(value)))
    if isinstance(value, tuple):
        return tuple(_bits(item) for item in value)
    return value


def _record_bits(value):
    return tuple((item.name, _bits(getattr(value, item.name))) for item in fields(value))


class RetainedBlockReaderTests(unittest.TestCase):
    def test_own_and_capsule_readers_preserve_dense_source_order_and_bytes(self):
        source, old = fixture(noncanonical=True)
        packets, capsule = candidate.project_frame(source, old)
        for packet in packets:
            own = source.endpoint_owners == packet.owner_id
            for name, _, tail in original._FIELDS:
                for kwargs, mask in (({}, own & (source.valid | source.quad_valid)),
                        ({"valid_only": True}, own & source.valid),
                        ({"quad_valid_only": True}, own & source.quad_valid)):
                    with self.subTest(owner=packet.owner_id, field=name, flags=kwargs):
                        actual = packet.own_original(name, **kwargs)
                        gold = getattr(source, name)[mask]
                        self.assertEqual((actual.dtype, actual.shape, actual.tobytes()),
                                         (gold.dtype, gold.shape, gold.tobytes()))
                        self.assertFalse(actual.flags.writeable)
                gold_unowned = getattr(source, name)[
                    source.quad_valid & (source.endpoint_owners < 0)]
                actual_unowned = capsule.gather_original(name)
                self.assertEqual((actual_unowned.dtype, actual_unowned.shape,
                                  actual_unowned.tobytes()),
                    (gold_unowned.dtype, gold_unowned.shape, gold_unowned.tobytes()))

    def test_independent_quad_filter_keeps_root_but_drops_invalid_quad_placeholder(self):
        source, old = fixture()
        args = {name: getattr(source, name).copy() for name, _, _ in original._FIELDS}
        args["quad_valid"][7] = False
        args["quads"].view(np.uint64)[7, 0, 0] = np.uint64(0x7ff8000000000043)
        changed = original.SourceFrame(stamp_ns=100, **args)
        dense = original.PairPacket(changed, 10, 1, old[0].signature,
                                    old[0].roles, old[0].face_indices)
        (packet,), _ = candidate.project_frame(changed, (dense,))
        self.assertEqual(packet.own_original("face_ids", valid_only=True).tolist(), [0, 1, 7])
        self.assertEqual(packet.own_original("face_ids", quad_valid_only=True).tolist(), [0, 1])
        self.assertEqual(packet.own_original("quads")[-1:].tobytes(),
                         changed.quads[7:8].tobytes())

    def test_occurrence_readers_and_normals_keep_both_incident_endpoints_and_repeats(self):
        source, old = fixture(noncanonical=True)
        packets, _ = candidate.project_frame(source, old)
        for gold, packet in zip(old, packets):
            self.assertEqual(packet.normals.tobytes(), source.normals[gold.face_indices].tobytes())
            self.assertFalse(packet.normals.flags.writeable)
            for ordinal, indices in enumerate(gold.face_indices):
                for name, _, _ in original._FIELDS:
                    actual = packet.occurrence_original(name, ordinal)
                    expected = getattr(source, name)[indices]
                    self.assertEqual((actual.dtype, actual.shape, actual.tobytes()),
                                     (expected.dtype, expected.shape, expected.tobytes()))
                self.assertEqual(packet.occurrence_normals(ordinal).tobytes(),
                                 source.normals[indices].tobytes())

    def test_reader_invalid_field_flags_and_occurrence_reject(self):
        source, old = fixture()
        (packet, *unused), capsule = candidate.project_frame(source, old)
        for reader in (packet.gather_original, packet.own_original, capsule.gather_original,
                       packet.lease.own_blocks[0].get):
            with self.assertRaisesRegex(ValueError, "source field"):
                reader("not_a_field")
        for ordinal in (-1, len(packet.roles), True, np.bool_(False), 0., "0"):
            with self.subTest(ordinal=ordinal):
                with self.assertRaisesRegex(ValueError, "ordinal"):
                    packet.occurrence_normals(ordinal)
        for kwargs in ({"valid_only": 1}, {"quad_valid_only": np.bool_(True)},
                       {"valid_only": True, "quad_valid_only": True}):
            with self.assertRaisesRegex(ValueError, "validity rule"):
                packet.own_original("model_roots", **kwargs)

    def test_legacy_source_packet_charge_and_payload_are_separate_exact_ledgers(self):
        source, old = fixture()
        packets, capsule = candidate.project_frame(source, old)
        dense = original.reachable_inventory(old)
        retained = candidate.inventory(packets + (packets[0],), (capsule, capsule))
        self.assertEqual(retained["legacy_source_packet_weighted_charge"], dense["support_records"])
        self.assertEqual(retained["legacy_source_packet_scalar_entries"], dense["scalar_entries"])
        self.assertEqual(retained["legacy_source_packet_payload_bytes"], dense["payload_bytes"])
        self.assertEqual(retained["source_events"], dense["source_tables"])
        self.assertEqual(retained["legacy_source_faces"], dense["source_faces"])
        self.assertEqual(retained["packets"], dense["packets"])
        self.assertEqual(retained["pair_occurrences"], dense["pair_occurrences"])
        small = candidate.inventory((packets[0],))
        gold_small = original.reachable_inventory((old[0],))
        self.assertEqual(small["legacy_source_packet_weighted_charge"], gold_small["support_records"])
        self.assertEqual(small["legacy_source_packet_scalar_entries"], gold_small["scalar_entries"])
        self.assertEqual(small["legacy_source_packet_payload_bytes"], gold_small["payload_bytes"])
        self.assertLess(small["retained_array_payload_bytes"], gold_small["payload_bytes"])
        self.assertEqual(small["retained_array_payload_bytes"],
                         small["face_payload_bytes"] + small["packet_payload_bytes"])

    def test_no_unowned_rows_still_produces_empty_typed_capsule(self):
        dense = _line_packet(1, True)
        packets, capsule = candidate.project_frame(dense.source, (dense,))
        self.assertEqual(capsule.blocks, ())
        self.assertIs(packets[0].lease.unowned, capsule)
        for name, dtype, tail in original._FIELDS:
            decoded = capsule.gather_original(name)
            self.assertEqual(decoded.shape, (0,) + tail)
            self.assertEqual(decoded.dtype, np.dtype(dtype))
            self.assertFalse(decoded.flags.writeable)

    def test_repeated_projection_is_explicitly_distinct_not_a_global_registry(self):
        source, old = fixture()
        first, _ = candidate.project_frame(source, old)
        second, _ = candidate.project_frame(source, old)
        self.assertIsNot(first[0].source_identity, second[0].source_identity)
        both = candidate.inventory((first[0], second[0]))
        one = candidate.inventory((first[0],))
        self.assertEqual(both["source_events"], 2)
        self.assertEqual(both["legacy_source_packet_weighted_charge"],
                         2 * one["legacy_source_packet_weighted_charge"])

    def test_factory_seals_frozen_records_and_explicit_representation(self):
        source, old = fixture()
        packets, capsule = candidate.project_frame(source, old)
        self.assertEqual(packets[0].source_identity.representation, candidate.REPRESENTATION)
        self.assertTrue(candidate.is_block_packet(packets[0]))
        self.assertTrue(candidate.is_unowned_capsule(capsule))
        self.assertFalse(candidate.is_block_packet(old[0]))
        self.assertFalse(candidate.is_unowned_capsule(source))
        for obj, name in ((packets[0], "lease"), (packets[0].lease, "own_blocks"),
                          (capsule, "blocks"), (capsule.frame, "stamp_ns"),
                          (capsule.blocks[0], "payload")):
            with self.assertRaises(FrozenInstanceError):
                setattr(obj, name, None)
        constructors = (
            lambda: candidate.FrameIdentity(1, 0, original.SOURCE_KIND),
            lambda: candidate.FaceBlock(None, 0, b""),
            lambda: candidate.UnownedCapsule(None, ()),
            lambda: candidate.KeyFrameLease(None, 7, 1, (-1, -1), (), (), None),
            lambda: candidate.BlockPacket(None, np.empty(0, np.uint8),
                                          np.empty((0, 2), np.int64), False))
        for constructor in constructors:
            with self.assertRaisesRegex(ValueError, "factories"):
                constructor()

    def test_invalid_unused_complete_source_cannot_be_accepted_by_projection(self):
        source, _ = fixture()
        args = {name: getattr(source, name).copy() for name, _, _ in original._FIELDS}
        # Face5 is not needed by owner10. Full validation must still reject it.
        args["normals"][5] = np.nan
        with self.assertRaisesRegex(ValueError, "valid source geometry"):
            original.SourceFrame(stamp_ns=100, **args)
        with self.assertRaisesRegex(ValueError, "validated source"):
            candidate.project_frame(object(), ())
        with self.assertRaisesRegex(ValueError, "block packet"):
            candidate.inventory((object(),))
        with self.assertRaisesRegex(ValueError, "unowned capsule"):
            candidate.inventory((), (object(),))

    def test_five_frame_confirmed_overlay_and_older_reliable_sixth_frame_match_dense(self):
        # Compare public block advancement with dense advancement, and retain
        # the separate chart/overlay oracle to exercise the normals reader.
        chart = compact.create_curve_state(7, 1, (-1, -1), (0., 0.), (1., 0.),
            normal_cosine=math.cos(math.pi / 6.), max_support_records=200000)
        dense_state, block_state, block_frames = chart, chart, {}
        for stamp in range(1, 8):
            dense = _line_packet(stamp, stamp == 1)
            (packet,), _ = candidate.project_frame(dense.source, (dense,))
            block_frames[stamp] = compact._chart_frame(chart, packet)
            dense_state = compact.advance_curve(dense_state, dense, available_before_ns=stamp)
            block_state = compact.advance_curve(block_state, packet, available_before_ns=stamp)
            for name in ("owner_id", "signed_side", "signature", "origin", "tangent",
                         "normal_cosine", "max_support_records", "source_frontier_ns",
                         "status", "reason"):
                self.assertEqual(_bits(getattr(block_state, name)),
                                 _bits(getattr(dense_state, name)), name)
            self.assertEqual(tuple(map(_record_bits, block_state.pieces)),
                             tuple(map(_record_bits, dense_state.pieces)))
            self.assertEqual(len(block_state.frames), len(dense_state.frames))
            for actual, expected in zip(block_state.frames, dense_state.frames):
                self.assertTrue(candidate.is_block_packet(actual.packet))
                self.assertEqual(actual.status, expected.status)
                self.assertEqual(actual.packet.stamp_ns, expected.packet.stamp_ns)
                self.assertEqual(actual.packet._has_reliable, expected.packet._has_reliable)
                self.assertEqual(actual.packet.roles.tobytes(), expected.packet.roles.tobytes())
                self.assertEqual(actual.packet.face_indices.tobytes(),
                                 expected.packet.face_indices.tobytes())
                self.assertEqual(tuple(map(_record_bits, actual.pieces)),
                                 tuple(map(_record_bits, expected.pieces)))
                for name, _, _ in original._FIELDS:
                    decoded = actual.packet.gather_original(name)
                    gold = getattr(expected.packet.source, name)[expected.packet.face_indices]
                    self.assertEqual((decoded.dtype, decoded.shape, decoded.tobytes()),
                                     (gold.dtype, gold.shape, gold.tobytes()), name)
            block_inventory = compact.inventory((block_state,))
            dense_inventory = compact.inventory((dense_state,))
            for name in ("support_records", "scalar_entries", "source_tables", "source_faces",
                         "packets", "pair_occurrences", "states", "retained_frames",
                         "affine_records", "exported_pieces", "contributor_records"):
                self.assertEqual(block_inventory[name], dense_inventory[name], name)
            self.assertEqual(block_inventory["legacy_source_packet_payload_bytes"],
                             dense_inventory["raw_payload_bytes"])
            selected = tuple(block_frames[frame.packet.stamp_ns] for frame in dense_state.frames)
            pieces, status, reason = arithmetic._overlay(chart, selected)
            self.assertEqual((status, reason), (dense_state.status, dense_state.reason))
            self.assertEqual(tuple(map(_record_bits, pieces)),
                             tuple(map(_record_bits, dense_state.pieces)))
            for actual, expected in zip(selected, dense_state.frames):
                self.assertEqual(actual.status, expected.status)
                self.assertEqual(tuple(map(_record_bits, actual.pieces)),
                                 tuple(map(_record_bits, expected.pieces)))
            if stamp >= 5:
                self.assertEqual(status, "confirmed")
                self.assertTrue(pieces)
        self.assertEqual([frame.packet.stamp_ns for frame in selected], [1, 3, 4, 5, 6, 7])
        self.assertEqual(len(selected), 6)
        self.assertEqual([frame.packet.stamp_ns for frame in block_state.frames],
                         [1, 3, 4, 5, 6, 7])
        self.assertEqual(len(block_state.frames), 6)


if __name__ == '__main__':
    unittest.main(verbosity=2)
