"""Prepared independent v5 reference/query tests; NOT RUN.

Requires root's real curve_reference + explicit BoundaryReference v5 integration.
No fake module, production builder, raw mask, vehicle, native or Unity oracle.
Array reconstruction is a synthetic payload check, not mapper save/load acceptance.
"""
from dataclasses import fields, replace
from fractions import Fraction
from pathlib import Path
import tempfile
import unittest

import numpy as np

from lane_mapping_lya_reference.curve_reference import CurveGeometry
from lane_mapping_lya_reference.paint_geometry import BoundaryConsensus, BoundaryReference
from lane_mapping_lya_reference.boundary_localization import BoundaryLaneLocalizer


ARRAY_FIELDS = ("segments", "normals", "owner", "frame_votes", "reliable_votes",
                "last_stamp_ns", "owner_status", "blocker_boxes", "blocker_owner")


def geometry(segments=(), *, normals=None, owners=None, statuses=(2,), boxes=(),
             box_owners=None, complete=True):
    segments = np.asarray(segments, dtype=np.float64).reshape(-1, 2, 2)
    count = len(segments)
    boxes = np.asarray(boxes, dtype=np.float64).reshape(-1, 2, 2)
    return CurveGeometry(
        segments=segments,
        normals=np.asarray([[0., 1.]] * count if normals is None else normals,
                           dtype=np.float64).reshape(-1, 2),
        owner=np.asarray([0] * count if owners is None else owners, dtype=np.int64),
        frame_votes=np.full(count, 5, dtype=np.int64),
        reliable_votes=np.ones(count, dtype=np.int64),
        last_stamp_ns=np.full(count, 80, dtype=np.int64),
        owner_status=np.asarray(statuses, dtype=np.int64),
        blocker_boxes=boxes,
        blocker_owner=np.asarray([0] * len(boxes) if box_owners is None else box_owners,
                                 dtype=np.int64), complete=complete)


def reference(curve, *, nodes=None, edges=(), full_in=None, full_out=None,
              spans=None, span_ids=None):
    count = len(curve.owner_status)
    nodes = np.asarray([[float(i), 0.] for i in range(count)] if nodes is None else nodes,
                       dtype=np.float64).reshape(count, 2)
    edges = np.asarray(edges, dtype=np.int64).reshape(-1, 2)
    kwargs = dict(
        directed_edges=edges,
        edge_frame_votes=np.full(len(edges), 5, dtype=np.int64),
        edge_reliable_votes=np.ones(len(edges), dtype=np.int64),
        edge_last_stamp_ns=np.full(len(edges), 80, dtype=np.int64),
        curve_geometry=curve)
    if full_in is not None:
        kwargs["full_in_degree"] = np.asarray(full_in, dtype=np.int64)
        kwargs["full_out_degree"] = np.asarray(full_out, dtype=np.int64)
    if spans is not None:
        spans = np.asarray(spans, dtype=np.float64).reshape(-1, 2)
        kwargs.update(
            continuation_intervals=spans,
            continuation_primitive=np.asarray([0] * len(spans) if span_ids is None else span_ids,
                                              dtype=np.int64),
            continuation_frame_votes=np.full(len(spans), 5, dtype=np.int64),
            continuation_reliable_votes=np.ones(len(spans), dtype=np.int64),
            continuation_last_stamp_ns=np.full(len(spans), 80, dtype=np.int64))
    return BoundaryReference(
        nodes, np.tile([0., 1.], (count, 1)),
        np.empty((0, 2, 2)), np.empty(0, dtype=np.int64),
        np.full(count, 10, dtype=np.int64), np.full(count, 5, dtype=np.int64),
        np.arange(count, dtype=np.int64), 100, **kwargs)


def match(ref, xy, normal=(0., 1.)):
    matcher = BoundaryLaneLocalizer(ref)
    result = matcher._correspondences(
        np.asarray(xy, dtype=np.float64).reshape(-1, 2), np.zeros(3),
        np.tile(np.asarray(normal, dtype=np.float64), (len(xy), 1)))
    return matcher, result


def horizontal(y):
    return ((-1., y), (1., y))


class CurveReferenceGeometryTests(unittest.TestCase):
    def test_exact_field_contract(self):
        self.assertEqual(tuple(field.name for field in fields(CurveGeometry)),
                         ARRAY_FIELDS + ("complete",))

    def test_input_arrays_are_independent_immutable_bytes(self):
        original = geometry([horizontal(.125)])
        kwargs = {name: getattr(original, name).copy() for name in ARRAY_FIELDS}
        frozen = CurveGeometry(**kwargs, complete=True)
        before = {name: getattr(frozen, name).tobytes() for name in ARRAY_FIELDS}
        for name, value in kwargs.items():
            value[...] = 0
            self.assertEqual(getattr(frozen, name).tobytes(), before[name])
            self.assertFalse(getattr(frozen, name).flags.writeable)
            with self.assertRaises(ValueError):
                getattr(frozen, name).flags.writeable = True

    def test_bad_geometry_fields_are_rejected(self):
        good = geometry([horizontal(0.)])
        bad = (
            {"complete": 1}, {"owner": np.array([1])},
            {"owner_status": np.array([3])}, {"frame_votes": np.array([0])},
            {"reliable_votes": np.array([6])}, {"last_stamp_ns": np.array([0])},
            {"normals": np.array([[0., 0.]])},
            {"normals": np.array([[1., 0.]])},
            {"segments": np.array([[[0., 0.], [0., 0.]]])},
            {"segments": np.array([[[0., 0.], [np.nan, 0.]]])},
            {"blocker_boxes": np.array([[[1., 0.], [0., 1.]]]),
             "blocker_owner": np.array([0])})
        for change in bad:
            with self.subTest(change=tuple(change)):
                with self.assertRaises(ValueError):
                    replace(good, **change)

    def test_subset_keeps_all_selected_boxes_and_occurrence_order(self):
        curve = geometry([horizontal(.125), horizontal(.25), horizontal(.375)],
            owners=[0, 2, 0], statuses=(0, 1, 2),
            boxes=[((-1., 0.), (1., 0.)), ((-.5, .5), (.5, .5)), ((-1., 0.), (1., 0.))],
            box_owners=[0, 1, 0])
        sliced = curve.subset(np.array([1, 0], dtype=np.int64))
        np.testing.assert_array_equal(sliced.owner_status, [1, 0])
        np.testing.assert_array_equal(sliced.owner, [1, 1])
        np.testing.assert_array_equal(sliced.blocker_owner, [1, 0, 1])
        self.assertEqual(sliced.segments.tobytes(), curve.segments[[0, 2]].tobytes())
        self.assertEqual(sliced.blocker_boxes.tobytes(), curve.blocker_boxes.tobytes())

    def test_crop_cannot_turn_incomplete_into_complete(self):
        curve = geometry([horizontal(0.)], statuses=(2, 0), complete=False)
        self.assertFalse(curve.subset(np.array([0])).complete)
        self.assertFalse(curve.subset(np.array([], dtype=np.int64)).complete)

    def test_standalone_unknown_boxes_survive_empty_and_reordered_subsets(self):
        curve = geometry([horizontal(.125)], statuses=(2, 0),
            boxes=[((-1., 0.), (1., 0.)), ((2., 2.), (3., 3.)), ((-1., 0.), (1., 0.))],
            box_owners=[-1, 1, -1])
        empty = curve.subset(np.empty(0, dtype=np.int64))
        self.assertEqual(len(empty.owner_status), 0)
        np.testing.assert_array_equal(empty.blocker_owner, [-1, -1])
        self.assertEqual(empty.blocker_boxes.tobytes(), curve.blocker_boxes[[0, 2]].tobytes())
        sliced = reference(curve).subset(np.array([1, 0], dtype=np.int64))
        np.testing.assert_array_equal(sliced.curve_geometry.blocker_owner, [-1, 0, -1])
        self.assertEqual(sliced.curve_geometry.blocker_boxes.tobytes(), curve.blocker_boxes.tobytes())
        with self.assertRaises(ValueError):
            replace(curve, blocker_owner=np.array([-2, 1, -1]))

    def test_standalone_unknown_is_only_a_local_veto_never_a_residual(self):
        curve = geometry([horizontal(.25)], boxes=[((-1., 0.), (1., 0.))], box_owners=[-1])
        _, blocked = match(reference(curve), [[0., .0625]])
        self.assertEqual(len(blocked[2]), 0)
        far = replace(curve, blocker_boxes=np.array([[[3., 3.], [4., 4.]]]))
        _, clear = match(reference(far), [[0., .0625]])
        np.testing.assert_array_equal(clear[2], [0])
        np.testing.assert_array_equal(clear[4], [-.1875])

    def test_subset_invalid_indices_do_not_mutate_original(self):
        curve = geometry([horizontal(0.)], statuses=(2, 0))
        before = tuple(getattr(curve, name).tobytes() for name in ARRAY_FIELDS)
        for indices in ([0, 0], [-1], [2], [.0], [[0]]):
            with self.subTest(indices=indices), self.assertRaises(ValueError):
                curve.subset(np.asarray(indices))
        self.assertEqual(before, tuple(getattr(curve, name).tobytes() for name in ARRAY_FIELDS))

    def test_graph_edge_and_meanline_are_not_nominal_v5_geometry(self):
        curve = geometry([horizontal(.25)], statuses=(2, 2))
        ref = reference(curve, nodes=[[-1., 0.], [1., 0.]], edges=[(0, 1)])
        _, result = match(ref, [[0., .0625]])
        np.testing.assert_array_equal(result[2], [0])
        np.testing.assert_array_equal(result[3], [[0., 1.]])
        np.testing.assert_array_equal(result[4], [-.1875])
        self.assertEqual(ref.boundary_version, 5)

    def test_zero_nominal_geometry_does_not_fall_back_to_graph_or_faces(self):
        curve = geometry(statuses=(1, 1), boxes=[((-1., 0.), (1., 0.))], box_owners=[0])
        ref = reference(curve, nodes=[[-1., 0.], [1., 0.]], edges=[(0, 1)])
        ref = replace(ref, support_segments=np.array([horizontal(0.)]),
                      segment_owner=np.array([0], dtype=np.int64))
        _, result = match(ref, [[0., .0625]])
        self.assertEqual(len(result[2]), 0)

    def test_unknown_nearer_box_blocks_farther_ready_curve(self):
        curve = geometry([horizontal(.25)], statuses=(2, 0),
                         boxes=[((-1., 0.), (1., 0.))], box_owners=[1])
        _, result = match(reference(curve), [[0., .0625]])
        self.assertEqual(len(result[2]), 0)

    def test_pending_signature_on_confirmed_node_remains_competitor(self):
        curve = geometry([horizontal(.25)], statuses=(2, 1),
                         boxes=[((-1., 0.), (1., 0.))], box_owners=[1])
        _, result = match(reference(curve), [[0., .0625]])
        self.assertEqual(len(result[2]), 0)

    def test_dual_signature_same_owner_does_not_choose_only_ready_key(self):
        curve = geometry([horizontal(.25)], statuses=(0,),
                         boxes=[((-1., 0.), (1., 0.))])
        _, result = match(reference(curve), [[0., .0625]])
        self.assertEqual(len(result[2]), 0)

    def test_blocker_distance_tie_is_closed_rejection(self):
        curve = geometry([horizontal(.25)], statuses=(2, 0),
                         boxes=[((-1., 0.), (1., 0.))], box_owners=[1])
        query = (Fraction(0), Fraction(1, 8))
        known_squared = (query[1] - Fraction(1, 4)) ** 2
        blocker_squared = query[1] ** 2
        self.assertEqual(known_squared, blocker_squared)
        _, result = match(reference(curve), [[float(x) for x in query]])
        self.assertEqual(len(result[2]), 0)

    def test_box_bounds_must_not_round_near_ties_into_clearance(self):
        # Every point-box lies on the independently known vertical segment;
        # all exact binary64 rational distances are equal, even after translation.
        for x in (1., np.nextafter(1., np.inf), np.nextafter(1., -np.inf), 1048576.):
            curve = geometry([((x, -1.), (x, 1.))], normals=[(1., 0.)], statuses=(2, 0),
                             boxes=[((x, 0.), (x, 0.))], box_owners=[1])
            for qx in (x-.5, np.nextafter(x, -np.inf)):
                exact = (Fraction.from_float(float(x)) - Fraction.from_float(float(qx))) ** 2
                self.assertGreaterEqual(exact, 0)
                _, result = match(reference(curve), [[qx, 0.]], normal=(1., 0.))
                self.assertEqual(len(result[2]), 0)

    def test_proven_farther_blocker_does_not_globally_disable_ready_geometry(self):
        curve = geometry([horizontal(0.)], statuses=(2, 0),
                         boxes=[((-1., .5), (1., .5))], box_owners=[1])
        _, result = match(reference(curve), [[0., .0625]])
        np.testing.assert_array_equal(result[2], [0])
        np.testing.assert_array_equal(result[4], [.0625])

    def test_incomplete_bundle_rejects_even_when_ready_piece_exists(self):
        _, result = match(reference(geometry([horizontal(0.)], complete=False)), [[0., .0625]])
        self.assertEqual(len(result[2]), 0)

    def test_nearest_terminal_does_not_retry_farther_curve(self):
        curve = geometry([((0., 0.), (.125, 0.)), horizontal(.25)],
                         owners=[0, 1], statuses=(2, 2))
        _, result = match(reference(curve), [[-.0625, .015625]])
        self.assertEqual(len(result[2]), 0)

    def test_existing_signed_gate_remains_signed_not_absolute(self):
        curve = geometry([horizontal(0.)], normals=[(0., -1.)])
        _, result = match(reference(curve), [[0., .0625]], normal=(0., 1.))
        self.assertEqual(len(result[2]), 0)
        _, reverse = match(reference(curve), [[0., .0625]], normal=(0., -1.))
        np.testing.assert_array_equal(reverse[2], [0])
        np.testing.assert_array_equal(reverse[4], [-.0625])

    def test_known_ineligible_opposite_side_retains_original_angle_selection(self):
        curve = geometry([horizontal(0.), horizontal(.25)],
                         normals=[(0., -1.), (0., 1.)], owners=[0, 1], statuses=(2, 2))
        _, result = match(reference(curve), [[0., .0625]])
        np.testing.assert_array_equal(result[2], [0])
        np.testing.assert_array_equal(result[4], [-.1875])

    def test_oblique_closest_foot_and_residual_use_same_actual_piece(self):
        curve = geometry([((-.5, -.375), (.5, .375))], normals=[(-.6, .8)])
        ref = reference(curve)
        matcher, result = match(ref, [[0., .125]], normal=(-.6, .8))
        targets = matcher._index.graph.query(np.array([[0., .125]]), np.array([[-.6, .8]]))[1]
        expected_foot = (Fraction(3, 50), Fraction(9, 200))
        expected_residual = Fraction(1, 8) * Fraction(4, 5)
        np.testing.assert_allclose(targets[0], [float(x) for x in expected_foot], rtol=0, atol=2e-15)
        np.testing.assert_array_equal(result[2], [0])
        np.testing.assert_allclose(result[4], [float(expected_residual)], rtol=0, atol=2e-15)

    def test_closed_exact_endpoint_keeps_finite_convention_without_mean_snap(self):
        curve = geometry([((0., .25), (.5, .25))])
        matcher, result = match(reference(curve, nodes=[[0., 0.]]), [[0., .3125]])
        np.testing.assert_array_equal(result[2], [0])
        targets = matcher._index.graph.query(np.array([[0., .3125]]), np.array([[0., 1.]]))[1]
        np.testing.assert_array_equal(targets, [[0., .25]])
        np.testing.assert_array_equal(result[4], [.0625])

    def test_graph_shared_mean_vertex_does_not_snap_curve_end_to_mean(self):
        curve = geometry([((-.5, .25), (0., .25)), ((0., .25), (.5, .25))],
                         statuses=(2, 2, 2))
        ref = reference(curve, nodes=[[-.5, 0.], [0., 0.], [.5, 0.]], edges=[(0, 1), (1, 2)])
        matcher, result = match(ref, [[0., .3125]])
        targets = matcher._index.graph.query(np.array([[0., .3125]]), np.array([[0., 1.]]))[1]
        np.testing.assert_array_equal(targets, [[0., .25]])
        np.testing.assert_array_equal(result[4], [.0625])

    def test_branch_permission_rejects_nearest_without_farther_retry(self):
        curve = geometry([horizontal(0.), horizontal(.25)], owners=[0, 1], statuses=(2, 2))
        ref = reference(curve, full_in=[2, 0], full_out=[2, 0], spans=[])
        _, result = match(ref, [[0., .0625]])
        self.assertEqual(len(result[2]), 0)

    def test_branch_permission_applies_to_actual_curve_alpha(self):
        curve = geometry([horizontal(.25)])
        ref = reference(curve, full_in=[2], full_out=[2], spans=[(.25, .75)])
        _, middle = match(ref, [[0., .3125]])
        np.testing.assert_array_equal(middle[2], [0])
        _, outside = match(ref, [[-.75, .3125]])
        self.assertEqual(len(outside[2]), 0)

    def test_v5_continuation_requires_original_five_frames(self):
        ref = reference(geometry([horizontal(.25)]), full_in=[2], full_out=[2],
                        spans=[(.25, .75)])
        for votes in range(1, 5):
            with self.subTest(votes=votes), self.assertRaises(ValueError):
                replace(ref, continuation_frame_votes=np.array([votes], dtype=np.int64))
        np.testing.assert_array_equal(match(ref, [[0., .3125]])[1][2], [0])

    def test_v5_exported_topology_requires_five_plus_one_but_legacy_does_not(self):
        ref = reference(geometry([horizontal(.25)], statuses=(2, 2)), edges=[(0, 1)])
        changes = [dict(edge_frame_votes=np.array([votes], dtype=np.int64))
                   for votes in range(1, 5)]
        changes.append(dict(edge_reliable_votes=np.array([0], dtype=np.int64)))
        for change in changes:
            with self.subTest(change=change), self.assertRaises(ValueError):
                replace(ref, **change)
            # Old explicit-graph contracts support caller-selected thresholds.
            legacy = replace(ref, curve_geometry=None, **change)
            self.assertEqual(legacy.boundary_version, 2)
            for name, value in change.items():
                np.testing.assert_array_equal(getattr(legacy, name), value)

    def test_v5_ready_status_requires_confirmed_owner_but_pending_is_retained(self):
        ready = reference(geometry(statuses=(2,)))
        for votes, reliable in ((4, 1), (5, 0)):
            with self.subTest(votes=votes, reliable=reliable), self.assertRaises(ValueError):
                replace(ready, votes=np.array([votes]), reliable_votes=np.array([reliable]))
        for status in (0, 1):
            pending = replace(reference(geometry(statuses=(status,))),
                              votes=np.array([1]), reliable_votes=np.array([0]))
            self.assertEqual(pending.curve_geometry.owner_status.tolist(), [status])
            self.assertEqual(len(pending.record_ids), 1)
            self.assertEqual(len(pending.curve_geometry.segments), 0)

    def test_reference_subset_preserves_global_degree_status_and_blocker(self):
        curve = geometry([horizontal(.25)], statuses=(2, 0, 1),
                         boxes=[((-1., 0.), (1., 0.)), ((2., 0.), (3., 0.))], box_owners=[1, 2])
        ref = reference(curve, full_in=[2, 1, 0], full_out=[2, 0, 1])
        sliced = ref.subset(np.array([1, 0], dtype=np.int64))
        np.testing.assert_array_equal(sliced.full_in_degree, [1, 2])
        np.testing.assert_array_equal(sliced.full_out_degree, [0, 2])
        np.testing.assert_array_equal(sliced.curve_geometry.owner_status, [0, 2])
        np.testing.assert_array_equal(sliced.curve_geometry.blocker_owner, [0])
        self.assertEqual(sliced.boundary_version, 5)
        _, result = match(sliced, [[0., .0625]])
        self.assertEqual(len(result[2]), 0)

    def test_v5_frozen_matcher_identity_includes_geometry_and_status(self):
        curve = geometry([horizontal(.25)], statuses=(2, 0),
                         boxes=[((-1., 0.), (1., 0.))], box_owners=[1])
        ref = reference(curve)
        matcher = BoundaryLaneLocalizer(ref)
        changes = (
            {"segments": np.array([horizontal(.375)])},
            {"normals": np.array([[0., -1.]])},
            {"owner": np.array([1])}, {"frame_votes": np.array([6])},
            {"reliable_votes": np.array([2])}, {"last_stamp_ns": np.array([79])},
            {"owner_status": np.array([2, 1])},
            {"blocker_boxes": np.array([[[-1., .125], [1., .125]]])},
            {"blocker_owner": np.array([0])}, {"complete": False})
        for change in changes:
            with self.subTest(change=tuple(change)):
                self.assertFalse(matcher.matches_anchor_source(
                    replace(ref, curve_geometry=replace(curve, **change))))

    def test_fresh_npz_payload_reconstruction_keeps_all_fields_and_query(self):
        curve = geometry([horizontal(.25)], statuses=(2, 0),
                         boxes=[((-1., 0.), (1., 0.))], box_owners=[1])
        payload = {name: getattr(curve, name) for name in ARRAY_FIELDS}
        payload["complete"] = np.asarray(curve.complete, dtype=np.bool_)
        with tempfile.TemporaryDirectory(prefix="v5_curve_reference_fixture_") as directory:
            scope = Path(directory).resolve()
            target = (scope / "fixture.npz").resolve()
            self.assertEqual(target.parent, scope)
            np.savez(target, **payload)
            with np.load(target, allow_pickle=False) as archive:
                reconstructed = CurveGeometry(**{name: archive[name].copy() for name in ARRAY_FIELDS},
                                              complete=bool(archive["complete"].item()))
        self.assertIsNot(reconstructed, curve)
        for name in ARRAY_FIELDS:
            self.assertEqual(getattr(reconstructed, name).tobytes(), getattr(curve, name).tobytes())
        original_matcher, original = match(reference(curve), [[0., .0625]])
        fresh_matcher, fresh = match(reference(reconstructed), [[0., .0625]])
        self.assertIsNot(original_matcher, fresh_matcher)
        for old, new in zip(original, fresh):
            self.assertEqual(old.tobytes(), new.tobytes())

    def test_legacy_reference_is_not_silently_upgraded(self):
        legacy = BoundaryReference(
            np.array([[-1., 0.], [1., 0.]]), np.array([[0., 1.], [0., 1.]]),
            np.empty((0, 2, 2)), np.empty(0, dtype=np.int64),
            np.full(2, 10, dtype=np.int64), np.full(2, 5, dtype=np.int64),
            np.arange(2, dtype=np.int64), 100,
            directed_edges=np.array([[0, 1]], dtype=np.int64),
            edge_frame_votes=np.array([5]), edge_reliable_votes=np.array([1]),
            edge_last_stamp_ns=np.array([80]))
        self.assertEqual(legacy.boundary_version, 2)
        _, result = match(legacy, [[0., .0625]])
        np.testing.assert_array_equal(result[2], [0])
        np.testing.assert_array_equal(result[4], [.0625])

    def test_legacy_custom_confirmation_config_is_not_overridden(self):
        consensus = BoundaryConsensus({"minimum_frame_votes": 3, "minimum_reliable_votes": 1})
        self.assertEqual(consensus.config["minimum_frame_votes"], 3)
        self.assertEqual(consensus.config["minimum_reliable_votes"], 1)

    def test_explicit_v5_rejects_incompatible_confirmation_config(self):
        for votes, reliable in ((3, 1), (5, 2), (6, 1)):
            with self.subTest(votes=votes, reliable=reliable), self.assertRaises(ValueError):
                BoundaryConsensus({"minimum_frame_votes": votes,
                                   "minimum_reliable_votes": reliable}, curve_geometry=True)


if __name__ == "__main__":
    unittest.main()
