"""Independent additional v5 guards, PREPARED NOT RUN; frozen29 unchanged.

Fraction oracles directly substitute input binary64 values, not subject bounds.
The small base-method stub in the metrics test isolates logging only; it does not
claim to test real scan geometry, raw-source completeness or runtime performance.
"""
from fractions import Fraction
import unittest
from unittest.mock import patch

import numpy as np

from lane_mapping_lya_reference.boundary_localization import (
    BoundaryLaneLocalizer, _curve_blocked, _curve_distance_upper)
from lane_mapping_lya_reference.curve_reference import CurveGeometry
from lane_mapping_lya_reference.mask_localization import LaneMapLocalizer
from lane_mapping_lya_reference.paint_geometry import BoundaryObservation, BoundaryReference


def rational(value):
    return Fraction.from_float(float(value))


def exact_feasible_squared(point, first, last, fraction):
    t = rational(fraction)
    return sum((rational(p) - (rational(a) + t * (rational(b) - rational(a)))) ** 2
               for p, a, b in zip(point, first, last))


def exact_box_squared(point, box):
    return sum(max(Fraction(0), rational(lower) - rational(p),
                   rational(p) - rational(upper)) ** 2
               for p, lower, upper in zip(point, box[0], box[1]))


def ready_reference(*, pieces=1, unknown=0):
    count = unknown + 1
    segments = np.array([[[-1., 0.], [1., 0.]]] * pieces, dtype=np.float64)
    boxes = np.array([[[-1., .5], [1., .5]]] * unknown, dtype=np.float64).reshape(-1, 2, 2)
    curve = CurveGeometry(segments, np.tile([0., 1.], (pieces, 1)),
        np.zeros(pieces, dtype=np.int64), np.full(pieces, 5, dtype=np.int64),
        np.ones(pieces, dtype=np.int64), np.full(pieces, 80, dtype=np.int64),
        np.r_[np.int64(2), np.zeros(unknown, dtype=np.int64)], boxes,
        np.arange(1, count, dtype=np.int64), True)
    return BoundaryReference(np.zeros((count, 2)), np.tile([0., 1.], (count, 1)),
        np.empty((0, 2, 2)), np.empty(0, dtype=np.int64),
        np.full(count, 10, dtype=np.int64), np.full(count, 5, dtype=np.int64),
        np.arange(count, dtype=np.int64), 100,
        directed_edges=np.empty((0, 2), dtype=np.int64),
        edge_frame_votes=np.empty(0, dtype=np.int64),
        edge_reliable_votes=np.empty(0, dtype=np.int64),
        edge_last_stamp_ns=np.empty(0, dtype=np.int64), curve_geometry=curve)


class CurveNumericGuardTests(unittest.TestCase):
    def test_upper_encloses_exact_binary64_feasible_point(self):
        cases = (
            ((-2., 3.), (0., 0.), (4., 2.), .25),
            ((6., -3.), (0., 0.), (4., 2.), .25),
            ((1., .5), (0., 0.), (4., 2.), .25),
            ((.1, -.3), (-.7, .9), (.2, -.4), .1),
            ((1.e7, -1.e7), (1.e7, -1.e7), (1.e7 - 2. ** -10, -1.e7 + 2. ** -9), .1))
        for point, first, last, t in cases:
            with self.subTest(point=point, fraction=t):
                upper = _curve_distance_upper(np.array([point]), np.array([first]),
                                              np.array([last]), np.array([t]))[0]
                self.assertTrue(np.isfinite(upper))
                self.assertGreaterEqual(rational(upper), exact_feasible_squared(point, first, last, t))

    def test_upper_endpoint_underflow_and_signed_zero(self):
        for y in (0., -0., 1.e-160, -1.e-160, np.nextafter(0., np.inf)):
            for t in (0., .5, 1.):
                point, first, last = (t, y), (0., 0.), (1., 0.)
                upper = _curve_distance_upper(np.array([point]), np.array([first]),
                                              np.array([last]), np.array([t]))[0]
                self.assertGreaterEqual(rational(upper), exact_feasible_squared(point, first, last, t))

    def test_box_exclusion_never_discards_exact_nearer_or_tied_box(self):
        points = np.array([[0., 0.], [np.nextafter(1., 0.), -0.], [1.e7, 1.e-160]])
        boxes = np.array([[[0., 0.], [0., 0.]], [[1., 0.], [1., 0.]],
                          [[-1., -.125], [1., .125]], [[1.e7, 0.], [1.e7, 0.]]])
        for bound in (0., 2. ** -104, .125, .5, 1.):
            blocked = _curve_blocked(points, np.full(len(points), bound), boxes)
            for index, point in enumerate(points):
                if any(exact_box_squared(point, box) <= rational(bound) for box in boxes):
                    self.assertTrue(blocked[index])

    def test_true_tie_blocks_but_well_separated_far_box_does_not(self):
        point = np.array([[0., .125]])
        first, last = np.array([[-1., .25]]), np.array([[1., .25]])
        upper = _curve_distance_upper(point, first, last, np.array([.5]))
        tied = np.array([[[-1., 0.], [1., 0.]]])
        distant = np.array([[[-1., .75], [1., .75]]])
        self.assertEqual(exact_box_squared(point[0], tied[0]),
                         exact_feasible_squared(point[0], first[0], last[0], .5))
        self.assertTrue(_curve_blocked(point, upper, tied)[0])
        self.assertFalse(_curve_blocked(point, upper, distant)[0])

    def test_all_chunks_nonfinite_failure_and_input_fp_state_are_preserved(self):
        points = np.array([[0., 0.]])
        boxes = np.array([[[0., 1.], [0., 1.]]] * 512 + [[[0., 0.], [0., 0.]]])
        arrays = (points, boxes)
        before = tuple(value.tobytes() for value in arrays)
        error_mode = np.geterr()
        try:
            np.seterr(all="raise")
            strict = np.geterr()
            self.assertTrue(_curve_blocked(points, np.array([0.]), boxes)[0])
            self.assertTrue(_curve_blocked(points, np.array([np.inf]), boxes[:0])[0])
            self.assertTrue(_curve_blocked(points, np.array([np.nan]), boxes[:0])[0])
            _curve_distance_upper(points, np.array([[0., 0.]]), np.array([[1., 0.]]), np.array([0.]))
            self.assertEqual(np.geterr(), strict)
        finally:
            np.seterr(**error_mode)
        self.assertEqual(before, tuple(value.tobytes() for value in arrays))

    def test_patch_capacity_includes_unknown_competing_owners(self):
        # Every box intersects the same original +/-0.65m patch window.
        # They are deliberately farther, so a failed cap must not be disguised
        # as geometric rejection by an accidentally close unknown box.
        reference = ready_reference(unknown=513)
        matcher = BoundaryLaneLocalizer(reference)
        self.assertGreater(514, matcher.config["max_patch_points"])
        _, _, selected, _, _ = matcher._correspondences(np.array([[0., .0625]]),
                                                       np.zeros(3), np.array([[0., 1.]]))
        self.assertEqual(len(selected), 0)

    def test_segmentation_does_not_inflate_normal_position_support(self):
        matcher = BoundaryLaneLocalizer(ready_reference(pieces=37))
        np.testing.assert_array_equal(matcher._index.normal_valid, [True])
        np.testing.assert_array_equal(matcher._index.normal_position_support, [1])

    def test_patch_capacity_counts_standalone_footprints_not_the_shared_minus_one(self):
        from dataclasses import replace
        for unknown, expected in ((511, 1), (512, 0)):
            reference = ready_reference(unknown=unknown)
            curve = replace(reference.curve_geometry,
                blocker_owner=np.full(unknown, -1, dtype=np.int64))
            matcher = BoundaryLaneLocalizer(replace(reference, curve_geometry=curve))
            self.assertEqual(matcher.config["max_patch_points"], 512)
            _, _, selected, _, _ = matcher._correspondences(np.array([[0., .0625]]),
                np.zeros(3), np.array([[0., 1.]]))
            self.assertEqual(len(selected), expected)

    def test_scan_metrics_identify_actual_v5_not_mean_chords(self):
        matcher = BoundaryLaneLocalizer(ready_reference())
        observation = BoundaryObservation(
            np.array([[0., 0.]]), np.array([[0., 1.]]),
            np.array([[[-1., 0.], [1., 0.]]]), np.array([0.]),
            np.array([True]), np.array([True]), np.array([True]),
            np.array([True]), np.array([True]), 100,
            np.array([[[-.5, 0.], [.5, 0.]]]), np.array([[0, 0]]), np.array([[0, 1]]),
            np.array([-1]), np.array([-1]), np.array([0]), np.array([0]),
            np.array([6]), np.array([.45]))
        matcher.last_seen_stamp_ns = 100
        matcher._boundary_observation = observation
        metrics = {}
        prepared = ((np.array([[0., 0.]]), np.array([[0., 1.]]), np.array([1.]), 1.), None)
        try:
            with patch.object(LaneMapLocalizer, "_prepare_scan_geometry", autospec=True,
                              return_value=prepared) as base:
                result, failure = matcher._prepare_scan_geometry(np.array([[0., 0.]]), None, metrics)
                base.assert_called_once()
        finally:
            matcher._boundary_observation = None
        self.assertIsNone(failure)
        self.assertIsNotNone(result)
        self.assertEqual(metrics["boundary_version"], 5)
        self.assertIn("curve", metrics["line_reference"])
        self.assertNotIn("mean_node", metrics["line_reference"])
        self.assertNotIn("chord", metrics["correspondence_extent"])


if __name__ == "__main__":
    unittest.main()
