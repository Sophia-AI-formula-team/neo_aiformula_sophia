"""Failure-log consistency through the real shared match control flow.

Only feature preparation/correspondences are controlled geometry doubles.
Original min_scan=30, min_inliers=24, overlap=.55 and all pose gates stay intact.
This is not a camera/trajectory-quality or runtime-performance test.
"""
import unittest

import numpy as np

from lane_mapping_lya_reference.boundary_localization import BoundaryLaneLocalizer
from lane_mapping_lya_reference.paint_geometry import BoundaryReference


class ReanchoredGeometry(BoundaryLaneLocalizer):
    def __init__(self, selected_count, *, weights=None, denominator=100., initial_count=60):
        x = np.linspace(0., 6., 60)
        points = np.column_stack((x, np.zeros(60)))
        normals = np.tile((0., 1.), (60, 1))
        segments = np.stack((points-np.array([.04, 0.]), points+np.array([.04, 0.])), axis=1)
        reference = BoundaryReference(points, normals, segments, np.arange(60),
            np.full(60, 5), np.ones(60, dtype=int), np.arange(60), 1)
        super().__init__(reference)
        self.selected_count = selected_count
        self.initial_count = initial_count
        self.weights = np.ones(60) if weights is None else np.asarray(weights, dtype=float)
        self.denominator = float(denominator)
        self.queries = []
        self._correction[:] = (.3, 0., 0.)
        self.last_accepted_stamp_ns = 100
        self.last_seen_stamp_ns = 100
        self.local = points.copy()

    def _prepare_scan_geometry(self, scan, support, metrics):
        metrics.update(scan_points=len(scan), boundary_original_foreground_mass=self.denominator,
            overlap_denominator='complete_original_foreground_mass_before_boundary_assignment')
        return (self.local, np.tile((0., 1.), (60, 1)), self.weights, self.denominator), None

    def _correspondences(self, local, estimate, scan_normals=None):
        self.queries.append(np.asarray(estimate).copy())
        count = self.initial_count if estimate[0] > .2 else self.selected_count
        selected = np.arange(count, dtype=int)
        self._last_correspondence_support_metrics = {
            'nearest_correspondences_before_support': len(local),
            'finite_support_rejected_correspondences': len(local)-count,
            'finite_support_covered_correspondences': count}
        world = local+estimate[:2]
        return world, local.copy(), selected, np.tile((0., 1.), (count, 1)), np.zeros(count)


class RawAnchorOverlapMetricsTests(unittest.TestCase):
    def run_failure(self, count, **kwargs):
        target = ReanchoredGeometry(count, **kwargs)
        self.assertEqual(target.config['min_overlap_ratio'], .55)
        self.assertEqual(target.config['min_inliers'], 24)
        self.assertEqual(target.config['min_scan_points'], 30)
        before = target.capture_tracking_state()
        before_correction = target._correction.tobytes()
        result = target.match(target.local, [0., 0., 0.], 200, prior_stamp_ns=199)
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, 'insufficient_overlap_after_raw_prior_anchor')
        self.assertEqual(len(target.queries), 2)
        np.testing.assert_array_equal(target.queries[0], [.3, 0., 0.])
        np.testing.assert_array_equal(target.queries[1], [0., 0., 0.])
        np.testing.assert_array_equal(result.pose_xyyaw, [.3, 0., 0.])
        self.assertEqual(target._correction.tobytes(), before_correction)
        self.assertEqual(target.last_accepted_stamp_ns, 100)
        self.assertEqual(target.accepted_frames, 0)
        self.assertEqual(target.rejected_frames, 1)
        self.assertIsNone(target._pending_result)
        self.assertIsNone(target._pending_data)
        self.assertIsNone(target._pending_motion_information)
        after = target.capture_tracking_state()
        self.assertEqual(before.config, after.config)
        self.assertEqual(before.correction_se2, after.correction_se2)
        self.assertEqual(before.last_accepted_stamp_ns, after.last_accepted_stamp_ns)
        self.assertIs(before.motion_information, after.motion_information)
        self.assertEqual(after.last_seen_stamp_ns, 200)
        return target, result.metrics

    def assert_consistent(self, target, metrics):
        count = target.selected_count
        numerator = float(np.sum(target.weights[:count]))
        self.assertEqual(metrics['overlap_evaluation_stage'], 'after_raw_prior_anchor')
        self.assertEqual(metrics['overlap_correspondence_count'], count)
        self.assertEqual(metrics['overlap_scan_point_count'], 60)
        self.assertEqual(metrics['overlap_weight_numerator'], numerator)
        self.assertEqual(metrics['overlap_weight_denominator'], target.denominator)
        self.assertEqual(metrics['overlap_weight_denominator'], metrics['scan_feature_weight_sum'])
        self.assertEqual(metrics['overlap_ratio'], float(numerator/target.denominator))
        self.assertEqual(metrics['unweighted_overlap_ratio'], float(count/60))
        self.assertEqual(metrics['finite_support_covered_correspondences'], count)
        self.assertEqual(metrics['finite_support_rejected_correspondences'], 60-count)
        self.assertEqual(metrics['overlap_denominator'],
                         'complete_original_foreground_mass_before_boundary_assignment')
        # Residual-based final inliers were not evaluated; do not invent them.
        self.assertEqual(metrics['inliers'], 0)
        self.assertEqual(metrics['inlier_ratio'], 0.)
        self.assertIsNone(metrics['rmse_after_m'])

    def test_weighted_failure_reports_recomputed_not_previous_ratio(self):
        target, metrics = self.run_failure(30)
        self.assert_consistent(target, metrics)
        self.assertEqual(metrics['overlap_ratio'], .3)
        self.assertNotEqual(metrics['overlap_ratio'], .6)

    def test_nonuniform_weights_do_not_substitute_count_ratio(self):
        weights = np.r_[np.full(30, .25), np.full(30, 1.75)]
        target, metrics = self.run_failure(30, weights=weights)
        self.assert_consistent(target, metrics)
        self.assertEqual(metrics['overlap_ratio'], .075)
        self.assertEqual(metrics['unweighted_overlap_ratio'], .5)

    def test_zero_correspondences_reports_zero_numerator_and_ratio(self):
        target, metrics = self.run_failure(0)
        self.assert_consistent(target, metrics)
        self.assertEqual(metrics['overlap_weight_numerator'], 0.)

    def test_count_only_failure_can_legitimately_have_ratio_above_threshold(self):
        weights = np.r_[np.full(23, 3.), np.full(37, .1)]
        target, metrics = self.run_failure(23, weights=weights)
        self.assert_consistent(target, metrics)
        self.assertGreaterEqual(metrics['overlap_ratio'], target.config['min_overlap_ratio'])
        self.assertLess(metrics['overlap_correspondence_count'], target.config['min_inliers'])

    def test_equal_minimum_count_still_rejects_insufficient_mass(self):
        target, metrics = self.run_failure(24)
        self.assert_consistent(target, metrics)

    def test_just_below_overlap_threshold_logs_exact_failure_value(self):
        target, metrics = self.run_failure(54)
        self.assert_consistent(target, metrics)
        self.assertEqual(metrics['overlap_ratio'], .54)

    def test_denominator_keeps_unassigned_foreground_not_only_selected_weights(self):
        target, metrics = self.run_failure(30, denominator=105.)
        self.assert_consistent(target, metrics)
        self.assertEqual(metrics['overlap_weight_denominator'], 105.)
        self.assertNotEqual(metrics['overlap_weight_denominator'], np.sum(target.weights))

    def test_first_query_failure_remains_first_query_with_no_reanchor_stage(self):
        target = ReanchoredGeometry(30, initial_count=40)
        result = target.match(target.local, [0., 0., 0.], 200, prior_stamp_ns=199)
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, 'insufficient_overlap')
        self.assertEqual(len(target.queries), 1)
        self.assertEqual(result.metrics['overlap_ratio'], .4)
        self.assertNotIn('overlap_evaluation_stage', result.metrics)


if __name__ == '__main__':
    unittest.main()
