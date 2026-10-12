"""Synthetic contracts only. Prepared, not run; no real junction acceptance."""
import copy
from dataclasses import replace
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from observer import (ObserverParameters, observe_lane_geometry, inspect_footprint, _runs,
    _normal_separation_interval, _finish_pairs)


class QuantizedCurveTests(unittest.TestCase):
    def setUp(self):
        self.mask = np.zeros((1080, 1920), np.uint8)
        self.camera = dict(K=[800., 0., 960., 0., 800., 120., 0., 0., 1.],
            R=[0., 0., 1., -1., 0., 0., 0., -1., 0.], t=[0., 0., 1.])

    def uv(self, x, y):
        return round(960-800*y/x), round(120+800*self.camera["t"][2]/x)

    def line(self, a, b, thickness=1):
        cv2.line(self.mask, self.uv(*a), self.uv(*b), 255, thickness)

    def main_lane(self, end=9.5, both=True):
        for sign in ((-1, 1) if both else (1,)):
            self.line((1.5, sign*1.5), (end, sign*1.5))

    def side_pair(self):
        self.line((3., 1.8), (3., 3.2))
        self.line((5.2, 1.8), (5.2, 3.2))

    def observe(self, **kwargs):
        return observe_lane_geometry(self.mask, self.camera, enabled=True, **kwargs)

    def assert_no_recognition(self, result):
        self.assertFalse(result["junction_recognized"])
        self.assertEqual(result["junction_status"], "unknown")
        self.assertFalse(result["confirmation_eligible"])
        self.assertFalse(result["capture_metadata_admitted"])
        for pair in result["pair_hypotheses"]:
            self.assertFalse(pair["junction_recognized"])
            self.assertNotIn("world_anchor", pair)
            self.assertNotIn("official_id", pair)

    def test_default_no_input_access_or_import(self):
        with patch("builtins.__import__", side_effect=AssertionError("disabled imported")):
            value = observe_lane_geometry(object(), object())
        self.assertEqual(value["status"], "disabled")
        self.assert_no_recognition(value)

    def test_only_boolean_true_enables(self):
        for value in (None, 1, 0, "true"):
            self.assertEqual(observe_lane_geometry(None, None, enabled=value)["reason"], "enabled_must_be_bool")

    def test_observed_points_are_exact_original_positive_rays(self):
        self.main_lane()
        old_mask, old_camera = self.mask.copy(), copy.deepcopy(self.camera)
        value = self.observe()
        self.assertTrue(value["observed_points"])
        for item in value["observed_points"]:
            u, v = item["uv"]
            self.assertGreater(self.mask[v, u], 0)
            self.assertEqual(item["pixel_id"], v*1920+u)
            x = 800/(v-120)
            np.testing.assert_allclose(item["ground_xyz"], [x, -(u-960)*x/800, 0], atol=1e-10)
        np.testing.assert_array_equal(self.mask, old_mask)
        self.assertEqual(self.camera, old_camera)

    def test_estimates_are_separate_and_keep_real_support_identity(self):
        self.main_lane()
        value = self.observe()
        real = {p["pixel_id"] for p in value["observed_points"]}
        self.assertTrue(value["curves"])
        for curve in value["curves"]:
            self.assertFalse(curve["raw_points_replaced"])
            self.assertFalse(curve["gap_filled"])
            self.assertIn("estimated", curve["polyline_kind"])
            self.assertTrue(set(curve["ordered_pixel_ids"]) <= real)
            for sample in curve["samples"]:
                self.assertTrue(sample["tangent_is_estimated"])
                self.assertTrue(set(sample["tangent_support_endpoint_ids"]) <= real)

    def test_continuous_uv_staircase_is_not_split_at_each_ground_row(self):
        cv2.line(self.mask, (350, 220), (650, 210), 255, 1)
        value = self.observe(parameters=replace(ObserverParameters(), max_forward_m=20, max_lateral_m=15))
        self.assertEqual(value["status"], "observed_geometry")
        self.assertLessEqual(len(value["curves"]), 2)
        self.assertTrue(any(len(curve["ordered_pixel_ids"]) > 100 for curve in value["curves"]))
        self.assert_no_recognition(value)

    def test_continuous_white_footprint_is_same_paint_not_two_boundaries(self):
        mask = np.zeros((20, 40), np.uint8)
        mask[5:15, 5:30] = 255
        result = inspect_footprint(mask, [8, 10], [25, 10])
        self.assertEqual(result["status"], "same_positive_paint_footprint")
        self.assertEqual(result["black_pixels"], 0)

    def test_black_gap_support_does_not_depend_on_component_identity(self):
        mask = np.zeros((30, 50), np.uint8)
        mask[3:25, 5:8] = 255
        mask[3:25, 35:38] = 255
        mask[3:5, 5:38] = 255  # Join the bands into the same connected component.
        _, labels = cv2.connectedComponents(mask)
        self.assertEqual(labels[15, 6], labels[15, 36])
        value = inspect_footprint(mask, [6, 15], [36, 15])
        self.assertEqual(value["status"], "separated_positive_paint_footprints")
        self.assertIn("not_drivable", value["black_pixel_means"])

    def test_one_pixel_black_hole_is_unresolved_not_independent_bands(self):
        mask = np.ones((5, 10), np.uint8)
        mask[2, 4] = 0
        self.assertEqual(inspect_footprint(mask, [1, 2], [8, 2])["status"], "unresolved_footprint")

    def test_intermediate_white_band_is_ambiguous(self):
        mask = np.zeros((5, 30), np.uint8)
        mask[2, [2, 3, 13, 14, 25, 26]] = 255
        self.assertEqual(inspect_footprint(mask, [2, 2], [26, 2])["status"], "additional_positive_bands_ambiguous")

    def test_clean_two_bands_can_support_only_unconfirmed_geometry(self):
        self.main_lane()
        self.side_pair()
        value = self.observe()
        supported = [p for p in value["pair_hypotheses"] if p["geometry_supported_under_assumptions"]]
        self.assertTrue(supported, value["reason"])
        for pair in supported:
            self.assertTrue(all(pair["checks"].values()))
            self.assertEqual(pair["status"], "paired_curve_support_under_local_plane_assumption")
            self.assertGreaterEqual(min(pair["conservative_observed_support_spans_m"]), 1.)
            for section in pair["matched_sections"]:
                self.assertGreaterEqual(section["width_interval_m"][0], 1.5)
                self.assertLessEqual(section["width_interval_m"][1], 6.)
                self.assertEqual(section["footprint"]["status"], "separated_positive_paint_footprints")
        self.assert_no_recognition(value)

    def test_curved_input_keeps_ordered_curve_without_global_line_fit(self):
        ys = np.linspace(1.8, 3.2, 40)
        for offset in (0., 2.2):
            uv = np.asarray([self.uv(3+offset+.08*(y-1.8)**2, y) for y in ys], np.int32)
            cv2.polylines(self.mask, [uv], False, 255, 1)
        value = self.observe()
        self.assertTrue(value["curves"])
        self.assertTrue(any(len(curve["estimated_uv_polyline"]) > 2 for curve in value["curves"]))
        self.assertTrue(all("ground_fit" not in curve for curve in value["curves"]))
        for pair in value["pair_hypotheses"]:
            if pair["geometry_supported_under_assumptions"]:
                self.assertTrue(pair["checks"]["monotonic_real_correspondence"])
        self.assert_no_recognition(value)

    def test_missing_main_after_is_not_filled(self):
        self.main_lane(end=5.5)
        self.side_pair()
        value = self.observe()
        self.assertFalse(any(p["geometry_supported_under_assumptions"] for p in value["pair_hypotheses"]))
        self.assert_no_recognition(value)

    def test_width_interval_merely_overlapping_limit_does_not_pass(self):
        self.main_lane()
        self.line((3., 1.8), (3., 3.2))
        self.line((4.5, 1.8), (4.5, 3.2))
        value = self.observe()
        overlapping = [p for p in value["pair_hypotheses"] if any(
            section["width_interval_m"][0] < 1.5 <= section["width_interval_m"][1]
            for section in p["matched_sections"])]
        self.assertTrue(overlapping)
        for pair in overlapping:
            self.assertFalse(pair["checks"]["complete_width_interval"])
            self.assertFalse(pair["geometry_supported_under_assumptions"])

    def test_missing_main_side_stays_unknown(self):
        self.main_lane(both=False)
        self.side_pair()
        value = self.observe()
        self.assertFalse(any(p["geometry_supported_under_assumptions"] for p in value["pair_hypotheses"]))

    def test_repeated_transverse_bands_remain_ambiguous(self):
        self.main_lane()
        for x in (3., 5.2, 7.4):
            self.line((x, 1.8), (x, 3.2))
        value = self.observe()
        self.assertFalse(any(p["geometry_supported_under_assumptions"] for p in value["pair_hypotheses"]))
        self.assert_no_recognition(value)

    def test_far_single_row_geometry_cannot_claim_high_precision(self):
        self.mask[165, 300:330] = 255
        self.mask[170, 400:430] = 255
        value = self.observe(parameters=replace(ObserverParameters(), max_forward_m=25, max_lateral_m=20))
        for pair in value["pair_hypotheses"]:
            self.assertFalse(pair["geometry_supported_under_assumptions"])
        self.assertTrue(any(p["pixel_quantization_bound_xy_m"][0] > .15 for p in value["observed_points"]))
        self.assert_no_recognition(value)

    def test_observed_runs_do_not_bridge_domain_gaps(self):
        runs = _runs([1, 2, 3, 4, 5, 6], {1, 2, 4, 5})
        self.assertEqual({tuple(run) for run, _ in runs}, {(1, 2), (4, 5)})
        self.assertTrue(all(not closed for _, closed in runs))

    def test_no_history_after_current_mask_disappears(self):
        self.main_lane()
        self.assertTrue(self.observe()["observed_points"])
        self.mask[:] = 0
        value = self.observe()
        self.assertEqual(value["observed_points"], [])
        self.assertEqual(value["pair_hypotheses"], [])

    def test_invalid_camera_and_work_budget_fail_closed(self):
        self.main_lane()
        self.side_pair()
        value = self.observe(parameters=replace(ObserverParameters(), max_pair_cells=1))
        self.assertEqual(value["status"], "unknown")
        self.assertEqual(value["pair_hypotheses"], [])
        self.camera["K"][0] = float("nan")
        self.assertEqual(self.observe()["status"], "unknown")

    def test_total_error_remains_unknown_separate_from_quantization(self):
        self.main_lane()
        value = self.observe()
        self.assertIn("unknown", value["projection_assumptions"]["total_uncertainty"])
        self.assertTrue(value["projection_assumptions"]["quantization_is_not_total_error"])
        self.assert_no_recognition(value)


class ReviewRepairTests(unittest.TestCase):
    def test_normal_rotation_crosses_lower_limit_without_position_error(self):
        value = _normal_separation_interval([1.501, 0.], [1., 0.], [0., 0.], 3.)
        self.assertGreater(1.501, 1.5)  # The old fixed-normal interval passed.
        self.assertLess(value["interval_m"][0], 1.5)
        self.assertAlmostEqual(value["interval_m"][1], 1.501)

    def test_normal_rotation_crosses_upper_limit_without_position_error(self):
        value = _normal_separation_interval([5.99, .8], [1., 0.], [0., 0.], 5.)
        self.assertLess(5.99, 6.)  # The old fixed-normal interval passed.
        self.assertGreater(value["interval_m"][1], 6.)

    def test_cone_internal_extrema_and_position_expansion_are_included(self):
        value = _normal_separation_interval([0., 2.], [1., 0.], [.03, .04], 10.)
        self.assertEqual(value["angular_only_interval_m"][0], 0.)
        self.assertAlmostEqual(value["position_bound_l2_m"], .05)
        self.assertEqual(value["interval_m"][0], 0.)
        self.assertAlmostEqual(value["interval_m"][1], value["angular_only_interval_m"][1]+.05)
        all_directions = _normal_separation_interval([3., 4.], [1., 0.], [0., 0.], 90.)
        self.assertEqual(all_directions["interval_m"], [0., 5.])

    def finish_fixture(self, with_competitor=False, first_fails=False):
        # Pure bookkeeping fixture: current unique main support; no image or
        # junction claim. Footprint equivalence is an explicit fixture input.
        arms = [dict(arm_id=i, samples=[]) for i in range(6)]
        observations = {}
        key = 1
        for side in (-1, 1):
            for start in (1., 8.):
                for index in range(12):
                    observations[key] = dict(ground_xyz=[start+index*.01, side*1.5, 0.],
                        pixel_quantization_bound_xy_m=[.001, .001], uv=[key, 1])
                    arms[5]["samples"].append(dict(pixel_id=key,
                        role="main_direction_support_under_assumptions"))
                    key += 1
        def pair(a, b, same=False, passes=True):
            return dict(arm_ids=[a, b], same_paint_footprint_explanation=same,
                observed_x_interval_m=[3., 5.], side=1,
                competing_interpretation_compatible=not same and passes,
                checks={"actual_separated_paint_footprints":not same,
                    "complete_width_interval":passes})
        pairs = [pair(0, 2, same=True), pair(1, 3, same=True),
            pair(0, 1, passes=not first_fails), pair(3, 2)]
        if with_competitor:
            pairs.append(pair(1, 4))
        groups = _finish_pairs(pairs, arms, observations, ObserverParameters(), np)
        return pairs, groups

    def test_duplicate_unordered_paint_pair_has_one_independent_representative(self):
        pairs, _ = self.finish_fixture()
        self.assertEqual(pairs[2]["pair_equivalence_id"], pairs[3]["pair_equivalence_id"])
        self.assertEqual(pairs[3]["duplicate_of_pair_id"], pairs[2]["pair_id"])
        self.assertTrue(pairs[2]["geometry_supported_under_assumptions"])
        self.assertFalse(pairs[3]["geometry_supported_under_assumptions"])
        self.assertFalse(pairs[3]["independent_support"])
        self.assertEqual(sum(pair["independent_support"] for pair in pairs), 1)

    def test_failing_duplicate_does_not_hide_passing_representative(self):
        pairs, _ = self.finish_fixture(first_fails=True)
        self.assertEqual(pairs[2]["duplicate_of_pair_id"], pairs[3]["pair_id"])
        self.assertFalse(pairs[2]["checks"]["complete_width_interval"])
        self.assertTrue(pairs[3]["geometry_supported_under_assumptions"])
        self.assertEqual(sum(pair["independent_support"] for pair in pairs), 1)

    def test_same_paint_support_group_stays_rejected(self):
        pairs, _ = self.finish_fixture()
        for pair in pairs[:2]:
            self.assertEqual(pair["paint_group_ids"][0], pair["paint_group_ids"][1])
            self.assertFalse(pair["checks"]["not_same_paint_support_group"])
            self.assertFalse(pair["geometry_supported_under_assumptions"])

    def test_equivalence_does_not_remove_third_group_competition(self):
        pairs, _ = self.finish_fixture(with_competitor=True)
        self.assertTrue(pairs[2]["competing_pair_arm_ids"])
        self.assertFalse(pairs[2]["checks"]["no_competing_boundary_pair"])
        self.assertFalse(any(pair["independent_support"] for pair in pairs))


if __name__ == "__main__":
    unittest.main()
