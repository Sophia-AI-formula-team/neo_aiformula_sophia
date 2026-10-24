"""Prepared synthetic evidence contracts. Not model/Unity/junction acceptance."""
import copy
from dataclasses import replace
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from observer import ObserverParameters, observe_lane_geometry, _runs


class CurveObserverTests(unittest.TestCase):
    def setUp(self):
        self.mask = np.zeros((720, 1280), np.uint8)
        self.camera = dict(K=[400., 0., 640., 0., 400., 120., 0., 0., 1.],
            R=[0., 0., 1., -1., 0., 0., 0., -1., 0.], t=[0., 0., 1.])

    def uv(self, x, y):
        return round(640-400*y/x), round(120+400/x)

    def line(self, a, b, thickness=1):
        cv2.line(self.mask, self.uv(*a), self.uv(*b), 255, thickness)

    def main(self):
        for sign in (-1, 1):
            self.line((1.5, sign*1.75), (9.5, sign*1.75))

    def observe(self, **kwargs):
        return observe_lane_geometry(self.mask, self.camera, enabled=True, **kwargs)

    def assert_unknown_identity(self, result):
        self.assertFalse(result["junction_recognized"])
        self.assertEqual(result["junction_status"], "unknown")
        for candidate in result["side_boundary_candidates"]:
            self.assertFalse(candidate["junction_recognized"])
            self.assertNotIn("station", candidate)
            self.assertNotIn("official_node_id", candidate)
            self.assertNotIn("world_position", candidate)

    def test_disabled_does_not_access_inputs_or_import_stack(self):
        with patch("builtins.__import__", side_effect=AssertionError("disabled import")):
            result = observe_lane_geometry(object(), object())
        self.assertEqual(result["status"], "disabled")
        self.assert_unknown_identity(result)

    def test_strict_boolean_optin(self):
        for value in (None, 0, 1, "true"):
            result = observe_lane_geometry(None, None, enabled=value)
            self.assertEqual(result["reason"], "enabled_must_be_bool")

    def test_exact_positive_pixel_rays_and_inputs_unchanged(self):
        self.main()
        original_mask, original_camera = self.mask.copy(), copy.deepcopy(self.camera)
        result = self.observe()
        self.assertEqual(result["status"], "observed_geometry")
        for p in result["observed_points"]:
            u, v = p["uv"]
            self.assertGreater(self.mask[v, u], 0)
            self.assertEqual(p["pixel_id"], v*1280+u)
            x = 400/(v-120)
            np.testing.assert_allclose(p["ground_xyz"], [x, -(u-640)*x/400, 0], atol=1e-10)
        np.testing.assert_array_equal(self.mask, original_mask)
        self.assertEqual(self.camera, original_camera)

    def test_all_contour_and_fragment_pixels_are_original_support(self):
        self.line((2, 1), (6, 1), 3)
        self.line((6, 1), (6, 3), 3)
        result = self.observe()
        projected = {p["pixel_id"] for p in result["observed_points"]}
        for curve in result["curves"]:
            for key in curve["original_contour_pixel_ids"]:
                self.assertGreater(self.mask[key//1280, key % 1280], 0)
        for f in result["fragments"]:
            self.assertTrue(set(f["ordered_pixel_ids"]) <= projected)
            self.assertTrue(set(f["inlier_pixel_ids"]) <= set(f["pixel_ids"]))
            self.assertTrue(set(f["support_endpoint_pixel_ids"]) <= set(f["pixel_ids"]))

    def test_one_connected_bend_keeps_multiple_local_directions(self):
        self.line((2, 1), (6, 1))
        self.line((6, 1), (6, 3))
        result = self.observe()
        useful = [f for f in result["fragments"] if f["local_linear_support"]]
        self.assertTrue(any(f["heading_to_forward_deg"] < 20 for f in useful))
        self.assertTrue(any(f["heading_to_forward_deg"] > 60 for f in useful))
        self.assertEqual(len({f["component_id"] for f in useful}), 1)
        self.assert_unknown_identity(result)
        self.assertEqual(result["side_boundary_candidates"], [])

    def test_connected_t_shape_keeps_local_branch_pixels_without_identifying_junction(self):
        self.line((2, 1), (7, 1))
        self.line((4, 1), (4, 3))
        result = self.observe()
        self.assertGreater(len(result["fragments"]), 2)
        self.assert_unknown_identity(result)
        self.assertEqual(result["side_boundary_candidates"], [])

    def test_domain_gap_splits_run_and_never_inserts_identity(self):
        runs = _runs([1, 2, 3, 4, 5, 6], {1, 2, 4, 5})
        self.assertEqual({tuple(run) for run, _ in runs}, {(1, 2), (4, 5)})
        self.assertTrue(all(not closed for _, closed in runs))

    def test_dash_gap_does_not_create_points_or_claim_observed_span(self):
        self.line((2, 1), (3, 1))
        self.line((3.3, 1), (4.5, 1))
        result = self.observe()
        self.assertTrue(result["fragment_gap_relations"])
        for relation in result["fragment_gap_relations"]:
            self.assertFalse(relation["gap_is_observed"])
            self.assertEqual(relation["inserted_pixel_count"], 0)
            self.assertEqual(relation["status"], "unconfirmed_gap_relation")
        original_count = int(np.count_nonzero(self.mask))
        self.assertEqual(result["positive_pixel_count"], original_count)
        self.assert_unknown_identity(result)

    def test_clean_side_pair_has_only_unconfirmed_real_positive_support(self):
        self.main()
        self.line((3, 2.3), (3, 3.8))
        self.line((5, 2.3), (5, 3.8))
        result = self.observe()
        self.assertTrue(result["side_boundary_candidates"], result["reason"])
        available = {p["pixel_id"] for p in result["observed_points"]}
        for candidate in result["side_boundary_candidates"]:
            for group in candidate["boundary_inlier_pixel_ids"]:
                self.assertTrue(set(group) <= available)
            for name in ("main_before_pixel_ids", "main_continuation_pixel_ids"):
                self.assertTrue(all(len(group) >= 12 for group in candidate[name].values()))
                self.assertTrue(all(set(group) <= available for group in candidate[name].values()))
        self.assert_unknown_identity(result)

    def test_without_real_main_continuation_side_pair_is_unknown(self):
        for sign in (-1, 1):
            self.line((1.5, sign*1.75), (5.5, sign*1.75))
        self.line((3, 2.3), (3, 3.8))
        self.line((5, 2.3), (5, 3.8))
        result = self.observe()
        self.assertEqual(result["side_boundary_candidates"], [])
        self.assert_unknown_identity(result)

    def test_single_missing_lane_cannot_authorize_side_pair(self):
        self.line((1.5, 1.75), (9.5, 1.75))
        self.line((3, 2.3), (3, 3.8))
        self.line((5, 2.3), (5, 3.8))
        self.assertEqual(self.observe()["side_boundary_candidates"], [])

    def test_repeated_side_stripes_are_ambiguous(self):
        self.main()
        for x in (3, 4, 5):
            self.line((x, 2.3), (x, 3.8))
        result = self.observe()
        self.assertEqual(result["side_boundary_candidates"], [])
        self.assert_unknown_identity(result)

    def test_crosswalk_across_corridor_does_not_become_side_pair(self):
        self.main()
        for x in (3, 3.5, 4, 4.5, 5):
            self.line((x, -2.5), (x, 2.5))
        result = self.observe()
        self.assertEqual(result["side_boundary_candidates"], [])
        self.assert_unknown_identity(result)

    def test_truncation_and_uncertainty_are_not_physical_endpoints(self):
        self.line((2, 0), (25, 0))
        cv2.line(self.mask, (0, 300), (20, 300), 255, 1)
        result = self.observe()
        flags = {v for f in result["fragments"] for v in f["truncation"]}
        self.assertIn("ground_domain_or_horizon_truncated", flags)
        for f in result["fragments"]:
            self.assertEqual(f["endpoint_kind"], "unknown_support_or_partition_extreme")
        self.assertIn("unknown", result["projection_assumptions"]["total_uncertainty"])

    def test_positive_pixel_budget_refuses_without_subsampling(self):
        self.main()
        result = self.observe(parameters=replace(ObserverParameters(), max_positive_pixels=3))
        self.assertEqual(result["status"], "unknown")
        self.assertIn("positive_pixel_budget_exceeded", result["reason"])
        self.assertEqual(result["observed_points"], [])

    def test_candidate_budget_retains_raw_geometry_but_refuses_selection(self):
        self.main()
        result = self.observe(parameters=replace(ObserverParameters(), max_candidate_fragments=1))
        self.assertGreater(len(result["observed_points"]), 10)
        self.assertIn("candidate_budget_exceeded", result["reason"])
        self.assertEqual(result["side_boundary_candidates"], [])

    def test_invalid_calibration_nonfinite_and_invalid_parameters_fail_closed(self):
        self.main()
        self.camera["t"][2] = float("nan")
        self.assertEqual(self.observe()["status"], "unknown")
        self.camera["t"][2] = 1.
        self.assertEqual(self.observe(parameters=replace(ObserverParameters(), min_inlier_fraction=2))["status"], "unknown")
        self.assertEqual(self.observe(parameters={})["status"], "unknown")

    def test_current_mask_loss_cannot_replay_old_curve_or_candidate(self):
        self.main()
        first = self.observe()
        self.assertTrue(first["observed_points"])
        self.mask[:] = 0
        result = self.observe()
        self.assertEqual(result["observed_points"], [])
        self.assertEqual(result["side_boundary_candidates"], [])
        self.assert_unknown_identity(result)


if __name__ == "__main__":
    unittest.main()
