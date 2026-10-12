"""Pure standard-library tests. No Unity, models, images, or source mutation."""
import copy
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT / "overlay"), str(ROOT / "before")]
import signal_bbox_range as bbox
import signal_panel_range as legacy


class BboxRangeTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT / "overlay/signal_bbox_range.json").read_text())
        self.config["enabled"] = True
        self.camera = copy.deepcopy(self.config["camera"])
        self.evidence = dict(fresh=True, state="red", panel=dict(
            state="red", confirmed_frames=2, bbox=[390., 160., 400., 170.]))

    def estimate(self):
        return bbox.estimate_bbox_range(self.evidence, self.camera, (360, 640, 3), self.config)

    def test_known_optical_depth_and_front_origin(self):
        self.evidence["panel"]["bbox"][3] = 160 + self.config["reference_height_px"]
        result = self.estimate()
        self.assertTrue(result["valid"])
        self.assertAlmostEqual(result["range_m"], 10)
        self.assertAlmostEqual(result["distance_m"], 9.784)

    def test_inverse_height(self):
        first = self.estimate()
        self.evidence["panel"]["bbox"][3] = 180
        self.assertAlmostEqual(self.estimate()["range_m"], first["range_m"] / 2)

    def test_inclusive_target_and_below_threshold(self):
        # Exact binary-representable threshold, independent of preset rounding.
        self.config.update(reference_depth_m=8, reference_height_px=10,
                           front_offset_m=0, target_height_px=10)
        self.assertTrue(self.estimate()["target_reached"])
        self.evidence["panel"]["bbox"][3] -= 0.01
        self.assertFalse(self.estimate()["target_reached"])

    def test_small_height_and_width_do_not_return_distance(self):
        for box in ([390, 160, 400, 163], [390, 160, 393, 170]):
            with self.subTest(box=box):
                self.evidence["panel"]["bbox"] = box
                result = self.estimate()
                self.assertEqual(result["reason"], "panel_too_small")
                self.assertIsNone(result["distance_m"])
                self.assertIsNone(result["target_reached"])

    def test_minimum_size_is_inclusive(self):
        self.evidence["panel"]["bbox"] = [390, 160, 394, 164]
        self.assertTrue(self.estimate()["valid"])

    def test_no_reuse_after_valid_then_small(self):
        self.assertTrue(self.estimate()["valid"])
        self.evidence["panel"]["bbox"][3] = 163
        self.assertIsNone(self.estimate()["distance_m"])

    def test_color_and_freshness_guards(self):
        original = copy.deepcopy(self.evidence)
        variants = [dict(original, fresh=False), dict(original, state="unknown"),
                    dict(original, panel=None)]
        for evidence in variants:
            self.evidence = evidence
            self.assertFalse(self.estimate()["valid"])

    def test_confirmation_mismatch_and_boolean_rejected(self):
        for value in (0, 1, True, 2.0, "2"):
            self.evidence["panel"]["confirmed_frames"] = value
            self.assertFalse(self.estimate()["valid"])
        self.evidence["panel"].update(confirmed_frames=2, state="green")
        self.assertFalse(self.estimate()["valid"])

    def test_green_keeps_same_distance_rule(self):
        red = self.estimate()
        self.evidence["state"] = self.evidence["panel"]["state"] = "green"
        self.assertEqual(self.estimate(), red)

    def test_bad_or_cropped_boxes(self):
        for box in (None, [0, 150, 10, 160], [630, 150, 640, 160],
                    [390, 160, 400, 360], [390, 170, 400, 160],
                    [True, 160, 400, 170], [390, 160, float("nan"), 170]):
            with self.subTest(box=box):
                self.evidence["panel"]["bbox"] = box
                self.assertFalse(self.estimate()["valid"])

    def test_resolution_or_camera_changes_are_rejected(self):
        for shape in ((720, 1280, 3), (360, 640, 4), (True, 640)):
            self.assertFalse(bbox.estimate_bbox_range(self.evidence, self.camera, shape, self.config)["valid"])
        self.camera["K"][4] *= 2
        self.assertEqual(self.estimate()["reason"], "camera_mismatch")

    def test_pixel_interval_is_not_zeroed(self):
        result = self.estimate()
        self.assertGreater(result["uncertainty_m"], 0)
        self.assertLess(result["front_interval_m"][0], result["distance_m"])
        self.assertGreater(result["front_interval_m"][1], result["distance_m"])
        self.assertIn("excludes", result["uncertainty_definition"])

    def test_invalid_configuration_fails_closed(self):
        original = copy.deepcopy(self.config)
        for key, value in (("reference_depth_m", float("inf")), ("min_height_px", True),
                           ("height_error_px", 0), ("reference_height_px", -1),
                           ("target_height_px", 3), ("enabled", "true")):
            with self.subTest(key=key):
                self.config = dict(original, **{key: value})
                self.assertFalse(self.estimate()["valid"])

    def test_inputs_are_not_mutated(self):
        before = copy.deepcopy((self.evidence, self.camera, self.config))
        self.estimate()
        self.assertEqual((self.evidence, self.camera, self.config), before)

    def test_disabled_dispatch_is_exact_legacy(self):
        fy, cy = self.camera["K"][4], self.camera["K"][5]
        height = fy * 0.286 / 10
        center = cy - fy * (0.95 - 0.6750197659) / 10
        self.evidence["panel"]["bbox"] = [390, center-height/2, 390+height, center+height/2]
        with patch.object(bbox, "_runtime_config", return_value={"enabled": False}):
            actual = bbox.estimate_panel_range(self.evidence, self.camera, (360, 640, 3))
        self.assertTrue(actual["valid"])
        self.assertEqual(actual, legacy.estimate_panel_range(self.evidence, self.camera, (360, 640, 3)))

    def test_enabled_dispatch_is_new_range_not_legacy(self):
        with patch.object(bbox, "_runtime_config", return_value=self.config):
            result = bbox.estimate_panel_range(self.evidence, self.camera, (360, 640, 3))
        self.assertEqual(result, self.estimate())
        self.assertEqual(result["source"], bbox.SOURCE)

    def test_invalid_config_dispatch_never_falls_back(self):
        with patch.object(bbox, "_runtime_config", return_value={"config_error": "bad JSON"}):
            result = bbox.estimate_panel_range(self.evidence, self.camera, (360, 640, 3))
        self.assertFalse(result["valid"])
        self.assertIsNone(result["distance_m"])

    def test_worker_patch_changes_only_range_import(self):
        import install
        raw = (ROOT / "before/worker.py").read_bytes()
        changed = install.worker_after(raw)
        self.assertEqual(changed.replace(b"from signal_bbox_range import estimate_panel_range",
                                         b"from signal_panel_range import estimate_panel_range"), raw)


if __name__ == "__main__":
    unittest.main(verbosity=2)
