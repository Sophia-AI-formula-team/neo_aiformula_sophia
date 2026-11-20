import unittest
from dataclasses import replace
from unittest.mock import patch

import numpy as np

from optimization_oa.geometry import GroundCamera
from optimization_oa.inputs import Cloud, align_clouds, boxes_to_mask, decode_raster
from optimization_oa.planner import PlannerConfig, plan, render_overlay


class TrialTest(unittest.TestCase):
    def setUp(self):
        x = np.linspace(.5, 6, 12)
        self.center = np.column_stack((x, np.zeros_like(x)))
        self.left = self.center + [0, 1.4]
        self.right = self.center - [0, 1.4]
        self.config = PlannerConfig(.55)
        projection = [[245, 0, 320, 0], [0, 245, 190, 0], [0, 0, 1, 0]]
        rotation = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        self.camera = GroundCamera(640, 360, projection, (.05, .06, .54), rotation)
        self.mask = np.zeros((360, 640), dtype=np.uint8)

    def solve(self, config=None):
        return plan(self.center, self.left, self.right, self.mask, config or self.config, self.camera)

    def assert_clear_band(self, result):
        occupied = result.band > 0
        self.assertFalse(np.any(occupied & (result.obstacles | ~result.allowed)))
        self.assertFalse(np.any(occupied[:180]))
        rows = np.flatnonzero(np.any(occupied, axis=1))
        self.assertTrue(np.all(np.diff(rows) == 1))
        self.assertGreater(result.metrics["near_band_width_px"], result.metrics["far_band_width_px"])
        self.assertEqual(render_overlay(result).shape, (360, 640, 3))

    def test_obstacles_on_both_sides_leave_room_for_the_full_reference_band(self):
        self.mask[230:248, 220:270] = 255
        self.mask[230:248, 390:440] = 255
        result = self.solve()
        self.assertEqual(result.reason, "keep_reference")
        np.testing.assert_array_equal(result.center_uv, result.reference_uv)
        self.assert_clear_band(result)

    def test_only_selected_centerline_is_converted_after_image_collision_checks(self):
        self.mask[227:242, 309:348] = 255
        convert = self.camera.image_to_ground
        with patch.object(self.camera, "image_to_ground", wraps=convert) as project:
            result = self.solve()
        self.assertEqual(result.reason, "detour")
        project.assert_called_once()
        np.testing.assert_array_equal(project.call_args.args[0], result.center_uv)
        np.testing.assert_allclose(result.path[:, :2], convert(result.center_uv))
        self.assert_clear_band(result)
        length_only = self.solve(replace(self.config, smoothness_weight=0, reference_weight=0))
        self.assertLessEqual(length_only.metrics["scored_length_m"], result.metrics["scored_length_m"])

    def test_clear_center_pixel_is_insufficient_when_the_band_clips_an_obstacle(self):
        free = self.solve()
        u, v = np.rint(free.center_uv[55]).astype(int)
        self.mask[v, u + 10] = 255
        self.assertEqual(self.mask[v, u], 0)
        self.assertEqual(free.band[v, u + 10], 255)
        answer = self.solve()
        self.assertNotEqual(answer.reason, "keep_reference")
        self.assertFalse(np.any(answer.band & self.mask))

    def test_no_space_produces_no_band_or_ground_path(self):
        self.mask[230:245, 120:540] = 255
        result = self.solve()
        self.assertEqual(result.reason, "no_feasible_candidate")
        self.assertEqual(len(result.path), 0)
        self.assertFalse(np.any(result.band))

    def test_roi_ignores_mask_above_the_original_image_half(self):
        self.mask[:180] = 255
        self.assertEqual(self.solve().reason, "keep_reference")

    def test_box_raster_matches_camera_mask_semantics(self):
        boxes = [dict(class_id=91, box_xyxy=[300, 130, 335, 249]),
                 dict(class_id=10, box_xyxy=[100, 90, 120, 200])]
        mask = boxes_to_mask(boxes, 640, 360)
        self.assertTrue(np.all(mask[130:250, 300:336] == 255))
        self.assertEqual(np.count_nonzero(mask), 120 * 36)
        self.assertEqual(decode_raster(bytes([2, 5, 11, 255]), 1, 1, 4, "bgra8")[0, 0].tolist(),
                         [11, 5, 2])

    def test_ego_motion_alignment_precedes_lane_projection(self):
        clouds = [Cloud(1_000_000_000, np.array([[2.0, 0.0]]))]
        odom = {1_000_000_000: (0, 0, 0), 2_000_000_000: (1, 0, 0)}
        aligned = align_clouds(clouds, odom, 1_500_000_000, 1_100_000_000)
        np.testing.assert_allclose(aligned, [[[1.5, 0]]])


if __name__ == "__main__":
    unittest.main()
