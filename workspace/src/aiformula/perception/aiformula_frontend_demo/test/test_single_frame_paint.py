import unittest

import cv2
import numpy as np

from aiformula_frontend_demo.common import decode_mask
from aiformula_frontend_demo.oa_stage import OaStage
from aiformula_frontend_demo.semantic import SemanticStage
from optimization_oa.geometry import GroundCamera


PITCH = np.deg2rad(1.)
CALIBRATION = dict(
    width=640, height=360, roi_top_ratio=.5, origin=[.055, .060, .55],
    projection=[[245.86, 0., 328.66, 0.], [0., 245.86, 189.01, 0.], [0., 0., 1., 0.]],
    rotation=[[0., np.sin(PITCH), np.cos(PITCH)], [-1., 0., 0.],
              [0., -np.cos(PITCH), np.sin(PITCH)]],
)
CAMERA = GroundCamera(**CALIBRATION)
CROSSING = [(2.55, 2.72, -1.9, 1.9)] + [
    (4.2, 5.9, y-.105, y+.105) for y in np.linspace(-1.25, 1.25, 6)
]


def ground_rect(x0, x1, y0, y1):
    uv, _ = CAMERA.ground_to_image([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
    return np.rint(uv).astype(np.int32)


class GroundFrame:
    def __init__(self, strokes, lane_mask=True, boxes=()):
        self.asphalt = np.zeros((CAMERA.height, CAMERA.width), np.uint8)
        self.paint = np.zeros_like(self.asphalt)
        cv2.fillPoly(self.asphalt, [ground_rect(.4, 16., -2.6, 2.6)], 1)
        for stroke in strokes:
            cv2.fillPoly(self.paint, [ground_rect(*stroke)], 1)
        self.bgr = np.full((*self.asphalt.shape, 3), (32, 115, 38), np.uint8)
        self.bgr[self.asphalt != 0] = 72
        self.bgr[self.paint != 0] = 234
        self.boxes = boxes
        self.lane_mask = lane_mask

    def infer(self, image):
        road = self.asphalt & ~self.paint if self.lane_mask else self.asphalt
        marking = self.paint if self.lane_mask else np.zeros_like(self.paint)
        return road, marking, list(self.boxes), np.zeros_like(road)


class SingleFramePaintTest(unittest.TestCase):
    def plan(self, scene):
        scene_observation = SemanticStage(scene, .5, 11.5, 1.8).step(
            scene.bgr, 1, "camera", "base_footprint", CALIBRATION)
        prediction, result = OaStage(.55, .08, 7.5).step(scene_observation)
        return scene_observation, prediction, result.band != 0

    def test_crosswalk_and_transverse_keep_through_path(self):
        observation, prediction, band = self.plan(GroundFrame(CROSSING))
        markings = {mark["kind"]: mark for mark in observation["markings"]}
        self.assertEqual(markings["crosswalk"]["stripes"], 6)
        self.assertEqual(len(markings["transverse"]["points_m"]), 2)
        self.assertEqual(observation["double_white_pairs"], 0)
        self.assertEqual(prediction["status"], "full_band")
        for kind in ("transverse", "crosswalk"):
            self.assertTrue(np.any(band & (decode_mask(observation[f"{kind}_png"]) != 0)))

    def test_near_paint_with_missing_lane_segmentation(self):
        scene = GroundFrame([(.78, 1.38, .23, .39)], lane_mask=False)
        mixed = (cv2.dilate(scene.paint, np.ones((3, 3), np.uint8)) != 0) & (scene.paint == 0)
        scene.bgr[mixed & (scene.asphalt != 0)] = 159
        observation, prediction, band = self.plan(scene)
        self.assertEqual(prediction["status"], "full_band")
        self.assertFalse(np.any(decode_mask(observation["solid_png"])))
        unknown = decode_mask(observation["unknown_png"]) != 0
        self.assertTrue(np.any(band & unknown & mixed))

    def test_dashed_with_two_visible_paint_runs(self):
        observation, prediction, _ = self.plan(
            GroundFrame([(1.1, 3.75, .52, .66), (4.65, 7.7, .52, .66)]))
        self.assertEqual(prediction["status"], "full_band")
        self.assertIn("dashed", {mark["kind"] for mark in observation["markings"]})
        self.assertFalse(np.any(decode_mask(observation["solid_png"])))

    def test_dim_painted_runs_with_missing_model_lane_mask(self):
        scene = GroundFrame([(1.1, 2.95, .52, .66), (4.05, 6.4, .52, .66)], lane_mask=False)
        scene.bgr[scene.paint != 0] = 107
        observation, _, _ = self.plan(scene)
        dashed = decode_mask(observation["dashed_png"]) != 0
        self.assertIn("dashed", {mark["kind"] for mark in observation["markings"]})
        gap = np.zeros_like(scene.paint)
        cv2.fillPoly(gap, [ground_rect(3.25, 3.72, .52, .66)], 1)
        self.assertFalse(np.any(dashed & (gap != 0)))

    def test_double_white_is_hard(self):
        observation, _, band = self.plan(
            GroundFrame([(1.2, 6.75, -.23, -.12), (1.2, 6.75, .12, .23)]))
        self.assertEqual(observation["double_white_pairs"], 1)
        self.assertFalse(np.any(band & (decode_mask(observation["double_white_png"]) != 0)))

    def test_obstacle_has_priority_over_paint(self):
        obstacle = dict(xyxy=[276, 213, 367, 290], label="vehicle", score=.99)
        observation, _, band = self.plan(GroundFrame(CROSSING, boxes=[obstacle]))
        obstacle_mask = decode_mask(observation["obstacle_png"]) != 0
        self.assertFalse(np.any(band & obstacle_mask))
        self.assertFalse(np.any((decode_mask(observation["drivable_png"]) != 0) & obstacle_mask))


if __name__ == "__main__":
    unittest.main()
