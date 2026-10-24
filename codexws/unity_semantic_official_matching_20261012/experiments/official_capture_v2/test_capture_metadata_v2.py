"""Prepared transport fixtures only; no model, Unity, projection or road proof.

No tests/imports were executed while this isolated candidate was authored.
"""
import copy
import importlib.machinery
import importlib.util
import json
import math
from pathlib import Path
import types
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parent


def load(name, filename):
    loader = importlib.machinery.SourceFileLoader(name, str(ROOT / filename))
    spec = importlib.util.spec_from_loader(name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


worker = load("capture_v2_candidate", "worker.py")
before_worker = load("capture_v1_installed_before", "worker.before.py.txt")
DIGEST = "a" * 64


def fixture():
    camera = {"K": [250., 0., 320., 0., 250., 180., 0., 0., 1.],
              "R": [0., 0., 1., -1., 0., 0., 0., -1., 0.], "t": [0., 0., .65]}
    snapshot = dict(sim_time=7.5, repository_projection=True, camera=camera,
        body_available=True, body_position_unity=[2., -.02, 3.],
        body_quaternion_xyzw=[0., 0., 0., 1.], body_euler_unity_deg=[0., 0., 0.],
        render_root_position_unity=[2., -.02, 3.], render_root_quaternion_xyzw=[0., 0., 0., 1.])
    return dict(schema="aiformula_junction_capture/v2", valid=True, reason="image_pose_stable",
        sequence=12, epoch=3, clock_kind="unity_time_as_double", capture_time=7.5,
        capture_realtime_sec=123.4, rgb_sha256=DIGEST, camera=copy.deepcopy(camera),
        image_pose=dict(source="unity_render_root", coordinate_frame="unity_world",
                        position_unity=[2., -.02, 3.], quaternion_xyzw=[0., 0., 0., 1.],
                        planar=dict(frame="render_root_in_odom_axes", x=3., y=-2., yaw=0.)),
        body_image_coincident=True, before=snapshot, after=copy.deepcopy(snapshot))


def invalidate(value, reason):
    value.update(valid=False, reason=reason, capture_time=None, image_pose=None)
    return value


class CaptureMetadataV2Tests(unittest.TestCase):
    def check(self, value, control=None):
        return worker.validate_capture_metadata(value, 12, DIGEST, control)

    def assert_rejected(self, value, control=None):
        result = self.check(value, control)
        self.assertFalse(result["valid"])
        self.assertTrue(result["reason"].startswith("capture_metadata_rejected:"), result)
        self.assertEqual(result["schema"], "aiformula_junction_capture/v2")
        self.assertNotIn("body_pose", result)
        self.assertNotIn("image_pose", result)
        self.assertNotIn("capture_time", result)

    def test_valid_echo_keeps_full_pose_height_clock_and_input(self):
        value = fixture()
        original = copy.deepcopy(value)
        self.assertIs(self.check(value), value)
        self.assertEqual(value, original)
        self.assertEqual(self.check(json.loads(json.dumps(value))), original)
        self.assertEqual(value["image_pose"]["position_unity"][1], -.02)
        self.assertNotIn("body_pose", value)

    def test_default_reply_and_model_calls_are_identical_to_installed_v1(self):
        replies = []
        for module in (before_worker, worker):
            image = types.SimpleNamespace(shape=(360, 640, 3))
            output = types.SimpleNamespace(combined_overlay="overlay", lane_mask="lane", obstacle_mask="obstacles",
                detections_payload=lambda: dict(traffic_light_state="unknown", detections=[]))
            model = mock.Mock(predict=mock.Mock(return_value=output))
            with mock.patch.object(module, "decode_request", return_value=(12, image, DIGEST)), \
                 mock.patch.object(module, "encode_png", side_effect=lambda value: value), \
                 mock.patch.object(module.time, "perf_counter", side_effect=[10., 10.025]):
                replies.append(module.infer(model, '{"sequence":12,"signal_panel_rgb":false}', {}))
            model.predict.assert_called_once_with(image)
        self.assertEqual(replies[0], replies[1])
        self.assertNotIn("junction_capture", replies[1])

    def test_default_lane_pipeline_receives_same_mask_and_request(self):
        replies, received = [], []
        mask = object()
        control = dict(epoch=3, sim_time=7.5, camera=fixture()["camera"], recover_projected_components=True)
        for module in (before_worker, worker):
            output = types.SimpleNamespace(combined_overlay="overlay", lane_mask=mask, obstacle_mask="obstacles",
                detections_payload=lambda: dict(traffic_light_state="green", detections=[]))
            def process(actual_mask, request):
                self.assertIs(actual_mask, mask)
                received.append(copy.deepcopy(request))
                return dict(valid=True, supported=False, mode="existing_result")
            pipeline = types.SimpleNamespace(process=process)
            with mock.patch.dict("sys.modules", {"upstream_adapter": types.SimpleNamespace(Pipeline=lambda: pipeline)}), \
                 mock.patch.object(module, "decode_request", return_value=(12, types.SimpleNamespace(shape=(360, 640, 3)), DIGEST)), \
                 mock.patch.object(module, "encode_png", side_effect=lambda value: "lane" if value is mask else value), \
                 mock.patch.object(module.time, "perf_counter", side_effect=[10., 10.025]):
                replies.append(module.infer(types.SimpleNamespace(predict=lambda _: output),
                    json.dumps(dict(sequence=12, lane_control=control)), dict(epoch=3, pipeline=pipeline)))
        self.assertEqual(received, [dict(control, sequence=12)] * 2)
        self.assertEqual(replies[0], replies[1])
        self.assertNotIn("junction_capture", replies[1])

    def test_body_translation_mismatch_has_image_pose_not_body_pose(self):
        value = fixture()
        for side in ("before", "after"):
            value[side]["body_position_unity"][2] += .001727
        value["body_image_coincident"] = False
        result = self.check(value)
        self.assertIs(result, value)
        self.assertTrue(result["valid"])
        self.assertFalse(result["body_image_coincident"])
        self.assertEqual(result["image_pose"]["planar"]["x"], 3.)
        self.assertNotEqual(result["before"]["body_position_unity"][2], result["image_pose"]["planar"]["x"])
        self.assertNotIn("body_pose", result)

    def test_body_rotation_mismatch_is_retained_without_substitution(self):
        value = fixture()
        q = [0., math.sin(.003), 0., math.cos(.003)]
        for side in ("before", "after"):
            value[side]["body_quaternion_xyzw"] = q[:]
        value["body_image_coincident"] = False
        self.assertEqual(self.check(value), value)
        self.assertEqual(value["image_pose"]["quaternion_xyzw"], [0., 0., 0., 1.])
        self.assertEqual(value["before"]["body_quaternion_xyzw"], q)

    def test_body_change_during_capture_is_raw_audit_only_if_render_is_stable(self):
        value = fixture()
        value["after"]["body_position_unity"][0] += .001
        value["body_image_coincident"] = False
        self.assertIs(self.check(value), value)
        self.assertNotEqual(value["before"]["body_position_unity"], value["after"]["body_position_unity"])

    def test_missing_body_does_not_invent_body_pose_or_erase_render_pose(self):
        value = fixture()
        for side in ("before", "after"):
            for name in ("body_position_unity", "body_quaternion_xyzw", "body_euler_unity_deg"):
                value[side].pop(name)
            value[side]["body_available"] = False
        value["body_image_coincident"] = False
        self.assertIs(self.check(value), value)
        self.assertTrue(value["valid"])
        self.assertEqual(value["image_pose"]["source"], "unity_render_root")

    def test_wrong_coincidence_claim_is_rejected(self):
        value = fixture()
        value["before"]["body_position_unity"][0] += .001
        self.assert_rejected(value)
        value = fixture()
        value["body_image_coincident"] = False
        self.assert_rejected(value)
        value["body_image_coincident"] = 1
        self.assert_rejected(value)

    def test_moving_render_position_is_invalid_not_tolerated(self):
        value = fixture()
        value["after"]["render_root_position_unity"][0] += 1e-9
        value["body_image_coincident"] = False
        self.assert_rejected(value)
        invalidate(value, "image_pose_time_or_calibration_changed_during_capture")
        self.assertEqual(self.check(value), value)
        self.assertIsNone(value["image_pose"])

    def test_moving_render_rotation_is_invalid(self):
        value = fixture()
        value["after"]["render_root_quaternion_xyzw"] = [0., math.sin(.001), 0., math.cos(.001)]
        value["body_image_coincident"] = False
        invalidate(value, "image_pose_time_or_calibration_changed_during_capture")
        self.assertIs(self.check(value), value)

    def test_time_or_camera_change_rejects_image_pose(self):
        for key in ("time", "camera"):
            with self.subTest(key=key):
                value = fixture()
                if key == "time":
                    value["after"]["sim_time"] += .000001
                else:
                    value["after"]["camera"]["K"][0] += 1
                invalidate(value, "image_pose_time_or_calibration_changed_during_capture")
                self.assertIs(self.check(value), value)

    def test_uncalibrated_projection_stays_invalid(self):
        value = fixture()
        for side in ("before", "after"):
            value[side]["repository_projection"] = False
        invalidate(value, "repository_projection_disabled")
        self.assertIs(self.check(value), value)

    def test_v1_invalid_is_not_automatically_upgraded(self):
        old = fixture()
        old.pop("image_pose")
        old.pop("body_image_coincident")
        for side in ("before", "after"):
            old[side]["body_position_unity"][2] += .001727
        old.update(schema="aiformula_junction_capture/v1", valid=False,
                   reason="body_render_pose_mismatch", capture_time=None, body_pose=None)
        self.assertIs(before_worker.validate_capture_metadata(old, 12, DIGEST), old)
        self.assertFalse(old["valid"])
        self.assert_rejected(old)

    def test_old_schema_and_ambiguous_flat_body_pose_are_rejected(self):
        value = fixture()
        value["schema"] = "aiformula_junction_capture/v1"
        self.assert_rejected(value)
        value = fixture()
        value["body_pose"] = dict(x=3., y=-2., yaw=0.)
        self.assert_rejected(value)

    def test_image_pose_source_frame_and_raw_snapshot_must_match(self):
        for key, replacement in (("source", "unity_rigidbody"), ("coordinate_frame", "camera"),
                                  ("position_unity", [2., -.02, 3.001]),
                                  ("quaternion_xyzw", [0., 0., 0., -1.])):
            with self.subTest(key=key):
                value = fixture()
                value["image_pose"][key] = replacement
                self.assert_rejected(value)

    def test_planar_conversion_uses_render_pose_with_correct_yaw_sign(self):
        value = fixture()
        angle = .4
        q = [0., math.sin(angle / 2), 0., math.cos(angle / 2)]
        for side in ("before", "after"):
            value[side]["render_root_quaternion_xyzw"] = q[:]
            value[side]["body_quaternion_xyzw"] = q[:]
        value["image_pose"]["quaternion_xyzw"] = q[:]
        value["image_pose"]["planar"]["yaw"] = -angle
        self.assertEqual(self.check(value), value)
        value["image_pose"]["planar"]["yaw"] = angle
        self.assert_rejected(value)
        value = fixture()
        value["image_pose"]["planar"]["frame"] = "rigidbody_odometry"
        self.assert_rejected(value)

    def test_full_nonplanar_quaternion_and_height_are_not_flattened(self):
        value = fixture()
        q = [math.sin(.2), 0., 0., math.cos(.2)]
        for side in ("before", "after"):
            value[side]["render_root_quaternion_xyzw"] = q[:]
            value[side]["body_quaternion_xyzw"] = q[:]
        value["image_pose"]["quaternion_xyzw"] = q[:]
        self.assertIs(self.check(value), value)
        self.assertEqual(value["image_pose"]["quaternion_xyzw"], q)
        self.assertEqual(value["image_pose"]["position_unity"][1], -.02)
        self.assertNotIn("ground_plane", value)

    def test_vertical_forward_has_no_usable_planar_heading(self):
        value = fixture()
        q = [math.sqrt(.5), 0., 0., math.sqrt(.5)]
        for side in ("before", "after"):
            value[side]["render_root_quaternion_xyzw"] = q[:]
            value[side]["body_quaternion_xyzw"] = q[:]
        invalidate(value, "image_heading_unavailable")
        self.assertIs(self.check(value), value)

    def test_hash_sequence_epoch_clock_and_lane_request_are_bound(self):
        for key, replacement in (("sequence", 13), ("sequence", True), ("epoch", -1),
                                  ("rgb_sha256", "b" * 64), ("clock_kind", "receipt_wall_time"),
                                  ("capture_time", 7.6)):
            with self.subTest(key=key, replacement=replacement):
                value = fixture()
                value[key] = replacement
                self.assert_rejected(value)
        value = fixture()
        control = dict(epoch=3, sim_time=7.5, camera=copy.deepcopy(value["camera"]))
        self.assertIs(self.check(value, control), value)
        for key, replacement in (("epoch", 4), ("epoch", True), ("camera", {}), ("sim_time", 8.0)):
            with self.subTest(control_key=key):
                bad = dict(control, **{key: replacement})
                self.assert_rejected(value, bad)

    def test_all_numeric_geometry_and_audit_values_must_be_finite(self):
        for replacement in (float("nan"), float("inf"), True, "7.5", None):
            with self.subTest(replacement=replacement):
                value = fixture()
                value["capture_time"] = replacement
                self.assert_rejected(value)
        for field in ("body_position_unity", "body_quaternion_xyzw", "body_euler_unity_deg",
                      "render_root_position_unity", "render_root_quaternion_xyzw"):
            value = fixture()
            value["before"][field][0] = float("nan")
            self.assert_rejected(value)
        value = fixture()
        value["before"]["render_root_quaternion_xyzw"] = [0, 0, 0, 0]
        self.assert_rejected(value)

    def test_invalid_capsule_cannot_keep_image_pose_or_capture_time(self):
        value = fixture()
        value["after"]["sim_time"] += 1
        value.update(valid=False, reason="image_pose_time_or_calibration_changed_during_capture")
        self.assert_rejected(value)

    def test_exact_keys_forbid_manual_map_plane_truth_and_pose_substitution(self):
        for field in ("junction_id", "lamp_truth", "cad_position", "receipt_pose", "ground_plane", "raycast"):
            value = fixture()
            value[field] = "unsupported"
            self.assert_rejected(value)
        value = fixture()
        value["image_pose"]["ground_y"] = 0.0
        self.assert_rejected(value)

    def test_enabled_metadata_does_not_change_original_lane_or_color(self):
        image = types.SimpleNamespace(shape=(360, 640, 3))
        mask = object()
        output = types.SimpleNamespace(combined_overlay="overlay", lane_mask=mask, obstacle_mask="obstacles",
            detections_payload=lambda: dict(traffic_light_state="red", detections=[]))
        control = dict(epoch=3, sim_time=7.5, camera=fixture()["camera"])
        for capsule in (fixture(), None, {"schema": "old-invalid"}):
            received = []
            def process(actual_mask, request):
                self.assertIs(actual_mask, mask)
                received.append(request)
                return dict(valid=True, supported=False, v=0, w=0)
            pipeline = types.SimpleNamespace(process=process)
            model = mock.Mock(predict=mock.Mock(return_value=output))
            with mock.patch.dict("sys.modules", {"upstream_adapter": types.SimpleNamespace(Pipeline=lambda: pipeline)}), \
                 mock.patch.object(worker, "decode_request", return_value=(12, image, DIGEST)), \
                 mock.patch.object(worker, "encode_png", side_effect=lambda value: "lane" if value is mask else value):
                reply = worker.infer(model, json.dumps(dict(sequence=12, lane_control=control,
                                    junction_capture=capsule)), dict(epoch=3, pipeline=pipeline))
            model.predict.assert_called_once_with(image)
            self.assertEqual(received, [dict(control, sequence=12)])
            self.assertEqual(reply["lane"], "lane")
            self.assertEqual(reply["traffic_light_state"], "red")
            self.assertEqual(reply["lane_control"], dict(valid=True, supported=False, v=0, w=0))

    def test_default_csharp_capture_and_calibration_paths_are_textually_unchanged(self):
        candidate = (ROOT / "AIFormulaRoadDetector.cs").read_text(encoding="utf-8-sig")
        before = (ROOT / "AIFormulaRoadDetector.before.cs.txt").read_text(encoding="utf-8-sig")
        self.assertIn("public bool EnableJunctionCaptureMetadata { get; set; }", candidate)
        settings = candidate.split("public sealed class Settings", 1)[1].split("public Settings Configuration", 1)[0]
        self.assertNotIn("EnableJunctionCaptureMetadata", settings)
        def block(text, start, end):
            return text.split(start, 1)[1].split(end, 1)[0]
        self.assertEqual(block(candidate, "JObject CameraCalibration()", "void LateUpdate()"),
                         block(before, "JObject CameraCalibration()", "void LateUpdate()"))
        self.assertEqual(block(candidate, "void LateUpdate()", "JObject CapturePoseSnapshot()"),
                         block(before, "void LateUpdate()", "JObject CapturePoseSnapshot()"))
        self.assertEqual(candidate.split("static Task<string> Observe", 1)[1], before.split("static Task<string> Observe", 1)[1])
        self.assertNotIn('["body_pose"]', candidate)


if __name__ == "__main__":
    unittest.main()
