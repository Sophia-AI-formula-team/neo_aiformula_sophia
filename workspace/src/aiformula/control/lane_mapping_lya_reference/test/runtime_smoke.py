#!/usr/bin/env python3
"""Real ROS 2 / real VectorNav message smoke test, with no vehicle connection.

Run after sourcing a Linux Foxy colcon install containing both lane packages and
vectornav_msgs, for example::

    python3 src/lane_mapping_lya_reference/test/runtime_smoke.py \
        --output /tmp/lane-smoke-result.json

This starts installed executables, not mock Node objects.  The recorder receives
a stationary synthetic two-line camera scene, CameraInfo, static TF and genuine
CommonGroup messages over DDS.  Separate follower probes verify command gating,
both immutable safety profiles and a synthetic ready-bundle handoff into REPEAT.
The bundle is generated independently from analytic circle geometry, not from
the stationary camera scene.  This is NOT a full-lap camera-to-route integration,
closed-loop vehicle simulation, real-track accuracy or hardware-stop test.
The repeat fixture explicitly selects a 0.2 m/s fixed reference; it is not a
validation of the production LYA's 2.0 m/s reference (see neo_upstream_smoke.py).

The only command-output topic is /lane_learning/cmd_vel on a private localhost
ROS domain.  No actuator topic, legacy LYA process or teacher supervisor runs.
Results and child logs are preserved; an existing result is never overwritten.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import sys
import tempfile
import time
import traceback
import uuid


class SmokeFailure(RuntimeError):
    pass


def synthetic_bundle(directory, frame_id):
    """An explicit analytic geometry fixture, not a claimed recorded lap."""
    import numpy as np
    from lane_mapping_lya_reference.route import build_route

    radius = 12.0
    angle = np.linspace(-math.pi / 2.0, 3.0 * math.pi / 2.0, 501)
    trajectory = np.column_stack((radius * np.cos(angle),
                                  radius + radius * np.sin(angle), angle + math.pi / 2.0))
    dense = np.linspace(-math.pi / 2.0, 3.0 * math.pi / 2.0, 2400, endpoint=False)
    lanes = np.vstack([np.column_stack((r * np.cos(dense), radius + r * np.sin(dense)))
                       for r in (radius - 1.2, radius + 1.2)])
    route = build_route(trajectory, lanes, {"reference_speed_mps": 0.2})
    coordinates = {"frame_id": frame_id, "origin_lla": [35.0, 139.0, 10.0],
                   "heading_source": "vectornav_yaw_ned_to_enu", "yaw_offset_rad": 0.0}
    route.update(coordinates)
    camera, extrinsic = repeat_calibration(frame_id[:-4])
    metadata = dict(coordinates, schema_version=1, finished=True,
                    camera_calibration=camera, static_extrinsic=extrinsic,
                    parameters={"mask_threshold": 127, "base_frame": extrinsic["parent_frame"],
                                "max_projection_sensitivity_m_per_px": 0.5,
                                "max_reliable_projection_sensitivity_m_per_px": 0.1},
                    fixture_kind="analytic_circle_not_recorded_camera_lap",
                    speed_policy="fixed_reference", reference_speed_mps=0.2)
    directory.mkdir(exist_ok=False)
    with (directory / "consensus.csv").open("x", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["east_m", "north_m", "frame_votes"])
        writer.writerows((float(x), float(y), 8) for x, y in lanes)
    metadata["data_sha256"] = {"consensus.csv": hashlib.sha256(
        (directory / "consensus.csv").read_bytes()).hexdigest()}
    for filename, body in (("route.json", route), ("metadata.json", metadata)):
        with (directory / filename).open("x", encoding="utf-8") as stream:
            json.dump(body, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
    manifest = dict(coordinates, schema_version=1, ready=True, bundle_id=uuid.uuid4().hex,
                    session_directory=str(directory), route_path="route.json",
                    metadata_path="metadata.json")
    for kind in ("route", "metadata"):
        manifest[kind + "_sha256"] = hashlib.sha256(
            (directory / (kind + ".json")).read_bytes()).hexdigest()
    with (directory / "bundle.json").open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return manifest, route


def repeat_calibration(token):
    """Known, same-frame camera geometry for the independent repeat fixture."""
    from lane_mapping_lya_reference.mapping_core import make_transform
    camera = {"frame_id": token + "_camera", "width": 640, "height": 360,
              "stamp_ns": 0,
              "distortion": [0.] * 5, "distortion_model": "plumb_bob",
              "matrix": [[254., 0., 320.], [0., 254., 180.], [0., 0., 1.]]}
    extrinsic = {"parent_frame": token + "_base", "child_frame": camera["frame_id"],
                 "matrix": make_transform([0., 0., 1.], [-.5, .5, -.5, .5]).tolist()}
    return camera, extrinsic


def repeat_mask_pixels(token):
    """Render the saved analytic lane circle, not unrelated straight lane pixels."""
    import numpy as np
    from lane_mapping_lya_reference.mapping_core import GroundLookup
    camera, extrinsic = repeat_calibration(token)
    lookup = GroundLookup(camera["width"], camera["height"], camera["matrix"],
                          np.asarray(extrinsic["matrix"]), 0.1)
    xy = lookup.points_xy
    radius = np.hypot(xy[:, :, 0], xy[:, :, 1] - 12.)
    foreground = lookup.stable & (np.minimum(abs(radius - 10.8), abs(radius - 13.2)) <= 0.065)
    return (foreground.astype(np.uint8) * 255).tobytes()


class Child:
    """Only this test's explicitly spawned process group may be stopped."""

    def __init__(self, command, log_path):
        self.command = command
        self.log_path = log_path
        self.log = log_path.open("x", encoding="utf-8")
        options = {"start_new_session": True}
        if os.name == "nt":
            options = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP |
                       subprocess.CREATE_NO_WINDOW}
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                        stdin=subprocess.DEVNULL, **options)

    def ensure_running(self):
        code = self.process.poll()
        if code is not None:
            raise SmokeFailure("Child exited with {}: {}; log={}".format(
                code, " ".join(self.command), self.log_path))

    def close(self):
        if self.process.poll() is None:
            if os.name == "nt":
                self.process.terminate()
            else:
                os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                if os.name == "nt":
                    self.process.kill()
                else:
                    os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=3.0)
        self.log.close()


def run_suite(report, directory):
    # Import after setting ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY in main().  Import
    # failures are captured into the same JSON report, rather than faking a pass.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from vectornav_msgs.msg import CommonGroup

    rclpy.init(args=[])
    probe = None
    active = None
    token = "lane_smoke_" + uuid.uuid4().hex[:10]
    prefix = "/" + token
    source_topics = {"vn": prefix + "/vectornav", "teacher": prefix + "/teacher",
                     "camera": prefix + "/camera_info", "mask": prefix + "/mask",
                     "estop": prefix + "/estop"}
    inputs = {"vn": True, "teacher": True, "camera": False, "mask": False}
    last_published = {key: 0.0 for key in inputs}
    period = {"vn": 0.02, "teacher": 0.05, "camera": 0.4, "mask": 0.10}
    follower_states = {}
    recorder_states = []
    commands = []
    cloud_widths = []
    recorder_poses = []
    source_counts = {key: 0 for key in inputs}
    teacher_values = {"speed": 0.2, "yaw_rate": 0.05}

    def check(name, condition, details=None):
        report["checks"].append({"name": name, "passed": bool(condition),
                                  "details": details or {}})
        if not condition:
            raise SmokeFailure(name + ": " + repr(details))

    def decoded(message):
        value = json.loads(message.data)
        if not isinstance(value, dict):
            raise SmokeFailure("State message is not a JSON object")
        return value

    try:
        probe = Node(token + "_probe")
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publishers = {
            "vn": probe.create_publisher(CommonGroup, source_topics["vn"], sensor),
            "teacher": probe.create_publisher(Twist, source_topics["teacher"], sensor),
            "camera": probe.create_publisher(CameraInfo, source_topics["camera"], sensor),
            "mask": probe.create_publisher(Image, source_topics["mask"], sensor),
            "estop": probe.create_publisher(Bool, source_topics["estop"], reliable),
        }
        probe.create_subscription(Twist, "/lane_learning/cmd_vel",
                                  lambda message: commands.append((time.monotonic(),
                                      float(message.linear.x), float(message.angular.z),
                                      float(message.linear.y), float(message.linear.z),
                                      float(message.angular.x), float(message.angular.y))), reliable)
        probe.create_subscription(String, "/lane_learning/recorder_state",
                                  lambda message: recorder_states.append(decoded(message)), durable)
        recorder_name = token + "_recorder"
        probe.create_subscription(PointCloud2, "/" + recorder_name + "/consensus",
                                  lambda message: cloud_widths.append(int(message.width)), durable)
        probe.create_subscription(PoseStamped, "/" + recorder_name + "/pose",
                                  lambda message: recorder_poses.append(message.header.stamp), sensor)
        static_tf = StaticTransformBroadcaster(probe)
        transform = TransformStamped()
        transform.header.frame_id = token + "_base"
        transform.child_frame_id = token + "_camera"
        transform.transform.translation.z = 1.0
        transform.transform.rotation.x = -0.5
        transform.transform.rotation.y = 0.5
        transform.transform.rotation.z = -0.5
        transform.transform.rotation.w = 0.5
        # Exactly zero stamp identifies the immutable static extrinsic expected
        # by the recorder.  The synthetic camera is mounted 1 m over flat ground.
        static_tf.sendTransform(transform)

        def common_group():
            message = CommonGroup()
            message.header.stamp = probe.get_clock().now().to_msg()
            message.header.frame_id = token + "_vectornav"
            message.group_fields = 0x0008 | 0x0040 | 0x0080 | 0x1000
            message.position.x = 35.0
            message.position.y = 139.0
            message.position.z = 10.0
            message.yawpitchroll.x = 90.0  # NED clockwise north -> ENU yaw zero.
            message.velocity.x = 0.0
            message.velocity.y = 0.0
            message.insstatus.mode = 2
            message.insstatus.gps_fix = True
            message.insstatus.time_error = False
            message.insstatus.imu_error = False
            message.insstatus.gps_error = False
            return message

        def camera_info():
            message = CameraInfo()
            message.header.frame_id = token + "_camera"
            message.width, message.height = 64, 48
            message.distortion_model = "plumb_bob"
            message.d = [0.0] * 5
            message.k = [40.0, 0.0, 32.0, 0.0, 40.0, 24.0, 0.0, 0.0, 1.0]
            message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            message.p = [40.0, 0.0, 32.0, 0.0, 0.0, 40.0, 24.0, 0.0,
                         0.0, 0.0, 1.0, 0.0]
            return message

        def mask_image():
            message = Image()
            message.header.stamp = probe.get_clock().now().to_msg()
            message.header.frame_id = token + "_camera"
            if inputs.get("repeat_scene", False):
                message.width, message.height, message.step = 640, 360, 640
                message.encoding = "mono8"
                message.data = repeat_pixels
                return message
            message.width, message.height, message.step = 64, 48, 64
            message.encoding = "mono8"
            pixels = bytearray(64 * 48)
            # Ground lines y=+/-0.8 m, z=0.  Lower rows include low-sensitivity
            # pixels; repeated live frames must therefore produce real consensus.
            for row in range(30, 48):
                for sign in (-1.0, 1.0):
                    column = int(round(32.0 + sign * 0.8 * (row - 24)))
                    for width in (-1, 0, 1):
                        pixels[row * 64 + column + width] = 255
            message.data = bytes(pixels)
            return message

        stop_heartbeat = {"publisher": None, "last": 0.0}

        def publish_inputs():
            now = time.monotonic()
            # Only enabled after the missing/pre-handover STOPPED negative tests.
            # This mimics the real supervisor's ongoing status publication; it
            # does not shorten the production stopped-settle or freshness gates.
            if stop_heartbeat["publisher"] is not None and now - stop_heartbeat["last"] >= 0.2:
                stop_heartbeat["publisher"].publish(String(data=json.dumps({
                    "state": "STOPPED", "stamp_ns": probe.get_clock().now().nanoseconds,
                    "synthetic_test_signal": True})))
                stop_heartbeat["last"] = now
            for key in ("vn", "teacher", "camera", "mask"):
                if not inputs[key] or now - last_published[key] < period[key]:
                    continue
                last_published[key] = now
                if key == "vn":
                    message = common_group()
                elif key == "teacher":
                    message = Twist()
                    message.linear.x = teacher_values["speed"]
                    message.angular.z = teacher_values["yaw_rate"]
                elif key == "camera":
                    message = camera_info()
                else:
                    message = mask_image()
                publishers[key].publish(message)
                source_counts[key] += 1

        def pump(duration):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                if active is not None:
                    active.ensure_running()
                publish_inputs()
                rclpy.spin_once(probe, timeout_sec=0.005)

        def wait_for(predicate, label, timeout=8.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if predicate():
                    return
                pump(0.02)
            raise SmokeFailure("Timed out waiting for " + label)

        def spawn(package, executable, name, parameters):
            command = ["ros2", "run", package, executable,
                       "--ros-args", "-r", "__node:=" + name]
            for key, value in parameters.items():
                if isinstance(value, bool):
                    value = "true" if value else "false"
                command.extend(("-p", key + ":=" + str(value)))
            child = Child(command, directory / (name + ".log"))
            report["children"].append({"package": package, "executable": executable,
                                       "pid": child.process.pid, "log": str(child.log_path),
                                       "command": command})
            return child

        def trigger(name, service):
            client = probe.create_client(Trigger, "/" + name + "/" + service)
            try:
                wait_for(client.service_is_ready, service + " service", timeout=8.0)
                future = client.call_async(Trigger.Request())
                wait_for(future.done, service + " response", timeout=8.0)
                result = future.result()
                if result is None:
                    raise SmokeFailure("Empty service response: " + service)
                return bool(result.success), str(result.message)
            finally:
                probe.destroy_client(client)

        def assert_zero_window(label):
            # Discard in-flight messages from the previous state, then inspect a
            # fresh window rather than merely finding one zero among old commands.
            pump(0.15)
            first = len(commands)
            pump(0.25)
            window = commands[first:]
            check(label, len(window) >= 2 and all(
                all(math.isfinite(value) and abs(value) < 1e-9 for value in row[1:])
                for row in window), {"observed_commands": len(window),
                                    "last_commands": [list(row[1:]) for row in window[-3:]]})

        pump(0.5)
        check("private_command_topic_initially_unowned",
              probe.count_publishers("/lane_learning/cmd_vel") == 0,
              {"topic": "/lane_learning/cmd_vel"})

        inputs.update(camera=True, mask=True, teacher=False)
        active = spawn("lane_mapping_lya_reference", "lap_recorder", recorder_name, {
            "vectornav_topic": source_topics["vn"], "mask_topic": source_topics["mask"],
            "camera_info_topic": source_topics["camera"],
            "lya_command_topic": source_topics["teacher"],
            "base_frame": token + "_base", "map_frame": token + "_map",
            "output_directory": str(directory / "recorder_runs"),
        })
        wait_for(lambda: recorder_states and recorder_states[-1].get("counters", {}).get(
            "masks_fused", 0) >= 8 and cloud_widths and max(cloud_widths) > 0,
                 "real CommonGroup + mask consensus", timeout=20.0)
        current = recorder_states[-1]
        check("recorder_real_messages_fuse_nonempty_consensus",
              current.get("state") == "recording" and len(recorder_poses) > 0,
              {"state": current, "largest_cloud_points": max(cloud_widths),
               "received_pose_messages": len(recorder_poses),
               "scene": "stationary flat ground, two synthetic lane lines; not a complete lap"})
        check("recorder_never_owns_command_output",
              probe.count_publishers("/lane_learning/cmd_vel") == 0)
        success, reason = trigger(recorder_name, "finish_lap")
        check("recorder_refuses_stationary_nonlap", not success, {"response": reason})
        active.close()
        active = None
        inputs.update(camera=False, mask=False, teacher=True, repeat_scene=True)
        repeat_pixels = repeat_mask_pixels(token)
        pump(0.3)
        manifest, synthetic_route = synthetic_bundle(directory / "synthetic_route", token + "_map")
        report["synthetic_bundle"] = {
            "path": str(directory / "synthetic_route" / "bundle.json"),
            "origin_lla": manifest["origin_lla"],
            "first_sample": synthetic_route["route_samples"][0],
            "source": "analytic circle, separate from recorder camera smoke",
            "route_sample_count": len(synthetic_route["route_samples"]),
            "speed_policy": synthetic_route["speed_policy"],
            "reference_speed_mps": synthetic_route["reference_speed_mps"],
            "production_speed_default_tested": False,
        }

        for profile, package, executable, forced, opposite in (
                ("reference", "lane_mapping_lya_reference", "reference_follower", "lya_reference", "fixed_only"),
                ("fixed", "lane_mapping_fixed", "fixed_follower", "fixed_only", "lya_reference")):
            node_name = token + "_" + profile
            inputs.update(vn=True, teacher=True)
            teacher_values.update(speed=0.2, yaw_rate=0.05)
            follower_states[profile] = []
            bundle_topic = prefix + "/" + profile + "/bundle"
            teacher_state_topic = prefix + "/" + profile + "/teacher_state"
            bundle_publisher = probe.create_publisher(String, bundle_topic, durable)
            teacher_state_publisher = probe.create_publisher(String, teacher_state_topic, durable)
            subscription = probe.create_subscription(
                String, "/" + node_name + "/control_state",
                lambda message, mode=profile: follower_states[mode].append(decoded(message)), durable)
            active = spawn(package, executable, node_name, {
                "safety_mode": opposite,  # The installed variant must override this.
                "vectornav_topic": source_topics["vn"],
                "mask_topic": source_topics["mask"],
                "base_frame": token + "_base",
                "teacher_command_topic": source_topics["teacher"],
                "emergency_stop_topic": source_topics["estop"],
                "ready_bundle_topic": bundle_topic,
                "teacher_state_topic": teacher_state_topic,
                # This existing stationary fixture deliberately tests 0.2 m/s.
                # Its bundle records the same fixed-reference policy, rather
                # than replaying a legacy speed profile under new defaults.
                "reference_speed_mps": 0.2,
                "maximum_speed_mps": 0.8,
                "maximum_yaw_rate_rps": 0.4,
                "accepted_teacher_max_speed_mps": 0.0,
                "log_directory": str(directory / (profile + "_logs")),
                # Keep every vehicle-output override false.  Exact default private
                # topic is deliberately required by the production deployment gate.
                "enable_vehicle_output": False,
                "motor_zero_passthrough_verified": False,
                "hardware_stop_verified": False,
            })
            states = follower_states[profile]

            def state_is(value):
                return bool(states) and states[-1].get("state") == value

            wait_for(lambda: state_is("DISARMED"), profile + " initial DISARMED")
            check(profile + "_profile_cannot_be_changed_by_parameter",
                  states[-1].get("safety_mode") == forced,
                  {"requested_parameter": opposite, "reported_mode": states[-1].get("safety_mode")})
            assert_zero_window(profile + "_disarmed_zero_output")
            success, reason = trigger(node_name, "arm")
            check(profile + "_explicit_arm", success, {"response": reason})
            wait_for(lambda: state_is("TEACH") and commands
                     and abs(commands[-1][1] - 0.2) < 1e-6
                     and abs(commands[-1][2] - 0.05) < 1e-6,
                     profile + " positive teacher output")
            check(profile + "_teach_forwards_private_teacher", abs(commands[-1][1] - 0.2) < 1e-6
                  and abs(commands[-1][2] - 0.05) < 1e-6,
                  {"speed_mps": commands[-1][1], "yaw_rate_rps": commands[-1][2]})

            inputs["teacher"] = False
            wait_for(lambda: state_is("HOLD"), profile + " teacher stale HOLD", timeout=3.0)
            assert_zero_window(profile + "_teacher_stale_zero_output")
            inputs["teacher"] = True
            pump(0.4)
            check(profile + "_teacher_recovery_does_not_autoarm", state_is("HOLD"))
            success, reason = trigger(node_name, "arm")
            check(profile + "_rearm_after_teacher_stale", success, {"response": reason})
            wait_for(lambda: state_is("TEACH"), profile + " TEACH recovery")

            inputs["vn"] = False
            wait_for(lambda: state_is("HOLD"), profile + " VectorNav stale HOLD", timeout=3.0)
            assert_zero_window(profile + "_vectornav_stale_zero_output")
            inputs["vn"] = True
            pump(0.4)
            check(profile + "_pose_recovery_does_not_autoarm", state_is("HOLD"))
            success, reason = trigger(node_name, "arm")
            check(profile + "_rearm_after_pose_stale", success, {"response": reason})
            wait_for(lambda: state_is("TEACH"), profile + " TEACH before estop")

            publishers["estop"].publish(Bool(data=True))
            wait_for(lambda: state_is("ESTOP"), profile + " manual estop latch", timeout=3.0)
            assert_zero_window(profile + "_manual_estop_zero_output")
            success, reason = trigger(node_name, "reset_estop")
            check(profile + "_cannot_reset_asserted_estop", not success,
                  {"response": reason})
            publishers["estop"].publish(Bool(data=False))
            pump(0.2)
            check(profile + "_estop_release_remains_latched", state_is("ESTOP"))
            success, reason = trigger(node_name, "reset_estop")
            check(profile + "_reset_released_estop", success, {"response": reason})
            wait_for(lambda: state_is("DISARMED"), profile + " reset DISARMED")
            assert_zero_window(profile + "_reset_does_not_autoarm")
            success, reason = trigger(node_name, "arm")
            check(profile + "_explicit_arm_required_after_reset", success,
                  {"response": reason})
            wait_for(lambda: state_is("TEACH") and commands[-1][1] > 0.1,
                     profile + " post-reset TEACH")
            success, reason = trigger(node_name, "stop")
            check(profile + "_operator_stop", success, {"response": reason})
            wait_for(lambda: state_is("HOLD"), profile + " stopped HOLD")
            assert_zero_window(profile + "_operator_stop_zero_output")

            # A ready notification is data only: it may neither arm nor start.
            bundle_publisher.publish(String(data=json.dumps(manifest)))
            if profile == "fixed":
                # Deliberately send a pre-handoff STOPPED. The follower must not
                # reuse it as proof that a newly requested shutdown completed.
                teacher_state_publisher.publish(String(data=json.dumps({
                    "state": "STOPPED", "stamp_ns": probe.get_clock().now().nanoseconds,
                    "synthetic_test_signal": True})))
            pump(0.12)
            check(profile + "_ready_notification_does_not_arm", state_is("HOLD"))
            success, reason = trigger(node_name, "start_repeat")
            check(profile + "_first_repeat_call_loads_bundle", success, {"response": reason})
            wait_for(lambda: state_is("HOLD") and states[-1].get("repeat_selected") is True,
                     profile + " first repeat call stays HOLD")
            check(profile + "_teacher_selection_after_prepare",
                  states[-1].get("teacher_enabled") is (profile == "reference"))
            assert_zero_window(profile + "_first_repeat_call_zero_output")
            # VN alone no longer qualifies as repeat localization.
            inputs["mask"] = False
            pump(1.1)
            success, reason = trigger(node_name, "start_repeat")
            check(profile + "_cannot_repeat_without_mask", not success,
                  {"response": reason})
            inputs["mask"] = True
            wait_for(lambda: states[-1].get("visual_ready") is True,
                     profile + " trusted mask acquisition")

            if profile == "fixed":
                inputs["teacher"] = False  # Synthetic managed teacher is now absent.
                success, reason = trigger(node_name, "start_repeat")
                check("fixed_rejects_preprepare_stopped_signal", not success and "STOPPED" in reason,
                      {"response": reason, "signal_was_sent_before_prepare": True})
            # Default one-second continuous stationary settle is not shortened.
            pump(1.1)
            if profile == "fixed":
                success, reason = trigger(node_name, "start_repeat")
                check("fixed_rejects_missing_postprepare_stopped", not success and "STOPPED" in reason,
                      {"response": reason})
                stop_heartbeat.update(publisher=teacher_state_publisher, last=0.0)
                pump(0.10)
            success, reason = trigger(node_name, "start_repeat")
            check(profile + "_second_repeat_call_explicitly_starts", success,
                  {"response": reason,
                   "managed_stop_signal_simulated": profile == "fixed"})
            wait_for(lambda: state_is("REPEAT") and commands[-1][1] > 0.05,
                     profile + " real DDS REPEAT output", timeout=5.0)
            check(profile + "_repeat_outputs_bounded_command",
                  0.0 < commands[-1][1] <= 0.8 and abs(commands[-1][2]) <= 0.4,
                  {"speed_mps": commands[-1][1], "yaw_rate_rps": commands[-1][2],
                   "pose_source": "actual CommonGroup in saved ENU origin",
                   "pose_is_stationary": True})
            if profile == "fixed":
                pump(0.45)
                check("fixed_repeat_does_not_need_lya_messages", state_is("REPEAT")
                      and states[-1].get("teacher_enabled") is False)
                inputs["mask"] = False
                wait_for(lambda: state_is("HOLD"), "fixed REPEAT stale mask HOLD", timeout=3.0)
                assert_zero_window("fixed_repeat_stale_mask_zero_output")
                inputs["mask"] = True
                wait_for(lambda: states[-1].get("visual_ready") is True,
                         "fixed trusted mask reacquisition")
                pump(1.1)
                check("fixed_mask_recovery_does_not_autostart", state_is("HOLD"))
                teacher_state_publisher.publish(String(data=json.dumps({
                    "state": "STOPPED", "stamp_ns": probe.get_clock().now().nanoseconds,
                    "synthetic_test_signal": True})))
                pump(0.05)
                success, reason = trigger(node_name, "start_repeat")
                check("fixed_explicit_restart_after_mask_recovery", success, {"response": reason})
                inputs["vn"] = False
                wait_for(lambda: state_is("HOLD"), "fixed REPEAT stale pose HOLD", timeout=3.0)
                assert_zero_window("fixed_repeat_stale_pose_zero_output")
                inputs["vn"] = True
            else:
                # Exercise the safety-reference branch with intentional command
                # disagreement. This is a control-gating test, not vehicle motion.
                teacher_values["yaw_rate"] = -0.4
                wait_for(lambda: state_is("FALLBACK") and commands[-1][2] < -0.3,
                         "reference disagreement FALLBACK", timeout=3.0)
                check("reference_disagreement_uses_live_lya", state_is("FALLBACK"),
                      {"speed_mps": commands[-1][1], "yaw_rate_rps": commands[-1][2]})
                inputs["teacher"] = False
                wait_for(lambda: state_is("HOLD"), "reference FALLBACK teacher stale HOLD", timeout=3.0)
                assert_zero_window("reference_fallback_stale_teacher_zero_output")
            active.close()
            active = None
            stop_heartbeat["publisher"] = None
            inputs["mask"] = False
            wait_for(lambda: probe.count_publishers("/lane_learning/cmd_vel") == 0,
                     profile + " owned command publisher removed", timeout=10.0)
            probe.destroy_subscription(subscription)
            probe.destroy_publisher(bundle_publisher)
            probe.destroy_publisher(teacher_state_publisher)

        report["observations"] = {
            "published_source_messages": source_counts,
            "received_command_messages": len(commands),
            "largest_recorder_consensus_points": max(cloud_widths) if cloud_widths else 0,
            "actual_vectornav_message_type": CommonGroup.__module__ + ".CommonGroup",
            "real_dds": True,
            "uses_test_json_vectornav_adapter": False,
            "ready_bundle_and_repeat_handoff_tested": True,
            "full_lap_camera_to_route_tested": False,
            "closed_loop_repeat_tracking_tested": False,
            "teacher_supervisor_process_tested": False,
            "production_speed_default_tested": False,
            "fixture_reference_speed_mps": 0.2,
            "managed_teacher_stop_signal_simulated": True,
            "hardware_emergency_stop_tested": False,
        }
    finally:
        if active is not None:
            active.close()
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", required=True, type=Path,
                        help="New JSON result filename; sibling unique directory preserves logs")
    parser.add_argument("--domain-id", type=int, default=None,
                        help="Isolated localhost domain (default: random 215..229)")
    args = parser.parse_args(argv)
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error("Refusing to overwrite existing result: " + str(output))
    domain = args.domain_id if args.domain_id is not None else random.SystemRandom().randint(215, 229)
    if not 0 <= domain <= 232:
        parser.error("ROS domain must be between 0 and 232")
    output.parent.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix="lane-runtime-smoke-", dir=str(output.parent)))
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_LOG_DIR"] = str(directory / "ros_logs")
    report = {
        "schema_version": 1, "passed": False,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
        "ros_domain_id": domain, "ros_localhost_only": True,
        "command_topic": "/lane_learning/cmd_vel", "vehicle_topics_used": False,
        "checks": [], "children": [], "log_directory": str(directory),
        "scope": "Real DDS package and synthetic-bundle REPEAT handoff; no complete camera lap, vehicle dynamics or hardware validation",
    }
    started = time.monotonic()
    try:
        run_suite(report, directory)
        report["passed"] = True
    except Exception as error:
        report["error"] = str(error)
        report["traceback"] = traceback.format_exc()
    finally:
        report["elapsed_s"] = time.monotonic() - started
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        with output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        print(json.dumps({"passed": report["passed"], "result": str(output),
                          "checks": len(report["checks"]), "log_directory": str(directory)}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
