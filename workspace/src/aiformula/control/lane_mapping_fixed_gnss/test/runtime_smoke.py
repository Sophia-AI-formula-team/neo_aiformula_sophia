#!/usr/bin/env python3
"""Native Foxy/DDS endpoint-window and safety smoke; synthetic, no vehicle IO.

Each isolated process constructs the real EndpointFollower and a probe, spinning
both in a SingleThreadedExecutor. All sensor inputs and service requests traverse
DDS; direct node access is read-only evidence, never a callback substitute.
Only /lane_learning_gnss/cmd_vel is used, in a private localhost ROS domain.
The stationary scene deliberately cannot produce a valid driven closed route.
This is not a complete lap/repeat test or proof of a physical emergency stop.
The synthetic teacher publishes 0.2 m/s; this is not production-speed validation.
"""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import random
import struct
import subprocess
import sys
import tempfile
import time
import traceback
import uuid


class SmokeFailure(RuntimeError):
    pass


def run_case(name, output):
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(random.SystemRandom().randrange(100, 180))
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import TransformStamped, Twist
    from sensor_msgs.msg import CameraInfo, Image, Imu
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from can_msgs.msg import Frame
    from vectornav_msgs.msg import GpsGroup
    from tf2_ros import StaticTransformBroadcaster
    from lane_mapping_fixed_gnss.node import EndpointFollower

    token = "gnss_smoke_" + uuid.uuid4().hex[:8]
    prefix = "/" + token
    report = dict(case=name, passed=False, checks=[], synthetic=True,
                  actual_ros_messages=True, domain_id=int(os.environ["ROS_DOMAIN_ID"]),
                  command_output_topic="/lane_learning_gnss/cmd_vel", physical_stop_tested=False,
                  production_speed_default_tested=False, fixture_teacher_speed_mps=0.2)
    node = probe = executor = None
    clients = []
    arguments = ["--ros-args"]
    settings = {
        "can_topic": prefix + "/can", "gyro_topic": prefix + "/gyro",
        "gnss_topic": prefix + "/gps", "mask_topic": prefix + "/mask",
        "camera_info_topic": prefix + "/camera", "base_frame": token + "_base",
        "map_frame": "lane_teach_local", "log_directory": str(output.parent / (name + "_logs")),
        # Explicit low-speed regression fixture, not the production LYA profile.
        "reference_speed_mps": 0.2, "maximum_speed_mps": 0.8, "maximum_yaw_rate_rps": 0.4,
        "accepted_teacher_max_speed_mps": 0.0,  # Unconfigured ceiling: private output only.
        "hardware_stop_verified": name != "default_guard",  # Synthetic private bus ONLY.
        "enable_vehicle_output": False, "motor_zero_passthrough_verified": False,
    }
    for key, value in settings.items():
        text = str(value).lower() if isinstance(value, bool) else str(value)
        arguments.extend(("-p", key + ":=" + text))
    rclpy.init(args=arguments)
    try:
        node = EndpointFollower()
        probe = Node(token + "_probe")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(probe)
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publishers = {
            "gyro": probe.create_publisher(Imu, settings["gyro_topic"], sensor),
            "wheel": probe.create_publisher(Frame, settings["can_topic"], sensor),
            "gps": probe.create_publisher(GpsGroup, settings["gnss_topic"], sensor),
            "camera": probe.create_publisher(CameraInfo, settings["camera_info_topic"], sensor),
            "mask": probe.create_publisher(Image, settings["mask_topic"], sensor),
            "teacher": probe.create_publisher(Twist, "/lane_learning/lya_cmd", reliable),
            "estop": probe.create_publisher(Bool, "/lane_learning_gnss/emergency_stop", durable),
        }
        commands, states = [], []
        probe.create_subscription(Twist, "/lane_learning_gnss/cmd_vel",
            lambda message: commands.append((float(message.linear.x), float(message.angular.z))), reliable)
        probe.create_subscription(String, "/lane_learning_gnss/state",
            lambda message: states.append(json.loads(message.data)), durable)
        static = StaticTransformBroadcaster(probe)
        transform = TransformStamped()
        transform.header.frame_id = settings["base_frame"]
        transform.child_frame_id = token + "_camera"
        transform.transform.translation.z = 1.0
        transform.transform.rotation.x, transform.transform.rotation.y = -.5, .5
        transform.transform.rotation.z, transform.transform.rotation.w = -.5, .5
        static.sendTransform(transform)  # Zero stamp: explicitly static mount.
        active = dict(gyro=False, wheel=False, camera=False, teacher=False, mask=False, gps=False)
        last = {key: 0.0 for key in active}
        periods = dict(gyro=.02, wheel=.02, camera=.3, teacher=.05, mask=.1, gps=.03)
        poison_orientation = False

        def check(label, condition, **details):
            report["checks"].append(dict(name=label, passed=bool(condition), details=details))
            if not condition:
                raise SmokeFailure(label + ": " + repr(details))

        def source(message, stamp=None):
            stamp = probe.get_clock().now().nanoseconds if stamp is None else stamp
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1000000000)
            return message

        def gyro(stamp=None):
            message = source(Imu(), stamp)
            message.header.frame_id = settings["base_frame"]
            message.angular_velocity.z = 0.0
            if poison_orientation:
                message.orientation.x = float("nan")
                message.orientation.y = 1000.0
                message.orientation.z = -1000.0
                message.orientation.w = float("nan")
            else:
                message.orientation.w = 1.0
            return message

        def wheel(stamp=None):
            message = source(Frame(), stamp)
            message.header.frame_id = settings["base_frame"]
            message.id, message.dlc, message.data = 1809, 8, [0] * 8
            message.is_rtr = message.is_extended = message.is_error = False
            return message

        def gps(stamp=None, bad=None, jump=False):
            message = source(GpsGroup(), stamp)
            message.header.frame_id = token + "_gps"
            message.group_fields, message.fix = (0x210 if bad == "missing_bits" else 0x230), 3
            message.poslla.x, message.poslla.y, message.poslla.z = (50., -50., 500.) if jump else (35., 139., 10.)
            message.posu.x = message.posu.y = 20.0 if bad == "uncertainty" else .2
            message.posu.z = .4
            return message

        camera_message = CameraInfo()
        camera_message.header.frame_id = token + "_camera"
        camera_message.width, camera_message.height = 640, 360
        camera_message.distortion_model = "plumb_bob"
        camera_message.d = [0.] * 5
        camera_message.k = [254., 0., 320., 0., 254., 180., 0., 0., 1.]
        camera_message.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        camera_message.p = [254., 0., 320., 0., 0., 254., 180., 0., 0., 0., 1., 0.]
        pixels = bytearray(640 * 360)
        for row in range(210, 360):
            for side in (-1., 1.):
                column = int(round(320 + side * 1.2 * (row - 180)))
                for offset in (-1, 0, 1):
                    if 0 <= column + offset < 640:
                        pixels[row * 640 + column + offset] = 255

        def publish():
            steady = time.monotonic()
            now = probe.get_clock().now().nanoseconds
            for key in ("gyro", "wheel", "camera", "teacher", "mask", "gps"):
                if not active[key] or steady - last[key] < periods[key]:
                    continue
                last[key] = steady
                if key == "gyro": message = gyro(now - 10000000)
                elif key == "wheel": message = wheel(now - 5000000)
                elif key == "camera": message = camera_message
                elif key == "teacher":
                    message = Twist()
                    message.linear.x, message.angular.z = .2, 0.
                elif key == "gps": message = gps(jump=True)
                else:
                    message = source(Image(), now - 1000000)
                    message.header.frame_id = token + "_camera"
                    message.width, message.height, message.step = 640, 360, 640
                    message.encoding, message.data = "mono8", bytes(pixels)
                publishers[key].publish(message)

        def pump(duration):
            until = time.monotonic() + duration
            while time.monotonic() < until:
                publish()
                executor.spin_once(timeout_sec=.005)

        def wait(predicate, label, timeout=6.):
            until = time.monotonic() + timeout
            while not predicate():
                if time.monotonic() >= until:
                    raise SmokeFailure("timeout: " + label + "; " + repr(node.safety.snapshot()))
                pump(.01)

        def call(service, pause_motion=False):
            client = probe.create_client(Trigger, "/lane_endpoint_follower/" + service)
            clients.append(client)
            wait(client.service_is_ready, service + " service discovery")
            if pause_motion:
                # Discover before pausing: discovery latency must not consume
                # the fresh stationary proof required by begin_teach. Drain
                # queued raw messages before the origin-reset service runs.
                active.update(gyro=False, wheel=False)
                pump(.04)
            pending = client.call_async(Trigger.Request())
            wait(pending.done, service + " response")
            response = pending.result()
            if response is None:
                raise SmokeFailure("empty service response")
            return response.success, response.message

        def zero_window(label):
            pump(.1)
            first = len(commands)
            pump(.2)
            window = commands[first:]
            check(label, len(window) >= 2 and all(v == 0. and w == 0. for v, w in window), count=len(window))

        def pose_bits():
            motion = node.motion.last_sample
            return struct.pack("!ddd", motion.x, motion.y, motion.yaw)

        wait(lambda: states and commands, "initial DDS state and zero command")
        check("default_disarmed", node.safety.state == "DISARMED")
        check("no_gnss_subscription_before_capture", node.gnss_subscription is None
              and probe.count_subscribers(settings["gnss_topic"]) == 0)
        check("private_command_bus_only", node.command_pub.topic_name == "/lane_learning_gnss/cmd_vel")
        zero_window("disarmed_outputs_only_zero")
        active.update(gyro=True, camera=True, teacher=True)
        wait(lambda: node.last_gyro_ns > 0 and node.camera is not None, "gyro/camera actual DDS")
        active["wheel"] = True
        wait(lambda: node.last_motion is not None, "native Honda wheel + raw gyro pose")
        pump(1.2)
        if name != "no_gnss_build":
            old_reset_count, old_generation, old_pose = node.motion.reset_count, node.safety.generation, pose_bits()
            success, reason = call("capture_start")
            check("explicit_capture_start_after_stopped_settle", success, response=reason)
            wait(lambda: probe.count_subscribers(settings["gnss_topic"]) == 1, "one GNSS-window DDS subscriber")
            check("exactly_one_gnss_subscriber_during_window", node.gnss_subscription is not None)
            stamp = probe.get_clock().now().nanoseconds
            if name == "future_gps": stamp += 100000000
            publishers["gps"].publish(gps(stamp, bad=name))
            wait(lambda: node.gnss_subscription is None, "GNSS subscription destruction after fix")
            wait(lambda: probe.count_subscribers(settings["gnss_topic"]) == 0, "GNSS graph removal")
            if name in ("future_gps", "missing_bits", "uncertainty"):
                check("invalid_fix_never_establishes_start_anchor", node.safety.start_anchor_id is None)
                check("invalid_optional_fix_does_not_arm", node.safety.state in ("HOLD", "DISARMED"))
                zero_window("invalid_gnss_never_actuates")
            else:
                check("valid_0x230_gps_anchor_accepted", node.safety.start_anchor_id is not None)
            check("gnss_fix_never_resets_odometry_or_invalidates_geometry",
                  node.motion.reset_count == old_reset_count and node.safety.generation == old_generation
                  and pose_bits() == old_pose and node.safety.phase == "START_ANCHOR_PENDING")
        else:
            check("teaching_can_skip_gnss_capture_entirely", node.safety.start_anchor_id is None
                  and node.gnss_subscription is None)
        success, reason = call("begin_teach", pause_motion=name == "startup_gap")
        check("explicit_local_origin_boundary_independent_of_gnss", success, response=reason)
        if name == "startup_gap":
            check("origin_reset_waits_for_first_native_motion", node.last_motion is None)
            pump(.35)
            check("missing_first_motion_latches_session_invalid",
                  node.session_motion_invalid and node.safety.state == "HOLD"
                  and node.last_motion is None,
                  reason=node.safety.reason)
            zero_window("post_begin_startup_gap_never_actuates")
            active.update(gyro=True, wheel=True)
            pump(1.2)
            check("restored_raw_stream_cannot_revive_invalid_session",
                  node.session_motion_invalid and node.safety.state == "HOLD")
            success, reason = call("arm_teach")
            check("startup_gap_requires_restart_and_new_lap", not success and "restart" in reason,
                  response=reason)
            success, reason = call("begin_teach")
            check("startup_gap_cannot_reset_same_session", not success, response=reason)
            zero_window("startup_gap_recovery_stays_zero")
            report["passed"] = True
            return report
        pump(1.2)  # Origin changes only at explicit begin_teach, not at a fix.
        success, reason = call("arm_teach")
        if name == "default_guard":
            check("default_hardware_proof_cannot_arm", not success and "physical" in reason, response=reason)
            zero_window("unverified_hardware_never_actuates")
            report["passed"] = True
            return report
        check("explicit_synthetic_private_arm", success, response=reason)
        wait(lambda: commands[-1][0] > .05 and node.safety.state == "TEACH", "bounded teacher output")
        check("bounded_teach_command", 0 < commands[-1][0] <= .8 and abs(commands[-1][1]) <= .4)
        before = pose_bits()
        active.update(gps=True, mask=True)
        poison_orientation = True
        pump(.8)
        check("runtime_gnss_spam_and_imu_orientation_have_bitwise_no_pose_influence",
              before == pose_bits() and node.gnss_subscription is None,
              orientation_contains_nan=True, gnss_coordinates_jump=True)
        check("orientation_not_used_as_heading", node.safety.state == "TEACH")
        wait(lambda: node.worker.engine.counts["accepted"] >= 5, "five actual stationary camera commits")
        check("full_masks_build_runtime_consensus", node.worker.engine.counts["accepted"] >= 5)
        active["gps"] = False
        if name in ("no_gnss_build", "future_gps", "missing_bits", "uncertainty"):
            check("missing_or_failed_gnss_does_not_block_native_mask_mapping",
                  node.safety.start_anchor_id is None and node.safety.state == "TEACH"
                  and node.gnss_subscription is None and node.worker.engine.counts["accepted"] >= 5)
            report["passed"] = True
            return report

        # A raw-motion fault after START invalidates this mapping session; each
        # is therefore a separate terminal case, not an automatic recovery test.
        if name in ("future_wheel", "future_gyro"):
            channel = "wheel" if name == "future_wheel" else "gyro"
            active[channel] = False
            future = probe.get_clock().now().nanoseconds + 200000000
            publishers[channel].publish(wheel(future) if channel == "wheel" else gyro(future))
            wait(lambda: node.safety.state == "HOLD", name + " HOLD", timeout=2.)
            frontier = node.motion.last_wheel_stamp_ns if channel == "wheel" else node.motion.last_gyro_stamp_ns
            check(name + "_not_fused", frontier != future)
            active[channel] = True
            pump(.2)
            check("motion_recovery_does_not_autoarm", node.safety.state == "HOLD")
            success, reason = call("arm_teach")
            check("raw_fault_requires_new_mapping_session", not success, response=reason)
            zero_window("invalid_motion_never_actuates")
            report["passed"] = True
            return report

        publishers["estop"].publish(Bool(data=True))
        wait(lambda: node.safety.state == "ESTOP", "manual DDS estop latch")
        zero_window("manual_estop_zero_output")
        success, _ = call("reset_estop")
        check("asserted_manual_estop_cannot_reset", not success)
        publishers["estop"].publish(Bool(data=False))
        pump(.1)
        check("manual_release_stays_latched", node.safety.state == "ESTOP")
        success, reason = call("reset_estop")
        check("released_estop_resets_disarmed", success and node.safety.state == "DISARMED", response=reason)
        zero_window("estop_reset_never_rearms")
        success, reason = call("arm_teach")
        check("explicit_arm_after_estop_reset", success, response=reason)
        success, reason = call("stop")
        check("explicit_stop_before_end_window", success, response=reason)
        pump(1.1)
        active["mask"] = False
        success, reason = call("finish_lap")
        check("finish_freezes_before_end_gnss_window", success, response=reason)
        wait(lambda: node.gnss_subscription is not None, "asynchronous freeze then end window")
        wait(lambda: probe.count_subscribers(settings["gnss_topic"]) == 1, "end window DDS subscriber")
        publishers["gps"].publish(gps())
        wait(lambda: node.gnss_subscription is None and node.safety.end_anchor_id is not None,
             "end fix accepted and subscriber destroyed")
        check("two_endpoint_windows_only", node.anchors.phase == "ready")
        wait(lambda: node.worker.failure is not None or node.safety.state == "HOLD", "stationary route refusal")
        pump(.3)
        check("stationary_scene_cannot_be_claimed_as_a_driven_route", node.bundle_path is None
              and node.safety.phase != "REPEAT" and node.safety.state == "HOLD")
        zero_window("incomplete_lap_never_starts_repeat")
        report["passed"] = True
        return report
    except Exception as error:
        report["error"] = str(error)
        report["traceback"] = traceback.format_exc()
        return report
    finally:
        if node is not None:
            report["final_state"] = node.safety.snapshot()
            report["logs"] = str(node.journal.directory)
            node.close()
        if executor is not None:
            executor.shutdown()
        if probe is not None:
            probe.destroy_node()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    cases = ("default_guard", "normal", "no_gnss_build", "startup_gap", "future_gps", "missing_bits",
             "uncertainty", "future_wheel", "future_gyro")
    parser.add_argument("--case", choices=cases)
    args = parser.parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise SystemExit("Refusing to overwrite " + str(output))
    if args.case:
        try:
            result = run_case(args.case, output)
        except Exception as error:
            result = dict(case=args.case, passed=False, error=str(error), traceback=traceback.format_exc())
    else:
        directory = Path(tempfile.mkdtemp(prefix="gnss-native-smoke-", dir=str(output.parent)))
        result = dict(passed=True, synthetic=True, actual_ros_messages=True,
                      generated_utc=datetime.now(timezone.utc).isoformat(), cases=[],
                      limitation="Stationary DDS safety/windows only; no full-lap repeat or physical hardware proof")
        for case in cases:
            child_output = directory / (case + ".json")
            log_path = directory / (case + ".log")
            with log_path.open("x", encoding="utf-8") as log:
                try:
                    child = subprocess.run([sys.executable, str(Path(__file__).resolve()),
                        "--case", case, "--output", str(child_output)], stdout=log,
                        stderr=subprocess.STDOUT, timeout=45, check=False)
                    data = json.loads(child_output.read_text(encoding="utf-8")) if child_output.exists() else {
                        "case": case, "passed": False, "error": "child produced no report"}
                    data.update(exit_code=child.returncode, process_log=str(log_path))
                except subprocess.TimeoutExpired:
                    data = dict(case=case, passed=False, error="native case exceeded 45 s", process_log=str(log_path))
            result["cases"].append(data)
            result["passed"] = result["passed"] and data.get("passed") is True and data.get("exit_code") == 0
    with output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps(dict(passed=result["passed"], report=str(output))))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
