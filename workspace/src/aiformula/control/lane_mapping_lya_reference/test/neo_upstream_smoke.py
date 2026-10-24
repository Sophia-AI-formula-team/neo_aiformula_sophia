#!/usr/bin/env python3
"""Isolated Foxy DDS test of the actual neo LYA process and mask publisher.

The real TeacherSupervisor starts the installed lya_0221 executable in two
isolated cases: its current source default and a ROS /** YAML override of 4.0.
Synthetic Odometry/Pose2D inputs exercise its original feedback law, then a real
control heartbeat requests shutdown. STOPPED is never fabricated by this test.

The optional road-detector publication check executes the production
publish_result method with real cv_bridge and DDS publishers, but bypasses all
model loading/inference. Neither test is vehicle or hardware-stop validation.
"""
import argparse
import ast
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import random
import sys
import tempfile
import time
import traceback
from types import SimpleNamespace
import uuid


MOTOR_TOPICS = (
    "/aiformula_control/game_pad/cmd_vel",
    "/aiformula_control/motor_controller/reference_signal",
)
PRIVATE_COMMAND = "/lane_learning/lya_cmd"


class SmokeFailure(RuntimeError):
    pass


def check(report, name, condition, **details):
    report["checks"].append(dict(name=name, passed=bool(condition), details=details))
    if not condition:
        raise SmokeFailure(name + ": " + repr(details))


def publication_method(source):
    """Keep the real method and its real deepcopy import, omit model imports."""
    import cv2
    import numpy as np

    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    detector = next(node for node in tree.body
                    if isinstance(node, ast.ClassDef) and node.name == "RoadDetector")
    method = next(node for node in detector.body
                  if isinstance(node, ast.FunctionDef) and node.name == "publish_result")
    imports = [node for node in tree.body
               if isinstance(node, ast.ImportFrom) and node.module == "copy"]
    module = ast.Module(body=imports + [method], type_ignores=[])
    namespace = {"cv2": cv2, "np": np}
    exec(compile(module, str(source), "exec"), namespace)
    return namespace["publish_result"]


def run_suite(report, directory, road_source, reference_override=None):
    # Never import ROS before main() establishes the isolated domain/locality.
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from geometry_msgs.msg import Pose2D, Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from lane_mapping_lya_reference.supervisor import TeacherSupervisor
    from trajectory_follower.lya_profile import REFERENCE_SPEED_MPS

    ros_arguments = ["--ros-args", "-p", "log_directory:=" + str(directory / "teacher")]
    expected_reference = float(REFERENCE_SPEED_MPS)
    report["reference_configuration"] = dict(source="current_lya_source_default",
        source_default_mps=REFERENCE_SPEED_MPS, effective_reference_mps=expected_reference)
    if reference_override is not None:
        expected_reference = float(reference_override)
        params_file = directory / "reference_override.yaml"
        # This is the only route by which the override enters the supervisor:
        # exercise the real ROS wildcard YAML parser, not a parameter double.
        with params_file.open("x", encoding="utf-8") as stream:
            stream.write("/**:\n  ros__parameters:\n    reference_speed_mps: "
                         + str(expected_reference) + "\n")
        ros_arguments.extend(["--params-file", str(params_file)])
        report["reference_configuration"].update(source="global_ros_params_file",
            effective_reference_mps=expected_reference, params_file=str(params_file),
            params_file_sha256=hashlib.sha256(params_file.read_bytes()).hexdigest())
    rclpy.init(args=ros_arguments)
    executor = SingleThreadedExecutor()
    probe = supervisor = None
    status, commands = [], []
    latest = {"teacher_enabled": True, "target_x": 0.0, "last_inputs": 0.0,
              "last_lane": 0.0, "last_ownership_check": 0.0}
    forbidden_maximum = {topic: 0 for topic in MOTOR_TOPICS}

    try:
        probe = Node("neo_upstream_smoke_" + uuid.uuid4().hex[:10], use_global_arguments=False)
        executor.add_node(probe)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        control = probe.create_publisher(String, "/lane_learning/control_state", durable)
        odometry = probe.create_publisher(Odometry, "/aiformula_sensing/gyro_odometry_publisher/odom", reliable)
        lanes = probe.create_publisher(Pose2D, "/filtered_lane_pose", reliable)
        omega = probe.create_publisher(Pose2D, "/filtered_omega_t", reliable)
        probe.create_subscription(Twist, PRIVATE_COMMAND,
            lambda msg: commands.append(dict(received_s=time.monotonic(),
                speed_mps=float(msg.linear.x), yaw_rate_rps=float(msg.angular.z))), reliable)
        probe.create_subscription(String, "/lane_learning/teacher_state",
            lambda msg: status.append(json.loads(msg.data)), durable)

        def ownership_check():
            counts = {topic: probe.count_publishers(topic) for topic in MOTOR_TOPICS}
            for topic, count in counts.items():
                forbidden_maximum[topic] = max(forbidden_maximum[topic], count)
            if any(counts.values()):
                raise SmokeFailure("unexpected motor-topic publisher in isolated domain: " + repr(counts))

        def pump():
            now = time.monotonic()
            if now - latest["last_inputs"] >= 0.02:
                stamp = probe.get_clock().now()
                control.publish(String(data=json.dumps(dict(stamp_ns=stamp.nanoseconds,
                    safety_mode="fixed_only", teacher_enabled=latest["teacher_enabled"]))))
                pose = Odometry()
                pose.header.stamp = stamp.to_msg()
                pose.header.frame_id, pose.child_frame_id = "odom", "base_link"
                pose.pose.pose.orientation.w = 1.0
                odometry.publish(pose)
                omega.publish(Pose2D(x=0.0, y=0.0, theta=0.0))
                latest["last_inputs"] = now
            if now - latest["last_lane"] >= 0.10:
                lanes.publish(Pose2D(x=latest["target_x"], y=0.0, theta=0.0))
                latest["last_lane"] = now
            executor.spin_once(timeout_sec=0.01)
            if now - latest["last_ownership_check"] >= 0.05:
                ownership_check()
                latest["last_ownership_check"] = now

        def wait_for(predicate, label, timeout=15.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                pump()
                if (supervisor is not None and latest["teacher_enabled"]
                        and supervisor._process.poll() is not None):
                    raise SmokeFailure("LYA exited unexpectedly: " + str(supervisor._process.returncode))
                if predicate():
                    return
            raise SmokeFailure("timeout: " + label)

        # Allow discovery, then refuse to attach test publishers to another ROS
        # application's domain. Localhost isolation alone does not guarantee it.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        nodes = probe.get_node_names_and_namespaces()
        check(report, "private_domain_has_only_probe_before_spawn",
              bool(nodes) and all(name == probe.get_name() and namespace == probe.get_namespace()
                                  for name, namespace in nodes), discovered_nodes=nodes)
        ownership_check()

        supervisor = TeacherSupervisor()
        executor.add_node(supervisor)
        child = supervisor._process
        report["child"] = dict(pid=child.pid, command=child.args, executable="lya_0221",
                               exitcode=None, stopped_from_real_supervisor=False)
        check(report, "actual_installed_lya_0221_started",
              Path(child.args[0]).name == "lya_0221" and child.poll() is None,
              executable=str(child.args[0]))
        check(report, "actual_motor_output_remapped_private",
              "/aiformula_control/game_pad/cmd_vel:=" + PRIVATE_COMMAND in child.args,
              command_topic=PRIVATE_COMMAND)
        check(report, "teacher_receives_effective_reference_at_startup",
              math.isfinite(expected_reference) and expected_reference > 0.0
              and supervisor.get_parameter("reference_speed_mps").value == expected_reference
              and "reference_speed_mps:=" + str(expected_reference) in child.args,
              configuration_source=report["reference_configuration"]["source"],
              source_reference_mps=REFERENCE_SPEED_MPS,
              effective_reference_mps=expected_reference,
              meaning="reference, not guaranteed constant LYA output")
        spec = importlib.util.find_spec("trajectory_follower.lya_0221")
        installed_source = Path(spec.origin)
        report["installed_lya_source"] = dict(path=str(installed_source),
            sha256=hashlib.sha256(installed_source.read_bytes()).hexdigest())
        wait_for(lambda: any(item.get("state") == "RUNNING" for item in status)
                 and probe.count_publishers(PRIVATE_COMMAND) == 1,
                 "actual running heartbeat and private publisher")
        check(report, "real_supervisor_running_heartbeat", True, child_pid=child.pid)

        # Identity odometry + zero target error yields v_t; a 1 m forward
        # target yields v_t + lambda_v * r, not a replayed speed curve. The
        # source-default case is not hardcoded to 2; the YAML case expects 4.15.
        for target_x, expected, label in ((0.0, expected_reference, "zero_error"),
                                         (1.0, expected_reference + 0.15, "forward_error")):
            latest["target_x"] = target_x
            started = time.monotonic()

            def matched():
                return [item for item in commands if item["received_s"] >= started
                        and math.isfinite(item["speed_mps"])
                        and abs(item["speed_mps"] - expected) < 1e-5
                        and math.isfinite(item["yaw_rate_rps"])]

            wait_for(lambda: len(matched()) >= 3, label + " LYA command")
            check(report, "real_lya_" + label + "_feedback_output", True,
                  source_reference_mps=REFERENCE_SPEED_MPS,
                  effective_reference_mps=expected_reference, target_x_m=target_x,
                  expected_speed_mps=expected, observed=matched()[-3:])

        if road_source is not None:
            import numpy as np
            from cv_bridge import CvBridge
            from sensor_msgs.msg import Image
            from std_msgs.msg import Header

            masks, annotated = [], []
            # The standard mask topic name is provided by neo's topic_list.
            mask_topic = "/aiformula_perception/road_detector/mask_image"
            publisher = probe.create_publisher(Image, mask_topic, reliable)
            image_publisher = probe.create_publisher(Image, "/neo_upstream_smoke/annotated", reliable)
            probe.create_subscription(Image, mask_topic, lambda msg: masks.append(msg), reliable)
            probe.create_subscription(Image, "/neo_upstream_smoke/annotated",
                                      lambda msg: annotated.append(msg), reliable)
            bridge = CvBridge()
            method = publication_method(road_source)
            detector = SimpleNamespace(cv_bridge=bridge, lane_mask_image_pub=publisher,
                                       annotated_mask_image_pub=image_publisher)
            mask = np.zeros((48, 80), dtype=np.uint8)
            mask[0, 0] = mask[-1, -1] = 1
            mask[:, 5] = 1
            mask[:, -6] = 1
            background = np.zeros((48, 80, 3), dtype=np.uint8)
            header = Header()
            header.frame_id = "neo_test_left_camera_optical_frame"
            header.stamp = probe.get_clock().now().to_msg()
            wait_for(lambda: probe.count_subscribers(mask_topic) == 1
                     and probe.count_subscribers("/neo_upstream_smoke/annotated") == 1,
                     "image DDS discovery")
            method(detector, background, mask, header)
            wait_for(lambda: bool(masks and annotated), "real mask and annotated DDS delivery")
            received = masks[-1]
            check(report, "production_mask_retains_full_frame_and_pixels",
                  received.width == 80 and received.height == 48
                  and received.encoding == "mono8"
                  and np.array_equal(bridge.imgmsg_to_cv2(received, "mono8"), mask * 255),
                  topic=mask_topic, width=received.width, height=received.height,
                  source_sha256=hashlib.sha256(road_source.read_bytes()).hexdigest())
            check(report, "production_mask_and_annotation_keep_original_header",
                  all(msg.header.frame_id == header.frame_id
                      and msg.header.stamp.sec == header.stamp.sec
                      and msg.header.stamp.nanosec == header.stamp.nanosec
                      for msg in (received, annotated[-1])), frame_id=header.frame_id)

        latest["teacher_enabled"] = False
        wait_for(lambda: any(item.get("state") == "STOPPED" and item.get("pid") == child.pid
                             for item in status)
                 and child.poll() is not None and not supervisor._group_alive(),
                 "actual STOPPED, child exit and owned process group exit", timeout=10.0)
        report["child"].update(exitcode=child.returncode, stopped_from_real_supervisor=True)
        supervisor._handler.flush()
        supervisor_events = [json.loads(line) for line in
            Path(supervisor._handler.baseFilename).read_text(encoding="utf-8").splitlines()
            if line.startswith("{")]
        check(report, "child_stop_requested_by_real_fixed_handoff_callback",
              any(item.get("event") == "stop_requested"
                  and item.get("reason") == "fixed_route_handoff" for item in supervisor_events),
              log_path=supervisor._handler.baseFilename)
        check(report, "real_fixed_handoff_stops_owned_lya_process_group", True,
              child_returncode=child.returncode, expected_stop=supervisor._expected_stop,
              stopped_heartbeats=sum(item.get("state") == "STOPPED" for item in status))
        wait_for(lambda: probe.count_publishers(PRIVATE_COMMAND) == 0,
                 "owned private LYA command publisher disappears", timeout=10.0)
        check(report, "private_command_publisher_removed_after_handoff", True)
        ownership_check()
        check(report, "no_real_motor_topic_publisher_observed", not any(forbidden_maximum.values()),
              maximum_observed_publisher_count=forbidden_maximum,
              limitation="sampled ROS graph in isolated domain; not a motor interlock")
        report["observations"] = dict(real_dds=True, real_installed_lya_process=True,
            real_supervisor_stop=True, fabricated_stopped_messages=False,
            command_messages=len(commands), teacher_state_messages=len(status),
            road_detector_publication_tested=road_source is not None,
            model_inference_tested=False, vehicle_tested=False)
    finally:
        # Only TeacherSupervisor's explicitly owned child/process group is
        # signalled. Never enumerate or kill unrelated ROS processes.
        if supervisor is not None:
            supervisor.close()
            executor.remove_node(supervisor)
            supervisor.destroy_node()
        if probe is not None:
            executor.remove_node(probe)
            probe.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--domain-id", type=int, default=None)
    parser.add_argument("--road-detector-source", type=Path, default=None,
                        help="Opt in to production publish_result + cv_bridge + DDS check; no inference")
    args = parser.parse_args(argv)
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error("Refusing to overwrite result: " + str(output))
    domain = args.domain_id if args.domain_id is not None else random.SystemRandom().randint(215, 229)
    if not 215 <= domain <= 229:
        parser.error("Use an isolated test domain in 215..229; never the vehicle domain")
    road_source = args.road_detector_source.expanduser().resolve() if args.road_detector_source else None
    if road_source is not None and not road_source.is_file():
        parser.error("Road detector source does not exist: " + str(road_source))
    output.parent.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix="neo-upstream-smoke-", dir=str(output.parent)))
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    report = dict(schema_version=2, passed=False, checks=[], cases=[],
        started_utc=datetime.now(timezone.utc).isoformat(),
        ros_distro=os.environ.get("ROS_DISTRO", "unknown"), ros_domain_id=domain,
        ros_localhost_only=True, log_directory=str(directory),
        scope="real neo upstream LYA feedback and owned-process handoff; optional mask publication",
        limits=["Synthetic upstream inputs, no perception inference or sensor drivers",
                "The 4.0 YAML case tests private supervisor propagation only; not GNSS motion-gate or driving approval",
                "No camera-to-map lap, mapping quality or closed-loop vehicle tracking",
                "No physical stop, real remote-control arbitration or motor-zero validation",
                "ROS graph absence is sampled diagnostic evidence, not hardware isolation proof"])
    started = time.monotonic()
    try:
        # Use separate DDS domains as well as a complete shutdown/init cycle;
        # stale discovery from the first case must not satisfy the second one.
        cases = (("source_default", domain, None),
                 ("global_yaml_override", 215 + (domain - 215 + 1) % 15, 4.0))
        for name, case_domain, reference_override in cases:
            case_directory = directory / name
            case_directory.mkdir()
            os.environ["ROS_DOMAIN_ID"] = str(case_domain)
            os.environ["ROS_LOG_DIR"] = str(case_directory / "ros_logs")
            case = dict(name=name, passed=False, checks=[], ros_domain_id=case_domain,
                        ros_localhost_only=True, log_directory=str(case_directory))
            report["cases"].append(case)
            case_started = time.monotonic()
            try:
                run_suite(case, case_directory, road_source if reference_override is None else None,
                          reference_override=reference_override)
                case["passed"] = True
            except Exception as error:
                case["error"], case["traceback"] = str(error), traceback.format_exc()
                raise
            finally:
                case["elapsed_s"] = time.monotonic() - case_started
                report["checks"].extend(dict(item, case=name) for item in case["checks"])
                with (case_directory / "result.json").open("x", encoding="utf-8") as stream:
                    json.dump(case, stream, indent=2, sort_keys=True, allow_nan=False)
                    stream.write("\n")
            check(report, "isolated_case_completed_" + name, case["passed"],
                  ros_domain_id=case_domain, log_directory=str(case_directory))
        report["passed"] = True
    except Exception as error:
        report["error"], report["traceback"] = str(error), traceback.format_exc()
    finally:
        report["elapsed_s"] = time.monotonic() - started
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        with output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        print(json.dumps(dict(passed=report["passed"], checks=len(report["checks"]),
                              result=str(output), log_directory=str(directory))))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
