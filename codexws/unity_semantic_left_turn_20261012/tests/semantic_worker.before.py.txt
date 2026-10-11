"""20 Hz stdio transport for MapAssist and the existing LYA classes.

This is an in-process ROS message/clock adapter, not a ROS or DDS runtime.
All traffic votes use actual monotonic arrival time. No lamp truth or route
reference enters the protocol. Simulation odometry is explicitly supplied.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace as NS

sys.dont_write_bytecode = True
MAX_LINE = 1024 * 1024


class Pose2D:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta


class Twist:
    def __init__(self):
        self.linear, self.angular = NS(x=0.0, y=0.0, z=0.0), NS(x=0.0, y=0.0, z=0.0)


class DataMessage:
    def __init__(self, data=None):
        self.data = data


def header():
    return NS(frame_id="odom", stamp=NS(sec=0, nanosec=0))


def pose():
    return NS(position=NS(x=0.0, y=0.0, z=0.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))


class PoseStamped:
    def __init__(self):
        self.header, self.pose = header(), pose()


class Odometry:
    def __init__(self):
        self.header, self.child_frame_id = header(), "base_link"
        self.pose = NS(pose=pose(), covariance=[0.0] * 36)
        self.twist = NS(twist=Twist(), covariance=[0.0] * 36)


class PathMessage:
    def __init__(self):
        self.header, self.poses = header(), []


class Publisher:
    def __init__(self):
        self.value, self.callback, self.count = None, None, 0

    def publish(self, message):
        self.value = copy.deepcopy(message)
        self.count += 1
        if self.callback is not None:
            self.callback(message)


def import_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def hash_file(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def sequence(value, name):
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return value


class SemanticTransport:
    def __init__(self, source_root, vision_root, signal_approach=False, diagnostic_route_output=None):
        self.source_root, self.vision_root = Path(source_root).resolve(), Path(vision_root).resolve()
        self.signal_approach = bool(signal_approach)
        self.sim_time = 0.0
        source = self.source_root / "semantic_local_planner_ros/semantic_local_planner_ros/map_assist.py"
        self.source_hash = hash_file(source)
        source_files = sorted(p for p in self.source_root.rglob("*")
                              if p.is_file() and p.suffix in (".py", ".yaml", ".osm")
                              and "__pycache__" not in p.parts)
        self.source_files = {p.relative_to(self.source_root).as_posix(): hash_file(p) for p in source_files}
        self.source_tree_hash = hashlib.sha256(json.dumps(self.source_files, sort_keys=True).encode()).hexdigest()
        self._install_ros_transport()
        sys.path.insert(0, str(self.source_root / "semantic_planner_core"))
        sys.path.insert(0, str(self.vision_root))
        self.map_module = import_file("semantic_trial_map_assist", source)
        self.upstream = import_file("semantic_trial_upstream_adapter", self.vision_root / "upstream_adapter.py")
        manifest = json.loads((self.vision_root / "upstream/provenance.json").read_text(encoding="utf-8"))
        if manifest["commit"] != self.upstream.COMMIT:
            raise ValueError("Upstream LYA revision differs from pinned adapter")
        for entry in manifest["files"]:
            if hash_file(self.vision_root / "upstream" / entry["path"]) != entry["sha256"]:
                raise ValueError("Upstream source hash mismatch: " + entry["path"])
        self.vision_commit = manifest["commit"]
        self.last_tick_sequence = -1
        self.last_vision_sequence = -1
        self.epoch = None
        self.total_vision_votes = 0
        self._new_chain()
        self.route_diagnostic = None
        self.route_diagnostic_error = None
        self.diagnostic_route_output = diagnostic_route_output
        if diagnostic_route_output is not None:
            try:
                import os
                from fusion_snapshot_capture import FusionSnapshotCapture, RouteDiagnosticWriter, reference_snapshot
                def context():
                    return dict(worker_pid=os.getpid(), tick_sequence=self.last_tick_sequence,
                        sim_time=self.sim_time, epoch=self.epoch,
                        source_hash=self.source_hash, source_tree_hash=self.source_tree_hash,
                        source_files=dict(self.source_files), vision_commit=self.vision_commit,
                        map_parameters=copy.deepcopy(self.map.p),
                        map_state_before=dict(odom=list(self.map.odom) if self.map.odom is not None else None,
                            odom_stamp=self.map.odom_stamp, odom_sequence=self.map.odom_seq,
                            recorded_samples=len(self.map.samples), recorded_distance_m=self.map.recorded_distance,
                            reference=reference_snapshot(self.map.reference)))
                capture = FusionSnapshotCapture(self.map_module, context)
                writer = RouteDiagnosticWriter(capture, diagnostic_route_output)
                capture.install()
                self.route_diagnostic = writer
            except Exception as error:
                self.route_diagnostic_error = repr(error)
                self._report_route_diagnostic("setup error: " + self.route_diagnostic_error)

    @staticmethod
    def _report_route_diagnostic(message):
        try:
            print("[route_diagnostic] " + message, file=sys.stderr)
        except Exception:
            pass

    def flush_route_diagnostic(self):
        if self.route_diagnostic is not None:
            try:
                self.route_diagnostic.flush()
            except Exception as error:
                # The helper also catches failures; retain an outer transport
                # boundary so diagnostics can never fail the control process.
                self.route_diagnostic_error = repr(error)
                self._report_route_diagnostic("flush error: " + self.route_diagnostic_error)

    def route_diagnostic_status(self):
        if self.route_diagnostic is not None:
            try:
                status = self.route_diagnostic.status()
                if self.route_diagnostic_error is not None:
                    status.update(error=self.route_diagnostic_error, exact_capture=False)
                return status
            except Exception as error:
                self.route_diagnostic_error = repr(error)
        return dict(enabled=False, output=self.diagnostic_route_output,
                    exact_capture=False, error=self.route_diagnostic_error)

    def _install_ros_transport(self):
        transport = self
        overrides = {"enable_traffic_control": True, "enable_velocity_gate": True,
                     "avoidance_initially_idle": True, "use_sim_time": True,
                     "enable_signal_approach": self.signal_approach,
                     # 8 m conservative range bound plus observed pixel margin targets an 8-9 m actual front gap.
                     "target_stop_distance_m": 8.0,
                     # Keep the established LYA cruise request until the stopping
                     # envelope limits it; sustained 0.8 m/s changed plant curvature.
                     "approach_max_speed_mps": 3.0,
                     # Simulator coast trials measured about 1.9 m/s^2. This
                     # conservative configuration is not a hardware calibration.
                     "approach_deceleration_mps2": 1.5}

        class Node:
            def __init__(self, name):
                self.name, self.parameters = name, {"use_sim_time": True}

            def declare_parameter(self, name, default):
                self.parameters[name] = overrides.get(name, default)

            def get_parameter(self, name):
                return NS(value=self.parameters[name])

            def create_publisher(self, *_):
                return Publisher()

            def create_subscription(self, message_type, topic, callback, qos):
                return NS(topic=topic, callback=callback)

            def create_timer(self, interval, callback, clock=None):
                # The caller supplies one control tick per current odom sample.
                return NS(interval=interval, callback=callback, clock=clock)

            def get_logger(self):
                def log(message):
                    print(f"[{self.name}] {message}", file=sys.stderr)
                return NS(info=log, warn=log, warning=log, error=log, debug=lambda *_: None)

            def get_clock(self):
                def now():
                    seconds = transport.sim_time
                    nanos = int(seconds * 1e9)
                    return NS(nanoseconds=nanos, to_msg=lambda: NS(sec=nanos // 10**9, nanosec=nanos % 10**9))
                return NS(now=now)

        names = ("rclpy", "rclpy.node", "rclpy.clock", "rclpy.qos", "geometry_msgs", "geometry_msgs.msg",
                 "nav_msgs", "nav_msgs.msg", "std_msgs", "std_msgs.msg", "ament_index_python", "ament_index_python.packages")
        modules = {name: ModuleType(name) for name in names}
        modules["rclpy.node"].Node = Node
        modules["rclpy.clock"].Clock = lambda **kwargs: NS(**kwargs)
        modules["rclpy.clock"].ClockType = NS(STEADY_TIME=3)
        modules["rclpy.qos"].QoSProfile = lambda **kwargs: NS(**kwargs)
        modules["rclpy.qos"].DurabilityPolicy = NS(TRANSIENT_LOCAL=1)
        for name, cls in (("Pose2D", Pose2D), ("PoseStamped", PoseStamped), ("Twist", Twist)):
            setattr(modules["geometry_msgs.msg"], name, cls)
        modules["nav_msgs.msg"].Odometry, modules["nav_msgs.msg"].Path = Odometry, PathMessage
        modules["std_msgs.msg"].Bool = modules["std_msgs.msg"].String = DataMessage
        modules["ament_index_python.packages"].get_package_share_directory = lambda name: str(self.source_root / name)
        for name, module in modules.items():
            if "." in name:
                parent, child = name.rsplit(".", 1)
                setattr(modules[parent], child, module)
        sys.modules.update(modules)

    def _new_chain(self):
        self.map = self.map_module.MapAssist()
        self.target = self.upstream.load_classes("perception/kalman_filter/kalman_filter/withoutkalman_0312.py",
                                                {"DataProcessingNode"})["DataProcessingNode"]()
        self.lya = self.upstream.load_classes("control/trajectory_follower/trajectory_follower/lya_0221.py",
                                             {"LYAController", "TrajectoryFollower"})["TrajectoryFollower"]()
        # Retain existing same-frame, local TF cancellation of the vision adapter.
        self.lya.current_pose = self.upstream.Message().pose
        for label in "abc":
            self.map.point_pubs[label].callback = getattr(self.target, "observation_callback_" + label)
        self.target.publisher_omega_t.callback = self.lya.omega_t_callback
        self.target.publisher_point.callback = self.lya.trajectory_callback
        self.lya.velocity_publisher.callback = self.map._on_controller_cmd
        self.last_vision_sequence = -1
        self.total_vision_votes = 0
        self.point_corridor = None
        self.map.cmd_pub.publish(Twist())

    def ready(self):
        return dict(type="ready", transport="in_process_ros_shim", control_tick_hz=20,
                    source_hash=self.source_hash, source_tree_hash=self.source_tree_hash,
                    source_files=self.source_files, vision_commit=self.vision_commit,
                    time_basis="real_monotonic_arrival", odom_source="caller_simulated_odometry",
                    lya_tf="existing_local_same_frame_cancellation", signal_approach=self.signal_approach,
                    semantic_revision="signal-distance-approach-v1")

    def _corridor_context(self, vision, image_time, age):
        """Validate an inferred target's scalar prior, separately from image TTL."""
        value = vision.get("corridor")
        if value is None:
            return None
        if not isinstance(value, dict) or value.get("source") not in ("current_right_inferred", "current_left_identity_checked_native"):
            raise ValueError("Unknown corridor source")
        stamp = finite(value["calibration_stamp"], "corridor.calibration_stamp")
        calibration_sequence = sequence(value["calibration_sequence"], "corridor.calibration_sequence")
        capture_age = finite(value["calibration_age_at_capture"], "corridor.calibration_age_at_capture")
        maximum = finite(value["max_calibration_age_sec"], "corridor.max_calibration_age_sec")
        width = finite(value["width_m"], "corridor.width_m")
        if (stamp < 0 or stamp > image_time or calibration_sequence < 1
                or calibration_sequence > vision["sequence"] or capture_age < 0
                or abs(capture_age - (image_time - stamp)) > 1e-6
                or maximum != 1.5 or width <= 0):
            raise ValueError("Invalid corridor calibration")
        received_age = capture_age + max(age, self.sim_time - image_time)
        if received_age > maximum:
            raise ValueError("Expired corridor calibration")
        return dict(source=value["source"], width_m=width,
                    calibration_sequence=calibration_sequence, calibration_stamp=stamp,
                    source_sequence=vision["sequence"], received_age=received_age,
                    received_at=time.monotonic(), max_age_sec=maximum)

    def _corridor_guard(self):
        value = self.point_corridor
        if value is None:
            return None
        elapsed = time.monotonic() - value["received_at"]
        age = max(self.sim_time - value["calibration_stamp"], value["received_age"] + elapsed)
        valid = elapsed >= 0 and 0 <= age <= value["max_age_sec"]
        return dict(source=value["source"], source_sequence=value["source_sequence"],
                    calibration_sequence=value["calibration_sequence"], width_m=value["width_m"],
                    calibration_age_sec=age, max_age_sec=value["max_age_sec"], valid=valid,
                    remaining_sec=max(0.0, value["max_age_sec"] - age) if valid else 0.0)

    def _receive_vision(self, vision):
        if vision is None:
            return "none"
        vision_sequence = sequence(vision["sequence"], "vision.sequence")
        if vision_sequence <= self.last_vision_sequence:
            return "duplicate_or_out_of_order"
        self.last_vision_sequence = vision_sequence
        age = finite(vision["age"], "vision.age")
        image_time = finite(vision["sim_time"], "vision.sim_time")
        if age < 0 or image_time > self.sim_time + 0.001:
            raise ValueError("Vision age/time is invalid")
        if max(age, self.sim_time - image_time) > self.map.p["point_timeout_sec"]:
            return "stale"
        state = str(vision["traffic_light_state"]).strip().lower()
        if state not in ("red", "green", "unknown"):
            raise ValueError("traffic_light_state must be red, green or unknown")
        if self.signal_approach:
            measured = vision.get("signal_range") or {}
            observation = dict(state=state, sequence=vision_sequence, age_sec=max(age, self.sim_time - image_time),
                               distance_m=measured.get("distance_m") if measured.get("valid") is True else None,
                               uncertainty_m=measured.get("uncertainty_m", 0.0) if measured.get("valid") is True else 0.0)
            self.map._on_signal_observation(DataMessage(json.dumps(observation, allow_nan=False)))
        else:
            self.map._on_light_state(DataMessage(state))
        self.total_vision_votes += 1
        if vision.get("supported") is not True:
            return "light_only_unsupported_lane"
        try:
            corridor = self._corridor_context(vision, image_time, age)
        except (ValueError, TypeError, KeyError, OverflowError):
            # Current color/range above remain real observations. This rejects
            # model ABC only; it does not make an old width or point fresh.
            return "light_only_invalid_corridor"
        points = vision["selected_points"]
        if not isinstance(points, list) or len(points) != 3:
            raise ValueError("supported vision requires exactly three selected_points")
        validated = []
        for index, point in enumerate(points):
            if not isinstance(point, list) or len(point) not in (2, 3):
                raise ValueError("Each selected point must have x,y and optional theta")
            values = [finite(value, f"point[{index}]") for value in point]
            validated.append(Pose2D(*values))
        for label, point in zip("abc", validated):
            self.map._point_cb(label)(point)
        self.point_corridor = corridor
        return "accepted"

    def tick(self, request):
        if request.get("type") != "tick":
            raise ValueError("Only type=tick is accepted")
        tick_sequence = sequence(request["sequence"], "sequence")
        if tick_sequence <= self.last_tick_sequence:
            raise ValueError("Tick sequence must increase")
        self.last_tick_sequence = tick_sequence
        self.sim_time = finite(request["sim_time"], "sim_time")
        if self.sim_time < 0:
            raise ValueError("sim_time must be nonnegative")
        vision = request.get("vision")
        if vision is not None and vision.get("epoch") != self.epoch:
            self.epoch = vision.get("epoch")
            self._new_chain()
        odom_values = {name: finite(request["odom"][name], "odom." + name) for name in ("x", "y", "yaw", "v", "w")}
        odom = Odometry()
        nanos = int(self.sim_time * 1e9)
        odom.header.stamp = NS(sec=nanos // 10**9, nanosec=nanos % 10**9)
        odom.pose.pose.position.x, odom.pose.pose.position.y = odom_values["x"], odom_values["y"]
        odom.pose.pose.orientation.z = math.sin(odom_values["yaw"] / 2)
        odom.pose.pose.orientation.w = math.cos(odom_values["yaw"] / 2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = odom_values["v"], odom_values["w"]
        vision_result = self._receive_vision(vision)
        self.map._on_odom(odom)
        point_count_before = self.map.point_pubs["c"].count
        command_count_before = self.map.cmd_pub.count
        self.map._tick()
        self.map._publish_status()
        status = json.loads(self.map.status_pub.value.data)
        cmd = self.map.cmd_pub.value or Twist()
        corridor_guard = self._corridor_guard()
        corridor_expired = corridor_guard is not None and corridor_guard["valid"] is not True
        if corridor_expired:
            status["semantic_velocity_gate_reason"] = status.get("velocity_gate_reason")
            status["velocity_gate_reason"] = "corridor_width_prior_expired"
        raw_cmd = self.lya.velocity_publisher.value
        target = self.target.publisher_point.value
        selected = [self.map.point_pubs[label].value for label in "abc"]
        return dict(type="tick", sequence=tick_sequence, sim_time=self.sim_time, valid=True,
                    v=0.0 if corridor_expired else float(cmd.linear.x),
                    w=0.0 if corridor_expired else float(cmd.angular.z), status=status,
                    semantic_v=float(cmd.linear.x), semantic_w=float(cmd.angular.z),
                    corridor_guard=corridor_guard,
                    selected_points=[[float(p.x), float(p.y)] for p in selected if p is not None],
                    selected_points_new=self.map.point_pubs["c"].count > point_count_before,
                    target=[float(target.x), float(target.y), float(target.theta)] if target else [],
                    controller_v=float(raw_cmd.linear.x) if raw_cmd else 0.0,
                    controller_w=float(raw_cmd.angular.z) if raw_cmd else 0.0,
                    command_new=self.map.cmd_pub.count > command_count_before,
                    vision_result=vision_result, vision_votes_total=self.total_vision_votes,
                    last_vision_sequence=self.last_vision_sequence, epoch=self.epoch,
                    source_hash=self.source_hash, source_tree_hash=self.source_tree_hash,
                    vision_commit=self.vision_commit)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", required=True)
    parser.add_argument("--vision-root", required=True)
    parser.add_argument("--signal-approach", action="store_true")
    parser.add_argument("--diagnostic-route-output", default=None)
    args = parser.parse_args()
    protocol = sys.stdout
    sys.stdout = sys.stderr

    def send(value):
        protocol.write(json.dumps(value, separators=(",", ":"), allow_nan=False) + "\n")
        protocol.flush()

    request = {}
    try:
        transport = SemanticTransport(args.source_root, args.vision_root, args.signal_approach,
                                      args.diagnostic_route_output)
        send(transport.ready())
        while True:
            line = sys.stdin.buffer.readline(MAX_LINE + 1)
            if not line:
                return 0
            if len(line) > MAX_LINE:
                raise ValueError("Input tick exceeds transport limit")
            request = json.loads(line)
            response = transport.tick(request)
            if args.diagnostic_route_output is not None:
                response["diagnostic_route"] = transport.route_diagnostic_status()
            send(response)
            # Keep sidecar I/O outside the fusion call and after normal command
            # transmission. Diagnostic failures never become control failures.
            transport.flush_route_diagnostic()
    except Exception as error:
        send(dict(type="error", sequence=request.get("sequence") if isinstance(request, dict) else None,
                  valid=False, v=0.0, w=0.0,
                  error=f"{type(error).__name__}: {error}"))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
