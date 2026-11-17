"""1115 joint-map Unity transport: native ABC -> pinned LYA -> native gate.

This is an in-process ROS message/clock shim, not ROS/DDS validation. It loads
the supplied semantic source without modifying it. Traffic uses real detector
colors; the separate bbox-distance protocol and legacy stop_wait gate are not
accepted. No ideal route, signal truth or alignment calibration is an input.
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
import uuid

sys.dont_write_bytecode = True
MAX_LINE = 1024 * 1024
EXPERIMENT_MODE = "semantic_joint_1115"


class Pose2D:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta


class Twist:
    def __init__(self):
        self.linear = NS(x=0.0, y=0.0, z=0.0)
        self.angular = NS(x=0.0, y=0.0, z=0.0)


class DataMessage:
    def __init__(self, data=None):
        self.data = data


def header():
    return NS(frame_id="odom", stamp=NS(sec=0, nanosec=0))


def pose():
    return NS(position=NS(x=0.0, y=0.0, z=0.0),
              orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))


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
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(name + " must be a finite number")
    return float(value)


def sequence(value, name):
    if type(value) is not int or value < 0:
        raise ValueError(name + " must be a nonnegative integer")
    return value


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                    encoding="utf-8")


def failed_request_snapshot(request):
    """Keep reviewed transport fields, never arbitrary caller metadata."""
    if not isinstance(request, dict):
        return {"input_type": type(request).__name__}
    result = {key: request[key] for key in (
        "type", "experiment_mode", "sequence", "sim_time", "lane_epoch",
        "avoidance_active", "branch_active") if key in request}
    if isinstance(request.get("odom"), dict):
        result["odom"] = {key: request["odom"][key] for key in ("x", "y", "yaw", "v", "w")
                          if key in request["odom"]}
    value = request.get("vision")
    if isinstance(value, dict):
        result["vision"] = {key: value[key] for key in (
            "sequence", "epoch", "sim_time", "age", "supported", "selected_points",
            "traffic_light_state", "vision_commit") if key in value}
        for field, keys in (("controller", ("v", "w", "target")),
                            ("corridor", ("source", "calibration_stamp", "calibration_sequence",
                                          "calibration_age_at_capture", "max_calibration_age_sec", "width_m"))):
            if isinstance(value.get(field), dict):
                result["vision"][field] = {key: value[field][key] for key in keys if key in value[field]}
        result["rejected_distance_fields_present"] = [key for key in ("signal_range", "distance_m") if key in value]
    elif "vision" in request:
        result["vision"] = None
    return result


class SemanticTransport:
    def __init__(self, source_root, vision_root, output_root=None):
        import yaml

        self.source_root = Path(source_root).resolve()
        self.vision_root = Path(vision_root).resolve()
        self.output_root = Path(output_root).resolve() if output_root else None
        self.sim_time, self.previous_sim_time = 0.0, None
        self.worker_boot_id = str(uuid.uuid4())
        self.started_wall, self.started_monotonic = time.time(), time.monotonic()
        self.last_tick_sequence = self.last_vision_sequence = -1
        self.epoch, self.last_point_source, self.point_corridor = None, None, None
        self.total_vision_votes, self.lya_command_count = 0, 0
        self.last_lya_source, self.first_odom_reference = None, []
        self.last_result, self.closed, self.evidence = None, False, None
        source = self.source_root / "semantic_local_planner_ros/semantic_local_planner_ros/map_assist.py"
        config = self.source_root / "semantic_local_planner_ros/config/map_assist.yaml"
        self.source_hash, self.config_hash = hash_file(source), hash_file(config)
        source_files = sorted(p for p in self.source_root.rglob("*")
                              if p.is_file() and p.suffix in (".py", ".yaml", ".osm")
                              and "__pycache__" not in p.parts)
        self.source_files = {p.relative_to(self.source_root).as_posix(): hash_file(p) for p in source_files}
        self.source_tree_hash = hashlib.sha256(json.dumps(self.source_files, sort_keys=True).encode()).hexdigest()
        document = yaml.safe_load(config.read_text(encoding="utf-8"))
        self.overrides = dict(document["map_assist"]["ros__parameters"])
        self.overrides.update(enable_traffic_control=True, enable_velocity_gate=True,
                              avoidance_initially_idle=True, use_sim_time=True, detour_lookahead_m=2.5, detour_entry_radius_m=1.8)
        manifest = json.loads((self.vision_root / "upstream/provenance.json").read_text(encoding="utf-8"))
        self.vision_commit = manifest["commit"]
        for entry in manifest["files"]:
            if hash_file(self.vision_root / "upstream" / entry["path"]) != entry["sha256"]:
                raise ValueError("Pinned LYA/LLP source mismatch: " + entry["path"])
        self.vision_adapter_hash = hash_file(self.vision_root / "upstream_adapter.py")
        self._install_ros_transport()
        sys.path.insert(0, str(self.source_root / "semantic_planner_core"))
        sys.path.insert(0, str(self.source_root / "semantic_local_planner_ros"))
        sys.path.insert(0, str(self.vision_root))
        self.map_module = import_file("semantic_joint_trial_map_assist", source)
        self.upstream = import_file("semantic_joint_trial_upstream_adapter", self.vision_root / "upstream_adapter.py")
        if self.upstream.COMMIT != self.vision_commit:
            raise ValueError("Pinned LYA adapter commit differs from provenance manifest")
        self.map = self.map_module.MapAssist()
        try:
            self.target = self.upstream.load_classes(
                "perception/kalman_filter/kalman_filter/withoutkalman_0312.py",
                {"DataProcessingNode"})["DataProcessingNode"]()
            self.lya = self.upstream.load_classes(
                "control/trajectory_follower/trajectory_follower/lya_0221.py",
                {"LYAController", "TrajectoryFollower"})["TrajectoryFollower"]()
            # Same-frame local TF cancellation is inherited from the existing
            # Unity adapter; map odometry never becomes an ideal LYA target.
            self.lya.current_pose = self.upstream.Message().pose
            for label in "abc":
                self.map.point_pubs[label].callback = getattr(self.target, "observation_callback_" + label)
            self.target.publisher_omega_t.callback = self.lya.omega_t_callback
            self.target.publisher_point.callback = self.lya.trajectory_callback
            self.lya.velocity_publisher.callback = self._on_lya_command
            self.map.cmd_pub.publish(Twist())
            if self.output_root is not None:
                self.output_root.mkdir(parents=True, exist_ok=True)
                if (self.output_root / "ready.json").exists():
                    raise ValueError("Output root already contains a run; use a unique directory")
                write_json(self.output_root / "ready.json", self.ready())
                self.evidence = (self.output_root / "semantic_joint_ticks.jsonl").open("x", encoding="utf-8")
        except Exception:
            self.map.close_fusion_worker()
            raise

    def _install_ros_transport(self):
        transport = self

        class Node:
            def __init__(self, name):
                self.name, self.parameters = name, {"use_sim_time": True}

            def declare_parameter(self, name, default):
                self.parameters[name] = transport.overrides.get(name, default)

            def get_parameter(self, name):
                return NS(value=self.parameters[name])

            def create_publisher(self, *_):
                return Publisher()

            def create_subscription(self, message_type, topic, callback, qos):
                return NS(topic=topic, callback=callback)

            def create_timer(self, interval, callback, clock=None):
                return NS(interval=interval, callback=callback, clock=clock)

            def get_logger(self):
                def log(message):
                    print("[" + self.name + "] " + str(message), file=sys.stderr)
                return NS(info=log, warn=log, warning=log, error=log, debug=lambda *_: None)

            def get_clock(self):
                def now():
                    nanos = int(transport.sim_time * 1e9)
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

    def ready(self):
        return dict(type="ready", experiment_mode=EXPERIMENT_MODE, worker_boot_id=self.worker_boot_id,
                    wall=self.started_wall, wall_time_basis="unix_seconds",
                    transport="in_process_ros_shim", control_tick_hz=self.map.p["publish_rate_hz"],
                    source_hash=self.source_hash, source_tree_hash=self.source_tree_hash,
                    source_files=self.source_files, config_hash=self.config_hash,
                    parameters=self.map.p, vision_commit=self.vision_commit,
                    vision_adapter_hash=self.vision_adapter_hash,
                    command_source="MapAssist_ABC_withoutkalman_0312_original_LYA_MapAssist_cmd_pub",
                    time_basis="real_monotonic_arrival", ros_time_basis="unchanged_Unity_simulation_time",
                    odom_source="caller_simulated_odometry", lya_tf="existing_local_same_frame_cancellation",
                    semantic_revision="1118_package_0.4.3", traffic_policy="native_joint_red_detour",
                    signal_approach=False, bbox_distance=False, persistence="not_implemented_in_supplied_package")

    def _on_lya_command(self, message):
        self.lya_command_count += 1
        now = time.monotonic()
        self.last_lya_source = dict(kind="detour" if self.map.traffic_mode == "detour" else "vision",
                                    received_at=now, tick_sequence=self.last_tick_sequence,
                                    vision=copy.deepcopy(self.last_point_source))
        self.map._on_controller_cmd(message)

    def _corridor_context(self, vision, image_time, age):
        value = vision.get("corridor")
        if value is None:
            return None
        if not isinstance(value, dict) or value.get("source") not in (
                "current_right_inferred", "current_left_identity_checked_native"):
            raise ValueError("Unknown corridor source")
        stamp = finite(value["calibration_stamp"], "corridor.calibration_stamp")
        calibration_sequence = sequence(value["calibration_sequence"], "corridor.calibration_sequence")
        capture_age = finite(value["calibration_age_at_capture"], "corridor.calibration_age_at_capture")
        maximum = finite(value["max_calibration_age_sec"], "corridor.max_calibration_age_sec")
        width = finite(value["width_m"], "corridor.width_m")
        if (stamp < 0 or stamp > image_time or calibration_sequence < 1
                or calibration_sequence > vision["sequence"] or capture_age < 0
                or abs(capture_age - (image_time - stamp)) > 1e-6 or maximum != 1.5 or width <= 0):
            raise ValueError("Invalid corridor calibration")
        received_age = capture_age + max(age, self.sim_time - image_time)
        if received_age > maximum:
            raise ValueError("Expired corridor calibration")
        return dict(source=value["source"], width_m=width, calibration_sequence=calibration_sequence,
                    calibration_stamp=stamp, source_sequence=vision["sequence"], received_age=received_age,
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
                    applicable=self.map.traffic_mode != "detour",
                    remaining_sec=max(0.0, value["max_age_sec"] - age) if valid else 0.0)

    def _receive_vision(self, vision):
        if vision is None:
            return "none"
        if not isinstance(vision, dict):
            raise ValueError("vision must be an object or null")
        if "signal_range" in vision or "distance_m" in vision:
            raise ValueError("bbox/range distance belongs to a separate experiment")
        vision_sequence = sequence(vision["sequence"], "vision.sequence")
        if vision_sequence <= self.last_vision_sequence:
            return "duplicate_or_out_of_order"
        if vision.get("epoch") != self.epoch:
            raise ValueError("Vision epoch differs from the continuous odometry session")
        self.last_vision_sequence = vision_sequence
        age = finite(vision["age"], "vision.age")
        image_time = finite(vision["sim_time"], "vision.sim_time")
        if age < 0 or image_time < 0 or image_time > self.sim_time + 0.001:
            raise ValueError("Invalid vision source age/time")
        source_age = max(age, self.sim_time - image_time)
        if source_age > self.map.p["point_timeout_sec"]:
            return "stale"
        state = str(vision["traffic_light_state"]).strip().lower()
        if state not in ("red", "green", "unknown"):
            raise ValueError("Invalid traffic light state")
        self.map._on_light_state(DataMessage(state))
        self.total_vision_votes += 1
        if vision.get("supported") is not True:
            return "light_only_unsupported_lane"
        if vision.get("vision_commit") != self.vision_commit:
            raise ValueError("Visual ABC provenance differs from the pinned LLP/LYA source")
        try:
            corridor = self._corridor_context(vision, image_time, age)
        except (ValueError, TypeError, KeyError, OverflowError):
            return "light_only_invalid_corridor"
        points = vision.get("selected_points")
        if not isinstance(points, list) or len(points) != 3:
            raise ValueError("supported vision requires three selected_points")
        validated = []
        for point in points:
            if not isinstance(point, list) or len(point) not in (2, 3):
                raise ValueError("Each ABC point requires x,y and optional theta")
            validated.append(Pose2D(*[finite(value, "point") for value in point]))
        self.last_point_source = dict(sequence=vision_sequence, sim_time=image_time,
                                      source_age_sec=source_age, received_at=time.monotonic())
        self.point_corridor = corridor
        for label, point in zip("abc", validated):
            self.map._point_cb(label)(point)
        return "accepted"

    def _map_data(self):
        node, outer = self.map, set(self.map.official.node_ids[:-1])
        graph = node.mapped_topology if node.mapped_topology is not None else node.topology
        aligned = node.mapped_topology is not None
        edges = sorted({tuple(sorted((first, second)))
                        for first, neighbours in graph.adjacency.items() for second in neighbours})
        tracker = node.detour_tracker
        return dict(frame_id=node.odom_frame,
                    learned_samples=[[s.x, s.y, s.yaw, bool(s.excluded)] for s in node.samples],
                    odom_reference_points=self.first_odom_reference,
                    reference_points=node.reference.points if node.reference is not None else [],
                    reference_trusted=node.reference.trusted if node.reference is not None else [],
                    graph_nodes=[dict(id=key, x=value[0], y=value[1],
                                      kind=("outer_anchor" if key in outer else "internal_estimate") if aligned
                                      else ("official_outer" if key in outer else "official_internal"))
                                 for key, value in sorted(graph.nodes.items())],
                    graph_edges=edges, graph_frame="odom" if aligned else "official_unaligned",
                    detour_points=tracker.points if tracker is not None else [],
                    detour_progress_m=tracker.progress_m if tracker is not None else None,
                    detour_segment_index=tracker.segment_index if tracker is not None else None,
                    path_revision=node.path_pub.count)

    def tick(self, request):
        if request.get("type") != "tick":
            raise ValueError("Only type=tick is accepted")
        if request.get("experiment_mode", EXPERIMENT_MODE) != EXPERIMENT_MODE:
            raise ValueError("Experiment mode differs from the joint-map worker")
        tick_sequence = sequence(request["sequence"], "sequence")
        if tick_sequence <= self.last_tick_sequence:
            raise ValueError("Tick sequence must increase")
        sim_time = finite(request["sim_time"], "sim_time")
        if sim_time < 0 or (self.previous_sim_time is not None and sim_time < self.previous_sim_time):
            raise ValueError("Unity simulation time must not rewind")
        epoch = sequence(request["lane_epoch"], "lane_epoch")
        if self.epoch is None:
            self.epoch = epoch
        elif self.epoch != epoch:
            raise ValueError("Lane epoch changed; start a new worker instead of implicitly resetting its map")
        self.last_tick_sequence = tick_sequence
        self.sim_time = self.previous_sim_time = sim_time
        values = {name: finite(request["odom"][name], "odom." + name) for name in ("x", "y", "yaw", "v", "w")}
        odom = Odometry()
        nanos = int(sim_time * 1e9)
        odom.header.stamp = NS(sec=nanos // 10**9, nanosec=nanos % 10**9)
        odom.pose.pose.position.x, odom.pose.pose.position.y = values["x"], values["y"]
        odom.pose.pose.orientation.z = math.sin(values["yaw"] / 2)
        odom.pose.pose.orientation.w = math.cos(values["yaw"] / 2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = values["v"], values["w"]
        # Sample publisher counters before any callback: color callbacks can
        # publish a real stop even before the main control timer executes.
        point_count, command_count = self.map.point_pubs["c"].count, self.map.cmd_pub.count
        lya_count = self.lya_command_count
        for field, callback in (("avoidance_active", self.map._on_avoidance),
                                ("branch_active", self.map._on_branch)):
            if field in request:
                if type(request[field]) is not bool:
                    raise ValueError(field + " must be boolean")
                callback(DataMessage(request[field]))
        vision_result = self._receive_vision(request.get("vision"))
        self.map._on_odom(odom)
        # Preserve the first odom curve before the asynchronous fusion result
        # can swap the active reference in _tick. This copy is display-only.
        if not self.first_odom_reference and self.map.reference is not None:
            self.first_odom_reference = copy.deepcopy(self.map.reference.points)
        self.map._tick()
        self.map._publish_status()
        status = json.loads(self.map.status_pub.value.data)
        cmd, raw = self.map.cmd_pub.value or Twist(), self.lya.velocity_publisher.value
        v, w = float(cmd.linear.x), float(cmd.angular.z)
        target = self.target.publisher_point.value
        selected = [self.map.point_pubs[label].value for label in "abc"]
        now = time.monotonic()
        source = self.last_lya_source
        source_age, source_sequence = None, None
        remaining = 0.0
        transport_reason = "forwarding_native_cmd_pub"
        if source is not None:
            source_age = max(0.0, now - source["received_at"])
            source_sequence = source["tick_sequence"]
            remaining = max(0.0, self.map.p["cmd_vel_timeout_sec"] - source_age)
            if source["kind"] == "vision" and source["vision"] is not None:
                observation = source["vision"]
                source_age = max(now - observation["received_at"] + observation["source_age_sec"],
                                 self.sim_time - observation["sim_time"])
                source_sequence = observation["sequence"]
                remaining = min(remaining, max(0.0, self.map.p["point_timeout_sec"] - source_age))
        corridor = self._corridor_guard()
        # Capture age remains an adapter boundary because Pose2D has no stamp.
        # The native gate result is retained separately for audit/visualization.
        if (v != 0.0 or w != 0.0) and remaining <= 0.0:
            v = w = 0.0
            transport_reason = "source_command_timeout"
        elif (v != 0.0 or w != 0.0) and corridor is not None and corridor["applicable"] and not corridor["valid"]:
            v = w = 0.0
            remaining = 0.0
            transport_reason = "corridor_width_prior_expired"
        if corridor is not None and corridor["applicable"]:
            remaining = min(remaining, corridor["remaining_sec"])
        result = dict(type="tick", experiment_mode=EXPERIMENT_MODE, worker_boot_id=self.worker_boot_id,
                      wall=time.time(), elapsed_wall_sec=now - self.started_monotonic,
                      sequence=tick_sequence, sim_time=sim_time, valid=True, epoch=self.epoch,
                      v=v, w=w, semantic_v=float(cmd.linear.x), semantic_w=float(cmd.angular.z),
                      status=status, odom=values, map_data=self._map_data(),
                      transport_gate_reason=transport_reason, corridor_guard=corridor,
                      command_gate=dict(reason=transport_reason if transport_reason != "forwarding_native_cmd_pub"
                                        else status["velocity_gate_reason"], policy="native_joint_MapAssist",
                                        source_kind=source["kind"] if source else None,
                                        source_sequence=source_sequence, source_age_sec=source_age,
                                        remaining_sec=remaining, raw_command_new=self.lya_command_count > lya_count),
                      selected_points=[[float(p.x), float(p.y)] for p in selected if p is not None],
                      selected_points_new=self.map.point_pubs["c"].count > point_count,
                      topo_selected_points_new=self.map.point_pubs["c"].count > point_count,
                      target=[float(target.x), float(target.y), float(target.theta)] if target else [],
                      controller_v=float(raw.linear.x) if raw else 0.0,
                      controller_w=float(raw.angular.z) if raw else 0.0,
                      command_new=self.map.cmd_pub.count > command_count,
                      vision_result=vision_result, vision_votes_total=self.total_vision_votes,
                      last_vision_sequence=self.last_vision_sequence,
                      source_hash=self.source_hash, source_tree_hash=self.source_tree_hash,
                      vision_commit=self.vision_commit)
        self.last_result = result
        if self.evidence is not None:
            self.evidence.write(json.dumps(result, separators=(",", ":"), allow_nan=False) + "\n")
            self.evidence.flush()
        return result

    def close(self):
        pending = self.map._fusion_future is not None or self.map._pending_fusion is not None
        if not self.closed:
            self.map.close_fusion_worker()
            if self.evidence is not None:
                self.evidence.close()
            if self.output_root is not None:
                write_json(self.output_root / "final_state.json", self.last_result)
            self.closed = True
        return dict(type="shutdown", experiment_mode=EXPERIMENT_MODE,
                    worker_boot_id=self.worker_boot_id, clean=True,
                    fusion_pending_on_close=pending, persistence="not_implemented_in_supplied_package")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", required=True)
    parser.add_argument("--vision-root", required=True)
    parser.add_argument("--output-root")
    args = parser.parse_args()
    protocol, sys.stdout = sys.stdout, sys.stderr

    def send(value):
        protocol.write(json.dumps(value, separators=(",", ":"), allow_nan=False) + "\n")
        protocol.flush()

    transport, request = None, {}
    try:
        transport = SemanticTransport(args.source_root, args.vision_root, args.output_root)
        send(transport.ready())
        while True:
            line = sys.stdin.buffer.readline(MAX_LINE + 1)
            if not line:
                return 0
            if len(line) > MAX_LINE:
                raise ValueError("Input exceeds transport limit")
            request = json.loads(line)
            if not isinstance(request, dict):
                raise ValueError("Input must be a JSON object")
            if request.get("type") == "shutdown":
                send(transport.close())
                return 0
            send(transport.tick(request))
    except Exception as error:
        failure = dict(type="error", experiment_mode=EXPERIMENT_MODE,
                       worker_boot_id=transport.worker_boot_id if transport else None,
                       sequence=request.get("sequence") if isinstance(request, dict) else None,
                       valid=False, v=0.0, w=0.0, error=f"{type(error).__name__}: {error}")
        print(failure["error"], file=sys.stderr)
        # A follower can replace its UI status during shutdown. Preserve the
        # original failure without changing any control/provenance decision.
        if transport is not None and transport.output_root is not None:
            try:
                write_json(transport.output_root / "error.json", failure)
                write_json(transport.output_root / "failed_request.json", failed_request_snapshot(request))
            except Exception as diagnostic_error:
                print(f"Could not persist transport failure: {diagnostic_error}", file=sys.stderr)
        send(failure)
        return 1
    finally:
        if transport is not None:
            transport.close()


if __name__ == "__main__":
    raise SystemExit(main())
