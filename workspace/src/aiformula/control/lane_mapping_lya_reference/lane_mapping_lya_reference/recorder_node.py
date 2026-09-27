"""Passive first-lap recorder: causal masks + VectorNav, never motor commands.

Finishing is a stopped-vehicle operation. A completed bundle is only data;
another explicitly armed node owns command selection and fixed-path execution.
"""

import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time
import uuid

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as PathMessage
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from vectornav_msgs.msg import CommonGroup

from lane_mapping_lya_reference.mapping_core import (
    CausalSampleBuffer, GroundLookup, SparseConsensusMap, VectorNavLocalizer,
    make_transform, transform_local_points,
)
from lane_mapping_lya_reference.route import RouteBuildError, build_route


def stamp_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json_exclusive(path, value):
    """Exclusive creation: snapshots and finished routes are never overwritten."""
    with Path(path).open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, ensure_ascii=False, sort_keys=True, indent=2,
                  allow_nan=False)
        stream.write("\n")
        stream.flush()


def decode_mask(message):
    if message.encoding not in ("mono8", "8UC1"):
        raise ValueError("mask_must_be_mono8_or_8UC1")
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width < 2 or height < 2 or step < width:
        raise ValueError("invalid_mask_dimensions")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < height * step:
        raise ValueError("truncated_mask_buffer")
    return raw[:height * step].reshape(height, step)[:, :width]


def point_cloud(header, xy, votes):
    points = np.asarray(xy, dtype=np.float32).reshape(-1, 2)
    packed = np.zeros((len(points), 4), dtype="<f4")
    packed[:, :2] = points
    packed[:, 3] = np.asarray(votes, dtype=np.float32)
    message = PointCloud2()
    message.header = header
    message.height, message.width = 1, len(points)
    message.fields = [PointField(name=name, offset=index * 4,
                                 datatype=PointField.FLOAT32, count=1)
                      for index, name in enumerate(("x", "y", "z", "intensity"))]
    message.is_bigendian, message.is_dense = False, True
    message.point_step, message.row_step = 16, 16 * len(points)
    message.data = packed.tobytes()
    return message


class LaneLapRecorder(Node):
    """Records continuous localization and ordered trace, not LYA actuations."""

    DEFAULTS = {
        "mask_topic": "/aiformula_perception/pub_mask_image",
        "vectornav_topic": "/vectornav/raw/common",
        "camera_info_topic": "/zed/zed_node/left/camera_info",
        "lya_command_topic": "/lane_learning/lya_cmd",
        "map_frame": "lane_map", "base_frame": "base_link",
        "camera_frame_override": "",
        "output_directory": "lane_lap_runs",
        "grid_resolution_m": 0.1, "consensus_min_frame_votes": 5,
        "consensus_min_reliable_frame_votes": 1,
        "max_projection_sensitivity_m_per_px": 0.5,
        "max_reliable_projection_sensitivity_m_per_px": 0.1,
        "mask_threshold": 127,
        "max_mask_age_ms": 1000.0, "max_pose_mask_gap_ms": 200.0,
        "max_vectornav_age_ms": 250.0, "future_tolerance_ms": 20.0,
        "yaw_offset_rad": 0.0, "position_jump_tolerance_m": 0.35,
        "maximum_speed_mps": 8.0,
        "max_candidate_cells": 200000, "max_confirmed_cells": 100000,
        "candidate_ttl_s": 30.0, "vectornav_buffer_size": 4096,
        "trace_min_distance_m": 0.05, "trace_min_yaw_rad": 0.03,
        "max_trace_points": 100000, "max_log_rows": 2000000,
        "max_session_duration_s": 3600.0,
        "finish_max_speed_mps": 0.15, "finish_stopped_duration_s": 0.5,
        "route_config_json": "{}",
    }

    def __init__(self):
        super().__init__("lane_lap_recorder")
        self.params = {}
        for name, default in self.DEFAULTS.items():
            self.declare_parameter(name, default)
            self.params[name] = self.get_parameter(name).value
        self._validate_parameters()
        self.group = MutuallyExclusiveCallbackGroup()
        self.phase, self.reason = "recording", "waiting_for_inputs"
        self.bundle = None
        self.sequence = 0
        self.last_mask_stamp = None
        self.last_pose_header = None
        self.last_stopped_ns = None
        self.stopped_since_steady_ns = None
        self.last_vn_receive_steady_ns = None
        self.trace = []
        self.camera = None
        self.extrinsic = None
        self.lookup = None
        self.counters = {"vectornav_accepted": 0, "vectornav_rejected": 0,
                         "masks_received": 0, "masks_fused": 0,
                         "masks_rejected": 0, "commands_observed": 0}
        self.last_rejection = None
        self.map_dirty = False
        self.session_started_steady = time.monotonic()
        self.localizer = VectorNavLocalizer(
            yaw_offset_rad=float(self.params["yaw_offset_rad"]),
            maximum_age_ms=float(self.params["max_vectornav_age_ms"]),
            future_tolerance_ms=float(self.params["future_tolerance_ms"]),
            position_tolerance_m=float(self.params["position_jump_tolerance_m"]),
            maximum_speed_mps=float(self.params["maximum_speed_mps"]))
        self.pose_buffer = CausalSampleBuffer(int(self.params["vectornav_buffer_size"]))
        self.map = SparseConsensusMap(
            float(self.params["grid_resolution_m"]),
            int(self.params["consensus_min_frame_votes"]),
            int(self.params["consensus_min_reliable_frame_votes"]),
            max_candidate_cells=int(self.params["max_candidate_cells"]),
            max_confirmed_cells=int(self.params["max_confirmed_cells"]),
            candidate_ttl_s=float(self.params["candidate_ttl_s"]))
        self._open_logs()
        reliable = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        sensor = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        vn_qos = QoSProfile(depth=100, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.state_pub = self.create_publisher(String, "/lane_learning/recorder_state", reliable)
        self.bundle_pub = self.create_publisher(String, "/lane_learning/route_bundle", reliable)
        self.pose_pub = self.create_publisher(PoseStamped, "~/pose", sensor)
        self.path_pub = self.create_publisher(PathMessage, "~/trajectory", reliable)
        self.map_pub = self.create_publisher(PointCloud2, "~/consensus", reliable)
        self.create_subscription(Image, str(self.params["mask_topic"]), self._on_mask,
                                 sensor, callback_group=self.group)
        self.create_subscription(CommonGroup, str(self.params["vectornav_topic"]),
                                 self._on_vectornav, vn_qos, callback_group=self.group)
        self.create_subscription(CameraInfo, str(self.params["camera_info_topic"]),
                                 self._on_camera_info, sensor, callback_group=self.group)
        self.create_subscription(Twist, str(self.params["lya_command_topic"]),
                                 self._on_command, sensor, callback_group=self.group)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_service(Trigger, "~/finish_lap", self._on_finish,
                            callback_group=self.group)
        self.create_service(Trigger, "~/save_snapshot", self._on_snapshot,
                            callback_group=self.group)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0, self._on_timer, clock=self.steady_clock,
                          callback_group=self.group)
        self._event("session_started", {"parameters": self.params})
        self._publish_state()
        self.get_logger().info("Passive lap recorder; logs: {}".format(self.session_dir))

    def _validate_parameters(self):
        if self.params["mask_topic"] == "/aiformula_perception/road_detector/mask_image":
            raise ValueError("Legacy ROI mask is forbidden; use the full-mask topic")
        positive = ("grid_resolution_m", "max_mask_age_ms", "max_pose_mask_gap_ms",
                    "max_vectornav_age_ms", "max_candidate_cells", "max_confirmed_cells",
                    "candidate_ttl_s", "vectornav_buffer_size", "max_trace_points",
                    "max_log_rows", "max_session_duration_s", "maximum_speed_mps",
                    "finish_stopped_duration_s")
        for name in positive:
            value = float(self.params[name])
            if not math.isfinite(value) or value <= 0:
                raise ValueError("{} must be finite and positive".format(name))
        for name in ("trace_min_distance_m", "trace_min_yaw_rad", "finish_max_speed_mps"):
            value = float(self.params[name])
            if not math.isfinite(value) or value < 0:
                raise ValueError("{} must be finite and nonnegative".format(name))
        reliable = float(self.params["max_reliable_projection_sensitivity_m_per_px"])
        candidate = float(self.params["max_projection_sensitivity_m_per_px"])
        if not 0 < reliable <= candidate < math.inf:
            raise ValueError("Projection gates must satisfy 0 < reliable <= candidate")
        if not 0 <= int(self.params["mask_threshold"]) <= 254:
            raise ValueError("mask_threshold must be in [0,254]")
        self.route_config = json.loads(str(self.params["route_config_json"]))
        if not isinstance(self.route_config, dict):
            raise ValueError("route_config_json must encode an object")

    def _open_logs(self):
        base = Path(str(self.params["output_directory"])).expanduser().resolve()
        base.mkdir(parents=True, exist_ok=True)
        self.session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid.uuid4().hex[:12]
        self.session_dir = base / self.session_id
        self.session_dir.mkdir(exist_ok=False)
        self.streams = {}
        self.writers = {}
        headers = {
            "trace": ["stamp_ns", "arrival_ros_ns", "east_m", "north_m", "yaw_rad",
                      "speed_mps", "latitude_deg", "longitude_deg", "altitude_m"],
            "commands": ["arrival_ros_ns", "steady_ns", "linear_x_mps", "angular_z_radps"],
            "masks": ["stamp_ns", "arrival_ros_ns", "selected_pose_stamp_ns", "age_ms",
                      "stable_pixels", "unique_cells", "reliable_cells", "callback_ms"],
        }
        for name, fields in headers.items():
            stream = (self.session_dir / (name + ".csv")).open("x", encoding="utf-8", newline="")
            self.streams[name] = stream
            self.writers[name] = csv.writer(stream)
            self.writers[name].writerow(fields)
            stream.flush()
        self.events = (self.session_dir / "events.jsonl").open("x", encoding="utf-8")
        self.log_rows = 0

    def _event(self, event, details=None):
        self.events.write(json.dumps({"event": event, "ros_ns": self._now_ns(),
                                      "steady_ns": time.monotonic_ns(),
                                      "details": details or {}}, allow_nan=False) + "\n")
        self.events.flush()

    def _row(self, name, values):
        if self.log_rows >= int(self.params["max_log_rows"]):
            self._fault("log_row_capacity_reached")
            return False
        self.writers[name].writerow(values)
        self.streams[name].flush()
        self.log_rows += 1
        return True

    def _now_ns(self):
        return int(self.get_clock().now().nanoseconds)

    def _next_sequence(self):
        self.sequence += 1
        return self.sequence

    def _fault(self, reason):
        if self.phase != "invalid":
            self.phase, self.reason = "invalid", reason
            self._event("lap_invalid", {"reason": reason})
            self.get_logger().error("Lap invalid: {}".format(reason))
            self._publish_state()

    def _reject(self, kind, reason):
        self.counters[kind + "_rejected"] += 1
        key = (kind, str(reason))
        if key != self.last_rejection:
            self._event("input_rejected", {"input": kind, "reason": str(reason)})
            self.last_rejection = key

    def _on_camera_info(self, message):
        self._next_sequence()
        if self.phase != "recording":
            return
        projection = np.asarray(message.p, dtype=float).reshape(3, 4)[:, :3]
        camera = projection if np.all(np.isfinite(projection)) and abs(np.linalg.det(projection)) > 1e-12 else np.asarray(message.k, dtype=float).reshape(3, 3)
        frame = str(message.header.frame_id)
        if (not frame or message.width < 2 or message.height < 2
                or not np.all(np.isfinite(camera)) or abs(np.linalg.det(camera)) < 1e-12
                or stamp_ns(message.header.stamp) < 0):
            self._event("invalid_camera_info")
            return
        data = {"width": int(message.width), "height": int(message.height),
                "frame_id": frame, "stamp_ns": stamp_ns(message.header.stamp),
                "matrix": camera.tolist(), "distortion": list(message.d),
                "distortion_model": str(message.distortion_model)}
        if self.camera is not None:
            if any(self.camera[key] != data[key] for key in
                   ("width", "height", "frame_id", "matrix", "distortion", "distortion_model")):
                self._fault("camera_calibration_changed")
            return
        self.camera = data
        self._event("camera_calibration_latched", data)

    def _on_vectornav(self, message):
        sequence = self._next_sequence()
        if self.phase not in ("recording", "ready"):
            return
        now = self._now_ns()
        received_steady = time.monotonic_ns()
        previous = self.localizer.last_sample
        try:
            sample = self.localizer.accept(message, now, sequence)
        except (ValueError, AttributeError) as error:
            self.last_stopped_ns = None
            self.stopped_since_steady_ns = None
            self._reject("vectornav", str(error))
            if self.phase == "recording" and str(error) in (
                    "vectornav_position_jump", "vectornav_nonincreasing_stamp",
                    "vectornav_heading_jump"):
                self._fault(str(error))
            return
        self.counters["vectornav_accepted"] += 1
        self.last_pose_header = message.header
        self.pose_pub.publish(self._pose_message(sample))
        gap_limit_ns = float(self.params["max_vectornav_age_ms"]) * 1e6
        if ((previous is not None and sample.stamp_ns - previous.stamp_ns > gap_limit_ns)
                or (self.last_vn_receive_steady_ns is not None
                    and received_steady - self.last_vn_receive_steady_ns > gap_limit_ns)):
            self.last_stopped_ns = None
            self.stopped_since_steady_ns = None
        self.last_vn_receive_steady_ns = received_steady
        if sample.speed_mps <= float(self.params["finish_max_speed_mps"]):
            if self.last_stopped_ns is None:
                self.last_stopped_ns = sample.stamp_ns
                self.stopped_since_steady_ns = received_steady
        else:
            self.last_stopped_ns = None
            self.stopped_since_steady_ns = None
        if self.phase != "recording":
            return
        self.pose_buffer.append(sample)
        if not self._row("trace", [sample.stamp_ns, now, sample.east_m, sample.north_m,
                                   sample.heading_rad, sample.speed_mps, sample.latitude_deg,
                                   sample.longitude_deg, sample.altitude_m]):
            return
        previous = self.trace[-1] if self.trace else None
        distance = math.inf if previous is None else math.hypot(
            sample.east_m - previous.east_m, sample.north_m - previous.north_m)
        yaw_change = math.inf if previous is None else abs(math.atan2(
            math.sin(sample.heading_rad - previous.heading_rad),
            math.cos(sample.heading_rad - previous.heading_rad)))
        if distance >= float(self.params["trace_min_distance_m"]) or yaw_change >= float(self.params["trace_min_yaw_rad"]):
            if len(self.trace) >= int(self.params["max_trace_points"]):
                self._fault("trace_capacity_reached_no_silent_truncation")
                return
            self.trace.append(sample)
        if self.counters["vectornav_accepted"] == 1:
            self._event("origin_latched", {"origin_lla": self.localizer.origin_lla})

    def _pose_message(self, sample):
        pose = PoseStamped()
        pose.header.frame_id = str(self.params["map_frame"])
        pose.header.stamp.sec = sample.stamp_ns // 1000000000
        pose.header.stamp.nanosec = sample.stamp_ns % 1000000000
        pose.pose.position.x, pose.pose.position.y = sample.east_m, sample.north_m
        pose.pose.orientation.z = math.sin(sample.heading_rad / 2)
        pose.pose.orientation.w = math.cos(sample.heading_rad / 2)
        return pose

    def _on_command(self, message):
        if self.phase != "recording":
            return
        values = [float(message.linear.x), float(message.angular.z)]
        if not all(math.isfinite(value) for value in values):
            self._event("nonfinite_teacher_command")
            return
        self.counters["commands_observed"] += 1
        self._row("commands", [self._now_ns(), time.monotonic_ns()] + values)

    def _ensure_lookup(self, message):
        if self.camera is None:
            raise ValueError("waiting_camera_info")
        frame = str(self.params["camera_frame_override"]) or str(message.header.frame_id)
        if (frame != self.camera["frame_id"] or int(message.width) != self.camera["width"]
                or int(message.height) != self.camera["height"]):
            raise ValueError("mask_calibration_frame_or_size_mismatch")
        if self.camera["stamp_ns"] > stamp_ns(message.header.stamp):
            raise ValueError("camera_info_newer_than_mask")
        if self.lookup is None:
            transform = self.tf_buffer.lookup_transform(str(self.params["base_frame"]), frame, Time())
            if stamp_ns(transform.header.stamp) != 0:
                raise ValueError("camera_tf_must_be_static")
            translation, rotation = transform.transform.translation, transform.transform.rotation
            matrix = make_transform([translation.x, translation.y, translation.z],
                                    [rotation.x, rotation.y, rotation.z, rotation.w])
            self.extrinsic = {"parent_frame": str(self.params["base_frame"]),
                              "child_frame": frame, "matrix": matrix.tolist()}
            self.lookup = GroundLookup(int(message.width), int(message.height),
                                       self.camera["matrix"], matrix,
                                       float(self.params["max_projection_sensitivity_m_per_px"]))
            self._event("static_extrinsic_latched", self.extrinsic)
        return self.lookup

    def _on_mask(self, message):
        sequence = self._next_sequence()
        if self.phase != "recording":
            return
        started = time.perf_counter_ns()
        self.counters["masks_received"] += 1
        source = stamp_ns(message.header.stamp)
        now = self._now_ns()
        if source <= 0 or now <= 0:
            self._reject("masks", "invalid_mask_clock")
            return
        if self.last_mask_stamp is not None and source <= self.last_mask_stamp:
            self._fault("mask_time_regression")
            return
        self.last_mask_stamp = source
        if not -float(self.params["future_tolerance_ms"]) <= (now - source) / 1e6 <= float(self.params["max_mask_age_ms"]):
            self._reject("masks", "stale_or_future_mask")
            return
        pose, _, reason = self.pose_buffer.latest_not_after(
            source, sequence, int(float(self.params["max_pose_mask_gap_ms"]) * 1e6))
        if pose is None:
            self._reject("masks", reason)
            return
        try:
            mask = decode_mask(message)
            lookup = self._ensure_lookup(message)
            local, sensitivity, counts = lookup.project_mask_with_sensitivity(
                mask, int(self.params["mask_threshold"]))
        except Exception as error:  # tf2 exception types differ between ROS releases.
            self._reject("masks", str(error))
            return
        world = transform_local_points(local, pose.east_m, pose.north_m, pose.heading_rad)
        keys = self.map.points_to_unique_keys(world)
        reliable = self.map.points_to_unique_keys(
            world[sensitivity <= float(self.params["max_reliable_projection_sensitivity_m_per_px"])])
        age = (self._now_ns() - source) / 1e6
        if self._now_ns() <= 0 or not -float(self.params["future_tolerance_ms"]) <= age <= float(self.params["max_mask_age_ms"]):
            self._reject("masks", "clock_or_deadline_changed_during_projection")
            return
        self.map.update(keys, reliable, stamp_ns=source)
        self.counters["masks_fused"] += 1
        self.map_dirty = True
        self.reason = "mapping"
        self._row("masks", [source, now, pose.stamp_ns, age, counts.stable_pixels,
                            len(keys), len(reliable), (time.perf_counter_ns() - started) / 1e6])
        if self.map.capacity_rejected_cells or self.map.confirmation_capacity_rejections:
            self._fault("map_capacity_reached_no_partial_route")

    def _publish_state(self):
        sample = self.localizer.last_sample
        state = {"schema_version": 1, "state": self.phase, "reason": self.reason,
                 "stamp_ns": self._now_ns(), "session_id": self.session_id,
                 "session_directory": str(self.session_dir),
                 "origin_lla": self.localizer.origin_lla, "trace_points": len(self.trace),
                 "confirmed_cells": len(self.map.confirmed_keys), "counters": self.counters,
                 "pose_stamp_ns": None if sample is None else sample.stamp_ns,
                 "ready": self.phase == "ready", "commands_are_observation_only": True}
        self.state_pub.publish(String(data=json.dumps(state, allow_nan=False)))

    def _on_timer(self):
        if (self.phase == "recording" and time.monotonic() - self.session_started_steady
                > float(self.params["max_session_duration_s"])):
            self._fault("session_duration_limit_reached")
        if self.map_dirty and self.last_pose_header is not None:
            header = Header()
            header.stamp = self.last_pose_header.stamp
            header.frame_id = str(self.params["map_frame"])
            points, votes = self.map.confirmed_points_and_votes()
            self.map_pub.publish(point_cloud(header, points, votes))
            self.map_dirty = False
        if self.trace:
            path = PathMessage()
            path.header = self._pose_message(self.trace[-1]).header
            path.poses = [self._pose_message(sample) for sample in self.trace]
            self.path_pub.publish(path)
        self._publish_state()

    def _metadata(self, finished):
        source_dir = Path(__file__).resolve().parent
        hashes = {name: sha256_file(source_dir / name) for name in
                  ("recorder_node.py", "mapping_core.py", "route.py")}
        return {"schema_version": 1, "session_id": self.session_id,
                "finished": bool(finished), "state": self.phase,
                "origin_lla": self.localizer.origin_lla,
                "frame_id": str(self.params["map_frame"]),
                "coordinate_convention": "ENU metres; body x forward, y left; yaw CCW from east",
                "heading_source": "vectornav_yaw_ned_to_enu",
                "yaw_offset_rad": float(self.params["yaw_offset_rad"]),
                "position_reference": "VectorNav LLA origin; zero antenna-to-base lever arm assumed",
                "camera_calibration": self.camera, "static_extrinsic": self.extrinsic,
                "parameters": self.params, "counters": dict(self.counters),
                "source_sha256": hashes, "stamp_ns": self._now_ns(),
                "causality": "only arrived VectorNav samples with header <= mask header; no interpolation or future data",
                "model_type": "pose-referenced semantic lane consensus, not SLAM",
                "commands_are_observation_only": True}

    def _save(self, finished=False):
        directory = self.session_dir / ("finished_" if finished else "snapshot_")
        directory = directory.with_name(directory.name + uuid.uuid4().hex[:12])
        directory.mkdir(exist_ok=False)
        points, votes = self.map.confirmed_points_and_votes()
        with (directory / "consensus.csv").open("x", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["east_m", "north_m", "frame_votes"])
            writer.writerows((float(point[0]), float(point[1]), int(vote))
                            for point, vote in zip(points, votes))
        with (directory / "trace.csv").open("x", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["stamp_ns", "east_m", "north_m", "yaw_rad", "speed_mps"])
            writer.writerows((sample.stamp_ns, sample.east_m, sample.north_m,
                              sample.heading_rad, sample.speed_mps) for sample in self.trace)
        metadata = self._metadata(finished)
        metadata["data_sha256"] = {name: sha256_file(directory / name)
                                   for name in ("consensus.csv", "trace.csv")}
        write_json_exclusive(directory / "metadata.json", metadata)
        self._event("snapshot_saved", {"directory": str(directory), "finished": finished})
        return directory, points

    def _on_snapshot(self, request, response):
        del request
        try:
            directory, _ = self._save()
            response.success, response.message = True, str(directory)
        except (OSError, ValueError) as error:
            response.success, response.message = False, str(error)
        return response

    def _on_finish(self, request, response):
        del request
        if self.phase != "recording":
            response.success, response.message = False, "Recorder is " + self.phase
            return response
        sample = self.localizer.last_sample
        now = self._now_ns()
        duration = float(self.params["finish_stopped_duration_s"]) * 1e9
        steady_now = time.monotonic_ns()
        if (sample is None or self.last_stopped_ns is None or now <= 0
                or not 0 <= now - sample.stamp_ns <= float(self.params["max_vectornav_age_ms"]) * 1e6
                or sample.stamp_ns - self.last_stopped_ns < duration
                or self.stopped_since_steady_ns is None
                or steady_now - self.stopped_since_steady_ns < duration
                or self.last_vn_receive_steady_ns is None
                or steady_now - self.last_vn_receive_steady_ns > float(self.params["max_vectornav_age_ms"]) * 1e6):
            response.success, response.message = False, "HOLD vehicle still with fresh VectorNav before finish_lap"
            return response
        if self.camera is None or self.extrinsic is None:
            response.success, response.message = False, "No calibrated lane map available"
            return response
        self.phase, self.reason = "finalizing", "building_validated_closed_route"
        self._publish_state()
        try:
            points, _ = self.map.confirmed_points_and_votes()
            trajectory = np.asarray([[item.east_m, item.north_m, item.heading_rad]
                                     for item in self.trace], dtype=float).reshape(-1, 3)
            route = build_route(trajectory, points, self.route_config)
            directory, _ = self._save(finished=True)
            route.update({"frame_id": str(self.params["map_frame"]),
                          "origin_lla": self.localizer.origin_lla,
                          "heading_source": "vectornav_yaw_ned_to_enu",
                          "yaw_offset_rad": float(self.params["yaw_offset_rad"])})
            write_json_exclusive(directory / "route.json", route)
            bundle = {"schema_version": 1, "bundle_id": uuid.uuid4().hex,
                      "session_directory": str(directory),
                      "route_path": "route.json", "route_sha256": sha256_file(directory / "route.json"),
                      "metadata_path": "metadata.json",
                      "metadata_sha256": sha256_file(directory / "metadata.json"),
                      "origin_lla": self.localizer.origin_lla,
                      "frame_id": str(self.params["map_frame"]),
                      "heading_source": "vectornav_yaw_ned_to_enu",
                      "yaw_offset_rad": float(self.params["yaw_offset_rad"]), "ready": True}
            write_json_exclusive(directory / "bundle.json", bundle)
            self.bundle = bundle
            self.phase, self.reason = "ready", "route_validated_await_explicit_operator_arm"
            self._event("route_ready", bundle)
            self.bundle_pub.publish(String(data=json.dumps(bundle, allow_nan=False)))
            response.success, response.message = True, str(directory / "bundle.json")
        except (RouteBuildError, ValueError, OSError) as error:
            self._fault("finish_failed: " + str(error))
            self._event("finish_failed", {"error": str(error),
                                          "diagnostics": getattr(error, "diagnostics", {})})
            response.success, response.message = False, str(error)
            try:
                self._save(finished=False)
            except (OSError, ValueError) as snapshot_error:
                self._event("failure_snapshot_failed", {"error": str(snapshot_error)})
        self._publish_state()
        return response

    def destroy_node(self):
        if hasattr(self, "events") and not self.events.closed:
            self._event("session_shutdown", {"state": self.phase, "counters": self.counters})
            for stream in self.streams.values():
                stream.close()
            self.events.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneLapRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
