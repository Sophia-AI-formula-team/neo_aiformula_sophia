"""ROS 2 orchestration. GNSS subscriptions exist only inside two stopped windows.

One callback group owns safety and all sensor admission; one bounded worker
owns map geometry. No GNSS-to-odometry bridge, synthetic CommonGroup, or INS
pose subscription exists. Restarting this node requires teaching a new lap.
"""
from collections import deque
from contextlib import contextmanager
from functools import wraps
import json
import math
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from can_msgs.msg import Frame
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as PathMessage
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from vectornav_msgs.msg import GpsGroup

from lane_mapping_lya_reference.controller import ClosedRouteController, Command, Pose, fresh
from lane_mapping_lya_reference.follower_node import RunJournal, finite_json
from lane_mapping_lya_reference.mapping_core import make_transform
from lane_mapping_lya_reference.recorder_node import point_cloud
from .anchors import EndpointAnchors, decode_vectornav_gps
from .control import RuntimeSafety
from .motion import CausalWheelGyroOdometry, decode_honda_rpm
from .worker import MapWorker


def source_ns(message):
    return int(message.header.stamp.sec) * 1000000000 + int(message.header.stamp.nanosec)


def serialized(function):
    @wraps(function)
    def call(self, *args, **kwargs):
        with self.lock:
            return function(self, *args, **kwargs)
    return call


class EndpointFollower(Node):
    DEFAULTS = {
        "can_topic": "/aiformula_sensing/vehicle_info",
        "gyro_topic": "/aiformula_sensing/zed_node/imu/data_raw",
        "gnss_topic": "/aiformula_sensing/vectornav/raw/gps",
        "mask_topic": "/aiformula_perception/pub_mask_image",
        "camera_info_topic": "/aiformula_sensing/zed_node/left/camera_info",
        "base_frame": "base_footprint", "map_frame": "lane_teach_local",
        "command_output_topic": "/lane_learning_gnss/cmd_vel",
        "enable_vehicle_output": False, "hardware_stop_verified": False,
        "motor_zero_passthrough_verified": False,
        "maximum_speed_mps": 0.8, "maximum_yaw_rate_rps": 0.4,
        "wheel_diameter_m": 0.254, "gyro_bias_radps": 0.0,
        "gyro_mount_quaternion": [0.0, 0.0, 0.0, 0.0],
        "max_gnss_horizontal_sigma_m": 1.5, "max_endpoint_distance_m": 3.0,
        "mask_max_age_s": 0.5, "pose_mask_gap_s": 0.15,
        "log_directory": "~/.ros/lane_learning_gnss", "route_config_json": "{}",
    }

    def __init__(self):
        super().__init__("lane_endpoint_follower")
        self.p = {}
        for key, default in self.DEFAULTS.items():
            self.declare_parameter(key, default)
            self.p[key] = self.get_parameter(key).value
        self._validate()
        self.lock = threading.RLock()
        self.group = MutuallyExclusiveCallbackGroup()
        self.sensor_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.state_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # Publisher/subscription resolved names work on Foxy as well as newer
        # ROS 2; Node.resolve_topic_name is not available on Foxy.
        self.command_pub = self.create_publisher(Twist, self.p["command_output_topic"], 1)
        self.teacher_subscription = self.create_subscription(Twist, "/lane_learning/lya_cmd",
            self._teacher, 1, callback_group=self.group)
        self.journal = RunJournal(self.p["log_directory"])
        self.safety = RuntimeSafety(config={
            "command_output_topic": self.command_pub.topic_name,
            "enable_vehicle_output": bool(self.p["enable_vehicle_output"]),
            "motor_zero_passthrough_verified": bool(self.p["motor_zero_passthrough_verified"]),
            "maximum_speed_mps": float(self.p["maximum_speed_mps"]),
            "maximum_yaw_rate_rps": float(self.p["maximum_yaw_rate_rps"]),
            "stopped_speed_mps": 0.05,
            "hardware_stop_verified": bool(self.p["hardware_stop_verified"]),
        }, event_sink=self.journal.write)
        resolved_output = self.command_pub.topic_name
        if resolved_output == self.teacher_subscription.topic_name:
            raise ValueError("resolved output cannot alias teacher input")
        if resolved_output != "/lane_learning_gnss/cmd_vel" and not all(self.p[x] for x in (
                "enable_vehicle_output", "hardware_stop_verified", "motor_zero_passthrough_verified")):
            raise ValueError("remapped vehicle output requires all deployment confirmations")
        self.motion = CausalWheelGyroOdometry(config={"wheel_diameter_m": self.p["wheel_diameter_m"]})
        self.anchors = EndpointAnchors(config={
            "max_horizontal_sigma_m": self.p["max_gnss_horizontal_sigma_m"],
            "max_endpoint_distance_m": self.p["max_endpoint_distance_m"],
        })
        self.pose_buffer = deque(maxlen=4096)
        self.last_motion = None
        self.last_mask_ns = 0
        self.last_gyro_ns = 0
        self.camera = None
        self.calibration = None
        self.gyro_rotation = None
        self.gyro_frame = None
        self.mount_candidate = None
        self.mount_stamps = set()
        self.session_motion_invalid = False
        self.session_started = False
        self.session_start_clocks = None
        self.start_reference_motion = None
        self.start_reference_receipt_ns = None
        self.start_reference_usable = False
        self.gnss_subscription = None
        self.finish_pending = False
        self.bundle_pending = False
        self.bundle_path = None
        self.teach_enabled = False
        self.repeat_prepared = False
        self.closing = False
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.worker = MapWorker(self._commit_guard)
        self.state_pub = self.create_publisher(String, "/lane_learning_gnss/state", self.state_qos)
        self.supervisor_pub = self.create_publisher(String, "/lane_learning_gnss/control_state", self.state_qos)
        self.map_pub = self.create_publisher(PointCloud2, "/lane_learning_gnss/consensus", self.state_qos)
        self.refined_pub = self.create_publisher(PointCloud2, "/lane_learning_gnss/refined_consensus", self.state_qos)
        self.pose_pub = self.create_publisher(PoseStamped, "/lane_learning_gnss/pose", 5)
        self.route_pub = self.create_publisher(PathMessage, "/lane_learning_gnss/route", self.state_qos)
        subscriptions = [
            (Frame, "can_topic", self._can), (Imu, "gyro_topic", self._gyro),
            (Image, "mask_topic", self._mask), (CameraInfo, "camera_info_topic", self._camera),
        ]
        for message_type, topic, callback in subscriptions:
            self.create_subscription(message_type, self.p[topic], callback, self.sensor_qos,
                                     callback_group=self.group)
        self.create_subscription(String, "/lane_learning_gnss/teacher_state", self._teacher_state,
                                 self.state_qos, callback_group=self.group)
        self.create_subscription(Bool, "/lane_learning_gnss/emergency_stop", self._manual_stop,
                                 self.state_qos, callback_group=self.group)
        for name, callback in (
                ("capture_start", self._capture_start), ("begin_teach", self._begin_teach),
                ("arm_teach", self._arm),
                ("finish_lap", self._finish), ("prepare_repeat", self._prepare),
                ("start_repeat", self._repeat), ("stop", self._stop),
                ("estop", self._estop), ("reset_estop", self._reset)):
            self.create_service(Trigger, "~/" + name, callback, callback_group=self.group)
        self.last_tick_ns = time.monotonic_ns()
        self.last_state_ns = 0
        self.create_timer(0.05, self._tick, callback_group=self.group,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._event("started", gnss_policy="start_and_end_only", input_provenance={
            "motion": "Honda RPM bytes + raw gyro angular_velocity only",
            "heading": "integrated local heading; not ENU",
            "gnss": "GpsGroup FIX|POSLLA|POSU; endpoint windows only"})

    def _validate(self):
        if self.p["map_frame"] != "lane_teach_local":
            raise ValueError("bundle coordinate frame is lane_teach_local")
        for name in ("maximum_speed_mps", "maximum_yaw_rate_rps", "wheel_diameter_m",
                     "max_gnss_horizontal_sigma_m", "max_endpoint_distance_m", "mask_max_age_s",
                     "pose_mask_gap_s"):
            value = float(self.p[name])
            if not math.isfinite(value) or value <= 0:
                raise ValueError("invalid positive parameter: " + name)
        if not math.isfinite(float(self.p["gyro_bias_radps"])):
            raise ValueError("invalid gyro bias")
        mounting = np.asarray(self.p["gyro_mount_quaternion"], dtype=float)
        if (mounting.shape != (4,) or not np.isfinite(mounting).all()
                or (np.any(mounting) and abs(np.linalg.norm(mounting) - 1.0) > 1e-5)):
            raise ValueError("gyro_mount_quaternion must be unit xyzw, or all-zero for TF calibration")
        if self.p["mask_max_age_s"] > 0.5 or self.p["pose_mask_gap_s"] > 0.2:
            raise ValueError("mask freshness cannot be disabled")
        output = self.p["command_output_topic"]
        if not isinstance(output, str) or not output.startswith("/"):
            raise ValueError("output topic must be absolute")
        if output != "/lane_learning_gnss/cmd_vel" and not all(self.p[x] for x in (
                "enable_vehicle_output", "hardware_stop_verified", "motor_zero_passthrough_verified")):
            raise ValueError("real output requires all three explicit deployment confirmations")
        if output == "/lane_learning/lya_cmd":
            raise ValueError("output cannot alias teacher input")
        self.route_config = json.loads(self.p["route_config_json"])
        if not isinstance(self.route_config, dict):
            raise ValueError("route_config_json must be an object")

    def _times(self):
        return self.get_clock().now().nanoseconds, time.monotonic_ns()

    def _event(self, event, **values):
        self.journal.write(finite_json(dict(event=event, **values)))

    def _stopped(self, now, steady):
        p = self.last_motion
        return (p is not None and fresh(p.stamp_ns, p.received_steady_ns, now, steady, 0.2)
                and p.speed <= self.safety.config["stopped_speed_mps"]
                and self.safety.motion_stopped_since_ns is not None
                and steady - self.safety.motion_stopped_since_ns >=
                    int(self.safety.config["stopped_settle_s"] * 1e9))

    def _close_gnss(self):
        if self.gnss_subscription is not None:
            self.destroy_subscription(self.gnss_subscription)
            self.gnss_subscription = None
            self._event("gnss_subscription_destroyed", phase=self.anchors.phase)

    def _subscribe_gnss(self):
        if self.gnss_subscription is not None:
            raise ValueError("GNSS window already open")
        self.gnss_subscription = self.create_subscription(GpsGroup, self.p["gnss_topic"],
            self._fix, QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT),
            callback_group=self.group)
        self._event("gnss_window_opened", phase=self.anchors.phase)

    def _fault(self, reason):
        self.safety.stop(reason)
        self.anchors.cancel_window(reason)
        self._close_gnss()
        self._send(Command())
        self._event("hold", reason=reason)

    def _motion_fault(self, reason):
        if self.session_motion_invalid:
            return
        if not self.session_started and self.start_reference_motion is not None:
            self.start_reference_usable = False
        if self.session_started:
            self.session_motion_invalid = True
            self.teach_enabled = False
            self.repeat_prepared = False
            reason = "motion continuity lost; restart and teach new lap: " + reason
        self._fault(reason)

    def _gnss_reject(self, reason):
        # GNSS has authority only over the repeat-start gate, never over map
        # completion, geometry, odometry, map generation or teach permission.
        self.anchors.cancel_window(reason)
        self.safety.close_anchor_window(reason)
        self._close_gnss()
        self._event("repeat_start_gnss_unavailable", reason=str(reason), map_affected=False)

    def _gyro_mount(self, frame, gyro_source):
        configured = np.asarray(self.p["gyro_mount_quaternion"], dtype=float)
        if np.any(configured):
            return make_transform([0, 0, 0], configured)[:3, :3]
        if frame == self.p["base_frame"]:
            return np.eye(3)
        transform = self.tf_buffer.lookup_transform(self.p["base_frame"], frame, Time())
        stamp = source_ns(transform)
        if stamp > gyro_source:
            if self.gyro_rotation is not None:
                # Do not consume newer calibration. The frozen past mounting
                # remains valid; a later causal sample will check it again.
                return self.gyro_rotation
            raise ValueError("waiting for already-arrived mounting transform at/before gyro")
        t, q = transform.transform.translation, transform.transform.rotation
        matrix = make_transform([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
        if stamp == 0:
            return matrix[:3, :3]
        # ZED publishes its fixed factory IMU mounting on /tf with a nonzero
        # stamp. Validate repeated identical mounting, NEVER use its attitude.
        if self.mount_candidate is None:
            self.mount_candidate = matrix
        if not np.allclose(matrix, self.mount_candidate, atol=1e-7, rtol=0):
            raise ValueError("IMU mounting transform changed; not a fixed extrinsic")
        self.mount_stamps.add(stamp)
        if len(self.mount_stamps) < 3:
            raise ValueError("waiting for three distinct identical IMU mounting calibrations")
        self.mount_stamps = set(sorted(self.mount_stamps)[-3:])
        return matrix[:3, :3]

    @serialized
    def _gyro(self, message):
        if self.session_motion_invalid:
            return
        now, steady = self._times()
        try:
            stamp = source_ns(message)
            if not fresh(stamp, steady, now, steady, 0.2, future_tolerance_s=0.0):
                raise ValueError("raw gyro source stale/future")
            frame = message.header.frame_id
            rotation = self._gyro_mount(frame, stamp)
            if self.gyro_rotation is None:
                self.gyro_rotation = rotation
                self.gyro_frame = frame
                self._event("gyro_mount_latched", frame=frame, rotation=rotation,
                            source_ns=stamp, orientation_field_used=False)
            if frame != self.gyro_frame or not np.allclose(rotation, self.gyro_rotation, atol=1e-7, rtol=0):
                raise ValueError("gyro frame or mounting changed")
            rates = np.array([message.angular_velocity.x, message.angular_velocity.y,
                              message.angular_velocity.z], dtype=float)
            if not np.isfinite(rates).all():
                raise ValueError("nonfinite raw gyro")
            # Deliberately NEVER inspect message.orientation or acceleration.
            yaw_rate = float((self.gyro_rotation @ rates)[2]) - self.p["gyro_bias_radps"]
            self.motion.accept_gyro(stamp, steady, yaw_rate)
            self.last_gyro_ns = stamp
        except Exception as error:
            self._motion_fault("raw gyro rejected: " + str(error))

    @serialized
    def _can(self, message):
        if self.session_motion_invalid:
            return
        if message.id != 1809:
            return  # Other CAN IDs are not wheel measurements.
        now, steady = self._times()
        try:
            left, right = decode_honda_rpm(message.id, message.data, message.dlc,
                                           message.is_rtr, message.is_extended, message.is_error)
            sample = self.motion.accept_wheels(source_ns(message), steady, left, right, now,
                                               now_steady_ns=steady)
            self.last_motion = sample
            pose = Pose(sample.x, sample.y, sample.yaw, sample.speed, sample.stamp_ns,
                        sample.received_steady_ns)
            if not self.safety.update_pose(pose, now, steady):
                raise ValueError("runtime pose admission failed")
            self.pose_buffer.append(sample)
            self.pose_pub.publish(self._pose_message(self.safety.arbiter.pose or pose))
            self._event("motion", source_ns=sample.stamp_ns, gyro_source_ns=sample.gyro_stamp_ns,
                        x=sample.x, y=sample.y, yaw=sample.yaw, speed=sample.speed)
        except Exception as error:
            self.last_motion = None
            self.pose_buffer.clear()
            self._motion_fault("wheel/gyro rejected: " + str(error))

    def _static_transform(self, frame):
        if not frame:
            raise ValueError("empty sensor frame")
        if frame == self.p["base_frame"]:
            return np.eye(4)
        transform = self.tf_buffer.lookup_transform(self.p["base_frame"], frame, Time())
        if source_ns(transform) != 0:
            raise ValueError("sensor transform must be static and already received")
        t, q = transform.transform.translation, transform.transform.rotation
        return make_transform([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])

    @serialized
    def _camera(self, message):
        try:
            matrix = list(message.k)
            if (message.width * message.height > 2000000 or min(message.width, message.height) < 2
                    or not all(math.isfinite(v) for v in matrix)
                    or message.distortion_model not in ("", "plumb_bob", "rational_polynomial")):
                raise ValueError("invalid camera calibration")
            value = dict(width=int(message.width), height=int(message.height),
                         camera_matrix=matrix, frame_id=message.header.frame_id)
            if self.camera is not None and value != self.camera[0]:
                raise ValueError("calibration changed; restart/new lap required")
            if self.camera is None:
                self.camera = value, source_ns(message)
        except Exception as error:
            self._fault(str(error))

    @serialized
    def _mask(self, message):
        if not self.teach_enabled and not self.repeat_prepared:
            return
        if self.finish_pending or self.bundle_pending:
            return
        now, steady = self._times()
        try:
            stamp = source_ns(message)
            if stamp <= self.last_mask_ns:
                raise ValueError("mask time regression")
            self.last_mask_ns = stamp
            if not fresh(stamp, steady, now, steady, self.p["mask_max_age_s"], 0.0):
                raise ValueError("mask stale or future")
            if not self.camera or self.camera[1] > stamp:
                raise ValueError("no already-arrived causal CameraInfo")
            camera = self.camera[0]
            if (message.width != camera["width"] or message.height != camera["height"]
                    or message.header.frame_id != camera["frame_id"]):
                raise ValueError("mask/calibration frame or size mismatch")
            if self.calibration is None:
                self.calibration = dict(camera, base_frame=self.p["base_frame"],
                    base_from_camera=self._static_transform(camera["frame_id"]).tolist(),
                    motion={"wheel_diameter_m": self.p["wheel_diameter_m"],
                            "gyro_bias_radps": self.p["gyro_bias_radps"],
                            "gyro_mount_rotation": self.gyro_rotation.tolist(),
                            "gyro_frame": self.gyro_frame, "gnss_used": False,
                            "orientation_field_used": False})
            if message.encoding not in ("mono8", "8UC1") or message.step < message.width:
                raise ValueError("full mono8 road detector mask required")
            expected = int(message.height) * int(message.step)
            if expected > 4000000 or len(message.data) != expected:
                raise ValueError("invalid/oversize mask buffer")
            prior = next((p for p in reversed(self.pose_buffer) if p.stamp_ns <= stamp), None)
            if prior is None or (stamp - prior.stamp_ns) * 1e-9 > self.p["pose_mask_gap_s"]:
                raise ValueError("no already-arrived wheel/gyro pose at or before mask")
            mask = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
            job = dict(kind="repeat" if self.repeat_prepared else "teach",
                       generation=self.safety.generation, stamp_ns=stamp, receipt_ns=steady,
                       prior=[prior.x, prior.y, prior.yaw], prior_stamp_ns=prior.stamp_ns,
                       mask=mask[:, :message.width].copy(), calibration=self.calibration)
            self.worker.submit(job)
        except Exception as error:
            if self.repeat_prepared:
                self.safety.visual_reject(str(error))
            self._event("mask_rejected", reason=str(error))

    @contextmanager
    def _commit_guard(self, job):
        with self.lock:
            now, steady = self._times()
            phase_allowed = (self.repeat_prepared if job["kind"] == "repeat" else
                             self.teach_enabled and not self.finish_pending and not self.bundle_pending)
            motion = self.last_motion
            allowed = (not self.closing and not self.session_motion_invalid and phase_allowed
                and job["generation"] == self.safety.generation
                and fresh(job["stamp_ns"], job["receipt_ns"], now, steady, self.p["mask_max_age_s"], 0.0)
                and motion is not None and fresh(motion.stamp_ns, motion.received_steady_ns,
                                                now, steady, 0.2, 0.0))
            if job["kind"] == "repeat":
                allowed = allowed and self.safety.visual_deadline(job["stamp_ns"], job["receipt_ns"],
                                                                  now, steady) is not None
                allowed = allowed and self.safety.state != "ESTOP"
                if self.safety.visual_streak < self.safety.config["visual_min_consecutive_matches"]:
                    allowed = (allowed and motion is not None and motion.speed <= self.safety.config["stopped_speed_mps"]
                               and self.safety.state in ("HOLD", "DISARMED"))
                allowed = (allowed and (steady - job["compute_started_ns"]) * 1e-6
                           <= self.safety.config["maximum_match_compute_ms"])
                correction = job.get("proposal_correction")
                if correction is None or motion is None:
                    allowed = False
                else:
                    dx, dy, angle = correction
                    distance = math.hypot(math.cos(angle) * motion.x - math.sin(angle) * motion.y + dx - motion.x,
                                          math.sin(angle) * motion.x + math.cos(angle) * motion.y + dy - motion.y)
                    allowed = (allowed and distance <= self.safety.config["visual_max_total_correction_m"]
                               and abs(angle) <= self.safety.config["visual_max_total_yaw_rad"])
            yield bool(allowed)

    @serialized
    def _fix(self, message):
        now, steady = self._times()
        if self.gnss_subscription is None or self.anchors.phase not in ("start_window", "end_window"):
            return  # A DDS callback queued before subscription destruction has no authority.
        try:
            fix = decode_vectornav_gps(message, steady)
            kind = "start" if self.anchors.phase == "start_window" else "end"
            self.anchors.observe_fix(fix, now, steady, stopped=self._stopped(now, steady))
            self._close_gnss()
            if kind == "start":
                ok, reason = self.safety.accept_start_anchor(str(fix.stamp_ns), now, steady)
                if not ok:
                    raise ValueError(reason)
                self.start_reference_motion = self.last_motion
                self.start_reference_receipt_ns = steady
                self.start_reference_usable = True
            else:
                anchors = self.anchors.snapshot()
                if anchors["ready_for_repeat"] and self.start_reference_usable:
                    ok, reason = self.safety.accept_end_anchor(str(fix.stamp_ns), now, steady,
                                                               proximity_validated=True)
                    if not ok:
                        raise ValueError(reason)
                else:
                    self.safety.close_anchor_window("repeat start proximity/association rejected")
                    self._event("repeat_start_proximity_rejected", validation=anchors["end"]["validation"],
                                reference_associated_with_local_start=self.start_reference_usable,
                                map_affected=False)
            self._event("endpoint_accepted", anchors=self.anchors.snapshot())
        except Exception as error:
            self._gnss_reject(str(error))

    @serialized
    def _teacher(self, message):
        now, steady = self._times()
        self.safety.teacher_command([message.linear.x, message.linear.y, message.linear.z,
                                     message.angular.x, message.angular.y, message.angular.z], now, now, steady)

    @serialized
    def _teacher_state(self, message):
        now, steady = self._times()
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError("teacher status must be an object")
            self.safety.teacher_state(value["state"], value["stamp_ns"], now, steady)
        except (ValueError, TypeError, KeyError):
            self._fault("malformed teacher heartbeat")

    @serialized
    def _manual_stop(self, message):
        if message.data:
            self.safety.estop("independent manual stop asserted", manual=True)
            self.anchors.cancel_window("manual emergency stop")
            self._close_gnss()
            self._send(Command())
        else:
            self.safety.release_manual_estop()

    @staticmethod
    def _reply(response, result):
        response.success, response.message = bool(result[0]), str(result[1])
        return response

    @serialized
    def _capture_start(self, request, response):
        now, steady = self._times()
        try:
            if self.safety.anchor_window(now, steady) != "start":
                raise ValueError("start-reference capture only before LYA teaching while stopped")
            self.anchors.open_start(now, steady, stopped=self._stopped(now, steady))
            self._subscribe_gnss()
            return self._reply(response, (True, "waiting for one fresh quality GNSS fix while stopped"))
        except Exception as error:
            return self._reply(response, (False, str(error)))

    @serialized
    def _begin_teach(self, request, response):
        now, steady = self._times()
        if self.session_started or self.session_motion_invalid:
            return self._reply(response, (False, "teach session already initialized or invalid"))
        if not self._stopped(now, steady):
            return self._reply(response, (False, "fresh stopped raw motion required before teach initialization"))
        reference = self.start_reference_motion
        if reference is not None:
            current = self.last_motion
            yaw_gap = math.atan2(math.sin(current.yaw - reference.yaw), math.cos(current.yaw - reference.yaw))
            associated = (math.hypot(current.x - reference.x, current.y - reference.y) <= 0.1
                          and abs(yaw_gap) <= 0.15 and steady - self.start_reference_receipt_ns <= 30000000000)
            self.start_reference_usable = self.start_reference_usable and associated
            self._event("repeat_start_reference_association", valid=self.start_reference_usable,
                        map_affected=False)
        result = self.safety.begin_teach_session(now, steady)
        if result[0]:
            self._close_gnss()
            self.motion.reset_origin(stopped=True)
            self.pose_buffer.clear()
            self.last_motion = None
            self.anchors.mark_teach_started()
            self.session_started = True
            self.session_start_clocks = (now, steady)
            self._event("map_session_started", gnss_used_for_mapping=False)
        return self._reply(response, result)

    @serialized
    def _arm(self, request, response):
        now, steady = self._times()
        if self.session_motion_invalid:
            return self._reply(response, (False, "motion continuity lost; restart/new lap required"))
        if not self.session_started or self.gnss_subscription is not None or self.camera is None:
            return self._reply(response, (False, "begin_teach and camera required; GNSS must be disconnected"))
        result = self.safety.arm_teach(now, steady)
        if result[0]:
            self.teach_enabled = True
        return self._reply(response, result)

    @serialized
    def _finish(self, request, response):
        now, steady = self._times()
        if self.session_motion_invalid or self.finish_pending or self.bundle_pending or not self.teach_enabled:
            return self._reply(response, (False, "not an active unfinished teach session"))
        result = self.safety.begin_end_anchor(now, steady)
        if result[0]:
            self.finish_pending = True
            self.teach_enabled = False
            self.worker.submit(dict(kind="freeze", generation=self.safety.generation))
            self._send(Command())
        return self._reply(response, result)

    @serialized
    def _prepare(self, request, response):
        now, steady = self._times()
        if self.session_motion_invalid:
            return self._reply(response, (False, "motion continuity lost; restart/new lap required"))
        result = self.safety.prepare_repeat(now, steady)
        if result[0]:
            self.repeat_prepared = True
            self.last_mask_ns = 0
            self._send(Command())
        return self._reply(response, result)

    @serialized
    def _repeat(self, request, response):
        if self.session_motion_invalid:
            return self._reply(response, (False, "motion continuity lost; restart/new lap required"))
        return self._reply(response, self.safety.start_repeat(*self._times()))

    @serialized
    def _stop(self, request, response):
        self._fault("operator stop; no automatic restart")
        return self._reply(response, (True, "HOLD"))

    @serialized
    def _estop(self, request, response):
        self.safety.estop("software emergency stop")
        self.anchors.cancel_window("software emergency stop")
        self._close_gnss()
        self._send(Command())
        return self._reply(response, (True, "ESTOP latched"))

    @serialized
    def _reset(self, request, response):
        return self._reply(response, self.safety.reset_estop())

    def _consume(self, now, steady):
        for _ in range(4):
            envelope = self.worker.take()
            if envelope is None:
                return
            job, kind = envelope["job"], envelope["kind"]
            if job["generation"] != self.safety.generation:
                self._event("worker_result_revoked", operation=job["kind"])
                continue
            if kind == "error":
                self.safety.stop("map worker: " + envelope["error"])
                self.bundle_pending = False
                self._event("map_worker_failed", reason=envelope["error"])
            elif kind == "freeze":
                # Start geometry-only bundle computation now, independently of
                # the separate optional GNSS window for repeat-start proximity.
                self.finish_pending = False
                self.bundle_pending = True
                self.worker.submit(dict(kind="bundle", root=str(self.journal.directory / "maps"),
                    route_config=self.route_config, generation=self.safety.generation))
                try:
                    self.anchors.open_end(now, steady, stopped=self._stopped(now, steady))
                    self._subscribe_gnss()
                except Exception as error:
                    self._gnss_reject(str(error))
            elif kind == "bundle":
                controller = ClosedRouteController(envelope["route"],
                    maximum_speed=self.p["maximum_speed_mps"], maximum_yaw_rate=self.p["maximum_yaw_rate_rps"])
                result = self.safety.set_ready(controller, envelope["path"], now, steady)
                if not result[0]:
                    self._fault(result[1])
                    continue
                self.bundle_path = envelope["path"]
                self.bundle_pending = False
                self._publish_route(envelope["route"])
                self._event("bundle_ready", path=self.bundle_path)
            elif kind == "repeat":
                result = envelope["result"]
                activated = False
                if envelope["committed"]:
                    activated = self.safety.visual_accept(job["stamp_ns"], job["receipt_ns"], job["generation"],
                        now, steady, correction_se2=result.correction_se2,
                        compute_ms=envelope["compute_ms"])
                else:
                    self.safety.visual_reject(result.reason if not result.accepted else "commit revoked")
                self._event("repeat_mask", committed=envelope["committed"], activated=activated, reason=result.reason,
                            source_ns=job["stamp_ns"], metrics=result.metrics, compute_ms=envelope["compute_ms"])
            elif kind == "teach":
                result = envelope["result"]
                self._event("teach_mask", result=result, source_ns=job["stamp_ns"],
                            prior_source_ns=job["prior_stamp_ns"], compute_ms=envelope["compute_ms"])
            if "map" in envelope:
                points, votes = envelope["map"]
                header = Header()
                header.frame_id = self.p["map_frame"]
                header.stamp.sec, header.stamp.nanosec = divmod(job["stamp_ns"], 1000000000)
                publisher = self.refined_pub if kind == "repeat" else self.map_pub
                publisher.publish(point_cloud(header, points, votes))

    @serialized
    def _tick(self):
        now, steady = self._times()
        dt = (steady - self.last_tick_ns) * 1e-9
        self.last_tick_ns = steady
        try:
            if (self.session_started and self.last_motion is None and not self.session_motion_invalid
                    and not fresh(self.session_start_clocks[0], self.session_start_clocks[1],
                                  now, steady, 0.2, 0.0)):
                self._motion_fault("no post-begin motion arrived before startup deadline")
            if self.session_started and self.last_motion is not None:
                if not fresh(self.last_motion.stamp_ns, self.last_motion.received_steady_ns,
                             now, steady, 0.2, 0.0) and not self.session_motion_invalid:
                    self._motion_fault("wheel/gyro input expired")
            if self.anchors.phase in ("start_window", "end_window"):
                try:
                    self.anchors.check_window(now, steady, stopped=self._stopped(now, steady))
                except Exception as error:
                    self._gnss_reject(str(error))
            if self.worker.failure or self.journal.failed:
                raise ValueError(self.worker.failure or self.journal.failed)
            self._consume(now, steady)
        except Exception as error:
            self._fault("runtime check: " + str(error))
        self._send(self.safety.tick(now, steady, dt))
        if self.anchors.phase not in ("start_window", "end_window"):
            self._close_gnss()
        if steady - self.last_state_ns >= 200000000:
            state = dict(self.safety.snapshot(), stamp_ns=now, gnss=self.anchors.snapshot(),
                         repeat_start_reference_usable=self.start_reference_usable,
                         session_motion_invalid=self.session_motion_invalid,
                         gnss_subscription_active=self.gnss_subscription is not None,
                         bundle_path=self.bundle_path, log_directory=str(self.journal.directory))
            self.state_pub.publish(String(data=json.dumps(finite_json(state), allow_nan=False)))
            self.supervisor_pub.publish(String(data=json.dumps(dict(stamp_ns=now,
                safety_mode="fixed_only", teacher_enabled=self.safety.teacher_enabled))))
            self.last_state_ns = steady

    def _pose_message(self, pose):
        message = PoseStamped()
        message.header.frame_id = self.p["map_frame"]
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(pose.stamp_ns, 1000000000)
        message.pose.position.x, message.pose.position.y = float(pose.x), float(pose.y)
        message.pose.orientation.z = math.sin(pose.yaw / 2)
        message.pose.orientation.w = math.cos(pose.yaw / 2)
        return message

    def _publish_route(self, route):
        path = PathMessage()
        path.header.frame_id = self.p["map_frame"]
        for sample in route["route_samples"]:
            path.poses.append(self._pose_message(Pose(sample["x_m"], sample["y_m"],
                sample["yaw_rad"], 0.0, self.get_clock().now().nanoseconds, time.monotonic_ns())))
        self.route_pub.publish(path)

    def _send(self, command):
        message = Twist()
        message.linear.x, message.angular.z = float(command.speed), float(command.yaw_rate)
        self.command_pub.publish(message)
        self._event("command", state=self.safety.state, speed=message.linear.x, yaw_rate=message.angular.z,
                    gnss_subscription_active=self.gnss_subscription is not None)

    def close(self):
        with self.lock:
            self.closing = True
            self.safety.stop("shutdown")
            self.anchors.cancel_window("shutdown")
            self._close_gnss()
            self._send(Command())
        self.worker.close()
        self.journal.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EndpointFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
