"""Single command owner. No node starts armed and no service bypasses estop."""
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import queue
import threading
import time
import uuid
from datetime import datetime, timezone

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path as NavPath
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger
from vectornav_msgs.msg import CommonGroup
from trajectory_follower.lya_profile import REFERENCE_SPEED_MPS, MAX_YAW_RATE_RPS

from lane_mapping_lya_reference.mapping_core import (
    CausalSampleBuffer, GroundLookup, VectorNavLocalizer,
)
from .mask_localization import LaneMapLocalizer
from .controller import (ClosedRouteController, Command, Pose, SafetyArbiter,
                         fresh, load_bundle)


def stamp_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def finite_json(value):
    """Rejections may contain infinite residuals; logs must remain strict JSON."""
    if isinstance(value, dict):
        return {str(key): finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [finite_json(item) for item in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def decode_repeat_mask(message):
    if message.encoding not in ("mono8", "8UC1"):
        raise ValueError("mask must be mono8 or 8UC1")
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width < 2 or height < 2 or step < width:
        raise ValueError("invalid mask dimensions")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < height * step:
        raise ValueError("truncated mask buffer")
    return raw[:height * step].reshape(height, step)[:, :width]


def load_repeat_context(bundle_path, manifest, metadata, params):
    """Use only the calibration and consensus protected by the bundle hashes."""
    root = Path(bundle_path).expanduser().resolve(strict=True).parent
    consensus_path = (root / manifest["metadata_path"]).parent / "consensus.csv"
    consensus_path = consensus_path.resolve(strict=True)
    try:
        consensus_path.relative_to(root)
    except ValueError:
        raise ValueError("consensus path escapes bundle directory")
    if consensus_path.stat().st_size > 32 * 1024 * 1024:
        raise ValueError("consensus file exceeds size limit")
    raw = consensus_path.read_bytes()
    if len(raw) > 32 * 1024 * 1024:
        raise ValueError("consensus file exceeds size limit")
    hashes = metadata.get("data_sha256")
    if not isinstance(hashes, dict):
        raise ValueError("bundle lacks consensus hash metadata")
    expected = hashes.get("consensus.csv")
    if not isinstance(expected, str) or hashlib.sha256(raw).hexdigest() != expected:
        raise ValueError("consensus.csv checksum mismatch or missing")
    reader = csv.DictReader(io.StringIO(raw.decode("utf-8")))
    if reader.fieldnames != ["east_m", "north_m", "frame_votes"]:
        raise ValueError("unsupported consensus columns")
    points, votes = [], []
    for row in reader:
        x, y, vote = float(row["east_m"]), float(row["north_m"]), int(row["frame_votes"])
        if not math.isfinite(x) or not math.isfinite(y) or vote <= 0:
            raise ValueError("invalid consensus cell")
        points.append((x, y))
        votes.append(vote)
        if len(points) > params["visual_max_anchor_points"]:
            raise ValueError("consensus exceeds visual anchor capacity")
    if len(points) < 30:
        raise ValueError("consensus has too few anchor points")
    camera, extrinsic = metadata.get("camera_calibration"), metadata.get("static_extrinsic")
    projection = metadata.get("parameters")
    if not all(isinstance(item, dict) for item in (camera, extrinsic, projection)):
        raise ValueError("bundle lacks original projection calibration")
    width, height = int(camera["width"]), int(camera["height"])
    if width < 2 or height < 2 or width * height > params["visual_max_image_pixels"]:
        raise ValueError("saved camera dimensions exceed limits")
    frame = str(camera["frame_id"])
    if not frame or extrinsic.get("child_frame") != frame or not extrinsic.get("parent_frame"):
        raise ValueError("saved calibration/extrinsic frame mismatch")
    if projection.get("base_frame") != extrinsic["parent_frame"]:
        raise ValueError("saved base frame/extrinsic mismatch")
    if (not isinstance(camera.get("distortion_model"), str)
            or not np.all(np.isfinite(np.asarray(camera.get("distortion"), dtype=float)))):
        raise ValueError("invalid saved distortion calibration")
    matrix = np.asarray(camera["matrix"], dtype=float)
    transform = np.asarray(extrinsic["matrix"], dtype=float)
    if (matrix.shape != (3, 3) or transform.shape != (4, 4)
            or not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(transform))
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-9)
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1.0, atol=1e-6)):
        raise ValueError("invalid saved camera/extrinsic matrix")
    sensitivity = float(projection["max_projection_sensitivity_m_per_px"])
    reliable_sensitivity = float(projection["max_reliable_projection_sensitivity_m_per_px"])
    threshold = int(projection["mask_threshold"])
    if (not 0 < reliable_sensitivity <= sensitivity or not math.isfinite(sensitivity)
            or not 0 <= threshold <= 255 or int(camera["stamp_ns"]) < 0):
        raise ValueError("invalid saved projection settings")
    config = json.loads(str(params["visual_matcher_config_json"]))
    if not isinstance(config, dict):
        raise ValueError("visual_matcher_config_json must be an object")
    anchor = np.asarray(points, dtype=float)
    anchor.setflags(write=False)
    lookup = GroundLookup(width, height, matrix, transform, sensitivity)
    matcher = LaneMapLocalizer(anchor, config=config)
    return dict(camera=dict(camera), lookup=lookup, matcher=matcher, anchor=anchor,
                anchor_votes=np.asarray(votes, dtype=float), threshold=threshold,
                anchor_sha256=expected, bundle_id=manifest.get("bundle_id"),
                reliable_sensitivity=reliable_sensitivity, last_refined_export_ns=0,
                refinement_publish_rate_hz=params["refinement_publish_rate_hz"])


def pack_cloud(points, votes):
    points = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    packed = np.zeros((len(points), 4), dtype="<f4")
    packed[:, :2], packed[:, 3] = points, np.asarray(votes, dtype=np.float32)
    return packed.tobytes(), len(points)


def cloud_message(source_ns, frame_id, packed):
    message = PointCloud2()
    message.header = Header()
    message.header.stamp.sec = int(source_ns // 1000000000)
    message.header.stamp.nanosec = int(source_ns % 1000000000)
    message.header.frame_id = frame_id
    message.height, message.width = 1, packed[1]
    message.is_bigendian, message.is_dense = False, True
    message.point_step, message.row_step = 16, 16 * packed[1]
    message.fields = [PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
                      for index, name in enumerate(("x", "y", "z", "frame_votes"))]
    message.data = packed[0]
    return message


class VisualCommitFence:
    """Serialize invalidation with the small matcher commit, never the ICP fit."""
    def __init__(self, clock=None):
        self.lock = threading.Lock()
        self.clock = clock or time.monotonic_ns
        self.generation = 0
        self.token = None
        self.closed = False

    def advance(self):
        with self.lock:
            self.generation += 1
            self.token = None
            return self.generation

    def authorize(self, job_id, generation, issued_ns, deadline_ns):
        with self.lock:
            if (self.closed or generation != self.generation or issued_ns is None
                    or deadline_ns is None or deadline_ns < issued_ns):
                return None
            self.token = dict(job_id=job_id, generation=generation,
                              issued_ns=issued_ns, deadline_ns=deadline_ns)
            return self.token

    def commit(self, token, job, matcher, result):
        with self.lock:
            if (self.closed or token is None or token is not self.token
                    or job["generation"] != self.generation
                    or token["generation"] != job["generation"]
                    or token["job_id"] != job["job_id"]):
                raise ValueError("visual commit authorization invalidated")
            self.token = None  # One use even when the deadline or matcher rejects.
            if not token["issued_ns"] <= self.clock() <= token["deadline_ns"]:
                raise ValueError("visual commit authorization expired")
            if not matcher.commit(result):
                raise ValueError("matcher rejected acknowledged proposal")

    def close(self):
        with self.lock:
            self.closed = True
            self.token = None


class LatestMaskWorker:
    """One compute job and one latest pending frame; no ROS/control mutation.

    The matcher is owned by this thread. A proposal is committed only after the
    control thread acknowledges that its generation, source time and pose are
    still valid. A shared fence rechecks/revokes that ACK at the actual commit.
    Until that ACK, later masks only replace the pending frame.
    """
    def __init__(self, commit_fence=None):
        self.commit_fence = commit_fence or VisualCommitFence()
        self.jobs, self.results, self.acks = queue.Queue(1), queue.Queue(2), queue.Queue(1)
        self.stopping = threading.Event()
        self.latest_refined = None
        self.last_committed_job = None
        self.thread = threading.Thread(target=self._run, name="mask-localization", daemon=True)
        self.thread.start()

    @staticmethod
    def _replace(target, value):
        replaced = False
        try:
            target.put_nowait(value)
        except queue.Full:
            try:
                target.get_nowait()
                replaced = True
            except queue.Empty:
                pass
            target.put_nowait(value)
        return replaced

    def submit(self, job):
        return self._replace(self.jobs, job)

    def acknowledge(self, job_id, accepted, generation=None,
                    issued_steady_ns=None, deadline_steady_ns=None):
        token = self.commit_fence.authorize(job_id, generation, issued_steady_ns,
                                            deadline_steady_ns) if accepted else None
        self._replace(self.acks, (job_id, token))

    def take_result(self):
        try:
            return self.results.get_nowait()
        except queue.Empty:
            return None

    def _run(self):
        while not self.stopping.is_set():
            try:
                job = self.jobs.get(timeout=0.05)
            except queue.Empty:
                continue
            context = job["context"]
            started = time.perf_counter_ns()
            result, error, counts = None, None, None
            try:
                mask = decode_repeat_mask(job["message"])
                local, sensitivity, counts = context["lookup"].project_mask_with_sensitivity(
                    mask, context["threshold"])
                # Full image with the saved metric uncertainty gate, not the
                # lane-publisher ROI. The matcher separately bounds workload.
                reliable = local[sensitivity <= context["reliable_sensitivity"]]
                result = context["matcher"].match(reliable, job["prior"], job["source_ns"],
                    prior_stamp_ns=job["prior_stamp_ns"], commit=False)
            except Exception as failure:
                error = str(failure)
            envelope = dict(kind="proposal", job=job, result=result, error=error,
                            compute_ms=(time.perf_counter_ns() - started) * 1e-6,
                            projected_pixels=counts.stable_pixels if counts else 0)
            self._replace(self.results, envelope)
            token = None
            while not self.stopping.is_set():
                try:
                    identity, decision = self.acks.get(timeout=0.05)
                except queue.Empty:
                    continue
                if identity == job["job_id"]:
                    token = decision
                    break
            if token is None or result is None:
                continue
            try:
                self.commit_fence.commit(token, job, context["matcher"], result)
                self.last_committed_job = job
                committed = dict(kind="committed", job=job, result=result)
                now = time.monotonic_ns()
                if now - context["last_refined_export_ns"] >= int(1e9 / context["refinement_publish_rate_hz"]):
                    xy, votes = context["matcher"].refined_points_and_votes()
                    snapshot = dict(kind="refinement", generation=job["generation"],
                                    source_ns=job["source_ns"], packed=pack_cloud(xy, votes))
                    self.latest_refined = snapshot
                    context["last_refined_export_ns"] = now
                    committed["refinement"] = snapshot
                self._replace(self.results, committed)
            except Exception as failure:
                self._replace(self.results, dict(kind="commit_failed", generation=job["generation"],
                                                error=str(failure)))

    def close(self):
        self.commit_fence.close()
        self.stopping.set()
        self.thread.join(timeout=2.0)
        if not self.thread.is_alive() and self.last_committed_job is not None:
            job = self.last_committed_job
            xy, votes = job["context"]["matcher"].refined_points_and_votes()
            self.latest_refined = dict(kind="refinement", generation=job["generation"],
                source_ns=job["source_ns"], packed=pack_cloud(xy, votes))


class RunJournal:
    """Bounded queue + rotating JSONL. Backpressure fails closed, never silent loss."""
    def __init__(self, root, max_bytes=5 * 1024 * 1024, max_files=4):
        label = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S") + "-" + uuid.uuid4().hex[:8]
        self.directory = Path(root).expanduser() / ("follower-" + label)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.max_bytes, self.max_files = max_bytes, max_files
        self.queue = queue.Queue(maxsize=256)
        self.failed = None
        self.closing = threading.Event()
        self.thread = threading.Thread(target=self._run, name="command-log", daemon=True)
        self.thread.start()

    def write(self, item):
        if self.failed:
            return
        try:
            self.queue.put_nowait(dict(wall_time_ns=time.time_ns(), **item))
        except queue.Full:
            self.failed = "command journal queue full"

    def _run(self):
        stream = None
        try:
            index, size = 0, 0
            stream = (self.directory / "commands-0.jsonl").open("x", encoding="utf-8")
            while not self.closing.is_set() or not self.queue.empty():
                try:
                    item = self.queue.get(timeout=0.05)
                except queue.Empty:
                    continue
                encoded = json.dumps(item, allow_nan=False, separators=(",", ":")) + "\n"
                if size + len(encoded.encode("utf-8")) > self.max_bytes:
                    stream.close()
                    index += 1
                    # Only files created in this unique run directory are retired.
                    if index >= self.max_files:
                        (self.directory / ("commands-{}.jsonl".format(index - self.max_files))).unlink()
                    stream = (self.directory / ("commands-{}.jsonl".format(index))).open("x", encoding="utf-8")
                    size = 0
                stream.write(encoded)
                stream.flush()
                size += len(encoded.encode("utf-8"))
        except Exception as error:
            self.failed = "command journal failed: " + str(error)
        finally:
            if stream is not None:
                stream.close()

    def close(self):
        self.closing.set()
        self.thread.join(timeout=2.0)


class FixedFollower(Node):
    def __init__(self, forced_safety_mode=None):
        super().__init__("lane_fixed_follower")
        defaults = {
            "safety_mode": "fixed_only",
            "command_output_topic": "/lane_learning/cmd_vel",
            "teacher_command_topic": "/lane_learning/lya_cmd",
            "vectornav_topic": "/aiformula_sensing/vectornav/raw/common",
            "mask_topic": "/aiformula_perception/road_detector/mask_image",
            "camera_frame_override": "",
            "visual_max_age_s": 0.5,
            "visual_max_pose_mask_gap_s": 0.15,
            "visual_min_consecutive_matches": 3,
            "visual_vn_buffer_size": 512,
            "visual_max_anchor_points": 100000,
            "visual_max_image_pixels": 2500000,
            "visual_matcher_config_json": "{}",
            "refinement_publish_rate_hz": 1.0,
            "emergency_stop_topic": "/lane_learning/emergency_stop",
            "emergency_stop_transient_local": False,
            "ready_bundle_topic": "/lane_learning/route_bundle",
            "route_bundle_path": "",
            "log_directory": "./lane_learning_logs",
            "control_rate_hz": 20.0,
            "reference_speed_mps": REFERENCE_SPEED_MPS,
            "maximum_speed_mps": 0.0,  # inherit the effective LYA reference
            "accepted_teacher_max_speed_mps": 0.0,
            "maximum_yaw_rate_rps": 0.4,
            "accepted_teacher_max_yaw_rate_rps": MAX_YAW_RATE_RPS,
            "maximum_acceleration_mps2": 0.5,
            "maximum_yaw_acceleration_rps2": 0.8,
            "maximum_lateral_acceleration_mps2": 0.35,
            "maximum_cross_track_m": 0.8,
            "maximum_heading_error_rad": 1.0,
            "start_distance_m": 1.0,
            "start_heading_error_rad": 0.6,
            "pose_timeout_s": 0.25,
            "teacher_timeout_s": 0.25,
            "stopped_speed_mps": 0.08,
            "stopped_settle_s": 1.0,
            "reference_speed_delta_mps": 0.35,
            "reference_yaw_delta_rps": 0.15,
            "reference_recovery_s": 0.5,
            "yaw_offset_rad": 0.0,
            "position_jump_tolerance_m": 0.35,
            "vectornav_maximum_speed_mps": 8.0,
            "enable_vehicle_output": False,
            "motor_zero_passthrough_verified": False,
            "hardware_stop_verified": False,
            "reject_competing_publishers": True,
            "require_managed_teacher_stop": True,
            "teacher_state_topic": "/lane_learning/teacher_state",
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if self.p["maximum_speed_mps"] == 0.0:
            self.p["maximum_speed_mps"] = self.p["reference_speed_mps"]
        if forced_safety_mode is not None:
            self.p["safety_mode"] = forced_safety_mode
        for key in defaults:
            if isinstance(defaults[key], float):
                number = float(self.p[key])
                nonnegative = key == "accepted_teacher_max_speed_mps"
                if (not math.isfinite(number) or (nonnegative and number < 0)
                        or (not nonnegative and key != "yaw_offset_rad" and number <= 0)):
                    raise ValueError("invalid parameter " + key)
        if not 5 <= self.p["control_rate_hz"] <= 100:
            raise ValueError("control_rate_hz must be between 5 and 100")
        if self.p["maximum_speed_mps"] < self.p["reference_speed_mps"]:
            raise ValueError("repeat speed cap cannot be below the fixed reference speed")
        for key in ("visual_min_consecutive_matches", "visual_vn_buffer_size",
                    "visual_max_anchor_points", "visual_max_image_pixels"):
            if isinstance(self.p[key], bool) or not isinstance(self.p[key], int) or self.p[key] <= 0:
                raise ValueError("invalid positive integer parameter " + key)
        if (self.p["visual_min_consecutive_matches"] < 2 or self.p["visual_vn_buffer_size"] > 4096
                or self.p["visual_max_anchor_points"] > 100000 or self.p["visual_max_image_pixels"] > 4000000):
            raise ValueError("visual capacity or confirmation count exceeds safety limits")
        self.output_topic = str(self.p["command_output_topic"])
        if not self.output_topic.startswith("/"):
            raise ValueError("command output topic must be absolute")
        if self.output_topic == self.p["teacher_command_topic"]:
            raise ValueError("command output must not be teacher input")
        self.arbiter = SafetyArbiter(
            maximum_speed=self.p["maximum_speed_mps"],
            maximum_yaw_rate=self.p["maximum_yaw_rate_rps"],
            pose_timeout_s=self.p["pose_timeout_s"],
            teacher_timeout_s=self.p["teacher_timeout_s"],
            stopped_speed=self.p["stopped_speed_mps"],
            stopped_settle_s=self.p["stopped_settle_s"],
            safety_mode=self.p["safety_mode"],
            reference_speed_delta=self.p["reference_speed_delta_mps"],
            reference_yaw_delta=self.p["reference_yaw_delta_rps"],
            reference_recovery_s=self.p["reference_recovery_s"],
            accepted_teacher_max_speed=self.p["accepted_teacher_max_speed_mps"],
            accepted_teacher_max_yaw_rate=self.p["accepted_teacher_max_yaw_rate_rps"],
            maximum_acceleration=self.p["maximum_acceleration_mps2"],
            maximum_yaw_acceleration=self.p["maximum_yaw_acceleration_rps2"],
            preserve_teacher_command=True,
        )
        self.localizer = self._new_localizer()
        self.journal = RunJournal(self.p["log_directory"])
        self.ready_path = str(self.p["route_bundle_path"])
        self.manifest = None
        self.last_tick_ns = time.monotonic_ns()
        self.last_ros_ns = None
        self.last_graph_check_ns = 0
        self.competing_publishers = False
        self.last_state = None
        self.pose_error = None
        self.arrival_seq = 0
        self.teacher_stopped = False
        self.teacher_state_received_ns = 0
        self.teacher_state_source_ns = 0
        self.repeat_prepare_ns = 0
        self.repeat_prepare_ros_ns = 0
        self.repeat_context = None
        self.visual_commit_fence = VisualCommitFence()
        self.visual_generation = self.visual_commit_fence.generation
        self.visual_job_id = 0
        self.visual_last_mask_stamp = None
        self.visual_last_accepted_stamp = None
        self.visual_last_arrival_steady_ns = None
        self.visual_streak = 0
        self.visual_correction = np.zeros(3, dtype=float)
        self.visual_metrics = {}
        self.visual_reason = "no saved map loaded"
        self.visual_last_rejection = None
        self.visual_pending_replacements = 0
        self.visual_last_refined = None
        self.raw_pose = None
        self.pose_buffer = CausalSampleBuffer(self.p["visual_vn_buffer_size"])
        sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        reliable_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        durable_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Twist, self.output_topic, reliable_qos)
        # Resolve ROS remappings: an apparently private configured name may be
        # remapped to the real actuator topic, and must not bypass the guard.
        self.output_topic = self.publisher.topic_name
        if self.output_topic != "/lane_learning/cmd_vel" and not all(self.p[name] for name in (
                "enable_vehicle_output", "motor_zero_passthrough_verified", "hardware_stop_verified")):
            self.journal.close()
            raise ValueError("refusing all vehicle-bus writes: motor zero and physical stop must be verified")
        if self.output_topic != "/lane_learning/cmd_vel" and self.p["accepted_teacher_max_speed_mps"] <= 0:
            self.journal.close()
            raise ValueError("vehicle output requires an explicitly approved teacher speed ceiling")
        self.state_publisher = self.create_publisher(String, "~/control_state", durable_qos)
        self.teacher_enabled_publisher = self.create_publisher(Bool, "~/teacher_enabled", durable_qos)
        self.pose_publisher = self.create_publisher(PoseStamped, "~/pose", sensor_qos)
        self.raw_pose_publisher = self.create_publisher(PoseStamped, "~/raw_pose", sensor_qos)
        self.route_publisher = self.create_publisher(NavPath, "~/route", durable_qos)
        self.anchor_publisher = self.create_publisher(PointCloud2, "~/anchor_consensus", durable_qos)
        self.refined_publisher = self.create_publisher(PointCloud2, "~/refined_consensus", durable_qos)
        self.visual_publisher = self.create_publisher(String, "~/visual_localization", reliable_qos)
        teacher_subscription = self.create_subscription(Twist, self.p["teacher_command_topic"], self._teacher, sensor_qos)
        if teacher_subscription.topic_name == self.output_topic:
            self.journal.close()
            raise ValueError("resolved teacher input aliases command output")
        self.create_subscription(CommonGroup, self.p["vectornav_topic"], self._vectornav, sensor_qos)
        self.create_subscription(Image, self.p["mask_topic"], self._mask, sensor_qos)
        stop_qos = durable_qos if self.p["emergency_stop_transient_local"] else reliable_qos
        self.create_subscription(Bool, self.p["emergency_stop_topic"], self._manual_stop, stop_qos)
        self.create_subscription(String, self.p["ready_bundle_topic"], self._bundle_ready, durable_qos)
        self.create_subscription(String, self.p["teacher_state_topic"], self._teacher_state, durable_qos)
        for name, callback in (("arm", self._arm), ("start_repeat", self._start_repeat),
                               ("stop", self._stop), ("estop", self._estop),
                               ("reset_estop", self._reset_estop),
                               ("resume_reference", self._resume_reference)):
            self.create_service(Trigger, "~/" + name, callback)
        self.mask_worker = LatestMaskWorker(self.visual_commit_fence)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / self.p["control_rate_hz"], self._tick,
                                       clock=self.steady_clock)
        self.journal.write({"event": "startup", "parameters": self.p,
                            "speed_policy": "fixed_reference",
                            "lya_source_default_mps": REFERENCE_SPEED_MPS,
                            "source_sha256": {name: hashlib.sha256(
                                Path(__file__).with_name(name).read_bytes()).hexdigest()
                                for name in ("follower_node.py", "mask_localization.py",
                                             "mapping_core.py", "controller.py")},
                            "resolved_output_topic": self.output_topic,
                            "teacher_source_stamp_available": False})
        self.get_logger().info("DISARMED; command journal: " + str(self.journal.directory.resolve()))
        self._publish_state()

    def _new_localizer(self, origin=None, yaw_offset=None):
        return VectorNavLocalizer(
            origin_lla=origin,
            yaw_offset_rad=self.p["yaw_offset_rad"] if yaw_offset is None else yaw_offset,
            maximum_age_ms=self.p["pose_timeout_s"] * 1000,
            position_tolerance_m=self.p["position_jump_tolerance_m"],
            maximum_speed_mps=self.p["vectornav_maximum_speed_mps"],
        )

    def _times(self):
        return int(self.get_clock().now().nanoseconds), time.monotonic_ns()

    def _deployment_guard(self):
        if self.output_topic != "/lane_learning/cmd_vel":
            if self.p["accepted_teacher_max_speed_mps"] <= 0:
                return "vehicle output requires an explicitly approved teacher speed ceiling"
            if not all(self.p[name] for name in (
                    "enable_vehicle_output", "motor_zero_passthrough_verified", "hardware_stop_verified")):
                return "vehicle output requires enable flag, verified motor zero bypass and verified hardware stop"
        if self.journal.failed:
            return self.journal.failed
        try:
            if self.p["reject_competing_publishers"] and self.count_publishers(self.output_topic) > 1:
                return "competing command publisher detected"
        except Exception as error:
            return "cannot verify command ownership: " + str(error)
        return None

    def _teacher(self, message):
        now, steady = self._times()
        accepted = self.arbiter.set_teacher((float(message.linear.x), float(message.linear.y),
            float(message.linear.z), float(message.angular.x), float(message.angular.y),
            float(message.angular.z)), now, steady)
        teacher_controls = (self.arbiter.state == "TEACH" or (
            self.arbiter.safety_mode == "lya_reference" and self.arbiter.state in ("REPEAT", "FALLBACK")))
        if not accepted or (teacher_controls and self.arbiter.teacher is not None
                            and self.arbiter.teacher.speed <= 0.0):
            self._send(Command())

    def _vectornav(self, message):
        now, steady = self._times()
        self.arrival_seq += 1
        try:
            sample = self.localizer.accept(message, now, self.arrival_seq)
            self.pose_buffer.append(sample)
            self.raw_pose = Pose(sample.east_m, sample.north_m, sample.heading_rad,
                                 sample.speed_mps, sample.stamp_ns, steady)
            self.pose_error = None
            if (self.arbiter.repeat_selected
                    and self.visual_streak < self.p["visual_min_consecutive_matches"]
                    and sample.speed_mps > self.p["stopped_speed_mps"]):
                # Revoke an already-issued ACK before the worker can commit an
                # acquisition correction. Keep healthy raw VN for reacquisition.
                self._invalidate_visual("vehicle moved during visual acquisition")
            self.raw_pose_publisher.publish(self._pose_message(self.raw_pose))
            self._update_control_pose(now, steady)
        except (ValueError, TypeError, AttributeError) as error:
            self.pose_error = str(error)
            self.arbiter.pose = None
            self.arbiter.stopped_since_ns = None
            self.arbiter.hold("VectorNav rejected: " + str(error))
            self.raw_pose = None
            self.pose_buffer.clear()
            self._invalidate_visual("VectorNav rejected: " + str(error))
            self._send(Command())

    def _pose_message(self, value):
        pose = PoseStamped()
        pose.header.stamp.sec = value.stamp_ns // 1000000000
        pose.header.stamp.nanosec = value.stamp_ns % 1000000000
        pose.header.frame_id = self.manifest.get("frame_id", "lane_map") if self.manifest else "teach_enu"
        pose.pose.position.x, pose.pose.position.y = value.x, value.y
        pose.pose.orientation.z = math.sin(value.yaw * 0.5)
        pose.pose.orientation.w = math.cos(value.yaw * 0.5)
        return pose

    def _visual_guard(self, now, steady):
        if self.repeat_context is None or self.visual_last_accepted_stamp is None:
            return "waiting for mask-to-map localization"
        if self.visual_streak < self.p["visual_min_consecutive_matches"]:
            return "waiting for consecutive trusted mask matches"
        age = (now - self.visual_last_accepted_stamp) * 1e-9
        received_age = (steady - self.visual_last_arrival_steady_ns) * 1e-9
        if now <= 0 or not -0.02 <= age <= self.p["visual_max_age_s"]:
            return "mask localization is stale or ROS clock invalid"
        if not 0 <= received_age <= self.p["visual_max_age_s"]:
            return "mask localization steady-clock deadline expired"
        return None

    def _update_control_pose(self, now, steady):
        if self.raw_pose is None:
            self.arbiter.pose = None
            return
        if self.arbiter.repeat_selected:
            if self._visual_guard(now, steady):
                self.arbiter.pose = None
                self.arbiter.stopped_since_ns = None
                return
            dx, dy, angle = self.visual_correction
            cosine, sine = math.cos(angle), math.sin(angle)
            raw = self.raw_pose
            corrected = Pose(cosine * raw.x - sine * raw.y + dx,
                             sine * raw.x + cosine * raw.y + dy,
                             math.atan2(math.sin(raw.yaw + angle), math.cos(raw.yaw + angle)),
                             raw.speed, raw.stamp_ns, raw.received_steady_ns)
        else:
            corrected = self.raw_pose
        self.arbiter.update_pose(corrected)
        self.pose_publisher.publish(self._pose_message(corrected))

    def _invalidate_visual(self, reason):
        # Invalidating a later input also invalidates older in-flight proposals.
        # They must not restore confidence after the rejection or clock reset.
        self.visual_generation = self.visual_commit_fence.advance()
        self.visual_streak = 0
        self.visual_last_accepted_stamp = None
        self.visual_last_arrival_steady_ns = None
        self.visual_reason = str(reason)
        if self.arbiter.repeat_selected:
            self.arbiter.pose = None
            self.arbiter.stopped_since_ns = None
            self.arbiter.hold("visual localization unavailable: " + str(reason))
            self._send(Command())
        if self.visual_last_rejection != str(reason):
            self.journal.write({"event": "visual_rejected", "reason": str(reason),
                                "generation": self.visual_generation})
            self.visual_last_rejection = str(reason)

    def _mask(self, message):
        self.arrival_seq += 1
        if self.repeat_context is None or not self.arbiter.repeat_selected:
            return
        now, steady = self._times()
        source = stamp_ns(message.header.stamp)
        if (source <= 0 or now <= 0 or not -0.02 <= (now - source) * 1e-9 <= self.p["visual_max_age_s"]):
            self._invalidate_visual("stale or future mask")
            return
        if self.visual_last_mask_stamp is not None and source <= self.visual_last_mask_stamp:
            self._invalidate_visual("mask timestamp did not advance")
            return
        self.visual_last_mask_stamp = source
        camera = self.repeat_context["camera"]
        frame = str(self.p["camera_frame_override"]) or str(message.header.frame_id)
        if (frame != camera["frame_id"] or int(message.width) != camera["width"]
                or int(message.height) != camera["height"] or source < int(camera["stamp_ns"])):
            self._invalidate_visual("mask does not match saved calibration or causal calibration time")
            return
        prior, _, reason = self.pose_buffer.latest_not_after(source, self.arrival_seq,
            int(self.p["visual_max_pose_mask_gap_s"] * 1e9))
        if prior is None:
            self._invalidate_visual(reason)
            return
        # Initial acquisition changes the map alignment: permit it only while
        # stopped in HOLD/DISARMED, never while a repeat command is being sent.
        acquiring = self.visual_streak < self.p["visual_min_consecutive_matches"]
        if acquiring and (self.arbiter.state not in ("HOLD", "DISARMED", "ESTOP")
                          or prior.speed_mps > self.p["stopped_speed_mps"]):
            self._invalidate_visual("visual acquisition requires stopped vehicle")
            return
        self.visual_job_id += 1
        job = dict(job_id=self.visual_job_id, generation=self.visual_generation,
                   context=self.repeat_context, message=message, source_ns=source,
                   received_steady_ns=steady, prior_stamp_ns=prior.stamp_ns,
                   prior=np.asarray([prior.east_m, prior.north_m, prior.heading_rad], dtype=float))
        if self.mask_worker.submit(job):
            self.visual_pending_replacements += 1

    def _consume_visual_results(self, now, steady):
        for _ in range(2):
            envelope = self.mask_worker.take_result()
            if envelope is None:
                return
            kind = envelope["kind"]
            if kind == "refinement":
                if envelope["generation"] == self.visual_generation:
                    self.visual_last_refined = envelope
                    self.refined_publisher.publish(cloud_message(envelope["source_ns"],
                        self.manifest["frame_id"], envelope["packed"]))
                continue
            if kind == "commit_failed":
                if envelope["generation"] == self.visual_generation:
                    self._invalidate_visual(envelope["error"])
                continue
            if kind == "committed":
                job, result = envelope["job"], envelope["result"]
                now, steady = self._times()
                if job["generation"] != self.visual_generation or job["context"] is not self.repeat_context:
                    continue
                raw = self.raw_pose
                if (now <= 0 or not -0.02 <= (now - job["source_ns"]) * 1e-9 <= self.p["visual_max_age_s"]
                        or not 0 <= (steady - job["received_steady_ns"]) * 1e-9 <= self.p["visual_max_age_s"]
                        or raw is None or self.pose_error or not fresh(raw.stamp_ns, raw.received_steady_ns,
                            now, steady, self.p["pose_timeout_s"])):
                    self._invalidate_visual("committed match no longer fresh at control activation")
                    continue
                if (self.visual_streak < self.p["visual_min_consecutive_matches"]
                        and (raw.speed > self.p["stopped_speed_mps"]
                             or self.arbiter.state not in ("HOLD", "DISARMED", "ESTOP"))):
                    self._invalidate_visual("vehicle moved before committed visual acquisition")
                    continue
                if (self.visual_last_accepted_stamp is None or
                        (job["source_ns"] - self.visual_last_accepted_stamp) * 1e-9 > self.p["visual_max_age_s"]):
                    self.visual_streak = 0
                self.visual_streak += 1
                self.visual_correction = np.asarray(result.correction_se2, dtype=float).copy()
                self.visual_last_accepted_stamp = job["source_ns"]
                self.visual_last_arrival_steady_ns = job["received_steady_ns"]
                self.visual_reason = result.reason
                self.visual_last_rejection = None
                self.visual_metrics = finite_json(result.metrics)
                self._update_control_pose(now, steady)
                if envelope.get("refinement") is not None:
                    snapshot = envelope["refinement"]
                    self.visual_last_refined = snapshot
                    self.refined_publisher.publish(cloud_message(snapshot["source_ns"],
                        self.manifest["frame_id"], snapshot["packed"]))
                self.journal.write(finite_json(dict(event="visual_committed", source_ns=job["source_ns"],
                    trusted_streak=self.visual_streak, correction_se2=self.visual_correction,
                    metrics=self.visual_metrics)))
                metrics = String()
                metrics.data = json.dumps(finite_json(dict(source_ns=job["source_ns"],
                    trusted_streak=self.visual_streak, correction_se2=self.visual_correction,
                    metrics=self.visual_metrics)), allow_nan=False)
                self.visual_publisher.publish(metrics)
                continue
            job, result = envelope["job"], envelope["result"]
            now, steady = self._times()
            reason = envelope["error"]
            if job["generation"] != self.visual_generation or job["context"] is not self.repeat_context:
                self.mask_worker.acknowledge(job["job_id"], False)
                continue
            if (now <= 0 or not -0.02 <= (now - job["source_ns"]) * 1e-9 <= self.p["visual_max_age_s"]
                    or not 0 <= (steady - job["received_steady_ns"]) * 1e-9 <= self.p["visual_max_age_s"]):
                reason = "mask match expired before commit"
            raw = self.raw_pose
            if (raw is None or self.pose_error or not fresh(raw.stamp_ns, raw.received_steady_ns,
                    now, steady, self.p["pose_timeout_s"])):
                reason = "VectorNav invalidated during mask matching"
            if (self.visual_streak < self.p["visual_min_consecutive_matches"] and raw is not None
                    and (raw.speed > self.p["stopped_speed_mps"]
                         or self.arbiter.state not in ("HOLD", "DISARMED", "ESTOP"))):
                reason = "vehicle moved during visual acquisition"
            if result is None or not result.accepted:
                reason = reason or (result.reason if result is not None else "mask matcher failed")
            self.visual_metrics = finite_json(result.metrics if result is not None else {})
            self.journal.write(finite_json({"event": "visual_match", "source_ns": job["source_ns"],
                "prior_stamp_ns": job["prior_stamp_ns"], "generation": job["generation"],
                "accepted": reason is None, "reason": reason or result.reason,
                "prior_xyyaw": job["prior"], "matched_xyyaw": result.pose_xyyaw if result else None,
                "correction_se2": result.correction_se2 if result else None,
                "metrics": self.visual_metrics, "compute_ms": envelope["compute_ms"],
                "projected_pixels": envelope["projected_pixels"]}))
            if reason:
                self.mask_worker.acknowledge(job["job_id"], False)
                self._invalidate_visual(reason)
                continue
            # Approval is not yet a successful commit. Only the worker's
            # committed notification may activate the correction for control.
            # Convert BOTH source-clock and receipt-clock remaining lifetimes to
            # one conservative steady deadline. Newer VN cannot extend this ACK.
            # The worker checks it under the same lock used by invalidation.
            visual_age_ns = int(self.p["visual_max_age_s"] * 1e9)
            pose_age_ns = int(self.p["pose_timeout_s"] * 1e9)
            deadline = min(steady + visual_age_ns - (now - job["source_ns"]),
                           job["received_steady_ns"] + visual_age_ns,
                           steady + pose_age_ns - (now - raw.stamp_ns),
                           raw.received_steady_ns + pose_age_ns)
            self.mask_worker.acknowledge(job["job_id"], True,
                generation=job["generation"], issued_steady_ns=steady, deadline_steady_ns=deadline)

    def _manual_stop(self, message):
        self.arbiter.manual_estop = bool(message.data)
        if message.data:
            self.arbiter.estop("manual emergency stop asserted")
            self._send(Command())
            self._publish_state()

    def _bundle_ready(self, message):
        # A notification can never change the active route or arm the vehicle.
        if len(message.data) < 65536 and not self.arbiter.repeat_selected:
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    raise ValueError("bundle notification must be an object")
                if payload.get("ready") is not True:
                    return
                self.ready_path = str(Path(payload["session_directory"]) / "bundle.json")
                self.journal.write({"event": "bundle_available", "path": self.ready_path})
            except (ValueError, KeyError, TypeError):
                self.journal.write({"event": "bundle_notification_rejected"})

    def _teacher_state(self, message):
        try:
            body = json.loads(message.data)
            if not isinstance(body, dict):
                raise ValueError("teacher state must be an object")
            source = body.get("stamp_ns")
            now, steady = self._times()
            if (isinstance(source, bool) or not isinstance(source, int) or source <= 0
                    or now <= 0 or not -20000000 <= now - source <= 1000000000
                    or source < self.repeat_prepare_ros_ns):
                raise ValueError("teacher state source stamp is invalid, stale or before handover")
            self.teacher_stopped = body.get("state") == "STOPPED"
            self.teacher_state_source_ns = source
            self.teacher_state_received_ns = steady
        except (ValueError, TypeError):
            self.teacher_stopped = False
            self.teacher_state_source_ns = 0
            self.teacher_state_received_ns = 0

    @staticmethod
    def _reply(response, success, message):
        response.success, response.message = bool(success), str(message)
        return response

    def _arm(self, request, response):
        issue = self._deployment_guard()
        result = (False, issue) if issue else self.arbiter.arm(*self._times())
        self._publish_state()
        return self._reply(response, *result)

    def _start_repeat(self, request, response):
        if self.arbiter.state == "ESTOP":
            return self._reply(response, False, "reset emergency stop first")
        issue = self._deployment_guard()
        if issue:
            self.arbiter.hold(issue)
            self._send(Command())
            return self._reply(response, False, issue)
        if self.arbiter.controller is None:
            self.arbiter.hold("loading route; teacher command forwarding stopped")
            self._send(Command())
            try:
                path = self.ready_path or self.get_parameter("route_bundle_path").value
                if not path:
                    raise ValueError("no ready bundle available")
                manifest, route, metadata = load_bundle(path)
                repeat_context = load_repeat_context(path, manifest, metadata, self.p)
                controller = ClosedRouteController(route,
                    reference_speed_mps=self.p["reference_speed_mps"],
                    maximum_speed=self.p["maximum_speed_mps"],
                    maximum_yaw_rate=self.p["maximum_yaw_rate_rps"],
                    maximum_acceleration=self.p["maximum_acceleration_mps2"],
                    maximum_yaw_acceleration=self.p["maximum_yaw_acceleration_rps2"],
                    maximum_lateral_acceleration=self.p["maximum_lateral_acceleration_mps2"],
                    maximum_cross_track=self.p["maximum_cross_track_m"],
                    maximum_heading_error=self.p["maximum_heading_error_rad"],
                    start_distance=self.p["start_distance_m"],
                    start_heading_error=self.p["start_heading_error_rad"])
                self.localizer = self._new_localizer(manifest["origin_lla"], manifest["yaw_offset_rad"])
                self.manifest = manifest
                self.repeat_context = repeat_context
                self.visual_generation = self.visual_commit_fence.advance()
                self.visual_streak = 0
                self.visual_last_mask_stamp = None
                self.visual_last_accepted_stamp = None
                self.visual_last_arrival_steady_ns = None
                self.visual_correction = np.zeros(3, dtype=float)
                self.visual_reason = "waiting for fresh mask acquisition in saved map"
                self.visual_last_refined = None
                self.raw_pose = None
                self.pose_error = None
                self.pose_buffer.clear()
                self.arbiter.pose = None  # Must receive fresh pose in the saved frame.
                self.repeat_prepare_ros_ns, self.repeat_prepare_ns = self._times()
                result = self.arbiter.prepare_repeat(controller)
                self.journal.write({"event": "route_loaded", "manifest": manifest,
                                    "matcher_configuration": repeat_context["matcher"].config})
                self._publish_route(route)
                self.anchor_publisher.publish(cloud_message(int(metadata.get("stamp_ns", 0)),
                    manifest["frame_id"], pack_cloud(repeat_context["anchor"], repeat_context["anchor_votes"])))
            except (ValueError, OSError, KeyError, TypeError) as error:
                result = False, "route rejected: " + str(error)
                self.arbiter.hold(result[1])
        else:
            now, steady = self._times()
            visual_issue = self._visual_guard(now, steady)
            if visual_issue:
                result = False, visual_issue
            elif (self.arbiter.safety_mode == "fixed_only" and self.p["require_managed_teacher_stop"]
                    and (not self.teacher_stopped
                         or now <= 0 or self.teacher_state_source_ns < self.repeat_prepare_ros_ns
                         or not -20000000 <= now - self.teacher_state_source_ns <= 1000000000
                         or self.teacher_state_received_ns < self.repeat_prepare_ns
                         or not 0 <= steady - self.teacher_state_received_ns <= 1000000000)):
                result = False, "managed LYA supervisor has not freshly confirmed STOPPED after handover"
            else:
                result = self.arbiter.start_repeat(now, steady)
            self.last_tick_ns = time.monotonic_ns()
        self._publish_state()
        return self._reply(response, *result)

    def _publish_route(self, route):
        message = NavPath()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.manifest["frame_id"]
        for row in route["route_samples"] + route["route_samples"][:1]:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y = row["x_m"], row["y_m"]
            pose.pose.orientation.z = math.sin(row["yaw_rad"] * 0.5)
            pose.pose.orientation.w = math.cos(row["yaw_rad"] * 0.5)
            message.poses.append(pose)
        self.route_publisher.publish(message)

    def _resume_reference(self, request, response):
        issue = self._deployment_guard() or self._visual_guard(*self._times())
        result = (False, issue) if issue else self.arbiter.resume_reference(*self._times())
        self._publish_state()
        return self._reply(response, *result)

    def _stop(self, request, response):
        self.arbiter.hold("operator stop; explicit resume required")
        self._send(Command())
        self._publish_state()
        return self._reply(response, True, self.arbiter.reason)

    def _estop(self, request, response):
        self.arbiter.estop("operator emergency stop service")
        self._send(Command())
        self._publish_state()
        return self._reply(response, True, self.arbiter.reason)

    def _reset_estop(self, request, response):
        result = self.arbiter.reset_estop()
        self._send(Command())
        self._publish_state()
        return self._reply(response, *result)

    def _tick(self):
        now, steady = self._times()
        dt = (steady - self.last_tick_ns) * 1e-9
        self.last_tick_ns = steady
        if self.last_ros_ns is not None and now < self.last_ros_ns:
            self.arbiter.hold("ROS clock regressed; restart localization before rearming")
            self.arbiter.pose = None
            self.raw_pose = None
            self.pose_buffer.clear()
            self._invalidate_visual("ROS clock regressed during repeat localization")
        self.last_ros_ns = now
        self._consume_visual_results(now, steady)
        # Worker-result processing and DDS serialization are not assumed free.
        # Re-read both clocks before permitting any nonzero control output.
        now, steady = self._times()
        if self.arbiter.repeat_selected:
            visual_issue = self._visual_guard(now, steady)
            if visual_issue and (self.arbiter.state in ("REPEAT", "FALLBACK")
                                 or self.visual_last_accepted_stamp is not None):
                # Confidence still accumulating is not a fault; it cannot arm.
                if self.visual_streak >= self.p["visual_min_consecutive_matches"] or self.arbiter.state in ("REPEAT", "FALLBACK"):
                    self._invalidate_visual(visual_issue)
        # Graph discovery is not a hardware interlock, but catches accidental
        # unmanaged LYA publishers. No process is killed from this node.
        if steady - self.last_graph_check_ns >= 100000000:
            self.last_graph_check_ns = steady
            issue = self._deployment_guard()
            if issue:
                self.arbiter.estop(issue)
        if self.journal.failed:
            self.arbiter.estop(self.journal.failed)
        try:
            command = self.arbiter.command(now, steady, dt)
        except Exception as error:
            self.arbiter.estop("unexpected control failure: " + str(error))
            command = Command()
        self._send(command)
        self._publish_state()

    def _send(self, command):
        message = Twist()
        message.linear.x, message.angular.z = float(command.speed), float(command.yaw_rate)
        self.publisher.publish(message)
        now, steady = self._times()
        pose = self.arbiter.pose
        self.journal.write({"event": "command", "stamp_ns": now, "steady_ns": steady,
            "state": self.arbiter.state, "reason": self.arbiter.reason,
            "speed_mps": command.speed, "yaw_rate_rps": command.yaw_rate,
            "pose_stamp_ns": pose.stamp_ns if pose else None,
            "pose_age_ms": (now - pose.stamp_ns) * 1e-6 if pose else None,
            "teacher_arrival_age_ms": (steady - self.arbiter.teacher_received_ns) * 1e-6
            if self.arbiter.teacher_received_ns else None,
            "teacher_raw": vars(self.arbiter.raw_teacher) if self.arbiter.raw_teacher else None,
            "teacher_limited": vars(self.arbiter.teacher) if self.arbiter.teacher else None,
            "raw_pose": vars(self.raw_pose) if self.raw_pose else None,
            "corrected_pose": vars(pose) if pose else None,
            "visual_source_ns": self.visual_last_accepted_stamp,
            "visual_trusted_streak": self.visual_streak,
            "visual_correction_se2": self.visual_correction.tolist(),
            "visual_match_metrics": self.visual_metrics,
            "tracking": self.arbiter.controller.last_metrics if self.arbiter.controller else {}})

    def _publish_state(self):
        teacher_enabled = not (self.arbiter.repeat_selected and self.arbiter.safety_mode == "fixed_only")
        body = dict(state=self.arbiter.state, reason=self.arbiter.reason,
                    safety_mode=self.arbiter.safety_mode, teacher_enabled=teacher_enabled,
                    repeat_selected=self.arbiter.repeat_selected,
                    visual_ready=self._visual_guard(*self._times()) is None if self.arbiter.repeat_selected else False,
                    visual_reason=self.visual_reason,
                    visual_trusted_streak=self.visual_streak,
                    visual_latest_only_replacements=self.visual_pending_replacements,
                    stamp_ns=int(self.get_clock().now().nanoseconds),
                    output_topic=self.output_topic)
        state = String()
        state.data = json.dumps(body, separators=(",", ":"))
        self.state_publisher.publish(state)
        flag = Bool()
        flag.data = teacher_enabled
        self.teacher_enabled_publisher.publish(flag)
        signature = self.arbiter.state, self.arbiter.reason
        if signature != self.last_state:
            self.journal.write(dict(event="state", **body))
            self.get_logger().info("{}: {}".format(*signature))
            self.last_state = signature

    def close(self):
        self.arbiter.hold("node shutting down")
        self._send(Command())
        self._publish_state()
        self.mask_worker.close()
        try:
            self._write_refinement_snapshot()
        except (OSError, ValueError, TypeError) as error:
            self.journal.write({"event": "refinement_snapshot_failed", "error": str(error)})
        self.journal.close()

    def _write_refinement_snapshot(self):
        snapshot = self.mask_worker.latest_refined
        if snapshot is None or self.manifest is None:
            return
        if snapshot["generation"] != self.visual_generation:
            # A previous committed layer is still valid evidence, but never save
            # another bundle's layer after a load-generation change.
            snapshot = self.visual_last_refined
            if snapshot is None:
                return
        path = self.journal.directory / "refined_consensus.csv"
        points = np.frombuffer(snapshot["packed"][0], dtype="<f4").reshape(-1, 4)
        with path.open("x", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("east_m", "north_m", "frame_votes"))
            writer.writerows((float(row[0]), float(row[1]), int(row[3])) for row in points)
        details = dict(schema_version=1, bundle_id=self.manifest.get("bundle_id"),
                       anchor_sha256=self.repeat_context["anchor_sha256"],
                       route_sha256=self.manifest["route_sha256"], route_changed=False,
                       anchor_changed=False, origin_lla=self.manifest["origin_lla"],
                       frame_id=self.manifest["frame_id"], source_ns=snapshot["source_ns"],
                       historical_layer=True, trusted_at_shutdown=False,
                       snapshot_generation=snapshot["generation"], current_generation=self.visual_generation,
                       cells=len(points), metrics=self.visual_metrics,
                       purpose="advisory committed mask consensus; never the localization anchor or fixed route")
        with (self.journal.directory / "refinement_metadata.json").open("x", encoding="utf-8") as stream:
            json.dump(finite_json(details), stream, indent=2, allow_nan=False)
        self.journal.write({"event": "refinement_snapshot_saved", "path": str(path), "cells": len(points)})


def main(args=None, forced_safety_mode=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FixedFollower(forced_safety_mode=forced_safety_mode)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.close()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_fixed(args=None):
    main(args=args, forced_safety_mode="fixed_only")


def main_reference(args=None):
    main(args=args, forced_safety_mode="lya_reference")
