"""Execute pinned Python class bodies with synchronous, in-process ROS transport.

No Unity pose, reference path, ideal mask or motor telemetry enters this chain.
Targets are in base_link; same-time odom->base and subtraction cancel, so the
ROS TF adapter uses an identity local odometry origin. DDS timing is not modeled.
Observed-center priority is adapter-only opt-in; default selection is unchanged.
"""
import ast
import hashlib
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace as NS
import numpy as np
from lane_publisher import LanePublisher

ROOT = Path(__file__).resolve().parent / "upstream"
PREFIX = "workspace/src/aiformula/"
COMMIT = "05f1651c8a14d56ab9a86a9767c52793403dcfdc"


def observed_triplet(cloud):
    """只选本帧已有点；5/6点短线缩短前视，不外推、不重复旧观测。"""
    n = len(cloud)
    indices = (2, 4, 6) if n >= 7 else (1, 3, 5) if n == 6 else (0, 2, 4) if n == 5 else None
    if indices is None:
        return None
    try:
        points = [cloud[i] for i in indices]
        if not all(len(p) >= 3 and all(math.isfinite(float(v)) for v in p[:3]) for p in points):
            return None
        x = [float(p[0]) for p in points]
        if not x[0] < x[1] < x[2] or x[2] - x[0] < 2.0 - 1e-6:
            return None
    except (IndexError, TypeError, ValueError, OverflowError):
        return None
    return indices


def observed_short_center_triplet(clouds):
    """Opt-in current 3/4-point center with explicit paired-boundary guards.

    These quality limits are adapter policy, not native LLP/LYA parameters.
    Input points are never extended, synthesized, reordered, or retained.
    """
    try:
        left, right, center = (np.asarray(c, dtype=float) for c in clouds)
        n = len(center)
        indices = (0, 1, 2) if n == 3 else (0, 1, 3) if n == 4 else None
        if indices is None or min(len(left), len(right)) <= indices[-1]:
            return None
        for cloud in (left, right, center):
            if cloud.ndim != 2 or cloud.shape[1] != 3 or not np.isfinite(cloud).all():
                return None
            x = cloud[:, 0]
            if not np.all(x > 0) or not np.all(np.diff(x) > 0) or x[-1]-x[0] < 1.0:
                return None
        x = center[:, 0]
        if any(x[0] < side[0, 0] or x[-1] > side[-1, 0] for side in (left, right)):
            return None
        # Endpoints above were checked before interpolation: no extrapolation.
        left_y = np.interp(x, left[:, 0], left[:, 1])
        right_y = np.interp(x, right[:, 0], right[:, 1])
        if np.max(np.abs(center[:, 1]-.5*(left_y+right_y))) > 1e-4:
            return None
        width = left_y-right_y
        if not np.isfinite(width).all() or not np.all(width > 0):
            return None
        if float(width.max()-width.min()) > .10*float(width.mean()):
            return None
        dx = x-x.mean()
        denominator = float(dx @ dx)
        left_slope = float(dx @ (left_y-left_y.mean()))/denominator
        right_slope = float(dx @ (right_y-right_y.mean()))/denominator
        heading_difference = abs(math.degrees(math.atan(left_slope)-math.atan(right_slope)))
        if not math.isfinite(heading_difference) or heading_difference > 15.0:
            return None
        return indices
    except (TypeError, ValueError, IndexError, OverflowError):
        return None


def select_observed_clouds(clouds, prefer_observed_center=False):
    left, right, center = clouds
    if prefer_observed_center is True:
        indices = observed_triplet(center)
        # Current common-range center only. Setting the existing native node's
        # indices also makes both real side counts satisfy its center branch.
        if indices and min(len(left), len(right)) > indices[-1]:
            return clouds, indices, "center", "adapter_opt_in_observed_center"
        indices = observed_short_center_triplet(clouds)
        if indices:
            return clouds, indices, "center", "adapter_opt_in_short_observed_center"
    original = 2 if min(len(left), len(right)) >= 7 else 0 if len(left) >= 7 else 1 if len(right) >= 7 else None
    # 旧的有效路径保留同一组点、同一原节点分支与补偿。
    if original is not None and len(clouds[original]) >= 7 and observed_triplet(clouds[original]) == (2, 4, 6):
        return clouds, (2, 4, 6), ("left_only", "right_only", "center")[original], "original_indices"
    indices = observed_triplet(center)
    if indices and min(len(left), len(right)) > indices[-1]:
        return clouds, indices, "center", "short_observed_center"
    for side in sorted((0, 1), key=lambda i: -len(clouds[i])):
        indices = observed_triplet(clouds[side])
        if indices:
            # 原节点仅按长度决定center/side；无共同center时必须明确选择可用侧。
            selected = [[], [], []]
            selected[side] = clouds[side]
            return selected, indices, ("left_only", "right_only")[side], "observed_side_recovery"
    return [[], [], []], (2, 4, 6), "default_straight", "no_observed_triplet"


class Pose2D:
    def __init__(self, x=0., y=0., theta=0.):
        self.x, self.y, self.theta = x, y, theta


class Message:
    def __init__(self):
        self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id="")
        self.pose = NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))
        self.linear, self.angular = NS(x=0., y=0., z=0.), NS(x=0., y=0., z=0.)


class Publisher:
    def __init__(self):
        self.value, self.callback = None, None

    def publish(self, value):
        self.value = value
        if self.callback:
            self.callback(value)


class Node:
    def __init__(self, *_):
        pass

    def create_subscription(self, *args):
        return args

    def create_publisher(self, *_):
        return Publisher()

    def get_logger(self):
        return NS(**{name: lambda *args: None for name in ("info", "debug", "warn", "warning", "error")})

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=lambda: NS(sec=0, nanosec=0)))


def load_classes(relative, names):
    path = ROOT / PREFIX / relative
    tree = ast.parse(path.read_text(encoding="utf-8"), str(path))
    classes = ast.Module(body=[n for n in tree.body if isinstance(n, ast.ClassDef) and n.name in names], type_ignores=[])
    namespace = dict(Node=Node, Pose2D=Pose2D, Bool=Message, PointCloud2=Message, Odometry=Message,
                     Twist=Message, PoseStamped=Message, math=math, np=np, time=time,
                     pc2=NS(read_points=lambda msg, **kwargs: msg),
                     Buffer=lambda: NS(transform=lambda pose, *args, **kwargs: pose),
                     TransformListener=lambda *args: None, TransformBroadcaster=lambda *args: None,
                     Duration=lambda **kwargs: None,
                     tf_transformations=NS(quaternion_from_euler=lambda r, p, y: (0, 0, math.sin(y/2), math.cos(y/2)),
                         euler_from_quaternion=lambda q: (0, 0, math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2)))))
    exec(compile(classes, str(path), "exec"), namespace)
    return namespace


class Pipeline:
    def __init__(self):
        manifest = json.loads((ROOT / "provenance.json").read_text(encoding="utf-8"))
        if manifest["commit"] != COMMIT:
            raise ValueError("Unexpected lane pipeline revision")
        for entry in manifest["files"]:
            if hashlib.sha256((ROOT / entry["path"]).read_bytes()).hexdigest() != entry["sha256"]:
                raise ValueError("Upstream source hash mismatch: " + entry["path"])
        self.publisher = LanePublisher()
        self.points = load_classes("perception/lane_points/lane_points/lane_0215.py", {"LaneLineSubscriber"})["LaneLineSubscriber"](2, 4, 6)
        self.target = load_classes("perception/kalman_filter/kalman_filter/withoutkalman_0312.py", {"DataProcessingNode"})["DataProcessingNode"]()
        self.lya = load_classes("control/trajectory_follower/trajectory_follower/lya_0221.py", {"LYAController", "TrajectoryFollower"})["TrajectoryFollower"]()
        self.lya.current_pose = Message().pose
        self.points.pose_publisher_a.callback = self.target.observation_callback_a
        self.points.pose_publisher_b.callback = self.target.observation_callback_b
        self.points.pose_publisher_c.callback = self.target.observation_callback_c
        self.target.publisher_omega_t.callback = self.lya.omega_t_callback
        self.target.publisher_point.callback = self.lya.trajectory_callback
        self.current_corridor = None

    def clouds_to_command(self, clouds, prefer_observed_center=False, selection=None, inferred_width_m=None):
        self.lya.velocity_publisher.value = None
        self.target.publisher_point.value = None
        self.target.A = self.target.B = self.target.C = None
        for publisher in (self.points.pose_publisher_a, self.points.pose_publisher_b,
                          self.points.pose_publisher_c):
            publisher.value = None
        selected, indices, mode, selection_reason = (select_observed_clouds(clouds, prefer_observed_center)
                                                    if selection is None else selection)
        self.points.a, self.points.b, self.points.c = indices
        self.points.left_callback(selected[0])
        self.points.right_callback(selected[1])
        previous_width = self.points.lane_width_m
        try:
            if inferred_width_m is not None:
                if mode != "right_only" or not math.isfinite(inferred_width_m) or inferred_width_m <= 0:
                    raise ValueError("Inferred width requires current right-only input")
                self.points.lane_width_m = inferred_width_m
            self.points.center_callback(selected[2])
        finally:
            self.points.lane_width_m = previous_width
        cmd = self.lya.velocity_publisher.value
        target = self.target.publisher_point.value
        published = [p.value for p in (self.points.pose_publisher_a, self.points.pose_publisher_b, self.points.pose_publisher_c)]
        supported = mode != "default_straight" and cmd is not None and target is not None and all(
            p is not None and math.isfinite(float(p.x)) and math.isfinite(float(p.y)) for p in published)
        return dict(valid=cmd is not None, supported=supported, mode=mode,
                    v=float(cmd.linear.x) if cmd else 0., w=float(cmd.angular.z) if cmd else 0.,
                    target=[float(target.x), float(target.y), float(target.theta)] if target else [],
                    selected_points=[[float(p.value.x), float(p.value.y)]
                                     for p in (self.points.pose_publisher_a, self.points.pose_publisher_b,
                                               self.points.pose_publisher_c) if p.value is not None],
                    selected_indices=[self.points.a, self.points.b, self.points.c],
                    frame="base_link", selection_source=mode, selection_reason=selection_reason,
                    omega_t=float(self.lya.lya.omega_t), counts=[len(c) for c in clouds])

    def process(self, mask, request):
        opted_in = request.get("recover_projected_components", False) is True
        before = ([None if line is None else line.copy() for line in self.publisher.lines], list(self.publisher.lost)) if opted_in else None
        clouds, debug = self.publisher.process(mask, request["camera"],
            recover_projected_components=opted_in)
        corridor, corridor_debug = None, None
        if opted_in:
            raw_clouds = clouds
            from current_corridor import CurrentCorridor
            if self.current_corridor is None:
                self.current_corridor = CurrentCorridor()
            choice = self.current_corridor.resolve(mask, request, clouds, self.publisher, before)
            clouds = choice["clouds"]
            result = self.clouds_to_command(clouds,
                selection=(clouds, choice["indices"], choice["mode"], choice["reason"]),
                inferred_width_m=choice.get("width_m"))
            corridor_debug = choice.get("evidence")
            if choice.get("width_m") is not None or choice.get("identity_checked_left") is True:
                identity_left = choice.get("identity_checked_left") is True
                corridor = dict(source="current_left_identity_checked_native" if identity_left else "current_right_inferred",
                    width_m=corridor_debug["width_m"],
                    calibration_sequence=corridor_debug["calibration_last_sequence"],
                    calibration_stamp=corridor_debug["calibration_last_stamp"],
                    calibration_age_at_capture=corridor_debug["calibration_age_at_capture"],
                    max_calibration_age_sec=corridor_debug["calibration_max_age_sec"])
                if identity_left:
                    corridor["native_left_width_m"] = float(self.points.lane_width_m)
                    if corridor_debug["raw_left_source"]["kind"] == "original_current_left":
                        corridor_debug["raw_left_source"]["publisher_debug"] = debug[0]
                elif corridor_debug["raw_right_source"]["kind"] == "original_current_right":
                    corridor_debug["raw_right_source"]["publisher_debug"] = debug[1]
        else:
            self.current_corridor = None
            result = self.clouds_to_command(clouds, prefer_observed_center=False)
        result.update(sequence=request["sequence"], sim_time=request["sim_time"], epoch=request["epoch"],
                      sides=debug, clouds=clouds, commit=COMMIT)
        if opted_in:
            result.update(corridor=corridor, corridor_debug=corridor_debug,
                          raw_clouds=raw_clouds, raw_counts=[len(c) for c in raw_clouds])
        if not all(math.isfinite(result[k]) for k in ("v", "w", "omega_t")):
            raise ValueError("Nonfinite upstream control output")
        return result
