"""Execute pinned Python class bodies with synchronous, in-process ROS transport.

No Unity pose, reference path, ideal mask or motor telemetry enters this chain.
Targets are in base_link; same-time odom->base and subtraction cancel, so the
ROS TF adapter uses an identity local odometry origin. DDS timing is not modeled.
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


def select_observed_clouds(clouds):
    left, right, center = clouds
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

    def clouds_to_command(self, clouds):
        self.lya.velocity_publisher.value = None
        self.target.publisher_point.value = None
        self.target.A = self.target.B = self.target.C = None
        for publisher in (self.points.pose_publisher_a, self.points.pose_publisher_b,
                          self.points.pose_publisher_c):
            publisher.value = None
        selected, indices, mode, selection_reason = select_observed_clouds(clouds)
        self.points.a, self.points.b, self.points.c = indices
        self.points.left_callback(selected[0])
        self.points.right_callback(selected[1])
        self.points.center_callback(selected[2])
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
        clouds, debug = self.publisher.process(mask, request["camera"])
        result = self.clouds_to_command(clouds)
        result.update(sequence=request["sequence"], sim_time=request["sim_time"], epoch=request["epoch"],
                      sides=debug, clouds=clouds, commit=COMMIT)
        if not all(math.isfinite(result[k]) for k in ("v", "w", "omega_t")):
            raise ValueError("Nonfinite upstream control output")
        return result
