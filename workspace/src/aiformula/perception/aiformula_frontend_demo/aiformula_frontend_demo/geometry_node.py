"""ROS 2 adapter for geometric hashing and point-to-curve alignment."""
from bisect import bisect_left
from collections import deque
import json
from time import monotonic

import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .common import as_json, interp_pose, ns, planar_pose
from .geometric import GeometryStage


class GeometryNode(Node):
    def __init__(self):
        super().__init__("geometric_hash")
        odom = self.declare_parameter("odom_topic", "/odom").value
        self.body = self.declare_parameter("body_frame", "base_link").value
        history = self.declare_parameter("history_keyframes", 12).value
        self.worker = GeometryStage(history=history)
        self.max_odom_gap_ns = round(self.declare_parameter("odom_max_gap_s", .35).value*1e9)
        self.wait_s = self.declare_parameter("odom_wait_s", 0.).value
        self.poses = deque(maxlen=500)
        self.pending = deque()
        self.last_observation_stamp = None
        if self.wait_s > 0:
            self.create_timer(.05, self.flush, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.output = self.create_publisher(String, "/front_demo/geometry/match", 5)
        self.create_subscription(Odometry, odom, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, "/front_demo/semantic/observation", self.on_obs, 5)

    def on_odom(self, msg):
        if msg.child_frame_id != self.body:
            return
        stamp = ns(msg.header.stamp)
        if self.poses and stamp < self.poses[-1][0]-1_000_000_000:
            self.poses.clear()
            if self.pending and self.pending[0][1] > stamp + self.max_odom_gap_ns:
                self.pending.clear()
        if not self.poses or stamp > self.poses[-1][0]:
            self.poses.append((stamp, planar_pose(msg.pose.pose)))
            self.flush()

    def on_obs(self, msg):
        try:
            obs = json.loads(msg.data)
            if obs["body_frame"] != self.body:
                return
            stamp = int(obs["stamp_ns"])
            if self.last_observation_stamp is not None and stamp < self.last_observation_stamp:
                self.pending.clear()
            self.last_observation_stamp = stamp
            self.pending.append((monotonic()+self.wait_s, stamp, obs))
            self.flush()
        except (ValueError, KeyError, TypeError) as error:
            self.get_logger().warning(f"geometry observation rejected: {error}")

    def flush(self):
        if not self.pending:
            return
        poses = list(self.poses)
        stamps = [item[0] for item in poses]
        while self.pending:
            deadline, stamp, obs = self.pending[0]
            i = bisect_left(stamps, stamp)
            at = poses[i] if i < len(poses) else None
            if at is None and monotonic() < deadline:
                return
            self.pending.popleft()
            before = poses[i-1] if i else None
            if at is not None and at[0] == stamp:
                pose = at[1]
            else:
                pose = interp_pose(stamp, before, at, self.max_odom_gap_ns)
            try:
                self.output.publish(String(data=as_json(self.worker.step(obs, pose))))
            except (ValueError, KeyError, TypeError) as error:
                self.get_logger().warning(f"geometry observation rejected: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = GeometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
