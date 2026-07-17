#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class LaneCloudToLyaPose(Node):
    def __init__(self):
        super().__init__('codex_lane_cloud_to_lya_pose')
        self.pose_pub = self.create_publisher(Pose2D, '/filtered_lane_pose', 10)
        self.omega_pub = self.create_publisher(Pose2D, '/filtered_omega_t', 10)
        self.received = 0
        self.create_subscription(
            PointCloud2,
            '/aiformula_perception/lane_line_publisher/lane_lines/center',
            self.cloud_callback,
            qos_profile_sensor_data)

    def cloud_callback(self, msg):
        points = list(pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not points:
            return
        # LLP may return fewer than seven valid samples on tight curves.
        # Use three ordered samples spanning every non-empty fitted center line.
        a, b, c = points[0], points[len(points) // 2], points[-1]
        theta_1 = math.atan2(b[1] - a[1], b[0] - a[0])
        theta_2 = math.atan2(c[1] - b[1], c[0] - b[0])
        self.pose_pub.publish(Pose2D(x=float(a[0]), y=float(a[1]), theta=theta_1))
        self.omega_pub.publish(Pose2D(
            x=theta_1, y=theta_2, theta=(theta_2 - theta_1) / 0.1))
        self.received += 1
        if self.received % 50 == 0:
            print(f'BRIDGE_COUNT={self.received} WIDTH={len(points)}', flush=True)


def main():
    rclpy.init()
    node = LaneCloudToLyaPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
