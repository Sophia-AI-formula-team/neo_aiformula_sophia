#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class StationaryRuntimeOdom(Node):
    def __init__(self):
        super().__init__('codex_stationary_runtime_odom')
        self.publisher = self.create_publisher(
            Odometry, '/aiformula_sensing/gyro_odometry_publisher/odom', 20)
        self.create_timer(0.05, self.publish_odom)

    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.orientation.w = 1.0
        self.publisher.publish(odom)


def main():
    rclpy.init()
    node = StationaryRuntimeOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
