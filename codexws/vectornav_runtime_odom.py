#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class RuntimeOdom(Node):
    def __init__(self):
        super().__init__('codex_vectornav_runtime_odom')
        self.publisher = self.create_publisher(
            Odometry, '/aiformula_sensing/gyro_odometry_publisher/odom', 20)
        self.create_subscription(
            Imu, '/aiformula_sensing/vectornav/imu', self.imu_callback,
            qos_profile_sensor_data)

    def imu_callback(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.orientation = msg.orientation
        odom.twist.twist.angular = msg.angular_velocity
        self.publisher.publish(odom)


def main():
    rclpy.init()
    node = RuntimeOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
