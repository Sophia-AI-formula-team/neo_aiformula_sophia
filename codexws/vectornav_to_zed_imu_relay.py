#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuRelay(Node):
    def __init__(self):
        super().__init__('codex_vectornav_to_zed_imu_relay')
        self.publisher = self.create_publisher(Imu, '/aiformula_sensing/zed_node/imu', 10)
        self.create_subscription(Imu, '/aiformula_sensing/vectornav/imu', self.publisher.publish, 10)


def main():
    rclpy.init()
    node = ImuRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
