"""Consume simulated CAN frames in isolation. This node never opens a CAN device."""
import rclpy
from can_msgs.msg import Frame
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    node = Node('virtual_receiver', namespace='/concept/replay')
    node.create_subscription(Frame, '/concept/replay/reference_signal', lambda _: None, 1)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
