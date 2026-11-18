import json
import time

import rclpy
from can_msgs.msg import Frame
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from .core import VehicleState


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class StateNode(Node):
    def __init__(self):
        super().__init__('concept_vehicle_state')
        self.state = VehicleState()
        self.publisher = self.create_publisher(String, '/concept/vehicle_state', 1)
        self.create_subscription(Frame, '/aiformula_sensing/vehicle_info', self.wheels, qos_profile_sensor_data)
        self.create_subscription(Imu, '/aiformula_sensing/vectornav/imu', self.gyro, qos_profile_sensor_data)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05, self.report, clock=self.steady)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def gyro(self, msg):
        if msg.angular_velocity_covariance[0] < 0 or not msg.header.frame_id:
            self.state.gyros.clear()
            return
        self.state.gyro(msg.angular_velocity.z, seconds(msg.header.stamp), self.now(), time.monotonic())

    def wheels(self, msg):
        self.state.wheels(msg.data, seconds(msg.header.stamp), self.now(), time.monotonic(),
                          frame_id=msg.id, dlc=msg.dlc, is_rtr=msg.is_rtr,
                          is_extended=msg.is_extended, is_error=msg.is_error)

    def report(self):
        data = self.state.sample(self.now(), time.monotonic())
        self.publisher.publish(String(data=json.dumps(data, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = StateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
