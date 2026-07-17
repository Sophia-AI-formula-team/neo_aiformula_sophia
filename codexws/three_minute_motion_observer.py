#!/usr/bin/env python3
import json
import math
import time

import rclpy
from can_msgs.msg import Frame
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2


class Observer(Node):
    def __init__(self):
        super().__init__('codex_three_minute_motion_observer')
        self.started = time.monotonic()
        self.counts = {}
        self.max_abs_linear = 0.0
        self.max_abs_angular = 0.0
        self.nonzero_cmd = 0
        self.reference_nonzero = 0
        self.cloud_widths = {name: set() for name in ('left', 'center', 'right')}
        self.create_subscription(Image, '/aiformula_sensing/zed_node/left_image/undistorted', lambda m: self.bump('camera'), 10)
        self.create_subscription(Image, '/aiformula_perception/lane_line_publisher/dynamic_roi', lambda m: self.bump('dynamic_roi'), 10)
        self.create_subscription(Image, '/aiformula_perception/lane_line_publisher/vehicle_fit_image', lambda m: self.bump('vehicle_fit'), 10)
        for name in ('left', 'center', 'right'):
            self.create_subscription(PointCloud2, f'/aiformula_perception/lane_line_publisher/lane_lines/{name}', lambda m, n=name: self.cloud(m, n), qos_profile_sensor_data)
        self.create_subscription(Pose2D, '/processed_point_a', lambda m: self.bump('processed_a'), 10)
        self.create_subscription(Pose2D, '/processed_point_b', lambda m: self.bump('processed_b'), 10)
        self.create_subscription(Pose2D, '/processed_point_c', lambda m: self.bump('processed_c'), 10)
        self.create_subscription(Pose2D, '/filtered_lane_pose', lambda m: self.bump('filtered_lane_pose'), 10)
        self.create_subscription(Pose2D, '/filtered_omega_t', lambda m: self.bump('filtered_omega_t'), 10)
        self.create_subscription(Odometry, '/aiformula_sensing/gyro_odometry_publisher/odom', lambda m: self.bump('odom'), 10)
        self.create_subscription(Twist, '/aiformula_control/game_pad/cmd_vel', self.cmd, 50)
        self.create_subscription(Frame, '/aiformula_control/motor_controller/reference_signal', self.reference, 100)
        self.create_subscription(Frame, '/aiformula_sensing/vehicle_info', lambda m: self.bump('vehicle_info'), 100)
        self.create_timer(180.0, self.finish)

    def bump(self, key):
        self.counts[key] = self.counts.get(key, 0) + 1

    def cloud(self, msg, name):
        self.bump(f'cloud_{name}')
        self.cloud_widths[name].add(int(msg.width))

    def cmd(self, msg):
        self.bump('cmd_vel')
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        self.max_abs_linear = max(self.max_abs_linear, abs(linear))
        self.max_abs_angular = max(self.max_abs_angular, abs(angular))
        if abs(linear) > 1e-6 or abs(angular) > 1e-6:
            self.nonzero_cmd += 1

    def reference(self, msg):
        self.bump('motor_reference')
        if any(int(v) != 0 for v in msg.data):
            self.reference_nonzero += 1

    def finish(self):
        result = {
            'elapsed_s': round(time.monotonic() - self.started, 3),
            'counts': self.counts,
            'cloud_widths': {k: sorted(v) for k, v in self.cloud_widths.items()},
            'cmd_vel_nonzero': self.nonzero_cmd,
            'cmd_vel_max_abs_linear_x': self.max_abs_linear,
            'cmd_vel_max_abs_angular_z': self.max_abs_angular,
            'motor_reference_nonzero_payload': self.reference_nonzero,
        }
        print('CODEX_RESULT=' + json.dumps(result, sort_keys=True), flush=True)
        rclpy.shutdown()


def main():
    rclpy.init()
    node = Observer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
