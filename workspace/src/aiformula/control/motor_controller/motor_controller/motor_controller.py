#!/usr/bin/env python
import rclpy
import math
import json
import time
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from .gate import MotorGate
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from can_msgs.msg import Frame
from typing import List


class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.declare_parameter('wheel.tread')
        self.declare_parameter('wheel.diameter')
        self.declare_parameter('wheel.gear_ratio')
        self.declare_parameter('publish_timer_loop_duration')
        self.tread = self.get_parameter('wheel.tread').get_parameter_value().double_value
        self.diameter = self.get_parameter('wheel.diameter').get_parameter_value().double_value
        self.gear_ratio = self.get_parameter('wheel.gear_ratio').get_parameter_value().double_value
        publish_timer_loop_duration = self.get_parameter(
            'publish_timer_loop_duration').get_parameter_value().double_value

        buffer_size = 10
        self.motion_enabled = bool(self.declare_parameter('motion_enabled', False).value)
        frame = str(self.declare_parameter('signal_frame_id', 'zed_left_camera_optical_frame').value)
        self.manual_enable_button = int(self.declare_parameter('manual_enable_button', 6).value)
        self.manual_turbo_button = int(self.declare_parameter('manual_turbo_button', 4).value)
        self.gate = MotorGate(frame)
        self.manual_sub = self.create_subscription(Twist, 'sub_manual_command', self.manual_callback, 1)
        self.joy_sub = self.create_subscription(Joy, 'sub_manual_joy', self.joy_callback, 1)
        self.signal_sub = self.create_subscription(String, '/semantic_drive/signal_capture', self.signal_callback, 1)
        self.status_sub = self.create_subscription(String, '/semantic_drive/map_status', self.status_callback, 1)
        self.diagnostics_pub = self.create_publisher(String, '~/gate_status', 1)
        self.owners_ok = False
        self.manual_owners_ok = False
        self.last_diagnostics = -math.inf
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.twist_sub = self.create_subscription(Twist, 'sub_speed_command', self.twist_callback, 1)
        self.can_pub = self.create_publisher(Frame, 'pub_can', buffer_size)
        self.publish_timer = self.create_timer(publish_timer_loop_duration, self.publish_canframe_callback, clock=self.steady_clock)
        self.frame_msg = Frame()
        self.frame_msg.header.frame_id = "can0"        # Default can0
        self.frame_msg.id = 0x210                      # MotorControler CAN ID : 0x210
        self.frame_msg.dlc = 8

        # ---- Sigmoid 拟合参数（稳态点拟合）----
        # 左转(正向)：odom = Lp/(1+exp(-kp*(x-x0p))) - Cp
        self.Lp   = 1.53908994
        self.kp   = 3.15496243
        self.x0p  = 3.36664054
        self.Cp   = -0.0621119

        # 右转(负向)：odom = Ln/(1+exp(-kn*(x-x0n))) - Cn
        # 注：拟合得 x0n≈-3.2212，因此这里直接使用 x-x0n 形式
        self.Ln   = 1.68283261
        self.kn   = 2.91627673
        self.x0n  = -3.22124235
        self.Cn   = 1.6954783

    def twist_callback(self, msg):
        self.gate.command((msg.linear.x, msg.angular.z), False, time.monotonic())

    def manual_callback(self, msg):
        self.gate.command((msg.linear.x, msg.angular.z), True, time.monotonic())

    def joy_callback(self, msg):
        if len(msg.buttons) <= max(self.manual_enable_button, self.manual_turbo_button):
            return  # A malformed Joy must not release a latched manual takeover.
        held = any(0 <= i < len(msg.buttons) and msg.buttons[i] != 0
                   for i in (self.manual_enable_button, self.manual_turbo_button))
        self.gate.joy(held, time.monotonic())

    def signal_callback(self, msg):
        self.gate.signal(msg.data, self.get_clock().now().nanoseconds, time.monotonic())

    def status_callback(self, msg):
        self.gate.status(msg.data, self.get_clock().now().nanoseconds, time.monotonic())

    def publish_canframe_callback(self):
        now = time.monotonic()
        # Detect accidental parallel LYA/teleop publishers before selecting input.
        if now - self.last_diagnostics >= 0.1:
            self.owners_ok = len(self.get_publishers_info_by_topic(self.twist_sub.topic_name)) <= 1
            self.manual_owners_ok = len(self.get_publishers_info_by_topic(self.manual_sub.topic_name)) <= 1
        velocity = self.gate.output(self.get_clock().now().nanoseconds, now,
                                    self.motion_enabled, self.owners_ok, self.manual_owners_ok)
        try:
            rpm = self.toRefRPM(*velocity)
            self.frame_msg.data = self.toCanCmd(rpm[0]) + self.toCanCmd(rpm[1])
        except (ValueError, OverflowError):
            self.frame_msg.data = [0] * 8
            self.gate.reason = 'invalid_motor_conversion'
        self.frame_msg.header.stamp = self.get_clock().now().to_msg()
        self.can_pub.publish(self.frame_msg)
        if now - self.last_diagnostics >= 0.1:
            self.diagnostics_pub.publish(String(data=json.dumps(dict(
                reason=self.gate.reason, motion_enabled=self.motion_enabled,
                manual_held=self.gate.manual_held, red_latched=self.gate.red_latched,
                signal_state=self.gate.signal_state, command=list(velocity),
                can_data=[int(v) for v in self.frame_msg.data], owners_ok=self.owners_ok,
                manual_owners_ok=self.manual_owners_ok))))
            self.last_diagnostics = now

    # --------- 工具：Sigmoid 反函数（把 odom(desired) 反推 cmd）---------
    @staticmethod
    def _inv_sigmoid(y, L, k, x0, C):
        # 数值夹紧，避免log溢出：y ∈ (-C, L-C)
        eps = 1e-6
        y_clamped = np.clip(y, -C + eps, L - C - eps)
        return x0 - (1.0 / k) * np.log(L / (y_clamped + C) - 1.0)

#  Velocity -> RPM Calc
#  V_right = (V + tread/2 * w), V_left = (V - tread / 2 * w ) [m/s]
#  w_right = V/r + (d/2r) * w   [rad/s]
#  w_left = V/r - (d/2r) * w    [rad/s]
#  rpm = w * 60 / 2* pi [rpm]
#  rpm = rpm * gear_ratio  [rpm]

    def toRefRPM(self, linear_velocity, angular_velocity):  # Calc Motor ref rad/s
        # --- (0) 车辆层：理想 omega_des -> 反算需要的 omega_cmd（按正/负分别用Sigmoid） ---
        if not all(math.isfinite(v) for v in (linear_velocity, angular_velocity)):
            return [0.0, 0.0]
        if abs(linear_velocity) <= 1e-9 and abs(angular_velocity) <= 1e-9:
            return [0.0, 0.0]
        omega_des = angular_velocity
        # Zero desired yaw has no inverse in the positive sigmoid fit.
        # Bypass it for straight driving; preserve both nonzero steering fits.
        if abs(omega_des) <= 1e-9:
            omega_cmd = 0.0
        elif omega_des > 0.0:
            # 左转（正向）
            omega_cmd = self._inv_sigmoid(omega_des, self.Lp, self.kp, self.x0p, self.Cp)
        else:
            # 右转（负向）
            omega_cmd = self._inv_sigmoid(omega_des, self.Ln, self.kn, self.x0n, self.Cn)

        # --- (1) 二轮差速：由 v 与 omega_cmd 推右/左轮的目标角速度（车体坐标） ---
        wheel_angular_velocities = np.array([
            (linear_velocity / (self.diameter * 0.5)) + (self.tread / self.diameter) * omega_cmd,  # right[rad/s]
            (linear_velocity / (self.diameter * 0.5)) - (self.tread / self.diameter) * omega_cmd   # left[rad/s]
        ])

        # # 如需直接不补偿：
        # minute_to_second = 60
        # rpm_no_comp = wheel_angular_velocities * (minute_to_second / (2 * math.pi))
        # return (rpm_no_comp * self.gear_ratio).tolist()

        # --- (1.5) 轮速层补偿（保持你原有结构/参数/索引不变） ---
        # |w_meas| = a * max(|w_cmd| - w0, 0)  =>  w_cmd = sign(w_des)*(|w_des|/a + w0)
        a_left = 0.834
        w0_left = 2.76
        a_right = 0.844
        w0_right = 2.81

        # 注意：沿用你原始写法与索引（index 0 为 right，却命名为 wL_cmd）
        wL_cmd = np.sign(wheel_angular_velocities[0]) * (
            abs(wheel_angular_velocities[0]) / a_left + w0_left)

        wR_cmd = np.sign(wheel_angular_velocities[1]) * (
            abs(wheel_angular_velocities[1]) / a_right + w0_right)

        # --- (2) 角速度 -> RPM 并乘齿比 ---
        minutetosecond = 60.0
        rpm_left = (wL_cmd * (minutetosecond / (2.0 * np.pi))) * self.gear_ratio
        rpm_right = (wR_cmd * (minutetosecond / (2.0 * np.pi))) * self.gear_ratio
        rpm = np.array([rpm_left, rpm_right], dtype=float)
        return rpm.tolist()

    @staticmethod
    def toCanCmd(rpm: float) -> List[int]:
        rounded = round(rpm)
        bytes = rounded.to_bytes(4, "little", signed=True)
        return list(bytes)


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
