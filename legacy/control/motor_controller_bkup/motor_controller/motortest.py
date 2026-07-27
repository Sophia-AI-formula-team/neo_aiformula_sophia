#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import List
import math
import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame

class ManualWheelToCAN(Node):
    """
    手动设定左右轮角速度 (wl, wr) [rad/s]，直接转换为 CAN 帧并发布到 `pub_can`。
    与原 MotorController 并行运行：不占用 `sub_speed_command`，互不干扰。
    关闭本节点后，原节点无需重启即可正常工作。
    """

    def __init__(self):
        super().__init__('manual_wheel_to_can')

        # === 读取参数 ===
        # 必要：齿比与发布频率；轮径/轮距不需要（直接用 wl/wr）
 #       self.gear_ratio = float(get_ros_parameter(self, 'wheel.gear_ratio', default_value=1.0))
     
        self.wl = 10
        self.wr = 10
        self.publish_hz = 50
        self.enabled = bool(True)

        # 动态参数更新
        self.add_on_set_parameters_callback(self._on_param_change)

        # === Publisher ===
        self.can_pub = self.create_publisher(Frame, 'pub_can', 10)
        self.wheel_publeft = self.create_publisher(Frame, 'wheelleft', 10)
        self.wheel_pubright = self.create_publisher(Frame, 'wheelright', 10)

        # 预构造 Frame 模板（与原节点一致）
        self.frame_msg = Frame()
        self.frame_msg.header.frame_id = 'can0'
        self.frame_msg.id = 0x210
        self.frame_msg.dlc = 8

        # 定时发布
        period = 1.0 / max(self.publish_hz, 1.0)
        self.timer = self.create_timer(period, self._publish_can)

        self.get_logger().info(
            f'[manual_wheel_to_can] 启动 | wl={self.wl:.3f} rad/s, wr={self.wr:.3f} rad/s, hz={self.publish_hz:.1f}, enabled={self.enabled}'
        )

    # === 工具：rad/s -> rpm（带齿比），再转 4 字节 little-endian（有符号） ===
    @staticmethod
    def _rpm_to_bytes(rpm: float) -> List[int]:
        rounded = int(round(rpm))
        b = rounded.to_bytes(4, 'little', signed=True)
        return list(b)

    def _publish_can(self):
        if not self.enabled:
            return

        wl = float(self.wl)
        wr = float(self.wr)

 
        # rad/s -> rpm
        minute_to_second = 60.0
        wl_rpm = wl * (minute_to_second / (2.0 * math.pi))
        wr_rpm = wr * (minute_to_second / (2.0 * math.pi))

        # 乘以齿比
        wl_cmd_rpm = wl_rpm
        wr_cmd_rpm = wr_rpm

        # 编码：右4字节 + 左4字节（与原代码一致）
        cmd_right = self._rpm_to_bytes(wr_cmd_rpm)
        cmd_left = self._rpm_to_bytes(wl_cmd_rpm)
        self.frame_msg.data = cmd_right + cmd_left

        # 发布
        self.can_pub.publish(self.frame_msg)
        self.wheel_publeft.publish(wl_rpm)
        self.wheel_pubright.publish(wr_rpm)
    # 动态参数
    def _on_param_change(self, params):
        for p in params:
            if p.name == 'manual.wl' and p.type_ is not None:
                self.wl = float(p.value)
                self.get_logger().info(f'更新 wl = {self.wl:.3f} rad/s')
            elif p.name == 'manual.wr' and p.type_ is not None:
                self.wr = float(p.value)
                self.get_logger().info(f'更新 wr = {self.wr:.3f} rad/s')
            elif p.name == 'publish_hz' and p.type_ is not None:
                self.publish_hz = float(p.value)
                new_period = 1.0 / max(self.publish_hz, 1.0)
                self.timer.cancel()
                self.timer = self.create_timer(new_period, self._publish_can)
                self.get_logger().info(f'更新发布频率 = {self.publish_hz:.1f} Hz')
            elif p.name == 'enabled' and p.type_ is not None:
                self.enabled = bool(p.value)
                self.get_logger().info(f'更新 enabled = {self.enabled}')
        return rclpy.parameter.SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = ManualWheelToCAN()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
