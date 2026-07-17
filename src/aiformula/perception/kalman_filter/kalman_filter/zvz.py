#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class VelocityLogger(Node):
    def __init__(self):
        super().__init__('velocity_logger')

        # ===== 参数配置 =====
        self.declare_parameter('sample_hz', 20.0)
        self.declare_parameter('out_dir', '')
        sample_hz = float(self.get_parameter('sample_hz').value)
        out_dir = self.get_parameter('out_dir').value or str(Path.cwd())

        # ===== 订阅话题 =====
        self.sub_cmd = self.create_subscription(Twist, '/aiformula_control/game_pad/cmd_vel', self.cb_cmd, 10)
        self.sub_odom = self.create_subscription(Odometry, '/aiformula_sensing/gyro_odometry_publisher/odom', self.cb_odom, 10)

        # ===== 最新值缓存 =====
        self.cmd_lin_x = None
        self.cmd_ang_z = None
        self.odom_lin_x = None
        self.odom_ang_z = None
        self.last_cmd_time = None
        self.last_odom_time = None

        # ===== CSV 文件初始化 =====
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = Path(out_dir) / f'velocity_log_{ts}.csv'
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'wall_time_iso',
            'cmd_linear_x (m/s)',
            'cmd_angular_z (rad/s)',
            'odom_linear_x (m/s)',
            'odom_angular_z (rad/s)',
            'cmd_age_ms',
            'odom_age_ms'
        ])
        self.csv_file.flush()

        # ===== 定时采样 =====
        period = 1.0 / sample_hz
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(f'Logging to: {self.csv_path}')
        self.get_logger().info(f'Sampling at {sample_hz:.2f} Hz')

    # ---------- 回调函数 ----------
    def cb_cmd(self, msg: Twist):
        self.cmd_lin_x = msg.linear.x
        self.cmd_ang_z = msg.angular.z
        self.last_cmd_time = datetime.now(timezone.utc)

    def cb_odom(self, msg: Odometry):
        self.odom_lin_x = msg.twist.twist.linear.x
        self.odom_ang_z = msg.twist.twist.angular.z
        self.last_odom_time = datetime.now(timezone.utc)

    # ---------- 定时写入 ----------
    def on_timer(self):
        now = datetime.now(timezone.utc)
        cmd_age = (now - self.last_cmd_time).total_seconds() * 1000 if self.last_cmd_time else None
        odom_age = (now - self.last_odom_time).total_seconds() * 1000 if self.last_odom_time else None

        row = [
            now.isoformat(),
            '' if self.cmd_lin_x is None else f'{self.cmd_lin_x:.6f}',
            '' if self.cmd_ang_z is None else f'{self.cmd_ang_z:.6f}',
            '' if self.odom_lin_x is None else f'{self.odom_lin_x:.6f}',
            '' if self.odom_ang_z is None else f'{self.odom_ang_z:.6f}',
            '' if cmd_age is None else f'{cmd_age:.1f}',
            '' if odom_age is None else f'{odom_age:.1f}'
        ]
        self.writer.writerow(row)
        self.csv_file.flush()

    def destroy_node(self):
        try:
            if hasattr(self, 'csv_file'):
                self.csv_file.flush()
                self.csv_file.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = VelocityLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
