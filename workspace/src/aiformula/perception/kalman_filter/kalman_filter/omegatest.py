#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')

        # ---- Publisher ----
        self.pub = self.create_publisher(Twist, '/aiformula_control/game_pad/cmd_vel', 10)

        # ---- 定时器参数 ----
        self.publish_rate = 0.1   # 发布频率 10 Hz
        self.update_interval = 1.5  # 每隔 1.5s 更新一次 omega
        self.timer_publish = self.create_timer(self.publish_rate, self.publish_callback)
        self.timer_update = self.create_timer(self.update_interval, self.update_omega_callback)

        # ---- 初始化变量 ----
        self.v = 2.0
        self.omega = -0.5
        self.max_omega = -5.0
        self.msg = Twist()
        self.stopped = False  # 标志是否已经停止

        self.get_logger().info("CmdVelPublisher started: v=2.0, ω starts at 0.5")

    def publish_callback(self):
        """以固定频率发布 cmd_vel"""
        self.msg.linear.x = self.v if not self.stopped else 0.0
        self.msg.angular.z = self.omega if not self.stopped else 0.0
        self.pub.publish(self.msg)
        self.get_logger().info(f"Publishing cmd_vel: v={self.msg.linear.x:.2f}, ω={self.msg.angular.z:.2f}")

    def update_omega_callback(self):
        """每1.5秒更新一次omega"""
        if self.stopped:
            return  # 已经停止则不再更新

        self.omega += -0.2
        if self.omega < self.max_omega:
            # 超过5后，等待一次周期后停止
            self.get_logger().info("Reached ω > 5. Will stop after this cycle.")
            self.stopped = True
            self.v = 0.0
            self.omega = 0.0
        else:
            self.get_logger().info(f"Omega increased to {self.omega:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
