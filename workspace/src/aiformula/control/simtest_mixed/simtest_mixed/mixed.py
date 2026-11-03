#!/usr/bin/env python3
"""4_变速转向组合验证：只发cmd_vel，机器人和bag由操作者另外启动。"""
import signal
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

CMD_TOPIC = '/aiformula_control/game_pad/cmd_vel'
PUBLISH_PERIOD_S = 0.1  # 10Hz，正常发布和退出补发零速统一使用
# 每项为（持续秒数，线速度命令m/s，角速度命令rad/s）。
# 用本项bag验证前三项拟合结果；不把本项用于同一轮参数拟合。
# 主序列20秒，命令路程积分11米；实际距离与转向均以实车记录为准。
STAGES = [(5, 0.0, 0.0), (2, 0.5, 0.0), (2, 1.5, 0.15),
          (2, 2.0, 0.0), (2, 1.0, -0.15), (2, 0.5, 0.0), (5, 0.0, 0.0)]


class CombinedValidationTest(Node):
    def __init__(self):
        super().__init__('test04_validation')
        self.publisher = self.create_publisher(Twist, CMD_TOPIC, 1)
        self.started = time.monotonic()
        self.stage = -1
        self.finished = False
        self.timer = self.create_timer(PUBLISH_PERIOD_S, self.update)
        self.get_logger().info('4_变速转向组合验证：先发5秒零速，然后执行预设组合动作。')

    def publish_command(self, speed, yaw_rate):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(yaw_rate)
        self.publisher.publish(msg)

    def update(self):
        now = time.monotonic()
        elapsed = now - self.started
        end = 0.0
        for index, (duration, speed, yaw_rate) in enumerate(STAGES):
            end += duration
            if elapsed < end:
                if index != self.stage:
                    self.get_logger().info(f'{elapsed:.1f}s：v={speed:.1f} m/s，omega={yaw_rate:+.2f} rad/s')
                    self.stage = index
                self.publish_command(speed, yaw_rate)
                return
       ## self.publish_command(0.0, 0.0)
        ##self.finished = True
        self.get_logger().info('20秒命令序列完成；请核对实际停车情况。')


def main(args=None):
    # 保留ROS连接，让Ctrl+C后仍能补发零速和零转向。
    rclpy.init(args=args)
    node = CombinedValidationTest()
    previous_term = signal.getsignal(signal.SIGTERM)

    def stop_requested(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop_requested)
    interrupted = False
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=PUBLISH_PERIOD_S)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        # 退出前补发1秒零速和零转向；这是发令，不是实车停车确认。
        previous_int = signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            deadline = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < deadline:
                node.publish_command(0.0, 0.0)
                time.sleep(PUBLISH_PERIOD_S)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            signal.signal(signal.SIGINT, previous_int)
            signal.signal(signal.SIGTERM, previous_term)
    return 1 if interrupted else 0


if __name__ == '__main__':
    raise SystemExit(main())
