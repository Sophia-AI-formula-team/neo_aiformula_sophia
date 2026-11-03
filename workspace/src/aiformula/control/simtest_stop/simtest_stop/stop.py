#!/usr/bin/env python3
"""2_正常零速停车：只发cmd_vel，机器人和bag由操作者另外启动。"""
import signal
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

CMD_TOPIC = '/aiformula_control/game_pad/cmd_vel'
PUBLISH_PERIOD_S = 0.1  # 10Hz，正常发布和退出补发零速统一使用
# 每项为（持续秒数，命令速度m/s），转向命令始终为0。
# 第12秒从2m/s命令直接切到0，观察10秒；不假定实际速度已达2m/s或已停稳。
# 主序列22秒；命令积分9米，实际行驶和停车距离仍需现场留余量。
STAGES = [(5, 0.0), (2, 0.5), (2, 1.0), (3, 2.0), (10, 0.0)]


class ZeroStopTest(Node):
    def __init__(self):
        super().__init__('test02_stop')
        self.publisher = self.create_publisher(Twist, CMD_TOPIC, 1)
        self.started = time.monotonic()
        self.stage = -1
        self.finished = False
        self.timer = self.create_timer(PUBLISH_PERIOD_S, self.update)
        self.get_logger().info('测试开始：先发5秒零速，逐档升至2m/s命令，再直接给零速。')

    def publish_speed(self, speed):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = 0.0
        self.publisher.publish(msg)

    def update(self):
        now = time.monotonic()
        elapsed = now - self.started
        end = 0.0
        for index, (duration, speed) in enumerate(STAGES):
            end += duration
            if elapsed < end:
                if index != self.stage:
                    self.get_logger().info(f'{elapsed:.1f}s：v={speed:.1f} m/s，omega=0')
                    if index == len(STAGES) - 1:
                        self.get_logger().info('进入零速停车观察段，继续发布零速。')
                    self.stage = index
                self.publish_speed(speed)
                return
      ##  self.publish_speed(0.0)
       ## self.finished = True
        self.get_logger().info('22秒命令序列完成；请核对实际停车情况。')


def main(args=None):
    # 保留ROS连接，让Ctrl+C后仍能补发零速。
    rclpy.init(args=args)
    node = ZeroStopTest()
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
        # 退出前补发1秒零速；这是发令，不是实车停车确认。
        previous_int = signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            deadline = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < deadline:
                node.publish_speed(0.0)
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
