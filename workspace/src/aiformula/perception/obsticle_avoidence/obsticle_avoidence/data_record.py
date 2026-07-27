#!/usr/bin/env python3
from __future__ import annotations

import csv
import datetime as _dt
from pathlib import Path

import psutil
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class DataRecorder(Node):
    """记录线速度、角速度、CPU占用、内存占用。"""

    def __init__(self) -> None:
        super().__init__("data_recorder")

        self._clock = self.get_clock()

        timestamp_str = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = Path(f"log_{timestamp_str}")
        out_dir.mkdir(exist_ok=True)

        self.data_f = open(out_dir / "system_and_vel.csv", "w", newline="")
        self.writer = csv.writer(self.data_f)
        self.writer.writerow(["timestamp", "v", "omega", "cpu_percent", "mem_percent"])

        cmd_vel_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            Twist,
            "/aiformula_control/game_pad/cmd_vel",
            self.cb_vel,
            cmd_vel_qos,
        )

        # 预热一次，避免第一次 cpu_percent 结果异常
        psutil.cpu_percent(interval=None)

    def _now(self) -> float:
        stamp = self._clock.now()
        return stamp.nanoseconds * 1e-9

    def cb_vel(self, msg: Twist) -> None:
        t = self._now()
        v = msg.linear.x
        omega = msg.angular.z
        cpu = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory().percent

        self.writer.writerow([t, v, omega, cpu, mem])

    def destroy_node(self) -> None:
        try:
            self.data_f.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()