from __future__ import annotations

import math
import random
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


CMD_TOPIC = "/aiformula_control/game_pad/cmd_vel"


class SineCmdPublisher(Node):
    def __init__(self) -> None:
        super().__init__("sine_cmd_publisher")
        self.declare_parameter("topic", CMD_TOPIC)
        self.declare_parameter("speed_min", 1.0)
        self.declare_parameter("speed_max", 5.0)
        self.declare_parameter("wavelength_min", 10.0)
        self.declare_parameter("wavelength_max", 35.0)
        self.declare_parameter("heading_amplitude", 0.35)
        self.declare_parameter("max_omega", 0.35)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("publish_stop_at_end", True)
        self.declare_parameter("random_seed", -1)

        self.topic = str(self.get_parameter("topic").value or CMD_TOPIC)
        self.speed_min = float(self.get_parameter("speed_min").value)
        self.speed_max = float(self.get_parameter("speed_max").value)
        if self.speed_max < self.speed_min:
            self.speed_min, self.speed_max = self.speed_max, self.speed_min
        self.wavelength_min = max(1e-6, float(self.get_parameter("wavelength_min").value))
        self.wavelength_max = max(self.wavelength_min, float(self.get_parameter("wavelength_max").value))
        self.heading_amplitude = abs(float(self.get_parameter("heading_amplitude").value))
        self.max_omega = max(1e-6, abs(float(self.get_parameter("max_omega").value)))
        self.publish_stop_at_end = bool(self.get_parameter("publish_stop_at_end").value)
        random_seed = int(self.get_parameter("random_seed").value)
        self.rng = random.Random(None if random_seed < 0 else random_seed)
        rate = max(1e-6, float(self.get_parameter("publish_rate_hz").value))

        self.cmd_pub = self.create_publisher(Twist, self.topic, 10)
        self.stopped = False
        self.command_count = 0
        self.segment_index = 0
        self.segment_start = time.monotonic()
        self.speed = 0.0
        self.wavelength = 0.0
        self.segment_duration = 1.0
        self.omega_peak = 0.0
        self.start_new_segment(self.segment_start)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().info(
            f"Publishing endless randomized sine cmd_vel to {self.topic}: "
            f"speed=[{self.speed_min}, {self.speed_max}] m/s, "
            f"wavelength=[{self.wavelength_min}, {self.wavelength_max}] m, max_omega={self.max_omega}"
        )

    def start_new_segment(self, now: float) -> None:
        self.segment_index += 1
        self.segment_start = now
        self.speed = self.rng.uniform(self.speed_min, self.speed_max)
        safe_min_wavelength = (self.heading_amplitude * 2.0 * math.pi * self.speed) / self.max_omega
        wavelength_low = max(self.wavelength_min, safe_min_wavelength)
        wavelength_high = max(wavelength_low, self.wavelength_max)
        self.wavelength = self.rng.uniform(wavelength_low, wavelength_high)
        self.segment_duration = max(1e-3, self.wavelength / max(self.speed, 1e-6))
        self.omega_peak = min(
            self.max_omega,
            (self.heading_amplitude * 2.0 * math.pi * self.speed) / self.wavelength,
        )
        self.get_logger().info(
            f"segment={self.segment_index} v={self.speed:.3f} m/s "
            f"wavelength={self.wavelength:.3f} m duration={self.segment_duration:.3f} s "
            f"omega_peak={self.omega_peak:.3f} rad/s"
        )

    def control_loop(self) -> None:
        now = time.monotonic()
        elapsed = now - self.segment_start
        if elapsed >= self.segment_duration:
            self.start_new_segment(now)
            elapsed = 0.0

        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = self.omega_peak * math.sin(2.0 * math.pi * elapsed / self.segment_duration)
        self.cmd_pub.publish(msg)
        self.command_count += 1
        if self.command_count == 1:
            self.get_logger().info(f"First sine command published on {self.topic}.")

    def publish_stop(self) -> None:
        if self.publish_stop_at_end and not self.stopped:
            self.cmd_pub.publish(Twist())
            self.stopped = True
            self.get_logger().info("Sine command complete; published stop command.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SineCmdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
