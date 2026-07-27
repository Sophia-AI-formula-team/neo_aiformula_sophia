#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class L2ThrottleJoyRemap(Node):
    def __init__(self):
        super().__init__("l2_throttle_joy_remap")
        self.declare_parameter("forward_axis", 2)
        self.declare_parameter("reverse_axis", 5)
        self.declare_parameter("target_axis", 1)
        self.declare_parameter("enable_button", 6)
        self.declare_parameter("mode_switch_axis", 7)
        self.declare_parameter("mode_switch_threshold", 0.5)
        self.declare_parameter("deadband", 0.01)
        self.declare_parameter("released_value", 1.0)
        self.declare_parameter("pressed_value", -1.0)

        self.forward_axis = int(self.get_parameter("forward_axis").value)
        self.reverse_axis = int(self.get_parameter("reverse_axis").value)
        self.target_axis = int(self.get_parameter("target_axis").value)
        self.enable_button = int(self.get_parameter("enable_button").value)
        self.mode_switch_axis = int(self.get_parameter("mode_switch_axis").value)
        self.mode_switch_threshold = float(self.get_parameter("mode_switch_threshold").value)
        self.deadband = float(self.get_parameter("deadband").value)
        self.released_value = float(self.get_parameter("released_value").value)
        self.pressed_value = float(self.get_parameter("pressed_value").value)
        self.trigger_mode = True

        self.publisher = self.create_publisher(Joy, "joy_out", 10)
        self.subscription = self.create_subscription(Joy, "joy_in", self.joy_callback, 10)

    @staticmethod
    def _ensure_len(values, index, fill_value):
        while len(values) <= index:
            values.append(fill_value)

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))

    def _trigger_to_throttle(self, raw_value):
        span = self.released_value - self.pressed_value
        if abs(span) < 1e-6 or not math.isfinite(raw_value):
            return 0.0

        throttle = (self.released_value - raw_value) / span
        throttle = self._clamp(throttle, 0.0, 1.0)
        if throttle < self.deadband:
            return 0.0
        return throttle

    def _update_mode(self, axes):
        self._ensure_len(axes, self.mode_switch_axis, 0.0)
        switch_value = axes[self.mode_switch_axis]
        old_mode = self.trigger_mode
        if switch_value > self.mode_switch_threshold:
            self.trigger_mode = True
        elif switch_value < -self.mode_switch_threshold:
            self.trigger_mode = False

        if self.trigger_mode != old_mode:
            mode_name = "trigger throttle" if self.trigger_mode else "original passthrough"
            self.get_logger().warning(f"Joystick control style switched to {mode_name}.")

    def joy_callback(self, msg):
        axes = list(msg.axes)
        buttons = list(msg.buttons)
        self._update_mode(axes)
        if not self.trigger_mode:
            self.publisher.publish(msg)
            return

        self._ensure_len(axes, self.forward_axis, self.released_value)
        self._ensure_len(axes, self.reverse_axis, self.released_value)
        self._ensure_len(axes, self.target_axis, 0.0)
        self._ensure_len(buttons, self.enable_button, 0)

        forward = self._trigger_to_throttle(axes[self.forward_axis])
        reverse = self._trigger_to_throttle(axes[self.reverse_axis])
        command = self._clamp(forward - reverse, -1.0, 1.0)
        if abs(command) < self.deadband:
            command = 0.0

        axes[self.target_axis] = command
        buttons[self.enable_button] = 1 if abs(command) > 0.0 else 0

        out = Joy()
        out.header = msg.header
        out.axes = axes
        out.buttons = buttons
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = L2ThrottleJoyRemap()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
