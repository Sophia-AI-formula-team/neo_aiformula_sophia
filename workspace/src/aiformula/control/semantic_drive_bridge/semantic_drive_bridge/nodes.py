"""Real rclpy nodes. Importing this module requires an installed ROS 2 runtime."""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .policy import CommandWatchdogPolicy, Rejected, SignalCapturePolicy, ZERO, validate_shadow_topic


def latest_qos():
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def parameter(node, name, default):
    return node.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value


def json_message(value):
    return String(data=json.dumps(value, allow_nan=False, separators=(",", ":")))


def twist_message(values):
    message = Twist()
    (message.linear.x, message.linear.y, message.linear.z,
     message.angular.x, message.angular.y, message.angular.z) = values
    return message


class SignalCaptureBridge(Node):
    def __init__(self):
        super().__init__("signal_capture_bridge")
        try:
            self.policy = SignalCapturePolicy(
                parameter(self, "expected_frame_id", ""),
                parameter(self, "max_age_sec", 0.5))
            input_topic = parameter(self, "input_topic", "/semantic_drive/signal_capture")
            output_topic = parameter(self, "output_topic", "/semantic_drive/map_signal")
            diagnostic_topic = parameter(self, "diagnostics_topic", "/semantic_drive/signal_bridge_status")
            self.publisher = self.create_publisher(String, output_topic, latest_qos())
            self.diagnostic_publisher = self.create_publisher(String, diagnostic_topic, latest_qos())
            self.subscription = self.create_subscription(String, input_topic, self.receive, latest_qos())
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(0.05, self.poll, clock=self.steady_clock)
            # Sample at startup too, so a rollback before the first input latches.
            self.policy.clock.observe(self.get_clock().now().nanoseconds, time.monotonic())
        except Exception:
            self.destroy_node()
            raise

    def receive(self, message):
        try:
            observation = self.policy.convert(
                message.data, self.get_clock().now().nanoseconds, time.monotonic())
        except Rejected:
            # No synthetic unknown and no re-publication of the last valid frame.
            return
        self.publisher.publish(json_message(observation))

    def poll(self):
        diagnostics = self.policy.poll(self.get_clock().now().nanoseconds, time.monotonic())
        diagnostics.update(clock_age_basis="receiver_ROS_clock_minus_source_header",
                           watchdog_clock="steady", restart_policy="full_chain_after_clock_rollback")
        self.diagnostic_publisher.publish(json_message(diagnostics))


class CommandWatchdog(Node):
    def __init__(self):
        super().__init__("command_watchdog")
        self.publisher = None
        self.output_validated = False
        try:
            self.policy = CommandWatchdogPolicy(parameter(self, "timeout_sec", 0.25))
            input_topic = parameter(self, "input_topic", "/semantic_drive/gated_cmd_vel")
            output_topic = validate_shadow_topic(parameter(self, "output_topic", "/semantic_drive/shadow_cmd_vel"))
            status_topic = parameter(self, "status_topic", "/semantic_drive/map_status")
            diagnostic_topic = parameter(self, "diagnostics_topic", "/semantic_drive/watchdog_status")
            self.publisher = self.create_publisher(Twist, output_topic, latest_qos())
            # Publisher.topic_name is the ROS-resolved name, including CLI remaps.
            # Do not publish even a shutdown message if a remap escaped shadow.
            validate_shadow_topic(self.publisher.topic_name)
            self.output_validated = True
            self.diagnostic_publisher = self.create_publisher(String, diagnostic_topic, latest_qos())
            self.command_subscription = self.create_subscription(Twist, input_topic, self.receive_command, latest_qos())
            self.status_subscription = self.create_subscription(String, status_topic, self.receive_status, latest_qos())
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(0.05, self.poll, clock=self.steady_clock)
            self.policy.clock.observe(self.get_clock().now().nanoseconds, time.monotonic())
            self.publish_zero()
        except Exception:
            self.destroy_node()
            raise

    def receive_command(self, message):
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        try:
            self.policy.receive_command(values, self.get_clock().now().nanoseconds, time.monotonic())
        except Rejected:
            self.publish_zero()

    def receive_status(self, message):
        try:
            self.policy.receive_status(message.data, self.get_clock().now().nanoseconds, time.monotonic())
        except Rejected:
            self.publish_zero()
            return
        if not self.policy.status["allowed"]:
            self.publish_zero()

    def poll(self):
        steady = time.monotonic()
        values = self.policy.output(self.get_clock().now().nanoseconds, steady)
        self.publisher.publish(twist_message(values))
        self.diagnostic_publisher.publish(json_message(self.policy.diagnostics(steady)))

    def publish_zero(self):
        if self.publisher is not None and self.output_validated:
            self.publisher.publish(twist_message(ZERO))

    def destroy_node(self):
        # Best effort only: SIGKILL, power loss, DDS delivery and motor watchdogs
        # are outside the guarantees of an application-side publisher.
        try:
            self.publish_zero()
        except Exception:
            pass
        return super().destroy_node()


def _main(node_type, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = node_type()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def signal_capture_main(args=None):
    _main(SignalCaptureBridge, args)


def command_watchdog_main(args=None):
    _main(CommandWatchdog, args)
