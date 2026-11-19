"""Foxy adapters for the single Concept CAN command path."""
import json
import math
import time

import rclpy
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist, TwistStamped
from motor_controller.gate import joystick_command
from rclpy.clock import Clock, ClockType
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from .core import Gate, ZERO, ZERO_FRAME
from .speed_setting import SpeedSetting

CAN_OUT = '/aiformula_control/motor_controller/reference_signal'
JOY = '/aiformula_control/joy_node/joy_l2_throttle'
MOTOR_IN = '/concept/internal/motor_can'
TWIST_OUT = '/concept/internal/motor_cmd'


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class GateNode(Node):
    def __init__(self):
        super().__init__('gate', namespace='/concept')
        self.speed_setting = SpeedSetting(self, 'max_auto_speed_mps', 5.)
        self.limit = self.speed_setting.initial
        if not self.valid_limit(self.limit):
            raise ValueError('max_auto_speed_mps must be in (0, 5]')
        self.add_on_set_parameters_callback(self.parameters)
        self.gate = Gate()
        self.state_t = -math.inf
        self.state_at = -math.inf
        self.sender = self.declare_parameter('can_sender_node', '/aiformula_control/socket_can_sender').value
        self.settings = {}
        for field, parameter, default in (
                ('enable_button', 'enable_button', 6), ('enable_turbo_button', 'enable_turbo_button', 4),
                ('axis_linear', 'axis_linear.x', 1), ('axis_angular', 'axis_angular.yaw', 3),
                ('scale_linear', 'scale_linear.x', 3.5), ('scale_linear_turbo', 'scale_linear_turbo.x', 3.6),
                ('scale_angular', 'scale_angular.yaw', .7), ('scale_angular_turbo', 'scale_angular_turbo.yaw', 1.)):
            self.settings[field] = self.declare_parameter(parameter, default).value
        self.cmd = self.create_publisher(Twist, TWIST_OUT, 1)
        self.can = self.create_publisher(Frame, CAN_OUT, 1)
        self.status = self.create_publisher(String, '/concept/gate/status', 1)
        self.create_subscription(String, '/concept/vehicle_state', self.vehicle_state, 1)
        self.create_subscription(TwistStamped, '/concept/auto_cmd', self.auto, 1)
        self.create_subscription(Joy, JOY, self.joy, qos_profile_sensor_data)
        self.create_subscription(Frame, MOTOR_IN, self.motor, 1)
        self.create_subscription(Bool, '/concept/gate/estop', self.estop, 1)
        self.create_service(SetBool, '/concept/gate/enable_output', self.enable)
        self.create_service(Trigger, '/concept/gate/resume', self.resume)
        self.create_service(Trigger, '/concept/gate/pause', self.pause)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02, self.tick, clock=self.steady)
        self.create_timer(.1, self.report, clock=self.steady)
        self.last_clock = None
        self.repeat_zero_until = -math.inf

    @staticmethod
    def valid_limit(value):
        return isinstance(value, float) and math.isfinite(value) and 0 < value <= 5.

    def parameters(self, params):
        for param in params:
            if param.name == 'speed_settings_directory':
                return SetParametersResult(successful=False, reason='Set speed_settings_directory at launch')
            if param.name == 'max_auto_speed_mps' and not self.valid_limit(param.value):
                return SetParametersResult(successful=False, reason='expected (0, 5] m/s')
        for param in params:
            if param.name == 'max_auto_speed_mps':
                try:
                    self.speed_setting.save(param.value)
                except (OSError, ValueError) as exc:
                    return SetParametersResult(successful=False, reason=str(exc))
                if param.value == self.limit:
                    continue
                self.limit = param.value
                self.gate.pause(self.now(), 'speed_limit_changed')
                self.stop()
        return SetParametersResult(successful=True)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_frame(self, data):
        msg = Frame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'can0'
        msg.id, msg.dlc, msg.data = 0x210, 8, list(data)
        self.can.publish(msg)

    def private_cmd(self, value):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = value
        self.cmd.publish(cmd)

    def stop(self):
        self.private_cmd(ZERO)
        self.publish_frame(ZERO_FRAME)
        if not self.gate.enabled:
            self.repeat_zero_until = time.monotonic() + .20

    def graph_ok(self):
        infos = self.get_subscriptions_info_by_topic(self.can.topic_name)
        subscribers = [s.node_namespace.rstrip('/') + '/' + s.node_name for s in infos]
        senders = [ns.rstrip('/') + '/' + name for name, ns in self.get_node_names_and_namespaces()
                   if 'socket_can_sender' in name or ns.rstrip('/') + '/' + name == self.sender]
        return (subscribers == [self.sender] and senders == [self.sender]
                and self.count_publishers(self.can.topic_name) == 1 and self.count_publishers(MOTOR_IN) == 1
                and self.count_publishers(self.cmd.topic_name) == 1
                and self.count_publishers('/concept/auto_cmd') <= 1
                and all(self.count_publishers('/concept/internal/unused_motor_' + name) == 0
                        for name in ('joy', 'session', 'map_status')))

    def fault(self, reason):
        self.gate.enabled = False
        self.gate.pause(self.now(), reason)
        self.stop()

    def vehicle_state(self, msg):
        try:
            data = json.loads(msg.data)
            stamp = data['t']
            valid = (data['valid'] is True and type(stamp) in (int, float)
                     and math.isfinite(stamp) and 0 <= self.now()-stamp <= .2)
        except (KeyError, ValueError, TypeError):
            valid = False
        if not valid:
            self.state_at = -math.inf
            if not self.gate.paused:
                self.gate.pause(self.now(), 'invalid_vehicle_state')
                self.stop()
        elif stamp > self.state_t:
            self.state_t = stamp
            self.state_at = time.monotonic()

    def state_fresh(self):
        return time.monotonic()-self.state_at <= .2 and 0 <= self.now()-self.state_t <= .2

    def joy(self, msg):
        try:
            held, command = joystick_command(msg.axes, msg.buttons, self.settings)
        except (ValueError, TypeError, IndexError):
            self.fault('invalid_joy')
            return
        old = self.gate.manual_held
        self.gate.joy(held, command, seconds(msg.header.stamp), self.now(), time.monotonic())
        if old != self.gate.manual_held:
            self.stop()

    def auto(self, msg):
        if (not self.gate.paused and not self.gate.manual_held
                and not -self.limit <= msg.twist.linear.x <= self.limit):
            self.gate.pause(self.now(), 'auto_speed_limit')
            self.stop()
            return
        if not all(map(math.isfinite, (msg.twist.linear.x, msg.twist.angular.z))):
            if not self.gate.paused:
                self.gate.pause(self.now(), 'invalid_auto_command')
                self.stop()
            return
        self.gate.command((msg.twist.linear.x, msg.twist.angular.z),
                          seconds(msg.header.stamp), self.now(), time.monotonic())

    def motor(self, msg):
        if msg.id != 0x210 or msg.dlc != 8 or msg.is_extended or msg.is_rtr or msg.is_error:
            self.fault('invalid_motor_frame')
            return
        if self.gate.enabled and not self.graph_ok():
            self.fault('graph_conflict')
            return
        if self.gate.enabled:
            self.publish_frame(self.gate.motor(bytes(msg.data), seconds(msg.header.stamp),
                                                self.now(), time.monotonic()))
            if not self.gate.enabled:
                self.repeat_zero_until = time.monotonic() + .20

    def tick(self):
        stamp = self.now()
        if self.last_clock is not None and stamp < self.last_clock:
            self.fault('clock_rewound')
        self.last_clock = stamp
        if self.gate.enabled and not self.graph_ok():
            self.fault('graph_conflict')
        if not self.gate.paused and not self.state_fresh():
            self.gate.pause(stamp, 'vehicle_state_timeout')
        was_enabled = self.gate.enabled
        cmd = self.gate.tick(stamp, time.monotonic())
        self.private_cmd(cmd)
        if was_enabled and not self.gate.enabled:
            self.repeat_zero_until = time.monotonic() + .20
        if cmd == ZERO and (self.gate.enabled or time.monotonic() < self.repeat_zero_until):
            self.publish_frame(ZERO_FRAME)

    def estop(self, msg):
        self.gate.emergency(msg.data, self.now())
        if msg.data:
            self.stop()

    def enable(self, request, response):
        ok = not request.data or self.graph_ok()
        response.success = ok and self.gate.enable(request.data, self.now())
        self.stop()
        response.message = self.gate.reason if ok else 'graph_conflict'
        return response

    def pause(self, _, response):
        self.gate.pause(self.now(), 'operator_pause')
        self.stop()
        response.success, response.message = True, self.gate.reason
        return response

    def resume(self, _, response):
        response.success = self.state_fresh() and self.gate.resume(self.now(), time.monotonic())
        response.message = self.gate.reason if response.success else 'resume_interlocked'
        return response

    def report(self):
        g = self.gate
        mode = ('estop' if g.estop else 'disabled' if not g.enabled else 'manual'
                if g.manual_held else 'paused' if g.paused else 'auto')
        self.status.publish(String(data=json.dumps(dict(output_enabled=g.enabled, paused=g.paused,
            manual_held=g.manual_held, estop=g.estop, mode=mode, reason=g.reason,
            t=self.now(), max_auto_speed_mps=self.limit, linear_mps=g.sent[0], yaw_rps=g.sent[1],
            manual_yaw_limit_rps=max(abs(self.settings['scale_angular']), abs(self.settings['scale_angular_turbo'])),
            auto_fresh=not g.paused and g.auto_is_fresh(self.now(), time.monotonic())))))


def main(args=None):
    rclpy.init(args=args)
    node = GateNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
