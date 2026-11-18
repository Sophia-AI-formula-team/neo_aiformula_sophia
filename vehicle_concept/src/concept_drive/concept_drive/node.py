"""Sensor-only V6.26 decision node. Physical output belongs to /concept/gate."""
import json
import math
import time
from threading import Lock
from uuid import uuid4

import cv2
import numpy as np
from geometry_msgs.msg import TwistStamped
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from concept_yolop.frame_codec import parse_frame_header, decode_frame_data
from .core import Drive
from .transport import Feedback, native_sensor, ros_command, stamp

CAPTURE_JITTER_NS = 10_000_000
MAX_CAPTURE_AGE = .5
MAX_STATE_AGE = .15


class DriverNode(Node):
    def __init__(self):
        super().__init__('drive', namespace='/concept')
        cv2.setNumThreads(1)
        # OpenCV builds the Lab conversion tables on first use. Do this before
        # accepting images, without creating road observations or a command.
        cv2.cvtColor(np.zeros((1, 1, 3), np.uint8), cv2.COLOR_BGR2LAB)
        self.target = float(self.declare_parameter('target_speed_mps', 3.4).value)
        hz = self.declare_parameter('planning_hz', 5.0).value
        if not math.isfinite(hz) or not 5. <= hz <= 15.:
            raise ValueError('planning_hz must be in [5, 15]')
        self.period_ns = round(1e9 / hz)
        self.limit = None
        self.gate_driving = False
        self.feedback = Feedback()
        self.lock = Lock()
        self.telemetry = MutuallyExclusiveCallbackGroup()
        self.reset_driver()
        self.add_on_set_parameters_callback(self.update_parameters)
        self.output = self.create_publisher(TwistStamped, '/concept/auto_cmd', 1)
        self.decisions = self.create_publisher(String, '/concept/decision', 1)
        self.create_subscription(String, '/concept/vehicle_state', self.state, 10, callback_group=self.telemetry)
        self.create_subscription(String, '/concept/gate/status', self.gate, 1, callback_group=self.telemetry)
        self.create_subscription(String, '/concept/perception/frame', self.frame, 1)
        self.create_service(Trigger, '~/restart', self.restart)

    def reset_driver(self):
        self.driver = Drive(self.target)
        self.session = uuid4().hex
        self.first_ns = self.last_ns = self.next_ns = None
        self.rate_skipped_total = 0
        self.locked_reason = None
        self.camera_key = None
        with self.lock:
            self.feedback.samples.clear()

    def restart(self, _, response):
        with self.lock:
            if self.gate_driving:
                response.success = False
                response.message = 'Pause the active physical gate before clearing driver memory.'
                return response
        self.reset_driver()
        response.success = True
        response.message = 'Cleared driver, timing and map. Awaiting fresh synchronized sensors.'
        return response

    def update_parameters(self, parameters):
        for p in parameters:
            if p.name == 'planning_hz':
                return SetParametersResult(successful=False, reason='Set planning_hz when launching the driver')
            if p.name == 'target_speed_mps':
                if p.type_ != p.Type.DOUBLE or not math.isfinite(p.value) or not 0. < p.value <= 6.:
                    return SetParametersResult(successful=False, reason='target_speed_mps must be a double in (0,6]')
                self.target = p.value
        return SetParametersResult(successful=True)

    def state(self, message):
        try:
            sample = json.loads(message.data)
            with self.lock:
                self.feedback.add(sample)
        except (KeyError, TypeError, ValueError):
            with self.lock:
                self.feedback.samples.clear()

    def gate(self, message):
        try:
            data = json.loads(message.data)
            limit = float(data['max_auto_speed_mps'])
            if math.isfinite(limit) and 0. < limit <= 6.:
                with self.lock:
                    self.limit = limit
                    self.gate_driving = data.get('output_enabled') is True and data.get('paused') is False
        except (KeyError, TypeError, ValueError):
            return

    def frame(self, message):
        now = lambda: self.get_clock().now().nanoseconds
        tic = time.monotonic()
        decision = dict(schema='concept.decision.v1', session=self.session, frame_id='base_footprint',
                        status='waiting', reason='', stamp=None, capture_age_s=None, sensor_age_s=None,
                        target_speed_mps=self.target, effective_speed_mps=None, selected_index=None,
                        whitebox_index=None, command=None, feasible_count=0, candidates=[], selected_path=[],
                        road_rows=[], objects=[], tracks=[], lines=[], edges={}, map=None, map_semantics=None)
        try:
            packet, ns, optical_frame, profile = parse_frame_header(message.data)
            capture = ns / 1e9
            decision.update(stamp=stamp(ns), geometry_profile=profile)
            age = (now() - ns) / 1e9
            if not 0 <= age <= MAX_CAPTURE_AGE:
                raise ValueError('image is stale or its stamp is in the future')
            if self.locked_reason:
                raise ValueError(self.locked_reason)
            camera_key = (optical_frame, profile)
            if self.camera_key is not None and camera_key != self.camera_key:
                self.locked_reason = 'camera or projection changed; pause and restart before reusing road memory'
                raise ValueError(self.locked_reason)
            if self.last_ns is not None:
                if ns <= self.last_ns:
                    raise ValueError('duplicate or out-of-order image')
                if ns < self.next_ns - CAPTURE_JITTER_NS:
                    self.rate_skipped_total += 1
                    return
                if ns - self.last_ns > 500_000_000:
                    self.locked_reason = 'image gap over 0.5 s; pause the gate and restart the driver'
                    raise ValueError(self.locked_reason)
            with self.lock:
                sample = self.feedback.at(capture, MAX_STATE_AGE)
                limit = self.limit
            pace = min(self.target, limit) if limit is not None else self.target
            f = decode_frame_data(packet)
            self.driver.set_pace(pace)
            decision.update(effective_speed_mps=pace, sensor_age_s=capture-sample['t'])
            if self.first_ns is None:
                self.first_ns = ns
            native = native_sensor(sample, (ns-self.first_ns)/1e9)
            det = {k:f[k] for k in ('boxes','labels','scores')}
            try:
                result = self.driver.step(f['bgr'], f['road'], f['lane'], det, native)
            except Exception:
                # A failure after partially updating temporal maps requires a common new epoch.
                self.locked_reason = 'incomplete planner step; pause the gate and restart the driver'
                raise
            self.camera_key = camera_key
            self.last_ns = ns
            periods = (ns - self.first_ns + CAPTURE_JITTER_NS) // self.period_ns + 1
            self.next_ns = self.first_ns + periods * self.period_ns
            cmd = ros_command(result.pop('command_native'))
            decision.update(result, command=cmd, status='ok')
            if profile != 'calibrated-v626' and not (
                    profile == 'frozen-unity-v626' and self.get_parameter('use_sim_time').value):
                decision.update(status='shadow', reason='camera geometry has no validated driving profile')
            elif cmd['linear_x_mps'] < 0 and sample.get('rpm_encoding') == 'forward_only_uint8':
                decision.update(status='shadow', reason='reverse wheel-feedback sign unverified')
            elif not 0 <= (now() - ns) / 1e9 <= MAX_CAPTURE_AGE:
                decision.update(status='late', reason='decision exceeded 0.5 s from acquisition')
            else:
                output = TwistStamped()
                output.header.frame_id = 'base_footprint'
                output.header.stamp.sec, output.header.stamp.nanosec = divmod(ns, 1_000_000_000)
                output.twist.linear.x = cmd['linear_x_mps']
                output.twist.angular.z = cmd['angular_z_radps']
                self.output.publish(output)
        except Exception as exc:
            decision.update(status='invalid', reason=str(exc))
        emitted = now()
        decision.update(emitted_stamp=stamp(emitted), processing_ms=(time.monotonic()-tic)*1000.,
                        rate_skipped_total=self.rate_skipped_total, planning_hz=1e9/self.period_ns)
        if decision['stamp'] is not None:
            decision['capture_age_s'] = (emitted-ns)/1e9
        self.decisions.publish(String(data=json.dumps(decision, allow_nan=False, separators=(',', ':'))))


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
