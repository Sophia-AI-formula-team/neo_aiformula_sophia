"""Sensor-only V6.26 decision node. Physical output belongs to /concept/gate."""
import gc
import json
import math
import time
from threading import Event, Lock
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

from concept_feasibility.response61515 import motor_profile
from concept_control_gate.speed_setting import SpeedSetting
from concept_yolop.frame_codec import decode_frame_data, parse_frame_header, validate_pixels, validated_detections
from concept_yolop.geometry import CALIBRATED, HONDA, UNITY
from .core import Drive
from .lap_map import encode_map
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
        self.speed_setting = SpeedSetting(self, 'target_speed_mps', 6.)
        self.target = self.speed_setting.initial
        hz = self.declare_parameter('planning_hz', 15.0).value
        if not math.isfinite(hz) or not 5. <= hz <= 15.:
            raise ValueError('planning_hz must be in [5, 15]')
        self.period_ns = round(1e9 / hz)
        self.limit = None
        self.gate_driving = False
        self.applied_command = (0., 0.)
        self.applied_stamp = -math.inf
        self.feedback = Feedback()
        self.lock = Lock()
        self.telemetry = MutuallyExclusiveCallbackGroup()
        self.planning_callbacks = MutuallyExclusiveCallbackGroup()
        self.restart_services = MutuallyExclusiveCallbackGroup()
        self.restart_requested = Event()
        self.speed_update_requested = Event()
        self.planning_state = Lock()
        self.reset_driver()
        self.output = self.create_publisher(TwistStamped, '/concept/auto_cmd', 1)
        self.decisions = self.create_publisher(String, '/concept/decision', 1)
        self.create_subscription(String, '/concept/vehicle_state', self.state, 10, callback_group=self.telemetry)
        self.create_subscription(String, '/concept/gate/status', self.gate, 1, callback_group=self.telemetry)
        self.onboard_camera = self.declare_parameter('onboard_camera', False).value
        self.add_on_set_parameters_callback(self.update_parameters)
        if not self.onboard_camera:
            self.create_subscription(String, '/concept/perception/frame', self.frame, 1, callback_group=self.planning_callbacks)
        self.create_service(Trigger, '~/restart', self.restart, callback_group=self.restart_services)
        # Leave the default group available for the built-in ROS parameter services.
        self.create_service(Trigger, '~/lap_map', self.lap_map, callback_group=self.planning_callbacks)

    def reset_driver(self):
        self.driver = Drive(self.target)
        self.session = uuid4().hex
        self.first_ns = self.last_ns = self.next_ns = None
        self.last_input_ns = None
        self.feedback_missed = False
        self.rate_skipped_total = 0
        self.locked_reason = None
        self.image_gap = False
        self.camera_key = None
        self.cached_lap_map = None
        with self.lock:
            self.feedback.samples.clear()

    def restart(self, _, response):
        self.restart_requested.set()
        try:
            with self.planning_state:
                with self.lock:
                    if self.gate_driving:
                        response.message = 'Pause the active physical gate before clearing driver memory.'
                        return response
                self.reset_driver()
                response.success = True
                response.message = 'Cleared driver, timing and map. Awaiting fresh synchronized sensors.'
                return response
        finally:
            self.restart_requested.clear()

    def lap_map(self, _, response):
        with self.planning_state:
            return self.map_snapshot(response)

    def map_snapshot(self, response):
        if self.last_ns is None:
            response.message = '等待第一帧有效地图'
            return response
        if self.driver.mapper.t != self.driver.previous_time:
            response.message = '最新规划未完成，保留上次地图'
            return response
        now = time.monotonic()
        if self.cached_lap_map is None or (
                self.cached_lap_map[0] != self.last_ns and now-self.cached_lap_map[1] >= 1.):
            try:
                message = encode_map(self.driver, self.session, stamp(self.last_ns))
            except (ValueError, TypeError, OverflowError) as exc:
                self.get_logger().warning('Could not export lap map: %s' % exc)
                response.message = '地图读取失败 · 保留上次地图'
                return response
            self.cached_lap_map = (self.last_ns, now, message)
        response.success = True
        response.message = self.cached_lap_map[2]
        return response

    def update_parameters(self, parameters):
        for p in parameters:
            if p.name in ('planning_hz','onboard_camera','speed_settings_directory'):
                return SetParametersResult(successful=False, reason='Set %s when launching the driver' % p.name)
        for p in parameters:
            if p.name == 'target_speed_mps':
                self.speed_update_requested.set()
                try:
                    with self.planning_state:
                        self.speed_setting.save(p.value)
                        self.target = p.value
                except (OSError, ValueError) as exc:
                    return SetParametersResult(successful=False, reason=str(exc))
                finally:
                    self.speed_update_requested.clear()
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
                    self.applied_command = (0., 0.)
                    self.applied_stamp = -math.inf
                    if data.get('mode') == 'auto' and data.get('output_enabled') and not data.get('paused'):
                        applied = (float(data.get('linear_mps', 0.)), -float(data.get('yaw_rps', 0.)))
                        applied_stamp = float(data.get('t', -math.inf))
                        if all(map(math.isfinite, (*applied, applied_stamp))):
                            self.applied_command = applied
                            self.applied_stamp = applied_stamp
                    self.gate_driving = data.get('output_enabled') is True and data.get('paused') is False
        except (KeyError, TypeError, ValueError):
            return

    def camera_frame(self, stamp, frame_id, profile, bgr, road, lane, det, timing_ms=None):
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        self.process_frame(local=(ns, frame_id, profile, dict(det, bgr=bgr, road=road, lane=lane), timing_ms))

    def frame(self, message):
        self.process_frame(message=message)

    def process_frame(self, message=None, local=None):
        if self.restart_requested.is_set() or self.speed_update_requested.is_set():
            return
        with self.planning_state:
            if not self.restart_requested.is_set() and not self.speed_update_requested.is_set():
                self.plan_frame(message, local)

    def plan_frame(self, message, local):
        now = lambda: self.get_clock().now().nanoseconds
        tic = time.monotonic()
        decision = dict(schema='concept.decision.v1', session=self.session, frame_id='base_footprint',
                        status='waiting', reason='', stamp=None, capture_age_s=None, sensor_age_s=None,
                        target_speed_mps=self.target, effective_speed_mps=None, selected_index=None,
                        whitebox_index=None, command=None, feasible_count=0, candidates=[], selected_path=[],
                        road_rows=[], objects=[], tracks=[], lines=[], edges={}, map=None, map_semantics=None)
        try:
            if local is None:
                packet, ns, optical_frame, profile = parse_frame_header(message.data)
            else:
                ns, optical_frame, profile, f, timing_ms = local
                if timing_ms is not None:
                    decision['camera_timing_ms'] = timing_ms
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
            previous_input = self.last_input_ns
            if previous_input is None or ns > previous_input:
                self.last_input_ns = ns
            if self.last_ns is not None:
                if ns <= self.last_ns:
                    raise ValueError('duplicate or out-of-order image')
                if ns < self.next_ns - CAPTURE_JITTER_NS:
                    self.rate_skipped_total += 1
                    return
                if ns - self.last_ns > 500_000_000:
                    self.image_gap = True
                    if previous_input is None or ns - previous_input > 500_000_000:
                        self.locked_reason = 'image gap over 0.5 s; pause the gate and restart the driver'
                    elif self.feedback_missed:
                        self.locked_reason = 'planning gap over 0.5 s after missing synchronized wheel/gyro feedback'
                    else:
                        self.locked_reason = 'planning gap over 0.5 s between completed frames'
                    raise ValueError(self.locked_reason)
            with self.lock:
                try:
                    sample = self.feedback.at(capture, MAX_STATE_AGE)
                except ValueError:
                    self.feedback_missed = True
                    raise
                limit = self.limit
                gate_age = now() * 1e-9 - self.applied_stamp
                applied = self.applied_command if 0. <= gate_age <= .3 else (0., 0.)
            pace = min(self.target, limit) if limit is not None else self.target
            if local is None:
                f = decode_frame_data(packet)
                det = {k:f[k] for k in ('boxes','labels','scores')}
            else:
                validate_pixels(f['bgr'], f['road'], f['lane'])
                det = validated_detections(f)
            self.driver.set_pace(pace)
            decision.update(effective_speed_mps=pace, sensor_age_s=capture-sample['t'])
            if self.first_ns is None:
                self.first_ns = ns
            native = native_sensor(sample, (ns-self.first_ns)/1e9)
            try:
                physical = profile in (HONDA, CALIBRATED)
                native['field_ground_projection'] = physical
                with motor_profile(physical, applied):
                    result = self.driver.step(f['bgr'], f['road'], f['lane'], det, native)
            except Exception:
                # A failure after partially updating temporal maps requires a common new epoch.
                self.locked_reason = 'incomplete planner step; pause the gate and restart the driver'
                raise
            self.camera_key = camera_key
            self.last_ns = ns
            self.feedback_missed = False
            periods = (ns - self.first_ns + CAPTURE_JITTER_NS) // self.period_ns + 1
            self.next_ns = self.first_ns + periods * self.period_ns
            cmd = ros_command(result.pop('command_native'), physical=physical)
            decision.update(result, command=cmd, status='ok', planner_reason=result.get('reason', ''))
            if profile not in (CALIBRATED, HONDA) and not (
                    profile == UNITY and self.get_parameter('use_sim_time').value):
                decision.update(status='blocked', reason='camera geometry has no validated driving profile')
            elif (cmd['linear_x_mps'] < 0 and sample.get('rpm_encoding') != 'signed_le_int32'
                  and not (profile == UNITY and self.get_parameter('use_sim_time').value)):
                decision.update(status='blocked', reason='reverse wheel-feedback sign unverified')
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
            decision.update(status='invalid', reason=str(exc), image_gap_reset_required=self.image_gap)
            if self.image_gap:
                decision['gap_reason'] = self.locked_reason
        emitted = now()
        decision.update(emitted_stamp=stamp(emitted), processing_ms=(time.monotonic()-tic)*1000.,
                        rate_skipped_total=self.rate_skipped_total, planning_hz=1e9/self.period_ns)
        if decision['stamp'] is not None:
            decision['capture_age_s'] = (emitted-ns)/1e9
        self.decisions.publish(String(data=json.dumps(decision, allow_nan=False, separators=(',', ':'))))


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    camera = None
    try:
        if node.onboard_camera:
            from concept_yolop.node import CameraNode
            # Planning, map snapshots and resets share one mutually exclusive group.
            # Telemetry and restart can run while a frame owns planning state.
            def on_image(*frame):
                node.camera_frame(*frame, timing_ms=camera.frame_timing_ms)
            camera = CameraNode(on_image, node.planning_callbacks)
            executor.add_node(camera)
            # Torch and the ROS graph live for the process lifetime. Repeated
            # full-heap scans of these objects stalled planning on the Jetson.
            # Future frame data and every subsequently created driver remain
            # in the normal cyclic collector, including across panel resets.
            gc.collect()
            gc.freeze()
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if camera is not None:
            gc.unfreeze()
            camera.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
