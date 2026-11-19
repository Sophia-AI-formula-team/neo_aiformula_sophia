"""Only simulated sensors enter ROS. Every command is correlated to its source image."""
import json
import math
import os
import socket
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from concept_drive.node import DriverNode
from concept_feasibility.response61515 import can_command
from concept_yolop.node import CameraNode

SCHEMA = 'concept-unity/1'
CAMERA = '/concept_sim/camera'
INFO = '/concept_sim/camera_info'
CALIBRATION = Path(__file__).with_name('honda.json')
FRAME = 'unity_honda_left_optical'
NS = 10 ** 9
MAX_JPEG = 2 * 1024 * 1024
TARGET = float(os.environ.get('CONCEPT_TARGET_MPS', '2'))
EVIDENCE = Path(os.environ.get('CONCEPT_EVIDENCE', '/evidence'))
TIMEOUT = 12.
FRAME_KEYS = frozenset(('schema', 'kind', 'seq', 'simulation_time_s', 'encoding', 'width', 'height',
                        'image_bytes', 'left_rpm', 'right_rpm', 'gyro_ros_radps', 'wheel_radius_m',
                        'applied_linear_mps', 'applied_angular_z_radps', 'applied_seq',
                        'applied_right_can_rpm', 'applied_left_can_rpm'))


def stamp_ns(msg):
    return msg.sec * NS + msg.nanosec


def stamp_dict_ns(value):
    return value['sec'] * NS + value['nanosec']


def wait_until(condition, reason, timeout=TIMEOUT):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(.002)
    raise TimeoutError(reason)


def json_message(value):
    return String(data=json.dumps(value, separators=(',', ':'), allow_nan=False))


def finite(header, key):
    value = float(header[key])
    if not math.isfinite(value):
        raise ValueError('non-finite ' + key)
    return value


def read_frame(stream, expected):
    line = stream.readline(8193)
    if not line:
        return None, None
    if len(line) > 8192 or not line.endswith(b'\n'):
        raise ValueError('invalid frame header')
    data = json.loads(line)
    if not isinstance(data, dict) or set(data) != FRAME_KEYS:
        raise ValueError('frame fields must contain exactly the documented onboard sensor and applied values')
    if (data.get('schema'), data.get('kind'), data.get('seq')) != (SCHEMA, 'frame', expected):
        raise ValueError('unexpected protocol, type or sequence')
    if (data.get('encoding'), data.get('width'), data.get('height')) != ('jpeg', 640, 360):
        raise ValueError('expected a 640x360 raw Honda-view JPEG')
    size = data['image_bytes']
    if type(size) is not int or not 0 < size <= MAX_JPEG:
        raise ValueError('image size outside protocol bounds')
    jpeg = stream.read(size)
    if len(jpeg) != size:
        raise EOFError('incomplete JPEG')
    return data, jpeg


def verify_applied(header, seq, last_ack):
    if type(header['applied_seq']) is not int or header['applied_seq'] != seq - 1:
        raise ValueError('host did not apply the preceding command')
    for name in ('left_can_rpm', 'right_can_rpm'):
        echo = header['applied_' + name]
        if type(echo) is not int or echo != (last_ack[name] if last_ack else 0):
            raise ValueError('host CAN echo differs from sent command: ' + name)
    for name in ('linear_mps', 'angular_z_radps'):
        if not math.isclose(finite(header, 'applied_' + name), last_ack[name] if last_ack else 0.,
                            rel_tol=1e-6, abs_tol=1e-6):
            raise ValueError('host drive-command echo differs: ' + name)


class SimIO(Node):
    def __init__(self):
        super().__init__('unity_link', namespace='/concept_sim')
        self.clock_pub = self.create_publisher(Clock, '/clock', 1)
        self.state_pub = self.create_publisher(String, '/concept/vehicle_state', 10)
        self.gate_pub = self.create_publisher(String, '/concept/gate/status', 1)
        self.image_pub = self.create_publisher(Image, CAMERA, 1)
        self.info_pub = self.create_publisher(CameraInfo, INFO, 1)
        self.create_subscription(String, '/concept/decision', self.on_decision, 4)
        self.create_subscription(TwistStamped, '/concept/auto_cmd', self.on_command, 4)
        self.restart_client = self.create_client(Trigger, '/concept/drive/restart')
        self.decision = None
        self.command = None
        self.bridge = CvBridge()
        config = json.loads(CALIBRATION.read_text())
        self.info = CameraInfo(width=config['width'], height=config['height'], p=config['projection'])
        self.info.header.frame_id = FRAME
        self.info.k = np.asarray(self.info.p).reshape(3, 4)[:, :3].ravel().tolist()

    def on_decision(self, msg):
        self.decision = json.loads(msg.data)

    def on_command(self, msg):
        self.command = (stamp_ns(msg.header.stamp), msg.twist.linear.x, msg.twist.angular.z)

    def reset(self, driver):
        self.gate_pub.publish(json_message(dict(mode='paused', max_auto_speed_mps=TARGET,
                                                output_enabled=False, paused=True)))
        wait_until(lambda: not driver.gate_driving, 'gate pause was not consumed')
        future = self.restart_client.call_async(Trigger.Request())
        wait_until(future.done, 'driver did not acknowledge its reset')
        if not future.result().success:
            raise RuntimeError(future.result().message)
        self.command = self.decision = None

    def publish_frame(self, driver, camera, data, jpeg, ns):
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img is None or img.shape != (360, 640, 3):
            raise ValueError('JPEG shape does not match camera')
        stamp = Time(sec=ns // NS, nanosec=ns % NS)
        self.clock_pub.publish(Clock(clock=stamp))
        self.info.header.stamp = stamp
        self.info_pub.publish(self.info)
        t = ns / NS
        sensor = dict(t=t, leftRpm=finite(data, 'left_rpm'), rightRpm=finite(data, 'right_rpm'),
                      gyro=finite(data, 'gyro_ros_radps'), wheelRadius=finite(data, 'wheel_radius_m'),
                      valid=True, source='unity wheel encoders + gyro', rpm_encoding='signed_le_int32')
        if sensor['wheelRadius'] <= 0:
            raise ValueError('wheel radius must be positive')
        self.state_pub.publish(json_message(sensor))
        self.gate_pub.publish(json_message(dict(mode='auto', paused=False, output_enabled=True,
                                                max_auto_speed_mps=TARGET, t=t,
                                                linear_mps=finite(data, 'applied_linear_mps'),
                                                yaw_rps=finite(data, 'applied_angular_z_radps'))))

        def telemetry_ready():
            with driver.lock:
                arrived = (driver.applied_stamp == t and driver.feedback.samples
                           and driver.feedback.samples[-1]['t'] == t)
            return (arrived and camera.info is not None and stamp_ns(camera.info.header.stamp) == ns
                    and driver.get_clock().now().nanoseconds == ns
                    and camera.get_clock().now().nanoseconds == ns)

        wait_until(telemetry_ready, 'matched clock, camera info, feedback or gate status missing')
        self.decision = self.command = None
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.frame_id, msg.header.stamp = FRAME, stamp
        self.image_pub.publish(msg)
        wait_until(lambda: self.decision and self.decision['stamp']
                   and stamp_dict_ns(self.decision['stamp']) == ns, 'no ROS decision for current frame')
        decision = self.decision
        v = w = 0.
        if decision['status'] == 'ok':
            wait_until(lambda: self.command and self.command[0] == ns, 'no ROS Twist for current frame')
            _, v, w = self.command
        right, left = can_command(v, -w, physical=True)
        return decision, dict(schema=SCHEMA, kind='command', seq=data['seq'], simulation_time_s=data['simulation_time_s'],
                              mode='auto' if decision['status'] == 'ok' else 'stop',
                              status=decision['status'], reason=decision['reason'], linear_mps=v, angular_z_radps=w,
                              right_can_rpm=int(right), left_can_rpm=int(left), selected_index=decision['selected_index'],
                              feasible_count=decision['feasible_count'], geometry_profile=decision.get('geometry_profile'))


def serve(sim, driver, camera, conn):
    sim.reset(driver)
    seq = 0
    initial_ns = None
    previous_ns = None
    last_ack = None
    directory = EVIDENCE / driver.session
    directory.mkdir(parents=True)
    with conn.makefile('rb') as stream, (directory / 'frames.jsonl').open('w', buffering=1) as log:
        while True:
            header, jpeg = read_frame(stream, seq)
            if header is None:
                return
            sim_ns = round(finite(header, 'simulation_time_s') * NS)
            if previous_ns is not None and not 66_666_500 <= sim_ns - previous_ns <= 500_000_000:
                raise ValueError('capture interval must be between 1/15 and 0.5 simulation seconds')
            if initial_ns is None:
                initial_ns = driver.get_clock().now().nanoseconds + NS - sim_ns
            verify_applied(header, seq, last_ack)
            start = time.monotonic()
            decision, ack = sim.publish_frame(driver, camera, header, jpeg, initial_ns + sim_ns)
            ack['wall_ms'] = round((time.monotonic() - start) * 1000, 2)
            log.write(json.dumps(dict(input=header, reply=ack, decision=decision), separators=(',', ':'), allow_nan=False) + '\n')
            conn.sendall((json.dumps(ack, separators=(',', ':'), allow_nan=False) + '\n').encode('ascii'))
            previous_ns, last_ack = sim_ns, ack
            seq += 1


def main():
    args = ['--ros-args']
    for key, val in dict(use_sim_time='true', onboard_camera='true', target_speed_mps=str(float(TARGET)),
                         image_topic=CAMERA, camera_info_topic=INFO, device='cuda',
                         weights=os.environ.get('CONCEPT_WEIGHTS', '/weights/B1.4.pth'),
                         calibration=str(CALIBRATION)).items():
        args += ['-p', key + ':=' + val]
    rclpy.init(args=args)
    driver = DriverNode()
    camera = CameraNode(lambda *f: driver.camera_frame(*f, timing_ms=camera.frame_timing_ms), driver.planning_callbacks)
    sim = SimIO()
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (sim, driver, camera):
        executor.add_node(node)
    runner = threading.Thread(target=executor.spin, daemon=True)
    runner.start()
    try:
        wait_until(lambda: sim.restart_client.service_is_ready() and sim.image_pub.get_subscription_count()
                   and sim.clock_pub.get_subscription_count() >= 2 and sim.state_pub.get_subscription_count()
                   and sim.info_pub.get_subscription_count() and sim.gate_pub.get_subscription_count()
                   and driver.output.get_subscription_count() and driver.decisions.get_subscription_count(),
                   'local ROS graph discovery failed', timeout=30.)
        with socket.socket() as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(('0.0.0.0', 16826))
            listener.listen(1)
            print('READY concept-unity/1 port 16826, B1.4 CUDA / Honda / ROS driver, target', TARGET, flush=True)
            while True:
                conn, _ = listener.accept()
                with conn:
                    try:
                        serve(sim, driver, camera, conn)
                    except Exception as exc:
                        print('Session ended:', repr(exc), flush=True)
                        try:
                            conn.sendall((json.dumps(dict(schema=SCHEMA, kind='error', reason=str(exc))) + '\n').encode())
                        except OSError:
                            pass
    finally:
        executor.shutdown()
        for node in (sim, camera, driver):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
