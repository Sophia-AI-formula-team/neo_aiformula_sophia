"""Exercise the actual Foxy adapters on isolated DDS with a software CAN receiver."""
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time

import rclpy
from can_msgs.msg import Frame
from geometry_msgs.msg import TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


ROOT = Path(os.environ.get('NEO_ROOT', '/neo'))
LOG = Path(os.environ.get('ROS_LOG_DIR', '/tmp/qa_gate/log'))
JOY = '/aiformula_control/joy_node/joy_l2_throttle'
FINAL = '/aiformula_control/motor_controller/reference_signal'


def require(condition, why):
    if not condition:
        raise AssertionError(why)


def expected_straight(speed):
    # Independent motor command oracle, in the order of the two signed CAN words.
    result = []
    for slope, offset in ((.834, 2.76), (.844, 2.81)):
        wheel = speed / .127
        motor = math.copysign(abs(wheel) / slope + offset, wheel)
        result.append(round(motor * 60 * 1.1 / (2 * math.pi)))
    return tuple(result)


class Fixture:
    def __init__(self):
        self.probe = Node('qa_concept_sensors')
        self.sink = Node('socket_can_sender', namespace='/aiformula_control')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.probe)
        self.executor.add_node(self.sink)
        self.status = {}
        self.feedback = {}
        self.frames = []
        self.vision_receipts = []
        self.procs = []
        self.logs = []
        self.joy_on = self.wheels_on = True
        self.manual = False
        self.speed = self.old_stamp = None
        self.seq = 0
        self.next_tx = 0
        self.start = time.monotonic()
        self.samples = []
        self.transition_start = None
        self.joy = self.probe.create_publisher(Joy, JOY, qos_profile_sensor_data)
        self.imu = self.probe.create_publisher(Imu, '/aiformula_sensing/vectornav/imu', qos_profile_sensor_data)
        self.wheels = self.probe.create_publisher(Frame, '/aiformula_sensing/vehicle_info', qos_profile_sensor_data)
        self.auto = self.probe.create_publisher(TwistStamped, '/concept/auto_cmd', 1)
        self.vision = self.probe.create_publisher(String, '/concept/perception/frame', 10)
        self.probe.create_subscription(String, '/concept/perception/frame', self.receive_vision, 10)
        self.probe.create_subscription(String, '/concept/gate/status', self.receive_status, 10)
        self.probe.create_subscription(String, '/concept/vehicle_state', self.receive_feedback, 10)
        self.sink.create_subscription(Frame, FINAL, self.receive_can, 50)
        self.enable = self.probe.create_client(SetBool, '/concept/gate/enable_output')
        self.resume = self.probe.create_client(Trigger, '/concept/gate/resume')

    def receive_vision(self, msg):
        self.vision_receipts.append((int(msg.data), time.monotonic()))

    def receive_can(self, msg):
        require(msg.id == 0x210 and msg.dlc == 8 and not any(
            (msg.is_rtr, msg.is_extended, msg.is_error)), 'unexpected CAN frame')
        self.frames.append((time.monotonic(), struct.unpack('<ii', bytes(msg.data))))

    def receive_status(self, msg):
        self.status = json.loads(msg.data)

    def receive_feedback(self, msg):
        self.feedback = json.loads(msg.data)

    def start_node(self, module, args):
        handle = (LOG / (module.split('.')[0] + '.log')).open('w')
        self.logs.append(handle)
        proc = subprocess.Popen([sys.executable, '-c',
                                 'from ' + module + ' import main; main()'] + args,
                                stdout=handle, stderr=subprocess.STDOUT)
        self.procs.append((module, proc))

    def launch(self):
        require(not self.probe.get_publishers_info_by_topic(FINAL), 'domain already has a CAN publisher')
        base = ROOT / 'workspace/src/aiformula'
        buttons = str(base / 'launchers/config/gamepad/dualshock4.yaml')
        motor = ['--ros-args', '-r', '__ns:=/concept/hardware',
                 '--params-file', str(base / 'vehicle/config/wheel.yaml'),
                 '--params-file', str(base / 'control/motor_controller/config/motor_controller.yaml'),
                 '--params-file', buttons]
        for source, target in (('sub_speed_command', 'motor_cmd'), ('pub_can', 'motor_can'),
                               (JOY, 'unused_motor_joy'), ('/semantic_drive/motor_session', 'unused_motor_session'),
                               ('/semantic_drive/map_status', 'unused_motor_map_status')):
            motor += ['-r', source + ':=/concept/internal/' + target]
        self.start_node('motor_controller.motor_controller', motor)
        self.start_node('concept_vehicle_state.node', [])
        self.start_node('concept_control_gate.node', ['--ros-args', '--params-file', buttons])

    def send(self):
        stamp = self.probe.get_clock().now().to_msg()
        gyro = Imu()
        gyro.header.stamp, gyro.header.frame_id = stamp, 'qa_imu'
        gyro.angular_velocity.z = .125
        self.imu.publish(gyro)
        if self.wheels_on:
            wheel = Frame()
            wheel.header.stamp, wheel.id, wheel.dlc = stamp, 0x711, 8
            wheel.data = [11, 0, 0, 0, 9, 0, 0, 0]
            self.wheels.publish(wheel)
        if self.joy_on:
            msg = Joy()
            msg.header.stamp = stamp
            msg.axes = [0., -.2, 0., 0.]
            msg.buttons = [0, 0, 0, 0, 0, 0, int(self.manual)]
            self.joy.publish(msg)
        if self.speed is not None:
            cmd = TwistStamped()
            cmd.header.stamp = self.old_stamp or stamp
            cmd.twist.linear.x = self.speed
            self.auto.publish(cmd)
        self.seq += 1
        self.vision.publish(String(data=str(self.seq)))

    def pump(self):
        now = time.monotonic()
        for name, process in self.procs:
            require(process.poll() is None, '{} exited: {}'.format(name, LOG / (name.split('.')[0] + '.log')))
        if now >= self.next_tx:
            self.send()
            self.next_tx = now + .025
        self.executor.spin_once(timeout_sec=.004)

    def until(self, predicate, why, timeout=3.0):
        end = time.monotonic() + timeout
        while not predicate():
            require(time.monotonic() < end, '{}; status={} feedback={} last CAN={}'.format(
                why, self.status, self.feedback, self.frames[-2:]))
            self.pump()

    def dwell(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.pump()

    def service(self, client, request, success=True):
        self.until(client.service_is_ready, 'service not discovered', 8)
        future = client.call_async(request)
        self.until(future.done, 'no service response')
        response = future.result()
        if success is not None:
            require(response.success is success, 'service returned ' + str(response))
        return response.message

    def mode(self, mode, reason=None):
        self.until(lambda: self.status.get('mode') == mode and (
            reason is None or self.status.get('reason') == reason), 'missing gate mode ' + mode)

    def mark(self):
        self.transition_start = time.monotonic(), len(self.frames), len(self.vision_receipts)

    def observe(self, label, expected, duration=.20, wholly_zero=False, require_zero_barrier=False):
        start, offset, vision = self.transition_start
        self.until(lambda: any(value == expected for _, value in self.frames[offset:]),
                   'CAN did not reach {}'.format(expected))
        match = next(i for i in range(offset, len(self.frames)) if self.frames[i][1] == expected)
        if require_zero_barrier:
            require(any(value == (0, 0) for _, value in self.frames[offset:match]),
                    label + ': no zero CAN between authorities')
        self.dwell(duration)
        window = self.frames[offset if wholly_zero else match:]
        require(len(window) >= 3 and all(value == expected for _, value in window),
                '{}: inconsistent CAN values {}'.format(label, sorted(set(v for _, v in window))))
        if expected != (0, 0) and not self.manual:
            require(self.status.get('mode') == 'auto' and self.status.get('auto_fresh')
                    and self.status.get('reason') == 'auto_command_received',
                    label + ': moving with a missing or stale command status ' + str(self.status))
        video = self.vision_receipts[vision:]
        require(len(video) >= 3, label + ': perception stream stalled')
        row = dict(stage=label, mode=self.status.get('mode'), reason=self.status.get('reason'),
                   rpm=expected, frames=len(window), camera_messages=len(video),
                   elapsed_s=round(time.monotonic()-self.start, 3),
                   transition_to_first_matching_can_ms=round(1000 * (self.frames[match][0]-start), 1))
        self.samples.append(row)
        print(json.dumps(row), flush=True)

    def test(self):
        self.launch()
        self.until(lambda: self.feedback.get('valid') and self.status.get('mode') == 'disabled'
                   and self.enable.service_is_ready(), 'nodes did not start', 15)
        require((self.feedback['leftRpm'], self.feedback['rightRpm'], self.feedback['gyro'])
                == (9, 11, .125), 'CAN/IMU did not traverse vehicle state')
        enable = SetBool.Request()
        enable.data = True
        # DDS discovery can finish on the probe before it finishes on the gate.
        self.until(lambda: self.service(self.enable, enable, success=None) == 'output_enabled',
                   'graph never became armable', 8)
        self.mode('paused', 'output_enabled')
        self.service(self.resume, Trigger.Request())
        self.mark()
        self.speed = .4
        self.mode('auto')
        self.observe('fresh autonomous command', expected_straight(.4))
        self.mark()
        self.manual = True
        self.mode('manual')
        self.observe('manual overrides live positive autonomous demand', expected_straight(-.7), require_zero_barrier=True)
        self.service(self.resume, Trigger.Request(), False)
        self.mark()
        self.manual = False
        self.mode('paused', 'manual_released')
        self.observe('release blocks continuously refreshed autonomous demand', (0, 0), .65)
        self.old_stamp = self.probe.get_clock().now().to_msg()
        self.dwell(.04)
        self.mark()
        self.service(self.resume, Trigger.Request())
        self.mode('auto', 'waiting_new_auto')
        self.observe('pre-resume stamp replayed after resume', (0, 0), .22, wholly_zero=True)
        self.mark()
        self.old_stamp, self.speed = None, .6
        self.observe('first fresh command after resume', expected_straight(.6))
        self.mark()
        self.speed = None
        self.mode('paused', 'auto_timeout')
        self.observe('autonomous publisher silent', (0, 0))
        self.mark()
        self.speed = .6
        self.service(self.resume, Trigger.Request())
        self.mode('auto')
        self.observe('fresh autonomous demand before joystick timeout', expected_straight(.6))
        self.mark()
        self.joy_on = False
        self.mode('paused', 'joy_timeout')
        self.observe('joystick disconnected while demand continues', (0, 0))
        self.service(self.resume, Trigger.Request(), False)
        self.joy_on = True
        self.dwell(.12)
        self.mark()
        self.service(self.resume, Trigger.Request())
        self.mode('auto')
        self.observe('fresh autonomous demand before feedback loss', expected_straight(.6))
        self.mark()
        self.wheels_on = False
        self.until(lambda: self.status.get('mode') == 'paused' and not self.feedback.get('valid'),
                   'no pause on missing wheel feedback')
        self.observe('CAN feedback lost while IMU and demand continue', (0, 0))
        self.service(self.resume, Trigger.Request(), False)
        require(self.feedback.get('reason') == 'wheel_timeout', 'unexpected feedback reason')
        self.wheels_on = True
        self.until(lambda: self.feedback.get('valid'), 'feedback did not recover')
        self.mark()
        self.service(self.resume, Trigger.Request())
        self.mode('auto')
        self.observe('fresh demand before motor converter loss', expected_straight(.6))
        # Keep the ROS graph alive while suspending the converter and all its timers.
        motor = next(proc for name, proc in self.procs if name == 'motor_controller.motor_controller')
        self.mark()
        motor.send_signal(signal.SIGSTOP)
        try:
            self.mode('disabled', 'motor_timeout')
            self.dwell(.25)
            start, offset, vision = self.transition_start
            remaining = self.frames[offset:]
            i = next(i for i, (_, words) in enumerate(remaining) if words == (0, 0))
            zeros = remaining[i:]
            require(len(zeros) >= 3, 'expected repeated final zero frames after motor timeout')
            require(all(words == (0, 0) for _, words in zeros), 'motion after motor timeout')
            self.service(self.resume, Trigger.Request(), False)
            camera_messages = len(self.vision_receipts) - vision
            require(camera_messages >= 3, 'perception stalled after motor timeout')
            row = dict(stage='motor converter suspended', mode=self.status['mode'],
                       reason=self.status['reason'], frames=len(zeros), rpm=(0, 0),
                       zero_span_ms=round(1000*(zeros[-1][0]-zeros[0][0]), 1),
                       camera_messages=camera_messages,
                       transition_to_first_matching_can_ms=round(1000*(remaining[i][0]-start), 1))
            self.samples.append(row)
            print(json.dumps(row), flush=True)
        finally:
            motor.send_signal(signal.SIGCONT)
        first = self.vision_receipts[0][0]
        gap = max(b[1]-a[1] for a, b in zip(self.vision_receipts, self.vision_receipts[1:]))
        require(self.vision_receipts[-1][0]-first+1 == len(self.vision_receipts),
                'perception sequence discontinuity')
        print(json.dumps(dict(result='PASS', synthetic_perception_messages=len(self.vision_receipts),
                              largest_perception_gap_ms=round(1000*gap, 1), stages=len(self.samples))), flush=True)

    def close(self):
        for _, proc in self.procs:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
        for _, proc in self.procs:
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        for handle in self.logs:
            handle.close()
        self.executor.shutdown()
        self.sink.destroy_node()
        self.probe.destroy_node()


def main():
    require(Path('/.dockerenv').is_file() and os.environ.get('ROS_DOMAIN_ID') == '69'
            and os.environ.get('ROS_LOCALHOST_ONLY') == '1',
            'run in an isolated Docker container, ROS_DOMAIN_ID=69 and ROS_LOCALHOST_ONLY=1')
    LOG.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    fixture = Fixture()
    try:
        fixture.test()
    finally:
        fixture.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
