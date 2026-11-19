"""ROS 2 smoke test with the actual DDS graph and the archived starting image.

Run from a sourced colcon workspace on a development computer. This test sends
only simulated camera/feedback and receives command proposals on a local domain.
"""
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import Trigger

from concept_drive.node import DriverNode
from concept_yolop.frame_codec import encode_frame
from concept_yolop.geometry import HONDA


def main():
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_DOMAIN_ID', '172')
    if os.environ['ROS_DOMAIN_ID'] == '0':
        raise ValueError('Select a dedicated local test domain')
    # The archived starting frame and expected action were recorded at 3.4 m/s.
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'target_speed_mps:=3.4'])
    driver = DriverNode()
    peer = Node('concept_driver_replay_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(peer)
    executor.add_node(driver)

    def until(condition):
        deadline = time.monotonic() + 15.
        while not condition():
            if time.monotonic() > deadline:
                raise TimeoutError('DDS delivery or subscription discovery timed out')
            executor.spin_once(timeout_sec=.03)

    clock = peer.create_publisher(Clock, '/clock', 10)
    state = peer.create_publisher(String, '/concept/vehicle_state', 10)
    camera = peer.create_publisher(String, '/concept/perception/frame', 10)
    gate = peer.create_publisher(String, '/concept/gate/status', 10)
    decisions, commands = [], []
    peer.create_subscription(String, '/concept/decision', lambda m: decisions.append(json.loads(m.data)), 10)
    peer.create_subscription(TwistStamped, '/concept/auto_cmd', commands.append, 10)
    restart = peer.create_client(Trigger, '/concept/drive/restart')
    until(lambda: all(p.get_subscription_count() for p in (camera, state, clock, gate, driver.decisions, driver.output)))
    assert restart.wait_for_service(timeout_sec=2.)
    root = Path(__file__).resolve().parents[1] / 'test/data/moving-lap-start'
    row = json.loads((root / 'expected.json').read_text())[0]
    sensor = dict(row['sensor'], gyro=-row['sensor']['gyro'], valid=True)
    image = cv2.imread(str(root / '00000_rgb.png'))

    def send_input(profile, sec, direct=False):
        clock.publish(Clock(clock=Time(sec=sec)))
        until(lambda: driver.get_clock().now().nanoseconds == sec * 10**9)
        state.publish(String(data=json.dumps(dict(sensor, t=float(sec)))))
        until(lambda: any(s['t'] == float(sec) for s in driver.feedback.samples))
        with np.load(root / '00000_perception.npz') as d:
            if direct:
                driver.camera_frame(Time(sec=sec), 'camera', profile, image, d['raw'], d['lane'],
                                    {key: d[key] for key in ('boxes', 'scores', 'labels')})
            else:
                wire = encode_frame(sec, 0, 'camera', image, d['raw'], d['lane'], d, profile)
                camera.publish(String(data=wire))
        until(lambda: bool(decisions))

    def gate_status(paused, limit):
        gate.publish(String(data=json.dumps(dict(output_enabled=True, paused=paused, max_auto_speed_mps=limit))))
        until(lambda: driver.limit == limit and driver.gate_driving == (not paused))

    try:
        send_input('frozen-unity-v626', 10)
        assert decisions[0]['status'] == 'ok' and decisions[0]['selected_index'] == row['selected']
        until(lambda: len(commands) == 1)
        command = commands[0]
        assert command.header.stamp.sec == 10 and command.header.frame_id == 'base_footprint'
        np.testing.assert_allclose([command.twist.linear.x, -command.twist.angular.z], row['control'], atol=1e-5)
        initial = decisions.pop()['session']
        gate_status(False, 1.)
        req = restart.call_async(Trigger.Request())
        until(req.done)
        assert not req.result().success
        gate_status(True, 1.)
        req = restart.call_async(Trigger.Request())
        until(req.done)
        assert req.result().success
        send_input('unverified-resize', 11)
        assert decisions[0]['session'] != initial and decisions[0]['status'] == 'blocked'
        assert decisions[0]['effective_speed_mps'] == 1. and len(commands) == 1
        decisions.clear()
        req = restart.call_async(Trigger.Request())
        until(req.done)
        assert req.result().success
        send_input(HONDA, 12)
        result = decisions[0]
        assert result['geometry_profile'] == HONDA and result['status'] == 'ok'
        until(lambda: len(commands) == 2)
        command = commands[-1]
        assert command.header.frame_id == 'base_footprint'
        assert (command.header.stamp.sec, command.header.stamp.nanosec) == (12, 0)
        assert command.twist.linear.x > 0
        np.testing.assert_allclose(
            [command.twist.linear.x, command.twist.angular.z],
            [result['command']['linear_x_mps'], result['command']['angular_z_radps']])
        decisions.clear()
        req = restart.call_async(Trigger.Request())
        until(req.done)
        assert req.result().success
        send_input(HONDA, 13, direct=True)
        until(lambda: len(commands) == 3)
        assert commands[-1].header.stamp.sec == 13
        for key in ('status', 'selected_index', 'command', 'feasible_count', 'selected_path', 'candidates'):
            assert decisions[0][key] == result[key], key
        decisions.clear()
        send_input(HONDA, 14)
        assert decisions[0]['status'] == 'invalid'
        assert decisions[0]['image_gap_reset_required'] is True
        assert decisions[0]['gap_reason'].startswith('image gap')
        assert len(commands) == 3
        req = restart.call_async(Trigger.Request())
        until(req.done)
        assert req.result().success
        decisions.clear()
        send_input(HONDA, 15)
        until(lambda: len(commands) == 4)
        assert decisions[0]['status'] == 'ok'
        assert commands[-1].header.stamp.sec == 15
        def follow_reverse_plan(*_args):
            return {'command_native': [-.2, 0.], 'reason': 'checked_slow_reverse'}

        req = restart.call_async(Trigger.Request())
        until(req.done)
        assert req.result().success
        driver.driver.step = follow_reverse_plan
        decisions.clear()
        sensor['rpm_encoding'] = 'signed_le_int32'
        sensor['leftRpm'] = sensor['rightRpm'] = -10
        send_input(HONDA, 16, direct=True)
        until(lambda: len(commands) == 5)
        assert decisions[0]['status'] == 'ok'
        assert commands[-1].header.stamp.sec == 16
        assert commands[-1].twist.linear.x == -.2

        req = restart.call_async(Trigger.Request())
        until(req.done)
        assert req.result().success
        driver.driver.step = follow_reverse_plan
        decisions.clear()
        sensor['rpm_encoding'] = 'forward_only_uint8'
        sensor['leftRpm'] = sensor['rightRpm'] = 0
        send_input(HONDA, 17, direct=True)
        assert decisions[0]['status'] == 'blocked' and len(commands) == 5
        req = restart.call_async(Trigger.Request())
        until(req.done)
        assert req.result().success
        driver.driver.step = lambda *_: dict(command_native=[0., .55], reason='checked_reverse_release')
        decisions.clear()
        sensor['rpm_encoding'] = 'signed_le_int32'
        send_input(HONDA, 18, direct=True)
        until(lambda: len(commands) == 6)
        assert decisions[0]['status'] == 'ok'
        assert commands[-1].header.stamp.sec == 18
        assert commands[-1].twist.linear.x == commands[-1].twist.angular.z == 0.
        assert decisions[0]['command'] == dict(linear_x_mps=0., angular_z_radps=0.)
        for nanosec in (200_000_000, 400_000_000, 600_000_000):
            decisions.clear()
            t = Time(sec=18, nanosec=nanosec)
            clock.publish(Clock(clock=t))
            until(lambda: driver.get_clock().now().nanoseconds == 18_000_000_000 + nanosec)
            # Fresh images keep arriving, but the last wheel/gyro pair is at 18.0.
            driver.camera_frame(t, 'camera', HONDA, image, None, None, {})
            until(lambda: bool(decisions))
            if nanosec <= 400_000_000:
                assert decisions[0]['reason'] == 'no recent wheel and gyro measurement at image capture'
        assert decisions[0]['image_gap_reset_required'] is True
        assert decisions[0]['gap_reason'] == 'planning gap over 0.5 s after missing synchronized wheel/gyro feedback'
        assert len(commands) == 6
        print('DDS verified command signs, capture stamp, selection, speed cap, restart, '
              'projection interlock, Honda direct-frame parity, image-gap restart and signed reverse publication and full-zero braking.')
    finally:
        peer.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
