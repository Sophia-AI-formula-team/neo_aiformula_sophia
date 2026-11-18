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


def main():
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_DOMAIN_ID', '172')
    if os.environ['ROS_DOMAIN_ID'] == '0':
        raise ValueError('Select a dedicated local test domain')
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
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

    def send_input(profile, sec):
        clock.publish(Clock(clock=Time(sec=sec)))
        until(lambda: driver.get_clock().now().nanoseconds == sec * 10**9)
        state.publish(String(data=json.dumps(dict(sensor, t=float(sec)))))
        until(lambda: any(s['t'] == float(sec) for s in driver.feedback.samples))
        with np.load(root / '00000_perception.npz') as d:
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
        assert decisions[0]['session'] != initial and decisions[0]['status'] == 'shadow'
        assert decisions[0]['effective_speed_mps'] == 1. and len(commands) == 1
        print('DDS verified command signs, capture stamp, checked selection, gate cap, restart and camera shadow.')
    finally:
        peer.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
