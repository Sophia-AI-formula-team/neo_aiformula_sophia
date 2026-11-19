"""Replay recorded Unity samples at wall-clock cadence through the production ROS nodes."""
import argparse
import collections
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from bridge import CAMERA, INFO, CALIBRATION, FRAME, NS, SimIO, json_message, stamp_dict_ns, stamp_ns, wait_until
from concept_drive.node import DriverNode
from concept_yolop.node import CameraNode


def percentiles(values):
    if not values:
        return {}
    return dict(zip(('min', 'p50', 'p95', 'max'), np.percentile(values, [0, 50, 95, 100]).round(2).tolist()))


def run(folder, hz, feedback_encoding):
    rows = [json.loads(s) for s in (folder / 'exchange.jsonl').read_text().splitlines()]
    images = [cv2.imread(str(folder / r['camera_file'])) for r in rows]
    if not images or any(img is None or img.shape != (360, 640, 3) for img in images):
        raise ValueError('no valid 640x360 images')
    target = 2.
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=false', '-p', 'target_speed_mps:=2.0',
                     '-p', 'onboard_camera:=true', '-p', 'device:=cuda', '-p', 'image_topic:=' + CAMERA,
                     '-p', 'camera_info_topic:=' + INFO, '-p', 'calibration:=' + str(CALIBRATION),
                     '-p', 'weights:=/weights/B1.4.pth'])
    driver = DriverNode()
    camera = CameraNode(lambda *f: driver.camera_frame(*f, timing_ms=camera.frame_timing_ms), driver.planning_callbacks)
    sim = SimIO()
    observer = Node('wall_diagnostics')
    decisions, commands = {}, {}

    def decision(msg):
        d = json.loads(msg.data)
        if d.get('stamp'):
            decisions[stamp_dict_ns(d['stamp'])] = d

    def command(msg):
        commands[stamp_ns(msg.header.stamp)] = (time.monotonic_ns(), msg.twist.linear.x, msg.twist.angular.z)

    observer.create_subscription(String, '/concept/decision', decision, 100)
    observer.create_subscription(TwistStamped, '/concept/auto_cmd', command, 100)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (sim, camera, driver, observer):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    stamps = []
    try:
        wait_until(lambda: sim.gate_pub.get_subscription_count() and sim.image_pub.get_subscription_count()
                   and sim.info_pub.get_subscription_count() and sim.state_pub.get_subscription_count()
                   and driver.output.get_subscription_count() == 2 and driver.decisions.get_subscription_count() == 2,
                   'ROS private graph missing', timeout=30)
        sim.info_pub.publish(sim.info)
        wait_until(lambda: camera.info is not None, 'missing camera info')
        start = time.monotonic_ns()
        for index, (row, img) in enumerate(zip(rows, images)):
            deadline = start + round(index * NS / hz)
            time.sleep(max(0., (deadline - time.monotonic_ns()) / NS))
            before = time.monotonic_ns()
            ns = sim.get_clock().now().nanoseconds
            stamps.append((ns, before, row['frame']['seq']))
            # DriverNode compares float seconds using ns / 1e9. At Unix epochs
            # integer division by 10**9 can round the same nanosecond differently.
            t = ns / 1e9 if feedback_encoding == 'driver' else ns / NS
            f = row['frame']
            sim.state_pub.publish(json_message(dict(t=t, leftRpm=f['left_rpm'], rightRpm=f['right_rpm'],
                wheelRadius=f['wheel_radius_m'], gyro=f['gyro_ros_radps'], valid=True,
                rpm_encoding='signed_le_int32', source='recorded Unity wheels and gyro')))
            sim.gate_pub.publish(json_message(dict(t=t, mode='auto', paused=False, output_enabled=True,
                max_auto_speed_mps=target, linear_mps=f['applied_linear_mps'], yaw_rps=f['applied_angular_z_radps'])))
            msg = sim.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg.header.frame_id = FRAME
            msg.header.stamp = Time(sec=ns // NS, nanosec=ns % NS)
            sim.image_pub.publish(msg)
        time.sleep(1.5)
        records = []
        for ns, before, seq in stamps:
            d, cmd = decisions.get(ns), commands.get(ns)
            records.append(dict(seq=seq, stamp_ns=ns,
                                feedback_roundoff_ns=(ns / NS - ns / 1e9)*1e9 if feedback_encoding == 'integer' else 0.,
                                decision=d['status'] if d else 'no_decision', reason=d.get('reason') if d else None,
                                command=list(cmd[1:]) if cmd else None, capture_to_twist_ms=(cmd[0]-before)/1e6 if cmd else None,
                                ros_camera_ms=d.get('camera_timing_ms') if d else None, planner_ms=d.get('processing_ms') if d else None))
        ages = [r['capture_to_twist_ms'] for r in records if r['command'] is not None]
        return dict(clock='system clock; timestamps assigned on replay publication of decoded recorded image',
                    input_count=len(records), requested_hz=hz, feedback_encoding=feedback_encoding,
                    measured_publish_hz=(len(stamps)-1)*NS/(stamps[-1][1]-stamps[0][1]) if len(stamps)>1 else None,
                    status_counts=collections.Counter(r['decision'] for r in records), command_count=len(ages),
                    capture_to_twist_ms=percentiles(ages), frames=records)
    finally:
        executor.shutdown()
        for node in (observer, sim, camera, driver):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument('recorded_folder', type=Path)
    args.add_argument('output', type=Path)
    args.add_argument('--hz', type=float, default=5.)
    args.add_argument('--feedback-encoding', choices=('driver', 'integer'), default='driver')
    options = args.parse_args()
    if not 0 < options.hz <= 15:
        args.error('cadence must be in (0,15] Hz')
    result = run(options.recorded_folder, options.hz, options.feedback_encoding)
    options.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'frames'}))
