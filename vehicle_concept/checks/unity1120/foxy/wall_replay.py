"""Replay onboard images without pausing the clock for inference or planning.

Run inside the standalone Unity/Foxy image, with the network disabled. This
measures ROS Image publication to receipt of the matching Twist on the desktop.
The archived wheels and gyro are a timing fixture, not a new driven trajectory.
"""
import argparse
import gc
import json
import os
import subprocess
import sys
import threading
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.executors import MultiThreadedExecutor

from bridge import (CALIBRATION, CAMERA, FRAME, INFO, NS, CameraNode, DriverNode, SimIO,
                    json_message, stamp_dict_ns, stamp_ns, wait_until)


class Meter(SimIO):
    def __init__(self):
        super().__init__()
        self.twists = {}
        self.decisions = {}

    def on_command(self, msg):
        self.twists[stamp_ns(msg.header.stamp)] = time.monotonic_ns()

    def on_decision(self, msg):
        decision = json.loads(msg.data)
        if decision.get('stamp'):
            self.decisions[stamp_dict_ns(decision['stamp'])] = decision


def replay(scene, output, mode, rate, seconds, split):
    rows = [json.loads(line) for line in (scene/'exchange.jsonl').read_text().splitlines()]
    params = dict(onboard_camera='false' if split else 'true', local_frame_transport='true' if split else 'false',
                  target_speed_mps='2.0', tracking_controller=mode,
                  image_topic=CAMERA, camera_info_topic=INFO, calibration=str(CALIBRATION),
                  planning_hz='15.0', device='cuda', weights='/weights/B1.4.pth')
    ros_args = ['--ros-args']
    for key, value in params.items():
        ros_args.extend(['-p', key+':='+value])
    if split:
        os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE','/opt/concept/src/concept/config/vehicle-fastdds.xml')
    rclpy.init(args=ros_args)
    driver, io = DriverNode(), Meter()
    vision = None
    if split:
        camera = None
        vision = subprocess.Popen([sys.executable, '-c', 'from concept_yolop.node import main; main()', *ros_args])
    else:
        camera = CameraNode(lambda *args: driver.camera_frame(*args, timing_ms=camera.frame_timing_ms))
    executor = MultiThreadedExecutor(num_threads=3)
    nodes = (io, driver) if split else (io, driver, camera)
    for node in nodes:
        executor.add_node(node)
    images = [io.bridge.cv2_to_imgmsg(cv2.imread(str(scene/row['camera_file'])), encoding='bgr8')
              for row in rows]
    gc.collect()
    gc.freeze()
    planner = threading.Thread(target=driver.run_onboard_planning)
    runner = threading.Thread(target=executor.spin, daemon=True)
    planner.start()
    runner.start()
    feedback_stop = threading.Event()
    feedback_thread = None
    try:
        wait_until(lambda: all(pub.get_subscription_count() for pub in (
            io.image_pub, io.info_pub, io.state_pub, io.gate_pub, driver.output)), 'ROS graph missing')
        io.info_pub.publish(io.info)
        if camera is not None:
            wait_until(lambda: camera.info is not None, 'camera info missing')

        latest_frame = 0

        def feedback():
            while not feedback_stop.is_set():
                publish_state(rows[latest_frame]['frame'])
                feedback_stop.wait(.02)

        def publish_state(row):
            ns = io.get_clock().now().nanoseconds
            io.state_pub.publish(json_message(dict(t=ns/NS, leftRpm=row['left_rpm'], rightRpm=row['right_rpm'],
                gyro=row['gyro_ros_radps'], wheelRadius=row['wheel_radius_m'], valid=True,
                source='offline Unity feedback', rpm_encoding='signed_le_int32')))
            io.gate_pub.publish(json_message(dict(mode='auto', paused=False, output_enabled=True,
                max_auto_speed_mps=2., t=ns/NS, linear_mps=row['applied_linear_mps'],
                yaw_rps=row['applied_angular_z_radps'])))

        def publish(i):
            nonlocal latest_frame
            latest_frame = i
            ns = io.get_clock().now().nanoseconds
            stamp = Time(sec=ns//NS, nanosec=ns%NS)
            image = images[i]
            image.header.stamp, image.header.frame_id = stamp, FRAME
            sent = time.monotonic_ns()
            io.image_pub.publish(image)
            return ns, sent

        feedback_thread = threading.Thread(target=feedback)
        feedback_thread.start()
        wait_until(lambda: len(driver.feedback.samples) >= 4, 'feedback history missing')
        for _ in range(16):
            publish(0)
            time.sleep(.18)
        if not io.twists or driver.locked_reason:
            raise RuntimeError('No matching commands during warm-up: '+str(
                Counter(d.get('reason') for d in io.decisions.values())))
        start = time.monotonic()
        sent = {}
        for n in range(rate*seconds):
            deadline = start+n/rate
            time.sleep(max(0., deadline-time.monotonic()))
            seq = n*5//rate % len(images)
            ns, mono = publish(seq)
            sent[ns] = mono
        time.sleep(.5)
        matched = sorted(set(sent) & set(io.twists))
        elapsed = [(io.twists[k]-sent[k])/1e6 for k in matched]
        arrivals = sorted(io.twists[k] for k in matched)
        source_decisions = [io.decisions[k] for k in sent if k in io.decisions]
        stages = {}
        for decision in source_decisions:
            for name, value in (decision.get('camera_timing_ms') or {}).items():
                if type(value) in (float, int):
                    stages.setdefault(name, []).append(value)
            for name in ('processing_ms', 'command_age_at_publish_ms'):
                if decision.get(name) is not None:
                    stages.setdefault(name, []).append(decision[name])
        summary = dict(controller=mode, transport='local_binary_dds' if split else 'onboard_callback',
                       wall_seconds=seconds, camera_hz=rate, sent=len(sent),
                       matched=len(matched), missing=len(sent)-len(matched),
                       measured_command_hz=len(matched)/seconds,
                       reasons=dict(Counter(d['reason'] for d in source_decisions)),
                       prefetched=sum(bool(d['camera_timing_ms'].get('pixel_prefetch_used')) for d in source_decisions),
                       median_stage_ms={k:float(np.median(v)) for k,v in stages.items()},
                       median_ms=float(np.median(elapsed)) if elapsed else None,
                       p95_ms=float(np.percentile(elapsed,95)) if elapsed else None,
                       max_ms=max(elapsed, default=None),
                       longest_arrival_gap_ms=max(np.diff(arrivals)/1e6, default=None),
                       eligible_under_200ms=sum(v <= 200 for v in elapsed))
        output.write_text(json.dumps(dict(summary=summary, sent=sent,
            received={k:io.twists[k] for k in matched}), indent=2))
        print(json.dumps(summary))
    finally:
        feedback_stop.set()
        if feedback_thread is not None:
            feedback_thread.join()
        if vision is not None:
            vision.terminate()
            vision.wait(timeout=10)
        driver.stop_onboard_planning()
        planner.join()
        executor.shutdown()
        gc.unfreeze()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--mode', choices=('lya','pure_pursuit'), required=True)
    parser.add_argument('--hz', type=int, default=15)
    parser.add_argument('--seconds', type=int, default=10)
    parser.add_argument('--split', action='store_true', help='Use the vehicle launch default: standalone B1.4 and binary DDS')
    args = parser.parse_args()
    replay(args.scene, args.output, args.mode, args.hz, args.seconds, args.split)
