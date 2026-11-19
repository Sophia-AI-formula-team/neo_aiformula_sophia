"""In-memory timing of the stock Concept node and its ROS endpoints."""
import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def probe(root, seconds):
    node = rclpy.create_node('concept_latency_probe')
    camera, frames, jpeg, display = {}, {}, {}, {}

    def raw(msg, table):
        table[stamp_ns(msg.header.stamp)] = time.time_ns()

    def frame(msg):
        received = time.time_ns()
        packet = json.loads(msg.data)
        stamp = packet['stamp']
        frames[stamp['sec'] * 10**9 + stamp['nanosec']] = received

    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Image, '/aiformula_sensing/zed_node/left_image/undistorted',
                             lambda msg: raw(msg, camera), qos)
    node.create_subscription(Image, '/concept/perception/overlay', lambda msg: raw(msg, display), qos)
    node.create_subscription(CompressedImage, '/concept/perception/overlay/compressed',
                             lambda msg: raw(msg, jpeg), qos)
    node.create_subscription(String, '/concept/perception/frame', frame, qos)
    decisions = []
    auto = []
    if os.environ.get('PROFILE_PLANNER') == '1':
        state = node.create_publisher(String, '/concept/vehicle_state', 10)
        node.create_timer(.025, lambda: state.publish(String(data=json.dumps(
            dict(t=time.time(), leftRpm=0, rightRpm=0, gyro=0, wheelRadius=.127,
                 source='isolated_stationary_timing_probe', valid=True)))))
        node.create_subscription(String, '/concept/decision',
                                 lambda msg: decisions.append(json.loads(msg.data)), qos)
        node.create_subscription(TwistStamped, '/concept/auto_cmd', auto.append, qos)
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        if os.environ.get('PROFILE_PLANNER') == '1':
            slim = [{k: row.get(k) for k in ('stamp', 'status', 'reason', 'capture_age_s',
                                              'processing_ms', 'emitted_stamp', 'feasible_count')}
                    for row in decisions]
            root.with_name('planner.json').write_text(json.dumps(dict(decisions=slim, auto=len(auto))))
        root.write_text(json.dumps(dict(camera=camera, frames=frames, jpeg=jpeg, display=display)))
        node.destroy_node()


def profile(root):
    from concept_yolop import node as vision
    import torch

    active, rows = {}, {}
    kernel = os.environ.get('CONCEPT_TEST_CUDNN')
    encode = vision.encode_frame

    def measured(key, fn):
        def run(*args, **kwargs):
            start = time.monotonic_ns()
            result = fn(*args, **kwargs)
            active[key] = (time.monotonic_ns() - start) / 1e6
            return result
        return run

    class Publisher:
        def __init__(self, publisher):
            self.publisher = publisher

        def publish(self, msg):
            start = time.monotonic_ns()
            self.publisher.publish(msg)
            active['publish_ms'] = (time.monotonic_ns() - start) / 1e6
            active['published_ns'] = time.time_ns()

    class Camera(vision.CameraNode):
        def on_image(self, msg):
            now = time.time_ns()
            start = time.monotonic_ns()
            active.clear()
            active['callback_ns'] = now
            super().on_image(msg)
            active['callback_ms'] = (time.monotonic_ns() - start) / 1e6
            rows[stamp_ns(msg.header.stamp)] = dict(active)

    vision.encode_frame = measured('encode_ms', encode)
    camera = Camera()
    camera.bridge.imgmsg_to_cv2 = measured('preprocess_ms', camera.bridge.imgmsg_to_cv2)
    camera.predictor.predict = measured('predict_ms', camera.predictor.predict)
    camera.frames = Publisher(camera.frames)
    camera.publish_compressed = measured('jpeg_ms', camera.publish_compressed)
    if kernel == 'heuristic':
        torch.backends.cudnn.deterministic = False
    elif kernel == 'autotune':
        torch.backends.cudnn.benchmark = True
        camera.predictor.predict(np.zeros((360, 640, 3), dtype=np.uint8))
    try:
        rclpy.spin(camera)
    except KeyboardInterrupt:
        pass
    finally:
        root.write_text(json.dumps(rows))
        camera.destroy_node()


def summarize(root):
    raw = json.loads((root / 'probe.json').read_text())
    rows = json.loads((root / 'profile.json').read_text())
    sample = [(stamp, rows[stamp]) for stamp in raw['frames'] if stamp in rows and 'published_ns' in rows[stamp]]
    sample = sample[3:]
    if not sample:
        raise RuntimeError('No paired frames after warmup')
    metrics = defaultdict(list)
    for stamp, row in sample:
        captured = int(stamp)
        for key, value in row.items():
            if key.endswith('_ms'):
                metrics[key].append(value)
        metrics['capture_to_node_ms'].append((row['callback_ns'] - captured) / 1e6)
        metrics['dds_to_consumer_ms'].append((raw['frames'][stamp] - row['published_ns']) / 1e6)
        metrics['capture_to_consumer_ms'].append((raw['frames'][stamp] - captured) / 1e6)
        if stamp in raw['camera']:
            metrics['capture_to_independent_camera_sub_ms'].append((raw['camera'][stamp] - captured) / 1e6)
            metrics['vision_sub_extra_wait_ms'].append((row['callback_ns'] - raw['camera'][stamp]) / 1e6)
        if stamp in raw['jpeg']:
            metrics['capture_to_jpeg_ms'].append((raw['jpeg'][stamp] - captured) / 1e6)
    report = dict(counts={key: len(v) for key, v in raw.items()}, paired=len(sample))
    report['vision_hz'] = round((len(sample) - 1) * 1e9 / (raw['frames'][sample[-1][0]] - raw['frames'][sample[0][0]]), 2)
    report['camera_hz'] = round((len(raw['camera']) - 1) * 1e9 / (max(raw['camera'].values()) - min(raw['camera'].values())), 2)
    report['stage_ms'] = {key: dict(zip(('min', 'p50', 'p95', 'max'), np.percentile(v, (0, 50, 95, 100)).round(3)))
                          for key, v in metrics.items()}
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=('probe', 'profile', 'summarize'))
    parser.add_argument('path', type=Path)
    parser.add_argument('--seconds', type=float, default=18)
    args = parser.parse_args()
    if args.mode == 'summarize':
        summarize(args.path)
    else:
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '198':
            raise SystemExit('Use isolated localhost domain 198')
        rclpy.init()
        try:
            probe(args.path, args.seconds) if args.mode == 'probe' else profile(args.path)
        finally:
            if rclpy.ok():
                rclpy.shutdown()
