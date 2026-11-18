"""Measure wall time from stamped sensor_msgs/Image to B1.4 perception over Foxy DDS."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from concept_yolop.weights import SHA256

CAMERA = '/aiformula_sensing/zed_node/left_image/undistorted'
FRAME = '/concept/perception/frame'
HZ = 15


def distribution(values):
    return {key: round(float(value), 3) for key, value in zip(
        ('min', 'p50', 'p95', 'max'), np.percentile(values, (0, 50, 95, 100)))}


class Probe(Node):
    def __init__(self, frames):
        super().__init__('concept_camera_wall_probe')
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.feed = self.create_publisher(Image, CAMERA, qos)
        self.create_subscription(String, FRAME, self.receive, 10)
        bridge = CvBridge()
        self.images = []
        for path in frames:
            if path.suffix == '.cdr':
                msg = deserialize_message(path.read_bytes(), Image)
            else:
                frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
                if frame is None or frame.shape != (360, 640, 3):
                    raise ValueError('Expected a readable original 640x360 frame: ' + str(path))
                msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.frame_id = 'replay_optical'
            self.images.append(msg)
        self.sent = {}
        self.received = {}

    def send(self, i):
        msg = self.images[i % len(self.images)]
        stamp = self.get_clock().now()
        msg.header.stamp = stamp.to_msg()
        self.sent[stamp.nanoseconds] = time.monotonic_ns()
        self.feed.publish(msg)

    def receive(self, msg):
        received = time.monotonic_ns()
        acquired = json.loads(msg.data)['stamp']
        stamp = acquired['sec']*1000000000 + acquired['nanosec']
        if stamp in self.sent:
            self.received[stamp] = (received, len(msg.data))


def experiment(probe, camera, count):
    def wait_until(deadline):
        while time.monotonic() < deadline:
            if camera.poll() is not None:
                raise RuntimeError('B1.4 node exited: ' + str(camera.returncode))
            rclpy.spin_once(probe, timeout_sec=max(0., min(.01, deadline - time.monotonic())))

    limit = time.monotonic() + 45
    while probe.feed.get_subscription_count() == 0 and time.monotonic() < limit:
        wait_until(time.monotonic() + .1)
    if not probe.feed.get_subscription_count():
        raise RuntimeError('Camera subscription did not appear')

    i = 0
    while len(probe.received) < 15:
        if time.monotonic() >= limit:
            raise RuntimeError('Camera produced fewer than 15 warm-up outputs')
        probe.send(i)
        i += 1
        wait_until(time.monotonic() + 1 / HZ)

    warmup_age = (probe.received[min(probe.received)][0]-probe.sent[min(probe.received)])/1e6
    probe.sent.clear()
    probe.received.clear()
    start = time.monotonic()
    for i in range(count):
        wait_until(start + i / HZ)
        probe.send(i)
    wait_until(time.monotonic() + .8)
    sent = sorted(probe.sent)
    arrivals = sorted(probe.received)
    if len(arrivals) < 2:
        raise RuntimeError('Fewer than two measured outputs')
    latencies = [(probe.received[s][0]-probe.sent[s])/1e6 for s in arrivals]
    input_duration = (probe.sent[sent[-1]]-probe.sent[sent[0]])/1e9
    output_duration = (probe.received[arrivals[-1]][0]-probe.received[arrivals[0]][0])/1e9
    return dict(model_sha256=SHA256, geometry='640x360', warmup_received=15,
                first_delivered_image_latency_ms=round(warmup_age, 3),
                sent=len(sent), received=len(arrivals), lost=len(sent)-len(arrivals),
                input_hz=round((len(sent)-1)/input_duration, 3),
                output_hz=round((len(arrivals)-1)/output_duration, 3),
                acquisition_to_perception_ms=distribution(latencies),
                camera_intervals_ms=distribution(np.diff([probe.sent[s] for s in sent])/1e6),
                perception_intervals_ms=distribution(np.diff([probe.received[s][0] for s in arrivals])/1e6),
                wire_bytes=distribution([probe.received[s][1] for s in arrivals]))


def profiled_camera(path, ros_args):
    from concept_yolop import node as camera_module

    # Instrument the production node in the child process, outside its source.
    # Keep timings in memory until shutdown to avoid disk I/O on camera frames.
    measurements = {}
    active = {}
    encode = camera_module.encode_frame

    def measure_encoder(*args, **kwargs):
        started = time.monotonic_ns()
        payload = encode(*args, **kwargs)
        active['encode_ms'] = (time.monotonic_ns()-started)/1e6
        return payload

    class Camera(camera_module.CameraNode):
        def on_image(self, msg):
            acquired_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            started = time.monotonic_ns()
            active.clear()
            active['callback_ros_ns'] = self.get_clock().now().nanoseconds
            super().on_image(msg)
            active['callback_ms'] = (time.monotonic_ns()-started)/1e6
            measurements[acquired_ns] = dict(active)

    camera_module.encode_frame = measure_encoder
    rclpy.init(args=ros_args)
    node = Camera()
    predictor = node.predictor.predict

    def measure_inference(image):
        started = time.monotonic_ns()
        result = predictor(image)
        active['predict_ms'] = (time.monotonic_ns()-started)/1e6
        return result

    node.predictor.predict = measure_inference
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        path.write_text(json.dumps(measurements))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frame_dir', type=Path, help='Recorded original *_rgb.png files')
    parser.add_argument('--weights', required=True)
    parser.add_argument('--format', choices=('png','zed-cdr'), default='png')
    parser.add_argument('--camera-dds-profile', type=Path, help='Optional Fast DDS XML on the image publisher only')
    parser.add_argument('--device', choices=('cuda', 'cpu'), default='cuda')
    parser.add_argument('--count', type=int, default=180)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', action='store_true', help='Attribute callback, inference and PNG time')
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        parser.error('Use a private local ROS domain for the recorded camera publisher')
    paths = sorted(args.frame_dir.glob('*.cdr' if args.format == 'zed-cdr' else '*_rgb.png'))[:args.count]
    if len(paths) != args.count or args.count < 2:
        parser.error('Supply at least --count distinct original images and --count >= 2')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    child_env = os.environ.copy()
    if args.camera_dds_profile:
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(args.camera_dds_profile.resolve(strict=True))
    rclpy.init()
    probe = Probe(paths)
    try:
        with args.output.with_suffix('.node.log').open('w') as log:
            detail = args.output.with_suffix('.profile.json')
            if args.profile:
                command = [sys.executable, __file__, '--profile-node', str(detail)]
            else:
                command = [sys.executable, '-c', 'from concept_yolop.node import main; main()']
            command += ['--ros-args', '-p', 'weights:=' + args.weights, '-p', 'device:=' + args.device]
            camera = subprocess.Popen(command, stdout=log, stderr=log, env=child_env)
            try:
                metrics = experiment(probe, camera, args.count)
                metrics.update(device=args.device, first_frame=paths[0].name, last_frame=paths[-1].name,
                               source_encoding=probe.images[0].encoding, camera_dds_profile=
                               args.camera_dds_profile.name if args.camera_dds_profile else None)
            finally:
                if camera.poll() is None:
                    camera.send_signal(signal.SIGINT)
                    try:
                        camera.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        camera.kill()
                        camera.wait()
        if args.profile:
            raw = json.loads(detail.read_text())
            matched = {s: raw[str(s)] for s in probe.sent if str(s) in raw}
            metrics['camera_callbacks'] = len(matched)
            metrics['sent_without_callback'] = len(probe.sent)-len(matched)
            metrics['outputs_missing_after_callback'] = len(matched)-len(probe.received)
            if matched:
                for stat in ('predict_ms', 'encode_ms', 'callback_ms'):
                    vals = [row[stat] for row in matched.values() if stat in row]
                    if vals:
                        metrics[stat] = distribution(vals)
                metrics['send_to_callback_ms'] = distribution([
                    (row['callback_ros_ns']-s)/1e6 for s, row in matched.items()])
        report = json.dumps(metrics, indent=2)
        args.output.write_text(report + '\n')
        print(report, flush=True)
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) > 2 and sys.argv[1] == '--profile-node':
        profiled_camera(Path(sys.argv[2]), sys.argv[3:])
    else:
        main()
