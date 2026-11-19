"""Save a matching rectified image and CameraInfo to measure the stopped vehicle."""
import argparse
import json
import os
from pathlib import Path
import tempfile
import time

from .geometry import CalibrationReceipt, optical_height


def install_receipt(source):
    content = source.expanduser().read_bytes()
    config = Path(os.environ.get('XDG_CONFIG_HOME') or Path.home() / '.config')
    destination = config / 'concept' / 'camera.json'
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as staged:
        stage_path = Path(staged.name)
        try:
            staged.write(content)
            staged.flush()
            os.fsync(staged.fileno())
            receipt = CalibrationReceipt(stage_path)
            os.replace(stage_path, destination)
        finally:
            stage_path.unlink(missing_ok=True)
    return destination, receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--capture', type=Path, metavar='NEW_DIRECTORY')
    mode.add_argument('--check', type=Path, metavar='RECEIPT_JSON')
    mode.add_argument('--install', type=Path, metavar='RECEIPT_JSON',
                      help='Validate and install the default Concept vehicle receipt')
    parser.add_argument('--mount-id')
    parser.add_argument('--optical-height-m', type=float,
                        help='Measured height of the left optical centre above level ground in metres')
    parser.add_argument('--image-topic', default='/aiformula_sensing/zed_node/left_image/undistorted')
    parser.add_argument('--camera-info-topic', default='/aiformula_sensing/zed_node/left/camera_info')
    args = parser.parse_args()
    if args.optical_height_m is not None:
        if not args.capture:
            parser.error('--optical-height-m is for --capture; receipt validation reads the stored height')
        try:
            optical_height(args.optical_height_m)
        except ValueError as exc:
            parser.error(str(exc))
    if args.check:
        receipt = CalibrationReceipt(args.check)
        print(f'Maximum held-out ground error: {receipt.max_error:.3f} m')
        return
    if args.install:
        destination, receipt = install_receipt(args.install)
        print(f'Installed {destination}. Maximum held-out ground error: {receipt.max_error:.3f} m')
        return
    if args.capture.exists() or not args.mount_id:
        parser.error('Choose a new capture directory and supply --mount-id')
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init(args=[])
    node = rclpy.create_node('concept_camera_measurement')
    images, infos = {}, {}

    def save(records, msg):
        key = (msg.header.frame_id, msg.header.stamp.sec, msg.header.stamp.nanosec, msg.width, msg.height)
        records[key] = msg
        if len(records) > 16:
            records.pop(next(iter(records)))

    node.create_subscription(Image, args.image_topic, lambda m: save(images, m), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, args.camera_info_topic, lambda m: save(infos, m), qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            keys = images.keys() & infos.keys()
            if not keys:
                continue
            key = max(keys, key=lambda k: k[1:3])
            info = infos[key]
            bgr = CvBridge().imgmsg_to_cv2(images[key], 'bgr8')
            args.capture.mkdir(parents=True, exist_ok=False)
            if not cv2.imwrite(str(args.capture / 'camera.png'), bgr):
                raise RuntimeError('Could not save camera.png')
            record = dict(image_topic=args.image_topic, frame_id=key[0], mount_id=args.mount_id,
                          width=info.width, height=info.height, projection=list(info.p),
                          fit_marks=[], ground_marks=[])
            if args.optical_height_m is not None:
                record['optical_height_m'] = args.optical_height_m
            out = args.capture / 'camera.json'
            out.write_text(json.dumps(record, indent=2), encoding='utf-8')
            print(out)
            return
        raise SystemExit('No matching image/CameraInfo within 15 s. Check topic names and source stamps.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
