import json
import os
import time
from collections import OrderedDict
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

from concept_yolop.frame_codec import decode_frame
from concept_yolop.geometry import HEIGHT, REFERENCE_K

assert os.environ['ROS_DOMAIN_ID'] == '198' and os.environ['ROS_LOCALHOST_ONLY'] == '1'
prior = json.loads(Path('honda-v1.4.0-ground-projection-prior.json').read_text())
source_h = np.asarray(prior['H_ground_to_left_rectified_pixels'])
freeze_h = REFERENCE_K @ [[1, 0, 0], [0, 0, HEIGHT], [0, 1, 0]]
warp = freeze_h @ np.linalg.inv(source_h)
rclpy.init()
node = rclpy.create_node('verify_honda_live_projection')
bridge = CvBridge()
frames = OrderedDict()
matches = []
profiles = set()
camera_info = {}
raw_count = 0

def intrinsics(message):
    camera_info['frame'] = message.header.frame_id
    camera_info['P'] = list(message.p)

def raw(message):
    global raw_count
    raw_count += 1
    t = message.header.stamp
    frames[t.sec * 10**9 + t.nanosec] = bridge.imgmsg_to_cv2(message, 'bgr8')
    if len(frames) > 24:
        frames.popitem(last=False)

def projected(message):
    packet = decode_frame(message.data)
    profiles.add(packet['geometry_profile'])
    original = frames.get(packet['stamp_ns'])
    if original is None or 'P' not in camera_info:
        return
    live_k = np.asarray(camera_info['P']).reshape(3, 4)[:, :3]
    current_warp = warp @ np.asarray(prior['K']) @ np.linalg.inv(live_k)
    expected = cv2.warpPerspective(original, current_warp, (640, 360))
    np.testing.assert_array_equal(expected, packet['bgr'])
    matches.append(int(np.any(expected != original, axis=-1).sum()))

node.create_subscription(Image, '/aiformula_sensing/zed_node/left_image/undistorted', raw, qos_profile_sensor_data)
node.create_subscription(CameraInfo, '/aiformula_sensing/zed_node/left/camera_info', intrinsics, qos_profile_sensor_data)
node.create_subscription(String, '/concept/perception/frame', projected, qos_profile_sensor_data)
try:
    deadline = time.monotonic() + 40
    while len(matches) < 12 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    print(json.dumps(dict(raw_frames=raw_count, camera_info=camera_info, profiles=sorted(profiles), same_stamp_exact_images=len(matches),
                          minimum_pixels_changed_from_raw=min(matches, default=0))), flush=True)
    assert len(matches) == 12 and min(matches) > 0 and profiles == {'honda-v1.4.0-ground'}
finally:
    node.destroy_node()
    rclpy.shutdown()
