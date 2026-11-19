"""Compare patched and stock FP32 on 12 live ZED frames retained only in RAM."""
import json
import os
import time
from collections import Counter
from types import MethodType

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from torch import nn

from concept_yolop.frame_codec import decode_frame, encode_frame
from concept_yolop.inference import Predictor
from concept_yolop.model import NativeCudaConv2d
from concept_yolop.weights import default_path


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '198':
        raise SystemExit('Use isolated localhost domain 198')
    predictor = Predictor(default_path())
    layers = [layer for layer in predictor.model.modules() if isinstance(layer, NativeCudaConv2d)]
    assert len(layers) == 2
    rclpy.init()
    node = rclpy.create_node('real_zed_output_parity')
    bridge = CvBridge()
    pictures = []
    last = 0

    def collect(msg):
        nonlocal last
        now = time.monotonic()
        if len(pictures) < 12 and now - last >= .3:
            pictures.append(bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
            last = now

    node.create_subscription(Image, '/aiformula_sensing/zed_node/left_image/undistorted',
                             collect, qos_profile_sensor_data)
    try:
        end = time.monotonic() + 18
        while len(pictures) < 12 and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    assert len(pictures) == 12, len(pictures)

    fixed = [predictor.predict(image) for image in pictures]
    for layer in layers:
        layer.forward = MethodType(nn.Conv2d.forward, layer)
    stock = [predictor.predict(image) for image in pictures]
    diffs = Counter(road=0, lane=0, boxes=0, labels=0, scores=0, png=0)
    detections = []
    for (bgr, road, lane, det), (_, road_ref, lane_ref, det_ref) in zip(fixed, stock):
        diffs['road'] += int(np.count_nonzero(road != road_ref))
        diffs['lane'] += int(np.count_nonzero(lane != lane_ref))
        for key in ('boxes', 'labels', 'scores'):
            diffs[key] += int(not np.array_equal(det[key], det_ref[key]))
        detections.append(len(det['labels']))
        msg = encode_frame(1, 0, 'camera', bgr, road, lane, det)
        decoded = decode_frame(msg)
        diffs['png'] += int(not all(np.array_equal(decoded[k], expected) for k, expected in
                                    zip(('bgr', 'road', 'lane'), (bgr, road, lane))))
    print(json.dumps(dict(frames=len(pictures), detections_per_frame=detections, differences=dict(diffs))), flush=True)
    assert sum(diffs.values()) == 0, diffs


if __name__ == '__main__':
    main()
