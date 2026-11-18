"""Run a real frame through B1.4 on the installed machine without starting ROS."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import torch
import torchvision

from .geometry import UNITY
from .frame_codec import decode_frame, encode_frame
from .inference import Predictor
from .weights import SHA256, default_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('--weights', type=Path, default=default_path())
    parser.add_argument('--device', choices=('cpu', 'cuda'), default='cuda')
    parser.add_argument('--compare', type=Path, help='Frozen perception.npz for this exact frame')
    opts = parser.parse_args()
    bgr = cv2.imread(str(opts.image), cv2.IMREAD_COLOR)
    if bgr is None:
        parser.error('Could not read image')
    bgr, road, lane, det = Predictor(opts.weights, opts.device).predict(bgr)
    wire = encode_frame(11, 23, 'camera', bgr, road, lane, det, UNITY if opts.compare else 'unverified-resize')
    packet = decode_frame(wire)
    for k, v in dict(bgr=bgr, road=road, lane=lane, **det).items():
        np.testing.assert_array_equal(packet[k], v)
    report = dict(weights=SHA256, device=opts.device, torch=torch.__version__,
                  torchvision=torchvision.__version__, detections=len(det['boxes']),
                  road_pixels=int(road.sum()), lane_pixels=int(lane.sum()), wire_bytes=len(wire.encode()))
    if opts.compare:
        with np.load(opts.compare, allow_pickle=False) as golden:
            np.testing.assert_array_equal(road, golden['raw'])
            np.testing.assert_array_equal(lane, golden['lane'])
            for k in ('boxes', 'scores', 'labels'):
                np.testing.assert_allclose(det[k], golden[k], rtol=0, atol=1.e-5)
        report['frozen_frame_matches'] = True
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
