"""Measure lossless PNG flags on the same predicted color and mask frames."""
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from concept_yolop.frame_codec import decode_frame, encode_frame, _PNG_COLOR
from concept_yolop.inference import Predictor
from concept_yolop.weights import default_path
from concept_yolop import frame_codec


print(cv2.__version__, 'explicit filter API', hasattr(cv2, 'IMWRITE_PNG_FILTER'))
predict = Predictor(default_path()).predict
packets = [predict(cv2.imread(str(path))) for path in sorted(Path(sys.argv[1]).glob('*_rgb.png'))]
for name, flags in (
        ('stock', _PNG_COLOR),
        ('implicit', []),
        ('rle-default-level', [cv2.IMWRITE_PNG_STRATEGY, cv2.IMWRITE_PNG_STRATEGY_RLE]),
        ('rle-explicit-level', [cv2.IMWRITE_PNG_COMPRESSION, 1, cv2.IMWRITE_PNG_STRATEGY, cv2.IMWRITE_PNG_STRATEGY_RLE]),
        ('uncompressed', [cv2.IMWRITE_PNG_COMPRESSION, 0])):
    frame_codec._PNG_COLOR = frame_codec._PNG_MASK = flags
    samples, sizes = [], []
    for repeat in range(3):
        for bgr, road, lane, det in packets:
            begin = time.monotonic_ns()
            msg = encode_frame(1, 0, 'camera', bgr, road, lane, det)
            samples.append((time.monotonic_ns() - begin) / 1e6)
            sizes.append(len(msg))
            decoded = decode_frame(msg)
            assert all(np.array_equal(decoded[k], src) for k, src in zip(('bgr', 'road', 'lane'), (bgr, road, lane)))
    print(json.dumps(dict(mode=name, flags=flags, p50_ms=round(float(np.median(samples)), 3),
                          p95_ms=round(float(np.percentile(samples, 95)), 3),
                          median_wire_bytes=int(np.median(sizes)))), flush=True)
