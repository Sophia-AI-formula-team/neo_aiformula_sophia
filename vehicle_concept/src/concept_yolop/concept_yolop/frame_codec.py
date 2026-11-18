"""Wire contract concept.perception.v1, shared by camera and planner nodes."""
import base64
import json
import struct

import cv2
import numpy as np

from .geometry import PROFILES, UNVERIFIED
from .weights import SHA256

SCHEMA = 'concept.perception.v1'
W, H = 640, 360


def _png(image):
    ok, buf = cv2.imencode('.png', image, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    if not ok:
        raise ValueError('Failed to encode a camera frame')
    return base64.b64encode(buf).decode('ascii')


def _image(encoded, channels):
    # Inspect dimensions before allocating decoded pixels.
    if not isinstance(encoded, str) or len(encoded) > 1600000:
        raise ValueError('Invalid encoded frame size')
    raw = base64.b64decode(encoded, validate=True)
    if len(raw) < 33 or raw[:16] != b'\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR':
        raise ValueError('Expected a PNG')
    width, height, bits, kind = struct.unpack('>IIBB', raw[16:26])
    if (width, height, bits, kind) != (W, H, 8, 2 if channels == 3 else 0):
        raise ValueError('Unexpected PNG dimensions or colour format')
    image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError('Incomplete camera PNG')
    if channels == 1:
        if np.any((image != 0) & (image != 255)):
            raise ValueError('Expected binary mask pixels')
        return image != 0
    return image


def encode_frame(sec, nanosec, frame_id, bgr, road, lane, detections, geometry_profile=UNVERIFIED):
    if geometry_profile not in PROFILES:
        raise ValueError("Invalid camera geometry profile")
    if (bgr.shape != (H, W, 3) or bgr.dtype != np.uint8 or road.shape != (H, W)
            or lane.shape != (H, W) or road.dtype != np.bool_ or lane.dtype != np.bool_):
        raise ValueError('Expected canonical BGR and boolean road/lane masks')
    return json.dumps({
        'schema': SCHEMA, 'model_sha256': SHA256, 'geometry_profile': geometry_profile,
        'stamp': {'sec': int(sec), 'nanosec': int(nanosec)}, 'frame_id': frame_id,
        'width': W, 'height': H, 'bgr_png': _png(bgr),
        'road_png': _png(road.astype(np.uint8) * 255),
        'lane_png': _png(lane.astype(np.uint8) * 255),
        'boxes': detections['boxes'].tolist(), 'scores': detections['scores'].tolist(),
        'labels': detections['labels'].tolist(),
    }, separators=(',', ':'), allow_nan=False)


def decode_frame(payload):
    if len(payload) > 4000000:
        raise ValueError('Oversize perception frame')
    obj = json.loads(payload)
    if (obj['schema'], obj['model_sha256'], obj['width'], obj['height']) != (SCHEMA, SHA256, W, H):
        raise ValueError('Unexpected perception contract')
    if obj.get('geometry_profile') not in PROFILES:
        raise ValueError('Missing camera geometry profile')
    sec, ns = obj['stamp']['sec'], obj['stamp']['nanosec']
    if type(sec) is not int or type(ns) is not int or not 0 <= ns < 1000000000:
        raise ValueError('Invalid acquisition time')
    if not isinstance(obj['frame_id'], str):
        raise ValueError('Expected optical frame ID')
    boxes = np.asarray(obj['boxes'], dtype=np.float32)
    if boxes.shape == (0,):
        boxes = boxes.reshape(0, 4)
    scores = np.asarray(obj['scores'], dtype=np.float32)
    labels = np.asarray(obj['labels'])
    n = len(scores)
    if (boxes.shape != (n, 4) or labels.shape != (n,) or scores.shape != (n,) or n > 100
            or not np.all(np.isfinite(boxes)) or not np.all(np.isfinite(scores))
            or np.any((scores < 0) | (scores > 1))
            or (n and (not np.issubdtype(labels.dtype, np.integer) or not np.isin(labels, [1,3,10,91]).all()))):
        raise ValueError('Invalid B1.4 detections')
    return dict(geometry_profile=obj['geometry_profile'], stamp_ns=sec * 1000000000 + ns, frame_id=obj['frame_id'],
                bgr=_image(obj['bgr_png'], 3), road=_image(obj['road_png'], 1),
                lane=_image(obj['lane_png'], 1), boxes=boxes, scores=scores, labels=labels.astype(np.int64))
