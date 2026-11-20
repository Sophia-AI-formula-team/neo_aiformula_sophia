"""Wire contract concept.perception.v1, shared by camera and planner nodes."""
import base64
import json
import math
import struct

import cv2
import numpy as np

from .geometry import PROFILES, UNVERIFIED
from .weights import SHA256

SCHEMA = 'concept.perception.v1'
LOCAL_SCHEMA = 'concept.perception.local.v1'
LOCAL_FRAME_TOPIC = '/concept/internal/perception/packed'
W, H = 640, 360
_HEADER_SIZE = struct.Struct('!I')
_MAX_HEADER = 65536
_BGR_SIZE = H * W * 3
_MASK_SIZE = H * W // 8
_BODY_SIZE = _BGR_SIZE + 2 * _MASK_SIZE


# A fixed scanline filter avoids trying five filters on every row. In OpenCV
# 4.2, specifying the compression level selects adaptive filtering. Leave the
# level unset on those builds to keep the encoder's fixed SUB filter.
_PNG_COLOR = _PNG_MASK = [cv2.IMWRITE_PNG_STRATEGY, cv2.IMWRITE_PNG_STRATEGY_RLE]
if hasattr(cv2, 'IMWRITE_PNG_FILTER'):
    _common = [cv2.IMWRITE_PNG_COMPRESSION, 1, *_PNG_COLOR, cv2.IMWRITE_PNG_FILTER]
    _PNG_COLOR = _common + [cv2.IMWRITE_PNG_FILTER_SUB]
    _PNG_MASK = _common + [cv2.IMWRITE_PNG_FILTER_NONE]


def _png(image):
    ok, buf = cv2.imencode('.png', image, _PNG_COLOR if image.ndim == 3 else _PNG_MASK)
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


def validate_pixels(bgr, road, lane):
    if (bgr.shape != (H, W, 3) or bgr.dtype != np.uint8 or road.shape != (H, W)
            or lane.shape != (H, W) or road.dtype != np.bool_ or lane.dtype != np.bool_):
        raise ValueError('Expected canonical BGR and boolean road/lane masks')


def encode_frame(sec, nanosec, frame_id, bgr, road, lane, detections, geometry_profile=UNVERIFIED):
    if geometry_profile not in PROFILES:
        raise ValueError("Invalid camera geometry profile")
    validate_pixels(bgr, road, lane)
    return json.dumps({
        'schema': SCHEMA, 'model_sha256': SHA256, 'geometry_profile': geometry_profile,
        'stamp': {'sec': int(sec), 'nanosec': int(nanosec)}, 'frame_id': frame_id,
        'width': W, 'height': H, 'bgr_png': _png(bgr),
        'road_png': _png(road.astype(np.uint8) * 255),
        'lane_png': _png(lane.astype(np.uint8) * 255),
        'boxes': detections['boxes'].tolist(), 'scores': detections['scores'].tolist(),
        'labels': detections['labels'].tolist(),
    }, separators=(',', ':'), allow_nan=False)


def parse_frame_header(payload):
    """Validate identity, geometry and time before committing to pixel decoding."""
    if len(payload) > 4000000:
        raise ValueError('Oversize perception frame')
    obj = json.loads(payload)
    return _validated_header(obj, SCHEMA)


def _validated_header(obj, schema):
    if (obj['schema'], obj['model_sha256'], obj['width'], obj['height']) != (schema, SHA256, W, H):
        raise ValueError('Unexpected perception contract')
    if obj.get('geometry_profile') not in PROFILES:
        raise ValueError('Missing camera geometry profile')
    sec, ns = obj['stamp']['sec'], obj['stamp']['nanosec']
    if type(sec) is not int or type(ns) is not int or not 0 <= ns < 1000000000:
        raise ValueError('Invalid acquisition time')
    if not isinstance(obj['frame_id'], str):
        raise ValueError('Expected optical frame ID')
    return obj, sec * 1000000000 + ns, obj['frame_id'], obj['geometry_profile']


def validated_detections(det):
    boxes = np.asarray(det['boxes'], dtype=np.float32)
    if boxes.shape == (0,):
        boxes = boxes.reshape(0, 4)
    scores = np.asarray(det['scores'], dtype=np.float32)
    labels = np.asarray(det['labels'])
    n = len(scores)
    if (boxes.shape != (n, 4) or labels.shape != (n,) or scores.shape != (n,) or n > 100
            or not np.all(np.isfinite(boxes)) or not np.all(np.isfinite(scores))
            or np.any((scores < 0) | (scores > 1))
            or (n and (not np.issubdtype(labels.dtype, np.integer) or not np.isin(labels, [1,3,10,91]).all()))):
        raise ValueError('Invalid B1.4 detections')
    return dict(boxes=boxes, scores=scores, labels=labels.astype(np.int64, copy=False))


def decode_frame_data(obj):
    """Unpack detections and pixels from a header-validated wire object."""
    det = validated_detections(obj)
    return dict(geometry_profile=obj['geometry_profile'], stamp_ns=obj['stamp']['sec'] * 1000000000 + obj['stamp']['nanosec'],
                frame_id=obj['frame_id'],
                bgr=_image(obj['bgr_png'], 3), road=_image(obj['road_png'], 1),
                lane=_image(obj['lane_png'], 1), **det)


def decode_frame(payload):
    obj, _, _, _ = parse_frame_header(payload)
    return decode_frame_data(obj)


def encode_local_frame(sec, nanosec, frame_id, bgr, road, lane, detections, geometry_profile, timings):
    """Pack one exact frame for the private same-host DDS route."""
    validate_pixels(bgr, road, lane)
    meta = {
        'schema': LOCAL_SCHEMA, 'model_sha256': SHA256, 'geometry_profile': geometry_profile,
        'stamp': {'sec': int(sec), 'nanosec': int(nanosec)}, 'frame_id': frame_id,
        'width': W, 'height': H, 'timings': timings,
        **{key: detections[key].tolist() for key in ('boxes', 'scores', 'labels')},
    }
    _validated_header(meta, LOCAL_SCHEMA)
    header = json.dumps(meta, separators=(',', ':'), allow_nan=False).encode('utf-8')
    if len(header) > _MAX_HEADER:
        raise ValueError('Oversize local perception header')
    return b''.join((_HEADER_SIZE.pack(len(header)), header, bgr.tobytes(),
                     np.packbits(road).tobytes(), np.packbits(lane).tobytes()))


def decode_local_frame(payload):
    """Validate and unpack the lossless private binary packet."""
    if not _HEADER_SIZE.size < len(payload) <= _HEADER_SIZE.size + _MAX_HEADER + _BODY_SIZE:
        raise ValueError('Invalid local perception packet size')
    view = memoryview(payload).cast('B')
    length, = _HEADER_SIZE.unpack_from(view)
    offset = _HEADER_SIZE.size + length
    if not 0 < length <= _MAX_HEADER or len(view) != offset + _BODY_SIZE:
        raise ValueError('Invalid local perception header size')
    obj = json.loads(view[_HEADER_SIZE.size:offset].tobytes())
    obj, ns, frame_id, profile = _validated_header(obj, LOCAL_SCHEMA)
    timings = obj['timings']
    if not isinstance(timings, dict) or any(type(value) not in (float, int) or not math.isfinite(value)
                                           for value in timings.values()):
        raise ValueError('Invalid local perception timings')
    frame = validated_detections(obj)
    raw = np.frombuffer(view, dtype=np.uint8, offset=offset)
    frame['bgr'] = raw[:_BGR_SIZE].reshape(H, W, 3)
    frame['road'] = np.unpackbits(raw[_BGR_SIZE:_BGR_SIZE + _MASK_SIZE]).reshape(H, W).view(np.bool_)
    frame['lane'] = np.unpackbits(raw[_BGR_SIZE + _MASK_SIZE:]).reshape(H, W).view(np.bool_)
    return ns, frame_id, profile, frame, timings
