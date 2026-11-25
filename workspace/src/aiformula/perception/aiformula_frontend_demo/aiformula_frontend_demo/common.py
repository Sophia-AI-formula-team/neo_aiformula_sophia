"""Shared frame and serialization helpers for the three demo nodes."""
import base64
import json
import math

import cv2
import numpy as np

from optimization_oa.geometry import GroundCamera, rotation_matrix

NS = 10**9
LANE_KEYS = ("center", "left", "right")


def ns(stamp):
    return int(stamp.sec) * NS + int(stamp.nanosec)


def image_bgr(msg):
    if hasattr(msg, "format"):
        formats = msg.format.lower()
        if formats and not any(x in formats for x in ("jpg", "jpeg", "png")):
            raise ValueError("compressed camera message is not JPEG/PNG")
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("empty or invalid camera image")
        return image
    modes = {"bgr8": (3, None), "rgb8": (3, cv2.COLOR_RGB2BGR),
             "bgra8": (4, cv2.COLOR_BGRA2BGR), "rgba8": (4, cv2.COLOR_RGBA2BGR),
             "mono8": (1, cv2.COLOR_GRAY2BGR)}
    if msg.encoding not in modes:
        raise ValueError(f"unsupported image encoding {msg.encoding}")
    channels, conversion = modes[msg.encoding]
    raw = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    image = raw[:, :msg.width*channels].reshape(msg.height, msg.width, channels)
    return cv2.cvtColor(image, conversion) if conversion is not None else image.copy()


def transform_matrix(tf):
    p, q = tf.translation, tf.rotation
    matrix = np.eye(4)
    matrix[:3, :3] = rotation_matrix([q.x, q.y, q.z, q.w])
    matrix[:3, 3] = (p.x, p.y, p.z)
    return matrix


def planar_pose(pose):
    p, q = pose.position, pose.orientation
    r = rotation_matrix([q.x, q.y, q.z, q.w])
    return np.array((p.x, p.y, math.atan2(r[1, 0], r[0, 0])))


def transform_pose(matrix):
    return np.array((matrix[0, 3], matrix[1, 3],
                     math.atan2(matrix[1, 0], matrix[0, 0])))


def interp_pose(t, a, b, max_gap=350_000_000):
    if a is None or b is None or not a[0] <= t <= b[0] or b[0]-a[0] > max_gap:
        return None
    if a[0] == b[0]:
        return np.asarray(a[1], dtype=float)
    delta = np.subtract(b[1], a[1])
    delta[2] = wrap(delta[2])
    return a[1] + delta * ((t-a[0])/(b[0]-a[0]))


def wrap(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))


def rotate(points, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray(points) @ np.array(((c, s), (-s, c)))


def to_world(points, pose):
    return rotate(points, pose[2]) + pose[:2]


def to_body(points, pose):
    return rotate(np.asarray(points) - pose[:2], -pose[2])


def camera_dict(info, base_from_optical, w, h, roi=.5):
    if not info.width or not info.height:
        raise ValueError("camera calibration has no image size")
    projection = np.asarray(info.p, dtype=float).reshape(3, 4).copy()
    if abs(projection[0, 0]) < 1e-6:
        projection[:, :3] = np.asarray(info.k, dtype=float).reshape(3, 3)
    projection[0] *= w/info.width
    projection[1] *= h/info.height
    result = dict(width=w, height=h, projection=projection.tolist(),
                  origin=base_from_optical[:3, 3].tolist(),
                  rotation=base_from_optical[:3, :3].tolist(), roi_top_ratio=roi)
    GroundCamera(**result)
    return result


def encode_mask(mask):
    pixels = (np.asarray(mask) > 0).astype(np.uint8) * 255
    ok, blob = cv2.imencode(".png", pixels)
    if not ok:
        raise ValueError("cannot encode semantic mask")
    return base64.b64encode(blob.tobytes()).decode("ascii")


def decode_mask(encoded):
    img = cv2.imdecode(np.frombuffer(base64.b64decode(encoded, validate=True), np.uint8),
                       cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError("cannot decode semantic mask")
    return img


def as_json(value):
    return json.dumps(value, ensure_ascii=True, allow_nan=False, separators=(",", ":"))


def lane_arrays(value):
    return tuple(np.asarray(value.get(key, []), dtype=float).reshape(-1, 2)
                 for key in LANE_KEYS)
