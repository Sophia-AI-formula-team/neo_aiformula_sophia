import json
import math
from collections import namedtuple

import numpy as np


Cloud = namedtuple("Cloud", "stamp_ns points")
DetectionFrame = namedtuple("DetectionFrame", "stamp_ns frame_id detections")
RasterFrame = namedtuple("RasterFrame", "stamp_ns frame_id pixels")


def decode_detections(text):
    payload = json.loads(text)
    header = payload["header"]
    stamp = header["stamp"]
    nanoseconds = int(stamp["sec"]) * 10**9 + int(stamp["nanosec"])
    frame = header["frame_id"]
    if nanoseconds <= 0 or not frame or not isinstance(payload["detections"], list):
        raise ValueError("invalid_detection_header")
    return DetectionFrame(nanoseconds, frame, payload["detections"])


def boxes_to_mask(detections, width, height):
    mask = np.zeros((height, width), dtype=np.uint8)
    for detection in detections:
        if detection["class_id"] not in (1, 3, 91):
            continue
        box = np.asarray(detection["box_xyxy"], dtype=float)
        if box.shape != (4,) or not np.all(np.isfinite(box)):
            raise ValueError("invalid_detection_box")
        left, top, right, bottom = (int(round(value)) for value in box)
        if right < left or bottom < top:
            raise ValueError("invalid_detection_box")
        left, top = max(0, left), max(0, top)
        right, bottom = min(width - 1, right), min(height - 1, bottom)
        if right >= left and bottom >= top:
            mask[top:bottom + 1, left:right + 1] = 255
    return mask


def decode_raster(data, width, height, step, encoding):
    channels = {"mono8": 1, "8UC1": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
    if channels is None or min(width, height) < 1 or step < width * channels:
        raise ValueError("unsupported_raster")
    image = np.ndarray((height, width, channels), dtype=np.uint8, buffer=data,
                       strides=(step, channels, 1))
    if channels == 1:
        return image[:, :, 0].copy()
    rgb = image[:, :, :3]
    return (rgb[:, :, ::-1] if encoding.startswith("bgr") else rgb).copy()


def cloud_xy(data, fields, width, height, row_step, point_step, big_endian):
    if width == 0 or height == 0:
        return np.empty((0, 2))
    dtype = np.dtype(">f4" if big_endian else "<f4")
    coords = []
    for axis in ("x", "y"):
        offset, datatype = fields[axis]
        if datatype != 7 or not 0 <= offset <= point_step - dtype.itemsize:
            raise ValueError("invalid_pointcloud_field")
        values = np.ndarray((height, width), dtype=dtype, buffer=data,
                            offset=offset, strides=(row_step, point_step))
        coords.append(values.ravel())
    return np.column_stack(coords).astype(float)


def matched_clouds(clouds, stamp, slop_ns):
    group = []
    for queue in clouds:
        if not queue:
            return None
        cloud = min(queue, key=lambda item: abs(item.stamp_ns - stamp))
        if abs(cloud.stamp_ns - stamp) > slop_ns:
            return None
        group.append(cloud)
    if max(c.stamp_ns for c in group) - min(c.stamp_ns for c in group) > slop_ns:
        return None
    return group


def interpolate_odom(odom, stamp, max_gap_ns):
    earlier = max((s for s in odom if s <= stamp), default=None)
    later = min((s for s in odom if s >= stamp), default=None)
    if earlier is None or later is None or later - earlier > max_gap_ns:
        return None
    a = np.asarray(odom[earlier], dtype=float)
    if earlier == later:
        return a
    b = np.asarray(odom[later], dtype=float)
    delta = b - a
    delta[2] = math.atan2(math.sin(delta[2]), math.cos(delta[2]))
    return a + delta * ((stamp - earlier) / (later - earlier))


def align_clouds(clouds, odom, target_stamp, max_gap_ns):
    target_pose = None
    aligned = []
    for cloud in clouds:
        if abs(cloud.stamp_ns - target_stamp) <= 1_000_000:
            aligned.append(cloud.points)
            continue
        if target_pose is None:
            target_pose = interpolate_odom(odom, target_stamp, max_gap_ns)
            if target_pose is None:
                return None
        source_pose = interpolate_odom(odom, cloud.stamp_ns, max_gap_ns)
        if source_pose is None:
            return None
        angle = source_pose[2] - target_pose[2]
        c, s = math.cos(angle), math.sin(angle)
        rotation = np.array(((c, s), (-s, c)))
        c, s = math.cos(target_pose[2]), math.sin(target_pose[2])
        translation = (source_pose[:2] - target_pose[:2]) @ np.array(((c, -s), (s, c)))
        aligned.append(cloud.points @ rotation + translation)
    return aligned
