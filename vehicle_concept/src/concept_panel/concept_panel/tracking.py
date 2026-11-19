"""Match the tracking inputs used for an issued decision to its exact camera frame."""
from collections import OrderedDict
import math

from concept_visualization.geometry import pixel


MODES = {'pure_pursuit': 'Pure Pursuit', 'lya': 'LYA'}
OWNERS = {'edge_recovery': '贴边回正', 'contingency': '备用动作', 'reverse': '倒车',
          'stop': '停车', 'unavailable': '无可用选点'}


def stamp_key(stamp):
    if not isinstance(stamp, dict):
        return None
    secs, nanos = stamp.get('sec'), stamp.get('nanosec')
    if type(secs) is int and type(nanos) is int and 0 <= nanos < 10**9:
        return secs, nanos
    return None


def used_points(tracking):
    mode = tracking.get('mode')
    if mode not in MODES or tracking.get('active') is not True or tracking.get('owner') != 'candidate':
        return ()
    points = tracking.get('points_zx_m')
    labels = ('P',) if mode == 'pure_pursuit' else ('A', 'B', 'C')
    if not isinstance(points, (tuple, list)) or len(points) != len(labels):
        return ()
    target = tracking.get('target_zx_m')
    if target != points[0]:
        return ()
    result = []
    for label, point in zip(labels, points):
        if not isinstance(point, (list, tuple)) or len(point) != 2 or not all(
                type(v) in (int, float) and math.isfinite(v) for v in point):
            return ()
        z, right = point
        uv = pixel(point) if z >= .55 and abs(right / z) <= 2. else None
        if uv is None or not (0 <= uv[0] < 640 and 0 <= uv[1] < 360):
            uv = None
        result.append((label, uv))
    return tuple(result)


class TrackingFrames:
    """DDS can deliver the camera and decision in either order. Keep only tracking inputs."""

    def __init__(self):
        self.frames = OrderedDict()
        self.session = None
        self.mode = None
        self.newest = None

    def put(self, decision):
        tracking = decision.get('tracking_controller') or {}
        mode = tracking.get('mode')
        session = decision.get('session')
        key = stamp_key(decision.get('stamp'))
        if session == self.session and self.newest is not None and key is not None and key < self.newest:
            return
        if session != self.session or mode != self.mode:
            self.frames.clear()
        self.session, self.mode = session, mode
        self.newest = key
        command = decision.get('command') or {}
        speed = command.get('linear_x_mps')
        yaw = command.get('angular_z_radps')
        checked_speed = tracking.get('checked_speed_mps')
        checked_yaw = tracking.get('checked_yaw_right_radps')
        if (not session or not key or decision.get('status') != 'ok' or
                not all(type(v) in (int, float) and math.isfinite(v)
                        for v in (speed, yaw, checked_speed, checked_yaw)) or
                speed <= 0. or decision.get('selected_index') is None or
                not math.isclose(speed, checked_speed, abs_tol=1e-7) or
                not math.isclose(yaw, -checked_yaw, abs_tol=1e-7)):
            self.frames.clear()
            return
        points = used_points(tracking)
        if not points:
            self.frames.clear()
            return
        self.frames[key] = points
        if len(self.frames) > 12:
            self.frames.popitem(last=False)
