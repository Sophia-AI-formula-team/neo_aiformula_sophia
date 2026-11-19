"""Recognition can run with any rectified 16:9 image. Metric driving needs a measured view."""
import json
from pathlib import Path

import numpy as np
import cv2

UNVERIFIED = 'unverified-resize'
CALIBRATED = 'calibrated-v626'
UNITY = 'frozen-unity-v626'
HONDA = 'honda-v1.4.0-ground'
PROFILES = (UNVERIFIED, CALIBRATED, UNITY, HONDA)
FX, FY, CX, CY, HEIGHT = 254.391622, 254.391622, 330.01302, 181.14964, .655
REFERENCE_K = np.array([[FX, 0., CX], [0., FY, CY], [0., 0., 1.]])


def measured_points(rows, width, height):
    marks = np.asarray(rows, dtype=float)
    if marks.ndim != 2 or marks.shape[1] != 4 or len(marks) < 4 or not np.isfinite(marks).all():
        raise ValueError('Need measured ground samples [image_u,image_v,right_m,forward_m]')
    u, v, right, forward = marks.T
    if (np.any(u < 0) or np.any(u >= width) or np.any(v < 0) or np.any(v >= height)
            or np.any(forward <= 0) or forward.min() > 3.0 or forward.max() < 7.0
            or right.min() > -.5 or right.max() < .5):
        raise ValueError('Ground check must cover nearby, distant and both lateral sides')
    return marks


def optical_height(value):
    if type(value) not in (float, int) or not np.isfinite(value) or value <= 0:
        raise ValueError('optical_height_m must be a finite, positive height in metres')
    return float(value)


def fit_rotation(source_k, marks, optical_height_m=HEIGHT):
    if source_k[0, 0] <= 0 or source_k[1, 1] <= 0 or np.linalg.matrix_rank(source_k) != 3:
        raise ValueError('Invalid rectified camera intrinsics')
    rotation = np.eye(3)
    if marks is not None:
        # Estimate orientation against ground rays from the physical camera.
        u, v, right, forward = marks.T
        sensor = np.linalg.solve(source_k, np.array([u, v, np.ones_like(u)])).T
        target = np.column_stack((right, np.full_like(right, optical_height_m), forward))
        sensor /= np.linalg.norm(sensor, axis=1)[:, None]
        target /= np.linalg.norm(target, axis=1)[:, None]
        a, s, b = np.linalg.svd(target.T @ sensor)
        if s[1] <= 1e-7:
            raise ValueError('Degenerate calibration marks')
        rotation = a @ np.diag([1., 1., np.linalg.det(a @ b)]) @ b
    # A level ground ray (right, measured height, forward) maps to the
    # same footprint in the frozen camera at its reference height.
    ground_scale = np.diag([1., HEIGHT / optical_height_m, 1.])
    return REFERENCE_K @ ground_scale @ rotation @ np.linalg.inv(source_k)


class CameraProjection:
    """Bind image projection and its visibility mask to the source CameraInfo."""
    def __init__(self, data):
        self.topic, self.frame_id = data['image_topic'], data['frame_id']
        self.width, self.height = int(data['width']), int(data['height'])
        self.projection = np.asarray(data['projection'], dtype=float).reshape(3, 4)
        if (not self.frame_id or self.width <= 0 or self.height <= 0 or self.width * 9 != self.height * 16
                or not np.isfinite(self.projection).all() or not data['mount_id']):
            raise ValueError('Incomplete rectified camera registration')

    def rectify(self, bgr):
        if self.warp is None:
            return bgr
        return cv2.warpPerspective(bgr, self.warp, (640, 360), flags=cv2.INTER_LINEAR)

    def restrict_predictions(self, road, lane, detections):
        if self.visible is None:
            return road, lane, detections
        boxes = detections['boxes']
        u = np.clip(((boxes[:, 0] + boxes[:, 2]) * .5).astype(int), 0, 639)
        v = np.clip(np.ceil(boxes[:, 3]).astype(int) - 1, 0, 359)
        keep = self.visible[v, u]
        return road & self.visible, lane & self.visible, {k: values[keep] for k, values in detections.items()}

    def matches(self, topic, image, info):
        if info is None:
            return False
        return (topic == self.topic and image.header.frame_id == self.frame_id == info.header.frame_id
                and (image.width, image.height) == (info.width, info.height) == (self.width, self.height)
                and np.allclose(np.array(info.p).reshape(3, 4), self.projection, rtol=0., atol=.001))


class HondaMountProjection(CameraProjection):
    profile = HONDA

    def __init__(self, data):
        super().__init__(data)
        if data['profile'] != HONDA:
            raise ValueError('Unknown nominal mount')
        h = optical_height(data['optical_height_m'])
        # Honda v1.4.0 left optical axis in forward/left/up body coordinates.
        # The xacro's optical RPY (-89, 0, -90) angles the camera up by 1 degree.
        angle = np.deg2rad(1.)
        c, s = np.cos(angle), np.sin(angle)
        optical_to_body = np.array([[0., s, c], [-1., 0., 0.], [0., -c, s]])
        # Each column multiplies [right_m, forward_m, 1] on level ground.
        relative_ground = np.array([[0., 1., -.055], [-1., 0., -.0600025], [0., 0., -h]])
        target = REFERENCE_K @ np.array([[1., 0., 0.], [0., 0., HEIGHT], [0., 1., 0.]])
        self.ground_to_virtual = target @ np.linalg.inv(optical_to_body.T @ relative_ground)
        self.update_intrinsics(self.projection)

    def update_intrinsics(self, projection):
        self.warp = self.ground_to_virtual @ np.linalg.inv(projection[:, :3])
        coverage = cv2.warpPerspective(np.ones((self.height, self.width), np.uint8),
                                       self.warp, (640, 360), flags=cv2.INTER_NEAREST)
        self.visible = coverage.astype(bool)
        self.projection = projection.copy()

    def matches(self, topic, image, info):
        if (info is None or topic != self.topic or image.header.frame_id != self.frame_id
                or info.header.frame_id != self.frame_id or (image.width, image.height) != (self.width, self.height)
                or (info.width, info.height) != (self.width, self.height)):
            return False
        projection = np.asarray(info.p, dtype=float).reshape(3, 4)
        if not np.array_equal(projection, self.projection):
            if (not np.isfinite(projection).all() or projection[0, 0] <= 0 or projection[1, 1] <= 0
                    or not np.allclose(projection[2], [0., 0., 1., 0.]) or np.any(projection[:, 3])):
                return False
            self.update_intrinsics(projection)
        return True


def load_projection(path):
    data = json.loads(Path(path).expanduser().read_text(encoding='utf8'))
    if data.get('profile') == HONDA:
        return HondaMountProjection(data)
    return CalibrationReceipt(path)


class CalibrationReceipt(CameraProjection):
    """Verify held-out ground marks before assigning the measured driving profile."""
    profile = CALIBRATED

    def __init__(self, path):
        data = json.loads(Path(path).expanduser().read_text(encoding='utf8'))
        super().__init__(data)
        # A fixed ray-to-ground transform is what matters to the existing planner.
        # Use actual camera pixels and independent vehicle-relative measured positions.
        marks = measured_points(data['ground_marks'], self.width, self.height)
        u, v, right, forward = marks.T
        self.warp = None
        self.visible = None
        self.optical_height_m = optical_height(data.get('optical_height_m', HEIGHT))
        fit = None
        if 'fit_marks' in data:
            fit = measured_points(data['fit_marks'], self.width, self.height)
            separation = np.linalg.norm(fit[:,None,2:] - marks[None,:,2:], axis=-1)
            if np.min(separation) < .02:
                raise ValueError('Use different physical marks for fitting and independent verification')
        if fit is not None or 'optical_height_m' in data:
            self.warp = fit_rotation(self.projection[:, :3], fit, self.optical_height_m)
            coverage = cv2.warpPerspective(np.ones((self.height, self.width), np.uint8),
                                           self.warp, (640, 360), flags=cv2.INTER_NEAREST)
            self.visible = coverage.astype(bool)
            check = self.warp @ np.array([u, v, np.ones_like(u)])
            if np.any(check[2] <= 0):
                raise ValueError('Verification points fall behind the virtual camera')
            u, v = check[:2] / check[2]
            if np.any((u < 0) | (u >= 640) | (v < 0) | (v >= 360)):
                raise ValueError('Verification points fall outside the frozen camera view')
        else:
            u = (u + .5) * 640 / self.width - .5
            v = (v + .5) * 360 / self.height - .5
        if np.any(v <= CY):
            raise ValueError('Ground projection crosses the optical horizon')
        estimated_z = HEIGHT * FY / (v - CY)
        error = np.hypot((u - CX) * estimated_z / FX - right, estimated_z - forward)
        if np.max(error) > .10:
            raise ValueError('Measured camera projection differs from V6.26 by more than 0.10 m')
        self.max_error = float(np.max(error))
