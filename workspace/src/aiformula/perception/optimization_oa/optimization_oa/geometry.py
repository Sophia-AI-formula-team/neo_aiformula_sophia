from dataclasses import dataclass

import numpy as np


def rotation_matrix(quaternion):
    x, y, z, w = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm((x, y, z, w))
    if not np.isfinite(norm) or norm <= 1e-9:
        raise ValueError("invalid_camera_rotation")
    x, y, z, w = np.asarray((x, y, z, w)) / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


@dataclass
class GroundCamera:
    width: int
    height: int
    projection: np.ndarray
    origin: np.ndarray
    rotation: np.ndarray
    roi_top_ratio: float = 0.5

    def __post_init__(self):
        self.projection = np.asarray(self.projection, dtype=float).reshape(3, 4)
        self.origin = np.asarray(self.origin, dtype=float).reshape(3)
        self.rotation = np.asarray(self.rotation, dtype=float).reshape(3, 3)
        if (self.width <= 0 or self.height <= 0 or not 0 <= self.roi_top_ratio < 1
                or not np.all(np.isfinite(self.projection))
                or not np.all(np.isfinite(self.origin)) or self.origin[2] <= 0
                or not np.all(np.isfinite(self.rotation))
                or not np.allclose(self.projection[:, 3], 0)
                or abs(np.linalg.det(self.projection[:, :3])) < 1e-10):
            raise ValueError("invalid_rectified_left_camera_model")

    def ground_to_image(self, xy):
        xy = np.asarray(xy, dtype=float)
        points = np.concatenate((xy, np.zeros((*xy.shape[:-1], 1))), axis=-1)
        optical = (points - self.origin) @ self.rotation
        projected = optical @ self.projection[:, :3].T
        with np.errstate(divide="ignore", invalid="ignore"):
            return projected[..., :2] / projected[..., 2:3], optical[..., 2]

    def image_to_ground(self, uv):
        uv = np.asarray(uv, dtype=float)
        pixels = np.column_stack((uv, np.ones(len(uv))))
        rays = pixels @ np.linalg.inv(self.projection[:, :3]).T @ self.rotation.T
        with np.errstate(divide="ignore", invalid="ignore"):
            distance = -self.origin[2] / rays[:, 2]
        if np.any(~np.isfinite(distance) | (distance <= 0)):
            raise ValueError("detection_contact_has_no_ground_intersection")
        return self.origin[:2] + rays[:, :2] * distance[:, None]

    def image_row_calibration(self, rows):
        """Return forward station and lateral scales for the native camera rows."""
        ground_plane = np.column_stack((np.eye(3)[:, :2], -self.origin))
        homography = self.projection[:, :3] @ self.rotation.T @ ground_plane
        inverse = np.linalg.inv(homography)
        center = (self.width - 1) / 2
        pixels = np.column_stack((np.full(len(rows), center), rows, np.ones(len(rows))))
        homogeneous = pixels @ inverse.T
        q = homogeneous[:, 2]
        q_left = q - center * inverse[2, 0]
        q_right = q + center * inverse[2, 0]
        if np.any(np.minimum(q_left, q_right) <= 0):
            raise ValueError("planning_rows_cross_ground_horizon")
        derivative = (inverse[:2, 0] * q[:, None] - homogeneous[:, :2] * inverse[2, 0])
        center_scale = np.linalg.norm(derivative, axis=1) / q**2
        min_scale = np.linalg.norm(derivative, axis=1) / np.maximum(q_left, q_right)**2
        return homogeneous[:, 0] / q, center_scale, min_scale
