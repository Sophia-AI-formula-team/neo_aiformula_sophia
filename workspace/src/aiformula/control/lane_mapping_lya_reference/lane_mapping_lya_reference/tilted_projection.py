"""Ground projection with a measured up direction, without changing calibration.

The plane is ``normal_body @ point_body == 0``: its offset is STILL the old
body-origin ground-plane assumption, not a measured suspension/road height.
The caller must supply one healthy, causal raw-IMU normal for the entire image
and use a compatible horizontal-heading frame for motion. This geometry class
does not infer attitude, check clocks, consume INS/GNSS, or compensate road bank.
J4 is pixel sensitivity only; it is not an attitude-uncertainty estimate.
"""

import math
from numbers import Integral
from typing import Sequence, Tuple

import numpy as np

from .mapping_core import GroundLookup, ProjectionCounts


class TiltedGroundLookup:
    """Fixed K/T and static rays; per-call intersections for active pixels + J4.

    ``normal_body`` is a finite unit up vector, with positive body-z component.
    Only unit-length roundoff (1e-6, sufficient for float32 normals) is repaired;
    arbitrary non-unit vectors are errors. The horizontal x axis is the body's
    forward axis projected onto the plane; y is ``normal cross x``. A nearly
    vertical forward axis has no numerically defined heading and is rejected.

    EXACT [0, 0, 1] delegates to the original GroundLookup, including its metric
    threshold, four-neighbour convention, stable gate, counts and float bits.
    Any nonzero tilt uses the dynamic path; there is no near-level shortcut.
    No frame normal, projected cloud or attitude-dependent result is cached.
    """

    def __init__(
        self,
        width: int,
        height: int,
        camera_matrix: Sequence[float],
        base_from_camera: np.ndarray,
        max_sensitivity_m_per_px: float,
    ) -> None:
        if any(isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral)
               or value <= 1 for value in (width, height)):
            raise ValueError("Image dimensions must be integers exceeding one pixel")
        if not math.isfinite(float(max_sensitivity_m_per_px)):
            raise ValueError("Projection sensitivity limit must be finite")
        self._level = GroundLookup(
            width, height, camera_matrix, base_from_camera, max_sensitivity_m_per_px)

        # The same two matrix products as GroundLookup, performed only once.
        u, v = np.meshgrid(np.arange(width, dtype=np.float64),
                           np.arange(height, dtype=np.float64))
        pixels = np.stack((u.ravel(), v.ravel(), np.ones(width * height)), axis=0)
        camera_rays = np.linalg.inv(self._level.camera_matrix) @ pixels
        rays = self._level.base_from_camera[:3, :3] @ camera_rays
        if not np.all(np.isfinite(rays)):
            raise ValueError("Calibration produces nonfinite camera rays")
        # bytes-backed storage cannot be made writable via setflags. Calibration
        # input arrays and per-call normal/mask arrays are never retained.
        self._base_rays = np.frombuffer(rays.T.copy().tobytes(), dtype=np.float64).reshape(-1, 3)
        self._origin = np.frombuffer(
            self._level.base_from_camera[:3, 3].tobytes(), dtype=np.float64)
        self._level.camera_matrix = np.frombuffer(
            self._level.camera_matrix.tobytes(), dtype=np.float64).reshape(3, 3)
        self._level.base_from_camera = np.frombuffer(
            self._level.base_from_camera.tobytes(), dtype=np.float64).reshape(4, 4)

    @property
    def width(self):
        return self._level.width

    @property
    def height(self):
        return self._level.height

    @property
    def camera_matrix(self):
        return self._level.camera_matrix

    @property
    def base_from_camera(self):
        return self._level.base_from_camera

    @property
    def max_sensitivity_m_per_px(self):
        return self._level.max_sensitivity_m_per_px

    def matches(self, width, height, camera_matrix, base_from_camera,
                max_sensitivity_m_per_px):
        """Match static calibration only; a normal never changes the cache key."""
        return self._level.matches(
            width, height, camera_matrix, base_from_camera, max_sensitivity_m_per_px)

    @staticmethod
    def _normal_and_basis(normal_body):
        source = np.asarray(normal_body)
        if source.shape != (3,) or source.dtype.kind not in "fiu":
            raise ValueError("normal_body must be a real numeric three-vector")
        normal = np.array(source, dtype=np.float64, copy=True)
        if not np.all(np.isfinite(normal)):
            raise ValueError("normal_body must be finite")
        length = math.hypot(*normal)
        if not math.isfinite(length) or abs(length - 1.0) > 1.0e-6:
            raise ValueError("normal_body must be a unit vector")
        level = bool(np.array_equal(normal, np.array([0.0, 0.0, 1.0])))
        normal /= length
        if normal[2] <= 0.0:
            raise ValueError("normal_body must point into the positive body-z hemisphere")
        forward_length = math.hypot(normal[1], normal[2])
        if forward_length <= 1.0e-9:
            raise ValueError("Horizontal heading is undefined for vertical body forward")
        # Stable form of normalize(ex - nx*n), avoiding 1 - nx**2 cancellation.
        heading_x = np.array([
            forward_length, -normal[0] * normal[1] / forward_length,
            -normal[0] * normal[2] / forward_length])
        heading_y = np.array([0.0, normal[2] / forward_length,
                              -normal[1] / forward_length])
        return normal, np.stack((heading_x, heading_y)), level

    def _intersect_indices(self, indices, normal, heading_basis, camera_height):
        """Intersect only the requested unique pixels; no attitude cache."""
        rays = self._base_rays[indices]
        denominator = rays @ normal
        metric = np.isfinite(denominator) & (denominator < -1.0e-9)
        scale = np.full(len(indices), np.nan, dtype=np.float64)
        points_xy = np.full((len(indices), 2), np.nan, dtype=np.float64)
        with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
            scale[metric] = -camera_height / denominator[metric]
            metric &= np.isfinite(scale) & (scale > 0.0)
            points_body = self._origin + rays[metric] * scale[metric, None]
            points_xy[metric] = points_body @ heading_basis.T
        metric &= np.all(np.isfinite(points_xy), axis=1)
        return points_xy, metric

    def project_mask_with_sensitivity(
        self, mask: np.ndarray, threshold: int, normal_body: Sequence[float]
    ) -> Tuple[np.ndarray, np.ndarray, ProjectionCounts]:
        """Return C-ordered horizontal XY/J4 and original projection counts.

        Non-active neighbours still contribute to J4. Out-of-image or
        metric-invalid neighbours contribute zero, exactly as in GroundLookup.
        There is no spatial ROI. Temporary work is bounded by five pixel
        references per active pixel, and at most one intersection per pixel.
        Invalid normal/calibration geometry raises; it NEVER falls back to flat.
        """
        normal, heading_basis, level = self._normal_and_basis(normal_body)
        if level:
            return self._level.project_mask_with_sensitivity(mask, threshold)
        if mask.shape != (self.height, self.width):
            raise ValueError("Mask shape does not match lookup calibration")
        camera_height = float(normal @ self._origin)
        if not math.isfinite(camera_height) or camera_height <= 0.0:
            raise ValueError("Camera must be above the body-origin ground plane")
        active = np.flatnonzero(np.asarray(mask) > int(threshold))
        raw_pixels = len(active)
        if raw_pixels == 0:
            return (np.empty((0, 2), dtype=np.float64), np.empty(0, dtype=np.float64),
                    ProjectionCounts(0, 0, 0))

        rows, columns = np.divmod(active, self.width)
        neighbours = np.column_stack((active - 1, active + 1,
                                      active - self.width, active + self.width))
        in_image = np.column_stack((columns > 0, columns + 1 < self.width,
                                    rows > 0, rows + 1 < self.height))
        indices = np.unique(np.concatenate((active, neighbours[in_image])))
        points, metric = self._intersect_indices(indices, normal, heading_basis, camera_height)
        centre_slots = np.searchsorted(indices, active)
        neighbour_slots = np.zeros(neighbours.shape, dtype=np.intp)
        neighbour_slots[in_image] = np.searchsorted(indices, neighbours[in_image])
        valid_pairs = (in_image & metric[centre_slots, None] & metric[neighbour_slots])
        differences = np.zeros(neighbours.shape, dtype=np.float64)
        # Invalid neighbours are never subtracted; finite overflow yields an
        # infinite sensitivity and therefore a rejection, not an accepted zero.
        with np.errstate(over="ignore", invalid="ignore"):
            centre_points = np.broadcast_to(points[centre_slots, None, :],
                                            (raw_pixels, 4, 2))
            deltas = centre_points[valid_pairs] - points[neighbour_slots[valid_pairs]]
            differences[valid_pairs] = np.linalg.norm(deltas, axis=1)
        sensitivity = np.max(differences, axis=1)
        metric_active = metric[centre_slots]
        stable = (metric_active & np.isfinite(sensitivity)
                  & (sensitivity <= self.max_sensitivity_m_per_px))
        counts = ProjectionCounts(raw_pixels, int(np.count_nonzero(metric_active)),
                                  int(np.count_nonzero(stable)))
        return points[centre_slots[stable]], sensitivity[stable], counts

    def project_mask(self, mask, threshold, normal_body):
        """Convenience XY/counts projection; a normal remains mandatory."""
        points, _, counts = self.project_mask_with_sensitivity(mask, threshold, normal_body)
        return points, counts
