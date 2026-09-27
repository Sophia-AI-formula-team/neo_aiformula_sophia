"""ROS-independent geometry and causal fusion primitives.

Vendored from aiformula lane_mapping_runtime/core.py (runtime v5).
Shared VectorNav localization below is used by both lap packages.

The runtime node deliberately keeps these functions independent of ROS so the
causality, projection, and consensus behavior can be tested without hardware.
"""

from collections import OrderedDict, deque
from dataclasses import dataclass, replace
import math
from numbers import Integral
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np


WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def geodetic_to_ecef(latitude_deg: float, longitude_deg: float, altitude_m: float) -> np.ndarray:
    """Convert WGS84 latitude/longitude/altitude to ECEF metres."""
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_lat = math.sin(latitude)
    cos_lat = math.cos(latitude)
    sin_lon = math.sin(longitude)
    cos_lon = math.cos(longitude)
    prime_vertical = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return np.array(
        [
            (prime_vertical + altitude_m) * cos_lat * cos_lon,
            (prime_vertical + altitude_m) * cos_lat * sin_lon,
            (prime_vertical * (1.0 - WGS84_E2) + altitude_m) * sin_lat,
        ],
        dtype=np.float64,
    )


class LocalEnuFrame:
    """Local ENU frame fixed at the first causally received VectorNav LLA."""

    def __init__(self, latitude_deg: float, longitude_deg: float, altitude_m: float) -> None:
        self.latitude_deg = float(latitude_deg)
        self.longitude_deg = float(longitude_deg)
        self.altitude_m = float(altitude_m)
        self._origin_ecef = geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m)
        latitude = math.radians(latitude_deg)
        longitude = math.radians(longitude_deg)
        sin_lat = math.sin(latitude)
        cos_lat = math.cos(latitude)
        sin_lon = math.sin(longitude)
        cos_lon = math.cos(longitude)
        self._ecef_to_enu = np.array(
            [
                [-sin_lon, cos_lon, 0.0],
                [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
                [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
            ],
            dtype=np.float64,
        )

    def convert(self, latitude_deg: float, longitude_deg: float, altitude_m: float) -> np.ndarray:
        """Return east, north, up in metres relative to this frame."""
        delta = geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m) - self._origin_ecef
        return self._ecef_to_enu @ delta


def quaternion_xyzw_to_rotation(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    """Return a 3x3 rotation matrix for an x,y,z,w quaternion."""
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("Quaternion must be finite and non-zero")
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def make_transform(
    translation_xyz: Sequence[float], quaternion_xyzw: Sequence[float]
) -> np.ndarray:
    """Build a homogeneous parent-from-child transform."""
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_xyzw_to_rotation(quaternion_xyzw)
    transform[:3, 3] = np.asarray(translation_xyz, dtype=np.float64)
    if transform[:3, 3].shape != (3,) or not np.all(np.isfinite(transform)):
        raise ValueError("Transform must contain finite translation and quaternion values")
    return transform


@dataclass(frozen=True)
class ProjectionCounts:
    raw_pixels: int
    metric_pixels: int
    stable_pixels: int

    @property
    def unresolved_pixels(self) -> int:
        return self.raw_pixels - self.metric_pixels

    @property
    def unstable_pixels(self) -> int:
        return self.metric_pixels - self.stable_pixels


class GroundLookup:
    """Per-pixel ground intersections and four-neighbour sensitivity.

    This applies no forward, lateral, or radial spatial ROI. The only
    projection filter is metric validity followed by the fixed image-space
    uncertainty gate expressed in metres per one-pixel perturbation.
    """

    def __init__(
        self,
        width: int,
        height: int,
        camera_matrix: Sequence[float],
        base_from_camera: np.ndarray,
        max_sensitivity_m_per_px: float,
    ) -> None:
        if width <= 1 or height <= 1:
            raise ValueError("Image dimensions must both exceed one pixel")
        if max_sensitivity_m_per_px <= 0.0:
            raise ValueError("Projection sensitivity limit must be positive")
        camera = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        transform = np.asarray(base_from_camera, dtype=np.float64)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError("base_from_camera must be a finite 4x4 matrix")
        if not np.all(np.isfinite(camera)) or abs(np.linalg.det(camera)) < 1.0e-12:
            raise ValueError("Camera matrix must be finite and invertible")

        self.width = int(width)
        self.height = int(height)
        self.camera_matrix = camera.copy()
        self.base_from_camera = transform.copy()
        self.max_sensitivity_m_per_px = float(max_sensitivity_m_per_px)

        u_grid, v_grid = np.meshgrid(
            np.arange(self.width, dtype=np.float64),
            np.arange(self.height, dtype=np.float64),
        )
        homogeneous_pixels = np.stack(
            (u_grid.reshape(-1), v_grid.reshape(-1), np.ones(self.width * self.height)), axis=0
        )
        camera_rays = np.linalg.inv(camera) @ homogeneous_pixels
        rotation = transform[:3, :3]
        origin = transform[:3, 3]
        base_rays = rotation @ camera_rays
        ray_z = base_rays[2]

        metric_flat = np.isfinite(ray_z) & (ray_z < -1.0e-9) & (origin[2] > 0.0)
        scale = np.full(ray_z.shape, np.nan, dtype=np.float64)
        scale[metric_flat] = -origin[2] / ray_z[metric_flat]
        metric_flat &= np.isfinite(scale) & (scale > 0.0)

        points = np.full((3, self.width * self.height), np.nan, dtype=np.float64)
        points[:, metric_flat] = origin[:, None] + base_rays[:, metric_flat] * scale[metric_flat]
        points_xy = points[:2].T.reshape(self.height, self.width, 2)
        metric = metric_flat.reshape(self.height, self.width)

        sensitivity = np.zeros((self.height, self.width), dtype=np.float64)
        horizontal_pair = metric[:, :-1] & metric[:, 1:]
        horizontal_delta = np.linalg.norm(points_xy[:, 1:] - points_xy[:, :-1], axis=2)
        horizontal_delta = np.where(horizontal_pair, horizontal_delta, 0.0)
        sensitivity[:, :-1] = np.maximum(sensitivity[:, :-1], horizontal_delta)
        sensitivity[:, 1:] = np.maximum(sensitivity[:, 1:], horizontal_delta)

        vertical_pair = metric[:-1, :] & metric[1:, :]
        vertical_delta = np.linalg.norm(points_xy[1:, :] - points_xy[:-1, :], axis=2)
        vertical_delta = np.where(vertical_pair, vertical_delta, 0.0)
        sensitivity[:-1, :] = np.maximum(sensitivity[:-1, :], vertical_delta)
        sensitivity[1:, :] = np.maximum(sensitivity[1:, :], vertical_delta)

        self.points_xy = points_xy
        self.metric_valid = metric
        self.sensitivity_m_per_px = sensitivity
        self.stable = (
            metric
            & np.isfinite(sensitivity)
            & (sensitivity <= self.max_sensitivity_m_per_px)
        )

    def matches(
        self,
        width: int,
        height: int,
        camera_matrix: Sequence[float],
        base_from_camera: np.ndarray,
        max_sensitivity_m_per_px: float,
    ) -> bool:
        return (
            self.width == int(width)
            and self.height == int(height)
            and math.isclose(self.max_sensitivity_m_per_px, float(max_sensitivity_m_per_px))
            and np.allclose(
                self.camera_matrix,
                np.asarray(camera_matrix).reshape(3, 3),
                atol=1.0e-12,
            )
            and np.allclose(self.base_from_camera, np.asarray(base_from_camera), atol=1.0e-12)
        )

    def project_mask(
        self, mask: np.ndarray, threshold: int
    ) -> Tuple[np.ndarray, ProjectionCounts]:
        """Return stable local x-forward/y-left points from a mono mask."""
        points, _, counts = self.project_mask_with_sensitivity(mask, threshold)
        return points, counts

    def project_mask_with_sensitivity(
        self, mask: np.ndarray, threshold: int
    ) -> Tuple[np.ndarray, np.ndarray, ProjectionCounts]:
        """Return points and their aligned J4 values (not calibrated error bars)."""
        if mask.shape != (self.height, self.width):
            raise ValueError(
                "Mask shape {} does not match calibration {}x{}".format(
                    mask.shape, self.width, self.height
                )
            )
        active = np.asarray(mask) > int(threshold)
        raw_pixels = int(np.count_nonzero(active))
        metric_active = active & self.metric_valid
        stable_active = active & self.stable
        counts = ProjectionCounts(
            raw_pixels=raw_pixels,
            metric_pixels=int(np.count_nonzero(metric_active)),
            stable_pixels=int(np.count_nonzero(stable_active)),
        )
        return (
            self.points_xy[stable_active],
            self.sensitivity_m_per_px[stable_active],
            counts,
        )

    def debug_bgr(
        self, mask: np.ndarray, threshold: int, reliable_sensitivity: Optional[float] = None
    ) -> np.ndarray:
        """Colour reliable=green, candidate-only=cyan, unstable=red, unresolved=orange."""
        if mask.shape != (self.height, self.width):
            raise ValueError("Mask shape does not match lookup")
        active = np.asarray(mask) > int(threshold)
        output = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        output[active & ~self.metric_valid] = (0, 128, 255)
        output[active & self.metric_valid & ~self.stable] = (0, 0, 255)
        output[active & self.stable] = (0, 255, 0)
        if reliable_sensitivity is not None:
            output[active & self.stable & (
                self.sensitivity_m_per_px > reliable_sensitivity
            )] = (255, 210, 0)
        return output


def transform_local_points(
    local_xy: np.ndarray, east_m: float, north_m: float, yaw_enu_rad: float
) -> np.ndarray:
    """Transform vehicle x-forward/y-left points into the local ENU map."""
    points = np.asarray(local_xy, dtype=np.float64).reshape(-1, 2)
    cosine = math.cos(yaw_enu_rad)
    sine = math.sin(yaw_enu_rad)
    rotation = np.array([[cosine, -sine], [sine, cosine]], dtype=np.float64)
    return points @ rotation.T + np.array([east_m, north_m], dtype=np.float64)


def pack_cell_indices(ix: np.ndarray, iy: np.ndarray) -> np.ndarray:
    """Pack signed int32 x/y indices into stable uint64 keys."""
    x = np.asarray(ix, dtype=np.int64)
    y = np.asarray(iy, dtype=np.int64)
    if np.any(x < np.iinfo(np.int32).min) or np.any(x > np.iinfo(np.int32).max):
        raise OverflowError("x cell index exceeds signed int32")
    if np.any(y < np.iinfo(np.int32).min) or np.any(y > np.iinfo(np.int32).max):
        raise OverflowError("y cell index exceeds signed int32")
    x_bits = x.astype(np.int32).view(np.uint32).astype(np.uint64)
    y_bits = y.astype(np.int32).view(np.uint32).astype(np.uint64)
    return (x_bits << np.uint64(32)) | y_bits


def unpack_cell_keys(keys: Sequence[int]) -> Tuple[np.ndarray, np.ndarray]:
    """Unpack uint64 keys into signed int64 x/y cell indices."""
    packed = np.asarray(keys, dtype=np.uint64)
    x_bits = (packed >> np.uint64(32)).astype(np.uint32)
    y_bits = (packed & np.uint64(0xFFFFFFFF)).astype(np.uint32)
    return x_bits.view(np.int32).astype(np.int64), y_bits.view(np.int32).astype(np.int64)


class SparseConsensusMap:
    """Past-only consensus with optional low-projection-sensitivity anchors.

    Total and reliable votes are both counted once per input frame, not once
    per pixel. Reliable evidence must be a subset of that frame's observations.
    No old image is reprocessed when a candidate later gains reliable support.
    """

    def __init__(
        self, resolution_m: float, minimum_frame_votes: int,
        minimum_reliable_frame_votes: int = 0,
        max_candidate_cells: int = 0, max_confirmed_cells: int = 0,
        candidate_ttl_s: float = 0.0,
    ) -> None:
        if not math.isfinite(resolution_m) or resolution_m <= 0.0:
            raise ValueError("Map resolution must be positive")
        if not _is_integer(minimum_frame_votes) or minimum_frame_votes <= 0:
            raise ValueError("Consensus threshold must be positive")
        if not _is_integer(minimum_reliable_frame_votes) or not (
            0 <= minimum_reliable_frame_votes <= minimum_frame_votes
        ):
            raise ValueError("Reliable threshold must be between zero and total threshold")
        for limit in (max_candidate_cells, max_confirmed_cells):
            if not _is_integer(limit) or limit < 0:
                raise ValueError("Map capacity limits must be non-negative integers")
        if not math.isfinite(candidate_ttl_s) or candidate_ttl_s < 0:
            raise ValueError("Candidate TTL must be finite and non-negative")
        if candidate_ttl_s > 0 and (
            not math.isfinite(candidate_ttl_s * 1.0e9) or candidate_ttl_s * 1.0e9 < 1
        ):
            raise ValueError("Positive candidate TTL must fit nanosecond precision")
        self.resolution_m = float(resolution_m)
        self.minimum_frame_votes = int(minimum_frame_votes)
        self.minimum_reliable_frame_votes = int(minimum_reliable_frame_votes)
        self.max_candidate_cells = int(max_candidate_cells)
        self.max_confirmed_cells = int(max_confirmed_cells)
        self.candidate_ttl_s = float(candidate_ttl_s)
        self._candidate_ttl_ns = int(candidate_ttl_s * 1.0e9)
        self.last_stamp_ns = None
        self.capacity_rejected_cells = 0
        self.confirmation_capacity_rejections = 0
        self.expired_candidate_cells = 0
        # One entry per unconfirmed candidate, not one entry per observation.
        self._unconfirmed_last_seen = OrderedDict()
        self.candidate_votes = {}
        self.candidate_reliable_votes = {}
        self.confirmed_keys = set()
        self._ordered_keys = np.empty(0, dtype=np.uint64)
        self._confirmed_points = np.empty((0, 2), dtype=np.float64)
        self._confirmed_votes = np.empty(0, dtype=np.float32)
        self._pending_confirmed_keys = []
        self._dirty_confirmed_votes = {}

    def points_to_unique_keys(self, world_xy: np.ndarray) -> np.ndarray:
        points = np.asarray(world_xy, dtype=np.float64).reshape(-1, 2)
        if points.size == 0:
            return np.empty(0, dtype=np.uint64)
        indices = np.floor(points / self.resolution_m + 0.5).astype(np.int64)
        return np.unique(pack_cell_indices(indices[:, 0], indices[:, 1]))

    @staticmethod
    def _unique_keys(frame_keys: Iterable[int]) -> np.ndarray:
        if isinstance(frame_keys, np.ndarray):
            keys = np.asarray(frame_keys, dtype=np.uint64).reshape(-1)
        else:
            keys = np.asarray(list(frame_keys), dtype=np.uint64).reshape(-1)
        # points_to_unique_keys already returns sorted, unique uint64 values.
        # Keep accepting arbitrary order and duplicates from other callers.
        if keys.size > 1 and not np.all(keys[1:] > keys[:-1]):
            keys = np.unique(keys)
        return keys

    def update(
        self, unique_frame_keys: Iterable[int],
        reliable_frame_keys: Optional[Iterable[int]] = None,
        stamp_ns: Optional[int] = None,
    ) -> List[int]:
        """Fuse one frame; refuse overflow without deleting confirmed geometry.

        Capacity counters count rejected cell-observations, not distinct cells.
        With TTL enabled, stamps are required and must be nondecreasing. Expiry
        uses measurement time; callers must not advance it to processing time
        before subsequently submitting older in-flight observations.
        """
        keys = self._unique_keys(unique_frame_keys)
        reliable = (
            np.empty(0, dtype=np.uint64) if reliable_frame_keys is None
            else self._unique_keys(reliable_frame_keys)
        )
        # Validate before mutation, including for an empty ordinary observation.
        if reliable.size:
            positions = np.searchsorted(keys, reliable)
            if np.any(positions >= keys.size) or not np.array_equal(keys[positions], reliable):
                raise ValueError("Reliable keys must be a subset of the current frame keys")
        if stamp_ns is not None:
            self.expire_candidates(stamp_ns)
        elif self.candidate_ttl_s > 0:
            raise ValueError("A measurement timestamp is required when candidate TTL is enabled")
        reliable_set = set(reliable.tolist())
        newly_confirmed = []
        for raw_key in keys.tolist():
            key = int(raw_key)
            if key not in self.candidate_votes and (
                self.max_candidate_cells > 0
                and len(self.candidate_votes) >= self.max_candidate_cells
            ):
                self.capacity_rejected_cells += 1
                continue
            new_count = self.candidate_votes.get(key, 0) + 1
            self.candidate_votes[key] = new_count
            if key in reliable_set:
                self.candidate_reliable_votes[key] = (
                    self.candidate_reliable_votes.get(key, 0) + 1
                )
            if self.candidate_ttl_s > 0 and key not in self.confirmed_keys:
                self._unconfirmed_last_seen[key] = int(stamp_ns)
                self._unconfirmed_last_seen.move_to_end(key)
            if (
                new_count >= self.minimum_frame_votes
                and self.candidate_reliable_votes.get(key, 0) >= self.minimum_reliable_frame_votes
            ):
                if key not in self.confirmed_keys:
                    if self.max_confirmed_cells > 0 and (
                        len(self.confirmed_keys) >= self.max_confirmed_cells
                    ):
                        self.confirmation_capacity_rejections += 1
                        continue
                    self.confirmed_keys.add(key)
                    self._unconfirmed_last_seen.pop(key, None)
                    self._pending_confirmed_keys.append(key)
                    newly_confirmed.append(key)
                self._dirty_confirmed_votes[key] = new_count
        return newly_confirmed

    def expire_candidates(self, stamp_ns: int) -> int:
        """Expire only unconfirmed cells; never retract latched global geometry."""
        stamp_ns = _checked_stamp(stamp_ns, self.last_stamp_ns)
        self.last_stamp_ns = stamp_ns
        if self.candidate_ttl_s <= 0:
            return 0
        cutoff = stamp_ns - self._candidate_ttl_ns
        count = 0
        while self._unconfirmed_last_seen:
            key, last_seen = next(iter(self._unconfirmed_last_seen.items()))
            if last_seen > cutoff:
                break
            self._unconfirmed_last_seen.popitem(last=False)
            del self.candidate_votes[key]
            self.candidate_reliable_votes.pop(key, None)
            count += 1
        self.expired_candidate_cells += count
        return count

    def confirmed_points_and_votes(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return independent, uint64-key-ordered consensus point/vote snapshots."""
        if not self.confirmed_keys:
            return np.empty((0, 2), dtype=np.float64), np.empty(0, dtype=np.float32)
        if self._pending_confirmed_keys:
            new_keys = np.asarray(sorted(self._pending_confirmed_keys), dtype=np.uint64)
            # Merge only newly latched keys into the previous sorted snapshot.
            # Existing coordinates never move in world space or need unpacking.
            new_positions = np.searchsorted(self._ordered_keys, new_keys)
            new_positions += np.arange(new_keys.size)
            size = self._ordered_keys.size + new_keys.size
            previous_positions = np.ones(size, dtype=bool)
            previous_positions[new_positions] = False
            ordered_keys = np.empty(size, dtype=np.uint64)
            points = np.empty((size, 2), dtype=np.float64)
            votes = np.empty(size, dtype=np.float32)
            ordered_keys[previous_positions] = self._ordered_keys
            ordered_keys[new_positions] = new_keys
            points[previous_positions] = self._confirmed_points
            ix, iy = unpack_cell_keys(new_keys)
            points[new_positions] = np.column_stack((ix, iy)) * self.resolution_m
            votes[previous_positions] = self._confirmed_votes
            self._ordered_keys = ordered_keys
            self._confirmed_points = points
            self._confirmed_votes = votes
            self._pending_confirmed_keys.clear()
        if self._dirty_confirmed_votes:
            keys = np.fromiter(self._dirty_confirmed_votes, dtype=np.uint64)
            positions = np.searchsorted(self._ordered_keys, keys)
            self._confirmed_votes[positions] = np.fromiter(
                self._dirty_confirmed_votes.values(), dtype=np.float32
            )
            self._dirty_confirmed_votes.clear()
        # Callers may alter/save a returned snapshot without corrupting caches.
        return self._confirmed_points.copy(), self._confirmed_votes.copy()

    def clear(self) -> None:
        self.candidate_votes.clear()
        self.candidate_reliable_votes.clear()
        self.confirmed_keys.clear()
        self._ordered_keys = np.empty(0, dtype=np.uint64)
        self._confirmed_points = np.empty((0, 2), dtype=np.float64)
        self._confirmed_votes = np.empty(0, dtype=np.float32)
        self._pending_confirmed_keys.clear()
        self._dirty_confirmed_votes.clear()
        self._unconfirmed_last_seen.clear()
        self.last_stamp_ns = None
        self.capacity_rejected_cells = 0
        self.confirmation_capacity_rejections = 0
        self.expired_candidate_cells = 0


def _is_integer(value: int) -> bool:
    return isinstance(value, Integral) and not isinstance(value, (bool, np.bool_))


def _checked_stamp(stamp_ns: int, previous_ns: Optional[int]) -> int:
    if not _is_integer(stamp_ns) or stamp_ns <= 0:
        raise ValueError("Measurement timestamp must be a positive integer in nanoseconds")
    if previous_ns is not None and stamp_ns < previous_ns:
        raise ValueError("Measurement timestamp regressed; reset the map before replaying")
    return int(stamp_ns)


class RollingConsensusMap:
    """Bounded recent-measurement consensus, distinct from the latched global map.

    A cell loses support when its contributing frames leave the time/frame
    window. No spatial ROI is applied. At capacity, sorted new keys are refused
    while observations of existing cells remain usable. Storage is bounded by
    max_cells cell records plus max_frames arrays of at most max_cells keys.
    """

    def __init__(
        self, resolution_m: float = 0.1, minimum_frame_votes: int = 2,
        minimum_reliable_frame_votes: int = 1, window_s: float = 0.75,
        max_cells: int = 20000, max_frames: int = 32,
    ) -> None:
        if not math.isfinite(resolution_m) or resolution_m <= 0:
            raise ValueError("Map resolution must be finite and positive")
        if not _is_integer(minimum_frame_votes) or minimum_frame_votes <= 0:
            raise ValueError("Consensus threshold must be a positive integer")
        if not _is_integer(minimum_reliable_frame_votes) or not (
            0 <= minimum_reliable_frame_votes <= minimum_frame_votes
        ):
            raise ValueError("Reliable threshold must be between zero and total threshold")
        if not math.isfinite(window_s) or window_s <= 0:
            raise ValueError("Rolling window must be finite and positive")
        if not math.isfinite(window_s * 1.0e9) or window_s * 1.0e9 < 1:
            raise ValueError("Rolling window must fit nanosecond precision")
        for limit in (max_cells, max_frames):
            if not _is_integer(limit) or limit <= 0:
                raise ValueError("Rolling capacity limits must be positive integers")
        self.resolution_m = float(resolution_m)
        self.minimum_frame_votes = int(minimum_frame_votes)
        self.minimum_reliable_frame_votes = int(minimum_reliable_frame_votes)
        self.window_s = float(window_s)
        self.max_cells = int(max_cells)
        self.max_frames = int(max_frames)
        self._window_ns = int(window_s * 1.0e9)
        self.candidate_votes = {}
        self.candidate_reliable_votes = {}
        self.confirmed_keys = set()
        self._frames = deque()
        self.last_stamp_ns = None
        self.capacity_rejected_cells = 0
        self.frame_capacity_evictions = 0
        self.expired_frames = 0
        self.expired_cells = 0

    @property
    def retained_frame_count(self) -> int:
        return len(self._frames)

    def _remove_oldest_frame(self) -> None:
        _, keys, reliable = self._frames.popleft()
        for raw_key in reliable:
            key = int(raw_key)
            count = self.candidate_reliable_votes[key] - 1
            if count:
                self.candidate_reliable_votes[key] = count
            else:
                del self.candidate_reliable_votes[key]
        for raw_key in keys:
            key = int(raw_key)
            count = self.candidate_votes[key] - 1
            if count:
                self.candidate_votes[key] = count
            else:
                del self.candidate_votes[key]
                self.expired_cells += 1
            if count < self.minimum_frame_votes or (
                self.candidate_reliable_votes.get(key, 0) < self.minimum_reliable_frame_votes
            ):
                self.confirmed_keys.discard(key)

    def expire(self, stamp_ns: int) -> int:
        """Remove contributions at/before measurement-time window start."""
        stamp_ns = _checked_stamp(stamp_ns, self.last_stamp_ns)
        self.last_stamp_ns = stamp_ns
        cutoff = stamp_ns - self._window_ns
        expired = 0
        while self._frames and self._frames[0][0] <= cutoff:
            self._remove_oldest_frame()
            expired += 1
        self.expired_frames += expired
        return expired

    def update(
        self, unique_frame_keys: Iterable[int],
        reliable_frame_keys: Optional[Iterable[int]] = None,
        stamp_ns: Optional[int] = None,
    ) -> List[int]:
        """Add one frame after time/frame eviction; return newly confirmed keys."""
        keys = SparseConsensusMap._unique_keys(unique_frame_keys)
        reliable = (
            np.empty(0, dtype=np.uint64) if reliable_frame_keys is None
            else SparseConsensusMap._unique_keys(reliable_frame_keys)
        )
        if reliable.size:
            positions = np.searchsorted(keys, reliable)
            if np.any(positions >= keys.size) or not np.array_equal(keys[positions], reliable):
                raise ValueError("Reliable keys must be a subset of the current frame keys")
        self.expire(stamp_ns)
        while len(self._frames) >= self.max_frames:
            self._remove_oldest_frame()
            self.frame_capacity_evictions += 1
        reliable_set = set(reliable.tolist())
        accepted = []
        accepted_reliable = []
        newly_confirmed = []
        for raw_key in keys:
            key = int(raw_key)
            if key not in self.candidate_votes and len(self.candidate_votes) >= self.max_cells:
                self.capacity_rejected_cells += 1
                continue
            accepted.append(key)
            count = self.candidate_votes.get(key, 0) + 1
            self.candidate_votes[key] = count
            if key in reliable_set:
                accepted_reliable.append(key)
                self.candidate_reliable_votes[key] = (
                    self.candidate_reliable_votes.get(key, 0) + 1
                )
            if count >= self.minimum_frame_votes and (
                self.candidate_reliable_votes.get(key, 0) >= self.minimum_reliable_frame_votes
            ) and key not in self.confirmed_keys:
                self.confirmed_keys.add(key)
                newly_confirmed.append(key)
        self._frames.append((
            int(stamp_ns), np.asarray(accepted, dtype=np.uint64),
            np.asarray(accepted_reliable, dtype=np.uint64),
        ))
        return newly_confirmed

    def confirmed_points_and_votes(self) -> Tuple[np.ndarray, np.ndarray]:
        keys = np.asarray(sorted(self.confirmed_keys), dtype=np.uint64)
        ix, iy = unpack_cell_keys(keys)
        points = np.column_stack((ix, iy)).astype(np.float64) * self.resolution_m
        votes = np.asarray([self.candidate_votes[int(key)] for key in keys], dtype=np.float32)
        return points, votes

    def clear(self) -> None:
        self.candidate_votes.clear()
        self.candidate_reliable_votes.clear()
        self.confirmed_keys.clear()
        self._frames.clear()
        self.last_stamp_ns = None
        self.capacity_rejected_cells = 0
        self.frame_capacity_evictions = 0
        self.expired_frames = 0
        self.expired_cells = 0


@dataclass(frozen=True)
class VectorNavSample:
    stamp_ns: int
    arrival_seq: int
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    east_m: float
    north_m: float
    up_m: float
    velocity_north_mps: float
    velocity_east_mps: float
    speed_mps: float
    heading_rad: Optional[float]
    ins_mode: int
    gps_fix: bool


@dataclass(frozen=True)
class MotionGateResult:
    accepted: bool
    reason: str
    displacement_m: float
    dt_s: float
    implied_speed_mps: float
    allowed_displacement_m: float


def check_motion_step(
    previous: Optional[VectorNavSample],
    current: VectorNavSample,
    position_tolerance_m: float,
    maximum_implied_speed_mps: float,
) -> MotionGateResult:
    """Check current LLA-derived motion using only the previous accepted sample."""
    if previous is None:
        return MotionGateResult(
            accepted=True,
            reason="",
            displacement_m=0.0,
            dt_s=0.0,
            implied_speed_mps=0.0,
            allowed_displacement_m=float(position_tolerance_m),
        )
    dt_s = (current.stamp_ns - previous.stamp_ns) / 1.0e9
    displacement_m = math.hypot(
        current.east_m - previous.east_m,
        current.north_m - previous.north_m,
    )
    if dt_s < 0.0:
        return MotionGateResult(
            False,
            "vectornav_time_regression",
            displacement_m,
            dt_s,
            math.inf,
            float(position_tolerance_m),
        )
    if dt_s == 0.0:
        accepted = displacement_m <= 1.0e-6
        return MotionGateResult(
            accepted,
            "" if accepted else "vectornav_duplicate_stamp_jump",
            displacement_m,
            0.0,
            0.0 if accepted else math.inf,
            1.0e-6,
        )
    reported_speed = max(previous.speed_mps, current.speed_mps)
    speed_limit = min(float(maximum_implied_speed_mps), reported_speed)
    allowed_displacement = float(position_tolerance_m) + speed_limit * dt_s
    implied_speed = displacement_m / dt_s
    accepted = displacement_m <= allowed_displacement
    return MotionGateResult(
        accepted,
        "" if accepted else "vectornav_position_jump",
        displacement_m,
        dt_s,
        implied_speed,
        allowed_displacement,
    )


class CausalSampleBuffer:
    """Arrival-gated, latest-header-not-after VectorNav buffer."""

    def __init__(self, maximum_size: int) -> None:
        if maximum_size <= 0:
            raise ValueError("Buffer size must be positive")
        self.maximum_size = int(maximum_size)
        self.samples: List[VectorNavSample] = []

    def append(self, sample: VectorNavSample) -> None:
        self.samples.append(sample)
        overflow = len(self.samples) - self.maximum_size
        if overflow > 0:
            del self.samples[:overflow]

    def latest_not_after(
        self,
        mask_stamp_ns: int,
        mask_arrival_seq: int,
        maximum_age_ns: int,
        fallback_heading_rad: Optional[float] = None,
    ) -> Tuple[Optional[VectorNavSample], Optional[int], str]:
        eligible = [
            sample
            for sample in self.samples
            if sample.arrival_seq < mask_arrival_seq and sample.stamp_ns <= mask_stamp_ns
        ]
        if not eligible:
            return None, None, "no_prior_vectornav"
        selected = max(eligible, key=lambda sample: (sample.stamp_ns, sample.arrival_seq))
        age_ns = int(mask_stamp_ns - selected.stamp_ns)
        if age_ns > int(maximum_age_ns):
            return None, age_ns, "stale_vectornav"
        heading_sources = [
            sample
            for sample in eligible
            if sample.heading_rad is not None and sample.stamp_ns <= selected.stamp_ns
        ]
        if heading_sources:
            heading_source = max(
                heading_sources, key=lambda sample: (sample.stamp_ns, sample.arrival_seq)
            )
            heading = heading_source.heading_rad
        else:
            heading = fallback_heading_rad
        if heading is None:
            return None, age_ns, "waiting_causal_heading"
        return replace(selected, heading_rad=heading), age_ns, "used"

    def clear(self) -> None:
        self.samples.clear()


class CausalCourseHeading:
    """NED-velocity course heading using current and past samples only."""

    def __init__(self, minimum_speed_mps: float) -> None:
        if minimum_speed_mps < 0.0:
            raise ValueError("Minimum heading speed cannot be negative")
        self.minimum_speed_mps = float(minimum_speed_mps)
        self.heading_rad: Optional[float] = None

    def update(
        self, velocity_north_mps: float, velocity_east_mps: float
    ) -> Tuple[Optional[float], float]:
        speed = math.hypot(velocity_north_mps, velocity_east_mps)
        if not math.isfinite(speed):
            raise ValueError("Velocity must be finite")
        if speed >= self.minimum_speed_mps:
            self.heading_rad = math.atan2(velocity_north_mps, velocity_east_mps)
        return self.heading_rad, speed

    def clear(self) -> None:
        self.heading_rad = None


class VectorNavLocalizer:
    """Strict CommonGroup -> fixed ENU pose, independent of camera callbacks.

    VectorNav yawpitchroll.x is clockwise degrees from NED north. The optional
    yaw offset is the calibrated sensor-to-vehicle planar mounting correction.
    This is not map relocalization: GNSS/INS repeatability remains a field gate.
    """

    def __init__(self, origin_lla=None, yaw_offset_rad=0.0,
                 maximum_age_ms=250.0, future_tolerance_ms=20.0,
                 position_tolerance_m=0.35, maximum_speed_mps=8.0,
                 heading_tolerance_rad=0.05, maximum_yaw_rate_radps=3.0):
        values = [yaw_offset_rad, maximum_age_ms, future_tolerance_ms,
                  position_tolerance_m, maximum_speed_mps,
                  heading_tolerance_rad, maximum_yaw_rate_radps]
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("Localizer parameters must be finite")
        if maximum_age_ms <= 0 or future_tolerance_ms < 0:
            raise ValueError("Invalid localization age limits")
        if position_tolerance_m < 0 or maximum_speed_mps <= 0:
            raise ValueError("Invalid localization motion limits")
        if heading_tolerance_rad < 0 or maximum_yaw_rate_radps <= 0:
            raise ValueError("Invalid localization heading limits")
        self.yaw_offset_rad = float(yaw_offset_rad)
        self.maximum_age_ns = int(maximum_age_ms * 1e6)
        self.future_tolerance_ns = int(future_tolerance_ms * 1e6)
        self.position_tolerance_m = float(position_tolerance_m)
        self.maximum_speed_mps = float(maximum_speed_mps)
        self.heading_tolerance_rad = float(heading_tolerance_rad)
        self.maximum_yaw_rate_radps = float(maximum_yaw_rate_radps)
        self.anchor = None
        if origin_lla is not None:
            origin = [float(value) for value in origin_lla]
            self._validate_lla(origin)
            self.anchor = LocalEnuFrame(*origin)
        self.last_sample = None

    @staticmethod
    def _validate_lla(values):
        if (len(values) != 3 or not all(math.isfinite(value) for value in values)
                or not -90.0 <= values[0] <= 90.0
                or not -180.0 <= values[1] <= 180.0):
            raise ValueError("invalid_vectornav_lla")

    @property
    def origin_lla(self):
        if self.anchor is None:
            return None
        return [self.anchor.latitude_deg, self.anchor.longitude_deg,
                self.anchor.altitude_m]

    def accept(self, message, now_ns, arrival_seq=0):
        stamp_ns = int(message.header.stamp.sec) * 1000000000 + int(
            message.header.stamp.nanosec)
        if now_ns <= 0 or stamp_ns <= 0:
            raise ValueError("invalid_vectornav_clock")
        age_ns = int(now_ns) - stamp_ns
        if age_ns < -self.future_tolerance_ns or age_ns > self.maximum_age_ns:
            raise ValueError("stale_or_future_vectornav")
        if self.last_sample is not None and stamp_ns <= self.last_sample.stamp_ns:
            raise ValueError("vectornav_nonincreasing_stamp")
        required_fields = 0x0008 | 0x0040 | 0x0080 | 0x1000
        if int(message.group_fields) & required_fields != required_fields:
            raise ValueError("missing_vectornav_yaw_position_velocity_status")
        status = message.insstatus
        if (int(status.mode) != 2 or not bool(status.gps_fix)
                or bool(status.time_error) or bool(status.imu_error)
                or bool(status.gps_error)):
            raise ValueError("unhealthy_vectornav_ins")
        lla = [float(message.position.x), float(message.position.y),
               float(message.position.z)]
        self._validate_lla(lla)
        north, east = float(message.velocity.x), float(message.velocity.y)
        yaw_deg = float(message.yawpitchroll.x)
        if not all(math.isfinite(value) for value in [north, east, yaw_deg]):
            raise ValueError("nonfinite_vectornav_attitude_or_velocity")
        speed = math.hypot(north, east)
        if speed > self.maximum_speed_mps:
            raise ValueError("vectornav_speed_above_limit")
        heading = math.atan2(
            math.sin(math.pi / 2.0 - math.radians(yaw_deg) + self.yaw_offset_rad),
            math.cos(math.pi / 2.0 - math.radians(yaw_deg) + self.yaw_offset_rad))
        anchor = self.anchor if self.anchor is not None else LocalEnuFrame(*lla)
        xyz = anchor.convert(*lla)
        sample = VectorNavSample(
            stamp_ns, int(arrival_seq), *lla, *[float(value) for value in xyz],
            north, east, speed, heading, int(status.mode), bool(status.gps_fix))
        gate = check_motion_step(self.last_sample, sample,
                                 self.position_tolerance_m, self.maximum_speed_mps)
        if not gate.accepted:
            raise ValueError(gate.reason)
        if self.last_sample is not None:
            difference = math.atan2(math.sin(heading - self.last_sample.heading_rad),
                                    math.cos(heading - self.last_sample.heading_rad))
            dt_s = (stamp_ns - self.last_sample.stamp_ns) / 1e9
            if abs(difference) > self.heading_tolerance_rad + self.maximum_yaw_rate_radps * dt_s:
                raise ValueError("vectornav_heading_jump")
        self.anchor = anchor
        self.last_sample = sample
        return sample
