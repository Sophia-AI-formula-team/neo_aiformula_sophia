"""Bounded lane scan-to-FROZEN-map localization and separate repeat consensus.

Only NumPy is required.  A caller must pair a mask with an already arrived,
not-newer VectorNav prior and enforce mask age, vehicle state and deadlines.
Runtime callers use ``match(..., commit=False)`` and ``commit(result)`` only
after their final freshness check.  Calls must be serialized by one worker.

Parallel lane geometry observes lateral position and heading, NOT position
along the lane. That component is reanchored to the current RAW VectorNav prior,
not allowed to accumulate through a previous SE2 correction. A rolling window
confirms new refinement cells; confirmed history persists within a fixed cap.
Neither refinement layer participates in matching or changes the route.
"""

from collections.abc import Mapping
from dataclasses import dataclass
import math
from numbers import Integral
import time
from types import MappingProxyType

import numpy as np

from .mapping_core import RollingConsensusMap, pack_cell_indices, unpack_cell_keys

try:
    import _lane_mapping_lya_native as _GRAPH_NATIVE
except ImportError:
    # Source-only tools retain the exact Python implementation and deadlines.
    # ROS builds compile the extension; CI explicitly verifies it is loaded.
    _GRAPH_NATIVE = None

if _GRAPH_NATIVE is not None and (
        getattr(_GRAPH_NATIVE, "API_VERSION", None) != 1 or
        getattr(_GRAPH_NATIVE, "RADIUS_GRAPH_API_VERSION", None) != 1 or
        getattr(_GRAPH_NATIVE, "LOCAL_SUPPORT_API_VERSION", None) != 1 or
        getattr(_GRAPH_NATIVE, "NEAREST_API_VERSION", None) != 1 or
        getattr(_GRAPH_NATIVE, "CROSS_SECTION_WEIGHTS_API_VERSION", None) != 1 or
        getattr(_GRAPH_NATIVE, "LOCAL_MOMENTS_API_VERSION", None) != 1 or not all(
            callable(getattr(_GRAPH_NATIVE, name, None)) for name in
            ("radius_graph", "local_support", "prepare_graph", "graph_forest", "connected_subsets",
             "nearest_grouped", "cross_section_weights", "local_moments"))):
    raise RuntimeError("incompatible lane graph native extension; rebuild this package")


def graph_backend_info():
    """Report execution mode without making a real-time acceptance claim."""
    return ({"backend": "python", "api_version": None, "source_sha256": None}
            if _GRAPH_NATIVE is None else
            {"backend": "native", "api_version": _GRAPH_NATIVE.API_VERSION,
             "source_sha256": _GRAPH_NATIVE.SOURCE_SHA256})


DEFAULT_CONFIG = {
    "max_anchor_points": 100000,
    "max_input_scan_points": 200000,
    "max_scan_points": 600,
    "max_scan_support_points": 4000,
    "min_scan_points": 30,
    "min_inliers": 24,
    "anchor_voxel_m": 0.08,
    "scan_voxel_m": 0.10,
    "hash_cell_m": 0.50,
    "normal_radius_m": 0.45,
    "normal_max_radius_m": 1.35,
    "normal_min_points": 6,
    "normal_connectivity_m": 0.22,
    "normal_max_angle_rad": math.pi / 6.0,
    "normal_max_variance_ratio": 0.15,
    "normal_max_thickness_m": 0.30,
    "max_patch_points": 512,
    "max_scan_range_m": 30.0,
    "match_distance_m": 0.65,
    "inlier_distance_m": 0.15,
    "min_overlap_ratio": 0.55,
    "max_rmse_m": 0.10,
    "huber_delta_m": 0.08,
    "normal_diversity_ratio": 0.03,
    "allow_longitudinal_correction": False,
    "observable_eigen_ratio": 0.01,
    "rotation_scale_m": 4.0,
    "max_iterations": 6,
    "convergence_translation_m": 0.001,
    "convergence_yaw_rad": 0.0002,
    "initial_max_translation_m": 0.45,
    "initial_max_yaw_rad": 0.12,
    "tracking_max_translation_m": 0.12,
    "tracking_max_yaw_rad": 0.04,
    "max_total_translation_m": 1.0,
    "max_total_yaw_rad": 0.25,
    "runtime_budget_ms": 40.0,
    "refinement_resolution_m": 0.10,
    "refinement_min_frame_votes": 3,
    "refinement_window_s": 10.0,
    "refinement_max_cells": 30000,
    "refinement_max_frames": 256,
}


def _configuration(config):
    result = dict(DEFAULT_CONFIG)
    if config is not None:
        if not isinstance(config, Mapping):
            raise ValueError("matcher config must be a mapping")
        unknown = set(config) - set(result)
        if unknown:
            raise ValueError("unknown matcher settings: " + ", ".join(sorted(map(str, unknown))))
        result.update(config)
    integer_keys = {"max_anchor_points", "max_input_scan_points", "max_scan_points",
                    "min_scan_points", "min_inliers", "normal_min_points", "max_scan_support_points",
                    "max_patch_points", "max_iterations", "refinement_min_frame_votes",
                    "refinement_max_cells", "refinement_max_frames"}
    for key, value in result.items():
        if key == "allow_longitudinal_correction":
            if not isinstance(value, bool):
                raise ValueError("allow_longitudinal_correction must be boolean")
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float, np.number)):
            raise ValueError("matcher setting must be numeric: " + key)
        if not math.isfinite(float(value)) or value <= 0:
            raise ValueError("matcher setting must be finite and positive: " + key)
        if key in integer_keys:
            if int(value) != value:
                raise ValueError("matcher setting must be an integer: " + key)
            result[key] = int(value)
        else:
            result[key] = float(value)
    for key in ("normal_max_variance_ratio", "min_overlap_ratio",
                "normal_diversity_ratio", "observable_eigen_ratio"):
        if result[key] > 1:
            raise ValueError(key + " must not exceed one")
    if not 3 <= result["min_inliers"] <= result["min_scan_points"] <= result["max_scan_points"]:
        raise ValueError("need 3 <= min_inliers <= min_scan_points <= max_scan_points")
    if result["max_scan_points"] > result["max_input_scan_points"]:
        raise ValueError("max_scan_points exceeds the input limit")
    if not result["max_scan_points"] <= result["max_scan_support_points"] <= 4000:
        raise ValueError("scan normal-support capacity must be between scan and 4000 points")
    if (result["normal_connectivity_m"] > result["normal_radius_m"]
            or result["normal_max_angle_rad"] >= math.pi / 2):
        raise ValueError("normal connectivity or direction bound is invalid")
    if result["normal_min_points"] < 3 or result["max_patch_points"] < result["normal_min_points"]:
        raise ValueError("normal support cannot fit in the patch budget")
    if result["inlier_distance_m"] > result["match_distance_m"]:
        raise ValueError("inlier distance exceeds correspondence distance")
    if result["max_rmse_m"] > result["inlier_distance_m"]:
        raise ValueError("RMSE gate exceeds the inlier distance")
    if result["refinement_min_frame_votes"] > result["refinement_max_frames"]:
        raise ValueError("refinement frame capacity cannot reach its vote threshold")
    if result["max_iterations"] > 30 or result["max_scan_points"] > 4000:
        raise ValueError("runtime iteration/scan budget exceeds the supported bound")
    if result["max_anchor_points"] > 100000 or result["max_patch_points"] > 2048:
        raise ValueError("anchor or neighborhood capacity exceeds the supported bound")
    if result["hash_cell_m"] / result["anchor_voxel_m"] > 16:
        raise ValueError("anchor voxel permits an unbounded dense hash bucket")
    if result["normal_max_radius_m"] < result["normal_radius_m"]:
        raise ValueError("maximum normal radius is smaller than the initial radius")
    if result["refinement_resolution_m"] < 0.01:
        raise ValueError("refinement resolution must be at least one centimetre")
    # Prevent pathological parameter files from requesting millions of hash cells.
    if max(result["normal_max_radius_m"], result["match_distance_m"]) / result["hash_cell_m"] > 8:
        raise ValueError("spatial query radius exceeds the bounded hash neighborhood")
    return result


def _wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def _rotation(yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.array([[cosine, -sine], [sine, cosine]], dtype=np.float64)


def _readonly(value):
    result = np.array(value, dtype=np.float64, copy=True)
    result.flags.writeable = False
    return result


def _pose(value):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3,) or not np.all(np.isfinite(result)) or np.max(np.abs(result[:2])) > 1e7:
        raise ValueError("prior pose must contain finite bounded x, y, yaw")
    result = result.copy()
    result[2] = _wrap(result[2])
    return result


def _points(value, maximum):
    result = np.asarray(value, dtype=np.float64)
    if result.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 2:
        raise ValueError("points must have shape N x 2")
    if len(result) > maximum:
        raise ValueError("point capacity exceeded")
    if not np.all(np.isfinite(result)) or np.max(np.abs(result)) > 1e7:
        raise ValueError("points must be finite bounded coordinates")
    return result


def _unique_cells(keys):
    """Lexicographic integer XY grouping without structured-array sorting.

    The row order, inverse labels and counts match np.unique(axis=0). Keeping
    inverse labels preserves the original-order floating-point accumulation.
    """
    if not len(keys):
        return keys.copy(), np.empty(0, dtype=np.int64), np.empty(0, dtype=np.int64)
    order = np.lexsort((keys[:, 1], keys[:, 0]))
    ordered = keys[order]
    first = np.empty(len(keys), dtype=bool)
    first[0] = True
    first[1:] = np.any(ordered[1:] != ordered[:-1], axis=1)
    starts = np.flatnonzero(first)
    inverse = np.empty(len(keys), dtype=np.int64)
    inverse[order] = np.cumsum(first) - 1
    counts = np.diff(np.append(starts, len(keys)))
    return ordered[first], inverse, counts


def _voxel_means(points, resolution):
    if not len(points):
        return np.empty((0, 2), dtype=np.float64)
    keys = np.floor(points / resolution).astype(np.int64)
    _, inverse, counts = _unique_cells(keys)
    sums = np.zeros((len(counts), 2), dtype=np.float64)
    np.add.at(sums, inverse, points)
    return sums / counts[:, None]


def _line_weights(points, normals, support_index, support_ids, cfg):
    """Normalize reliable votes within a connected current paint cross-section.

    Connectivity must stay inside the tangent slab, not take a detour through
    a remote (or nearby) U bridge. Full support defines structure only: the
    numerator/denominator count ONLY the reliable scan indices. The slab is
    centered on their full-support seed; the reliable voxel mean can differ.
    """
    if not len(points):
        return np.empty(0)
    cloud = support_index.anchor
    if (_GRAPH_NATIVE is not None and support_index._native_graph is not None
            and len(cloud) <= 4000 and len(points) <= 4000
            and cloud.dtype == normals.dtype == np.float64
            and cloud.ndim == 2 and cloud.shape[1] == 2
            and normals.shape == (len(points), 2)
            and support_ids.dtype == np.int64 and support_ids.shape == (len(points),)
            and np.all(np.isfinite(cloud)) and np.all(np.isfinite(normals))
            and (not len(cloud) or np.max(np.abs(cloud)) <= 1e7)
            and np.max(np.abs(normals)) <= 2.0 and np.geterr()["under"] == "ignore"):
        # Preserve the original slab/radius arithmetic and duplicate reliable
        # IDs. Finite bounded coordinates/unit-like normals avoid overflow;
        # unusual numeric inputs or strict underflow keep NumPy's error policy.
        result = _GRAPH_NATIVE.cross_section_weights(
            support_index._native_graph, len(cloud), cloud.tobytes(),
            normals.tobytes(), support_ids.tobytes(), float(cfg["scan_voxel_m"] * 0.5),
            float(cfg["normal_max_radius_m"] ** 2))
        return np.frombuffer(result, dtype=np.float64).copy()
    reliable = np.zeros(len(cloud), dtype=bool)
    reliable[support_ids] = True
    unique_ids = int(np.sum(reliable)) == len(support_ids)
    reliable_bits = int.from_bytes(np.packbits(reliable, bitorder="little").tobytes(), "little")
    byte_count = (len(cloud) + 7) // 8
    adjacency = support_index._adjacency_bits
    if adjacency is None:
        raise ValueError("current scan cross-section exceeds bounded graph capacity")
    weights = np.empty(len(points))
    for start in range(0, len(points), 64):
        stop = min(start + 64, len(points))
        seeds = support_ids[start:stop]
        dx = cloud[None, :, 0] - cloud[seeds, None, 0]
        dy = cloud[None, :, 1] - cloud[seeds, None, 1]
        along = dx * -normals[start:stop, None, 1] + dy * normals[start:stop, None, 0]
        allowed = ((np.abs(along) <= cfg["scan_voxel_m"] * 0.5)
                   & (dx * dx + dy * dy <= cfg["normal_max_radius_m"] ** 2))
        packed = np.packbits(allowed, axis=1, bitorder="little")
        if _GRAPH_NATIVE is not None and support_index._native_graph is not None:
            reached = _GRAPH_NATIVE.connected_subsets(
                support_index._native_graph, len(cloud), packed.tobytes(), seeds.tolist())
            selected = np.unpackbits(np.frombuffer(reached, dtype=np.uint8).reshape(len(seeds), byte_count),
                                     axis=1, bitorder="little")[:, :len(cloud)]
            # Indexing by every reliable ID preserves duplicate multiplicity.
            weights[start:stop] = 1.0 / np.sum(selected[:, support_ids], axis=1)
            continue
        for row, seed in enumerate(seeds):
            mask = int.from_bytes(packed[row].tobytes(), "little")
            reached = frontier = 1 << int(seed)
            while frontier and reached != mask:
                bit = frontier & -frontier
                frontier ^= bit
                added = adjacency[bit.bit_length() - 1] & mask & ~reached
                reached |= added
                frontier |= added
            if unique_ids:
                # bin/count also works on ROS Foxy's Python 3.8 (no bit_count).
                count = bin(reached & reliable_bits).count("1")
            else:
                # Preserve multiplicity if rounding associates two reliable
                # voxel means with the same full-support seed.
                selected = np.unpackbits(np.frombuffer(reached.to_bytes(byte_count, "little"),
                                                      dtype=np.uint8), bitorder="little")
                count = int(np.sum(selected[support_ids]))
            weights[start + row] = 1.0 / count
    return weights


@dataclass(frozen=True)
class CommittedLocalizationState:
    """Small immutable same-session tracking handoff, never map evidence.

    Capturing this state does not traverse an anchor or build a spatial index.
    It is valid only while raw odometry and the map coordinate frame remain
    continuous; it is not a relocalization seed for a restarted session.
    """

    config: Mapping
    correction_se2: tuple
    last_accepted_stamp_ns: object
    last_seen_stamp_ns: object

    def __post_init__(self):
        config = MappingProxyType(_configuration(self.config))
        correction = np.asarray(self.correction_se2, dtype=np.float64)
        if correction.shape != (3,) or not np.all(np.isfinite(correction)):
            raise ValueError("tracking correction must contain three finite values")
        for name in ("last_accepted_stamp_ns", "last_seen_stamp_ns"):
            stamp = getattr(self, name)
            if stamp is not None:
                if isinstance(stamp, bool) or not isinstance(stamp, Integral) or stamp < 0:
                    raise ValueError("tracking timestamp must be a nonnegative integer or None")
                object.__setattr__(self, name, int(stamp))
        if self.last_accepted_stamp_ns is not None and (
                self.last_seen_stamp_ns is None
                or self.last_accepted_stamp_ns > self.last_seen_stamp_ns):
            raise ValueError("committed tracking timestamp cannot be newer than seen timestamp")
        object.__setattr__(self, "config", config)
        object.__setattr__(self, "correction_se2", tuple(float(value) for value in correction))


@dataclass(frozen=True)
class MatchResult:
    accepted: bool
    pose_xyyaw: np.ndarray
    correction_se2: np.ndarray
    reason: str
    stamp_ns: int
    metrics: dict


def _local_pca_moments(difference, support, counts):
    """Statistics of canonical 0/1 connected support and its exact row counts.

    Only the same native-f64 arithmetic takes the bounded kernel. NumPy keeps
    ownership of eigenvectors/gates and all other dtypes or strict underflow
    behavior. The sole production caller receives paired support/counts from
    _local_support; arbitrary inconsistent counts or noncanonical bool bytes
    are not this private helper's input contract. Native errors propagate.
    """
    if (_GRAPH_NATIVE is not None and difference.dtype == np.float64
            and difference.ndim == 3 and difference.shape[2] == 2
            and support.dtype == np.bool_ and support.shape == difference.shape[:2]
            and difference.shape[0] <= 4000 and difference.shape[1] <= 2048
            and np.geterr()["under"] == "ignore"
            and np.all(np.isfinite(difference)) and np.all(np.abs(difference) <= 2e7)):
        means, covariance = _GRAPH_NATIVE.local_moments(
            difference.tobytes(), support.tobytes(), difference.shape[0], difference.shape[1])
        # Bytes retain immutable ownership; pending concatenation below creates
        # its own storage. No writable view can corrupt these kernel results.
        return (np.frombuffer(means, dtype=np.float64).reshape(-1, 2),
                np.frombuffer(covariance, dtype=np.float64).reshape(-1, 2, 2))
    weights = support.astype(np.float64)
    safe_count = np.maximum(counts, 1.0)
    means = np.sum(difference * weights[:, :, None], axis=1) / safe_count[:, None]
    centered = difference - means[:, None, :]
    covariance = np.einsum("nki,nkj,nk->nij", centered, centered, weights) / safe_count[:, None, None]
    return means, covariance


class _FrozenIndex:
    """Spatial-hash batches: no full anchor-by-scan distance matrix per frame."""

    def __init__(self, anchor, cfg, normal_targets=None, normal_queries=None, direction_support=None,
                 reuse_index=None):
        self.anchor = anchor
        # Cache equality must compare construction-time settings, not a caller's mutable alias.
        self.cfg = MappingProxyType(dict(cfg))
        self.cell = cfg["hash_cell_m"]
        self.cells = {}
        keys = np.floor(anchor / self.cell).astype(np.int64)
        for index, key in enumerate(keys):
            self.cells.setdefault((int(key[0]), int(key[1])), []).append(index)
        self._cell_geometry = {key: anchor[indices].tobytes() for key, indices in self.cells.items()}
        self.components = self._components()
        self._component_sizes = np.bincount(self.components, minlength=len(anchor))
        self.normals = np.zeros_like(anchor)
        self.line_points = anchor.copy()
        self.normal_valid = np.zeros(len(anchor), dtype=bool)
        self.normal_radii = np.zeros(len(anchor), dtype=np.float64)
        self.normal_thickness = np.zeros(len(anchor), dtype=np.float64)
        self.normal_position_support = np.zeros(len(anchor), dtype=np.int64)
        self.direction_supported_points = 0
        self.direction_borrowed = np.zeros(len(anchor), dtype=bool)
        self.reused_geometry_points = 0
        self.patch_truncations = 0
        self._patch_cache = {}
        self.query_nearest = None
        if normal_queries is not None:
            found = self.nearest(normal_queries, valid_only=False)
            self.query_nearest = found
            normal_targets = np.unique(found[found >= 0])
        requested = np.ones(len(anchor), dtype=bool)
        if normal_targets is not None:
            requested[:] = False
            requested[np.asarray(normal_targets, dtype=np.int64)] = True
        cached = np.zeros(len(anchor), dtype=bool)
        if reuse_index is not None and reuse_index.cfg == cfg:
            for key, all_indices in self.cells.items():
                if self._cell_geometry[key] != reuse_index._cell_geometry.get(key):
                    continue
                current = np.asarray(all_indices, dtype=np.int64)
                previous = np.asarray(reuse_index.cells[key], dtype=np.int64)
                eligible = (~self.direction_borrowed[current] & ~reuse_index.direction_borrowed[previous]
                            & reuse_index.computed_normals[previous])
                dependencies = np.where(reuse_index.normal_valid[previous],
                                        reuse_index.normal_radii[previous], cfg["normal_max_radius_m"])
                for radius in np.unique(dependencies[eligible]):
                    reach = int(math.ceil(radius / self.cell))
                    # A previously valid smallest-scale fit depends only on
                    # that radius and all smaller attempted radii. Invalid
                    # geometry depends on the entire maximum neighborhood.
                    # Compare complete cells, including patch-truncation input.
                    same = all(self._cell_geometry.get((key[0] + dx, key[1] + dy))
                               == reuse_index._cell_geometry.get((key[0] + dx, key[1] + dy))
                               for dx in range(-reach, reach + 1) for dy in range(-reach, reach + 1))
                    if not same:
                        continue
                    subset = eligible & (dependencies == radius)
                    new, old = current[subset], previous[subset]
                    for name in ("normals", "line_points", "normal_valid", "normal_radii",
                                 "normal_thickness", "normal_position_support"):
                        getattr(self, name)[new] = getattr(reuse_index, name)[old]
                    cached[new] = True
                    self.reused_geometry_points += len(new)
        radii = [cfg["normal_radius_m"]]
        while radii[-1] < cfg["normal_max_radius_m"]:
            radii.append(min(radii[-1] * 2.0, cfg["normal_max_radius_m"]))
        # Choose the SMALLEST scale passing the unchanged line-support test.
        # Thin lane marks retain small-scale geometry. A rasterized/accumulated
        # wide band may require a larger neighborhood to reveal its long axis.
        # Isotropic patches and intersections are not rescued by relaxing ratio.
        for radius in radii:
            pending = []
            pending_count = 0

            def flush_pca():
                nonlocal pending_count
                if not pending:
                    return
                batch_indices = np.concatenate([part[0] for part in pending])
                batch_means = np.concatenate([part[1] for part in pending])
                batch_counts = np.concatenate([part[2] for part in pending])
                batch_covariance = np.concatenate([part[3] for part in pending])
                eigenvalues, eigenvectors = np.linalg.eigh(batch_covariance)
                valid = ((batch_counts >= cfg["normal_min_points"]) &
                         (eigenvalues[:, 1] > 1e-5) &
                         (eigenvalues[:, 0] <= cfg["normal_max_thickness_m"] ** 2) &
                         (eigenvalues[:, 0] <= cfg["normal_max_variance_ratio"] * eigenvalues[:, 1]))
                chosen = batch_indices[valid]
                normals = eigenvectors[valid, :, 0]
                self.normals[chosen] = normals
                self.normal_valid[chosen] = True
                self.normal_radii[chosen] = radius
                self.normal_thickness[chosen] = np.sqrt(np.maximum(eigenvalues[valid, 0], 0.0))
                self.normal_position_support[chosen] = batch_counts[valid].astype(np.int64)
                # Keep the original per-row arithmetic and smallest valid scale.
                offset = np.sum(batch_means[valid] * normals, axis=1)
                self.line_points[chosen] = anchor[chosen] + normals * offset[:, None]
                pending.clear()
                pending_count = 0

            for key, all_indices in self.cells.items():
                indices = np.asarray(all_indices, dtype=np.int64)
                indices = indices[requested[indices] & ~self.normal_valid[indices] & ~cached[indices]]
                if not len(indices):
                    continue
                candidate_indices = self._patch(key, radius)
                if len(candidate_indices) < cfg["normal_min_points"]:
                    continue
                targets = anchor[indices]
                candidates = anchor[candidate_indices]
                difference = candidates[None, :, :] - targets[:, None, :]
                support = ((np.sum(difference * difference, axis=2) <= radius * radius)
                           & (self.components[indices, None] == self.components[candidate_indices][None, :]))
                support, counts = self._local_support(indices, candidate_indices, support, return_counts=True)
                # These rows cannot pass at this radius. Keep them eligible for
                # the next radius, but do not construct/eigendecompose a PCA
                # that the unchanged support-count gate necessarily rejects.
                enough = counts >= cfg["normal_min_points"]
                if not np.any(enough):
                    continue
                indices, difference, support = indices[enough], difference[enough], support[enough]
                counts = counts[enough].astype(np.float64)
                means, covariance = _local_pca_moments(difference, support, counts)
                # The covariance reduction above remains per cell, with its
                # original candidate order. Only independent 2x2 solves are
                # combined, in bounded batches; no cross-cell PCA is introduced.
                start = 0
                while start < len(indices):
                    stop = min(len(indices), start + 256 - pending_count)
                    pending.append((indices[start:stop], means[start:stop],
                                    counts[start:stop], covariance[start:stop]))
                    pending_count += stop - start
                    start = stop
                    if pending_count == 256:
                        flush_pca()
            # Apply this radius before deciding which rows need a larger one.
            flush_pca()
        if direction_support is not None:
            self._past_direction_support(direction_support)
        self.normals.flags.writeable = False
        self.normal_valid.flags.writeable = False
        self.line_points.flags.writeable = False
        self.normal_radii.flags.writeable = False
        self.normal_thickness.flags.writeable = False
        self.normal_position_support.flags.writeable = False
        self.direction_borrowed.flags.writeable = False
        self.computed_normals = requested | self.direction_borrowed | cached
        self.computed_normals.flags.writeable = False
        self.thickness_p95_m = (float(np.percentile(self.normal_thickness[self.normal_valid], 95))
                                if np.any(self.normal_valid) else None)

    def _past_direction_support(self, support):
        """Use past paint direction without importing its unconfirmed position.

        The caller owns the strictly-past source frontier. Only normals may be
        borrowed. Line centers/thickness and all matching positions remain
        calculated from the original CONFIRMED anchor, within its own component.
        """
        cfg = self.cfg
        unresolved = np.flatnonzero(~self.normal_valid)
        if not len(unresolved):
            return
        # Past low-reliability paint may fill a missing direction, never replace
        # geometry already observable from confirmed map points. Unconditional
        # replacement caused self-reinforcing heading/lateral drift in replay.
        hint = _FrozenIndex(support, cfg, normal_queries=self.anchor[unresolved])
        nearest = np.full(len(self.anchor), -1, dtype=np.int64)
        nearest[unresolved] = hint.query_nearest
        covered = nearest >= 0
        covered[covered] &= hint.normal_valid[nearest[covered]]
        covered[covered] &= np.linalg.norm(self.anchor[covered] - support[nearest[covered]], axis=1) <= cfg["normal_connectivity_m"]
        self.direction_supported_points = 0
        for key, all_indices in self.cells.items():
            indices = np.asarray(all_indices, dtype=np.int64)
            indices = indices[covered[indices]]
            if not len(indices):
                continue
            candidates = self._patch(key, cfg["normal_max_radius_m"])
            difference = self.anchor[candidates][None, :, :] - self.anchor[indices, None, :]
            support_mask = ((np.sum(difference * difference, axis=2) <= cfg["normal_max_radius_m"] ** 2)
                            & (self.components[indices, None] == self.components[candidates][None, :]))
            support_mask, counts = self._local_support(indices, candidates, support_mask, return_counts=True)
            enough = counts >= cfg["normal_min_points"]
            if not np.any(enough):
                continue
            indices, difference, support_mask = indices[enough], difference[enough], support_mask[enough]
            counts = counts[enough]
            normals = hint.normals[nearest[indices]]
            projected = np.einsum("nki,ni->nk", difference, normals)
            mean = np.sum(projected * support_mask, axis=1) / np.maximum(counts, 1)
            variance = np.sum((projected - mean[:, None]) ** 2 * support_mask, axis=1) / np.maximum(counts, 1)
            valid = ((counts >= cfg["normal_min_points"])
                     & (variance <= cfg["normal_max_thickness_m"] ** 2))
            selected = indices[valid]
            self.normals[selected] = normals[valid]
            self.direction_borrowed[selected] = True
            self.normal_valid[selected] = True
            self.normal_thickness[selected] = np.sqrt(np.maximum(variance[valid], 0.))
            self.normal_position_support[selected] = counts[valid]
            self.normal_radii[selected] = cfg["normal_max_radius_m"]
            self.line_points[selected] = self.anchor[selected] + normals[valid] * mean[valid, None]
            self.direction_supported_points += len(selected)

    def _local_support(self, targets, candidates, allowed, return_counts=False):
        """Component containing each seed in its own radius-induced graph.

        A remote U-shaped bridge must not join two locally separate paint bands.
        Compact Python integer bitsets bound memory to the existing patch cap;
        only vertices inside each query circle may be traversed. This is exact
        connectivity, not a truncated flood-fill that could invent a thin line.
        """
        if (_GRAPH_NATIVE is not None and self._native_graph is not None
                and targets.dtype == np.int64 and candidates.dtype == np.int64
                and targets.ndim == candidates.ndim == 1 and allowed.dtype == np.bool_
                and allowed.shape == (len(targets), len(candidates))):
            # The native routine validates this exact forest and computes the
            # same seed-induced support. Keep the original in-place mask API.
            packed_support, packed_counts = _GRAPH_NATIVE.local_support(
                self._native_graph, len(self.anchor), candidates.tobytes(),
                targets.tobytes(), allowed.tobytes(), self._forest_parents.tobytes())
            allowed[...] = np.frombuffer(packed_support, dtype=np.bool_).reshape(allowed.shape)
            counts = np.frombuffer(packed_counts, dtype=np.int64).copy()
            return (allowed, counts) if return_counts else allowed
        # Runtime callers already apply this filter. Enforce it here as well:
        # an equal-sized set containing other components is not a whole-component
        # certificate for this seed. Native flood naturally excludes such nodes.
        allowed &= self.components[targets, None] == self.components[candidates][None, :]
        # A connected candidate subset is not support for a seed outside it.
        # In particular, bounded patch truncation can exclude a target. Check
        # membership before either whole-component or spanning-tree shortcuts.
        seed_allowed = np.any(allowed & (targets[:, None] == candidates[None, :]), axis=1)
        allowed[~seed_allowed] = False
        counts = np.sum(allowed, axis=1, dtype=np.int64)
        incomplete = np.flatnonzero(seed_allowed & (counts != self._component_sizes[self.components[targets]]))
        if not len(incomplete):
            return (allowed, counts) if return_counts else allowed
        if self._adjacency_bits is not None:
            positions = np.full(len(self.anchor), -1, dtype=np.int64)
            positions[candidates] = np.arange(len(candidates))
            parents = self._forest_parents[candidates]
            edges = np.flatnonzero(parents >= 0)
            edges = edges[positions[parents[edges]] >= 0]
            parent_positions = positions[parents[edges]]
            edge_count = np.sum(allowed[incomplete[:, None], edges]
                                & allowed[incomplete[:, None], parent_positions], axis=1)
            incomplete = incomplete[edge_count != counts[incomplete] - 1]
            if not len(incomplete):
                return (allowed, counts) if return_counts else allowed
            # Live windows are <=4000 points: share their exact sparse graph
            # encoded as at most 2 MB of bitsets across every PCA radius/query.
            # Larger full-map indexes retain the bounded local-patch fallback.
            size = len(self.anchor)
            global_allowed = np.zeros((len(incomplete), size), dtype=bool)
            global_allowed[:, candidates] = allowed[incomplete]
            masks = np.packbits(global_allowed, axis=1, bitorder="little")
            if _GRAPH_NATIVE is not None and self._native_graph is not None:
                reached = _GRAPH_NATIVE.connected_subsets(
                    self._native_graph, size, masks.tobytes(), targets[incomplete].tolist())
                bits = np.unpackbits(np.frombuffer(reached, dtype=np.uint8).reshape(
                    len(incomplete), (size + 7) // 8), axis=1, bitorder="little")[:, :size]
                allowed[incomplete] = bits[:, candidates]
                counts[incomplete] = np.sum(allowed[incomplete], axis=1, dtype=np.int64)
                return (allowed, counts) if return_counts else allowed
            for number, row in enumerate(incomplete):
                seed = int(targets[row])
                if not global_allowed[number, seed]:
                    allowed[row] = False
                    continue
                mask = int.from_bytes(masks[number].tobytes(), "little")
                reached = frontier = 1 << seed
                while frontier and reached != mask:
                    bit = frontier & -frontier
                    frontier ^= bit
                    added = self._adjacency_bits[bit.bit_length() - 1] & mask & ~reached
                    reached |= added
                    frontier |= added
                bits = np.unpackbits(np.frombuffer(reached.to_bytes((size + 7) // 8, "little"), dtype=np.uint8),
                                     bitorder="little")
                allowed[row] = bits[candidates]
            counts[incomplete] = np.sum(allowed[incomplete], axis=1, dtype=np.int64)
            return (allowed, counts) if return_counts else allowed
        count = len(candidates)
        positions = {int(index): position for position, index in enumerate(candidates)}
        # Cheap exact certificate for ordinary continuous bands: an induced
        # subset of a spanning forest is connected iff it retains n-1 edges.
        # Only uncertified rows need flood-fill; this cannot admit a U bridge
        # outside the query circle, since both endpoints must be allowed.
        forest = [(position, positions[other]) for position, index in enumerate(candidates)
                  for other in self._forest_children[int(index)] if other in positions]
        if forest:
            edges = np.asarray(forest, dtype=np.int64)
            connected_count = np.sum(allowed[incomplete[:, None], edges[:, 0]]
                                     & allowed[incomplete[:, None], edges[:, 1]], axis=1)
            needed = counts[incomplete] - 1
            incomplete = incomplete[connected_count != needed]
            if not len(incomplete):
                return (allowed, counts) if return_counts else allowed
        adjacency = []
        points = self.anchor[candidates]
        for start in range(0, count, 64):
            delta = points[start:start + 64, None, :] - points[None, :, :]
            connected = np.sum(delta * delta, axis=2) <= self.cfg["normal_connectivity_m"] ** 2
            adjacency.extend(int.from_bytes(row.tobytes(), "little") for row in
                             np.packbits(connected, axis=1, bitorder="little"))
        byte_count = (count + 7) // 8
        for row in incomplete:
            seed = positions.get(int(targets[row]))
            if seed is None or not allowed[row, seed]:
                allowed[row] = False
                continue
            mask = int.from_bytes(np.packbits(allowed[row], bitorder="little").tobytes(), "little")
            reached = frontier = 1 << seed
            while frontier and reached != mask:
                bit = frontier & -frontier
                frontier ^= bit
                added = adjacency[bit.bit_length() - 1] & mask & ~reached
                reached |= added
                frontier |= added
            allowed[row] = np.unpackbits(np.frombuffer(reached.to_bytes(byte_count, "little"), dtype=np.uint8),
                                         bitorder="little")[:count]
        counts[incomplete] = np.sum(allowed[incomplete], axis=1, dtype=np.int64)
        return (allowed, counts) if return_counts else allowed

    def _components(self):
        """Keep disconnected paint bands out of each other's PCA support.

        Radius growth may reveal one thick connected ribbon, but must never
        invent a transverse line through separate crosswalk bars. Connectivity
        is determined from this observation/reference alone, before matching.
        """
        count = len(self.anchor)
        parent = np.arange(count, dtype=np.int64)
        self._forest_children = [[] for _ in range(count)]
        self._adjacency_bits = [0] * count if count <= 4000 else None
        self._adjacency_packed = None
        self._native_graph = None
        self._forest_parents = np.full(count, -1, dtype=np.int64)
        radius = self.cfg["normal_connectivity_m"]
        buckets = {}
        keys = np.floor(self.anchor / radius).astype(np.int64)

        def root(index):
            while parent[index] != index:
                parent[index] = parent[parent[index]]
                index = parent[index]
            return index

        for index, key in enumerate(keys):
            buckets.setdefault((int(key[0]), int(key[1])), []).append(index)
        if _GRAPH_NATIVE is not None and count <= 4000 and self.anchor.dtype == np.float64:
            # Keep the original Python floor keys and exactly the same five-cell
            # capacity check. Only the bounded pair arithmetic moves to C.
            for key, members in buckets.items():
                candidate_count = len(members) + sum(
                    len(buckets.get((key[0] + dx, key[1] + dy), ()))
                    for dx, dy in ((-1, -1), (-1, 0), (-1, 1), (0, -1)))
                if len(members) > 512 or candidate_count > 2048:
                    raise ValueError("normal connectivity bucket capacity exceeded")
            self._adjacency_packed = _GRAPH_NATIVE.radius_graph(
                self.anchor.tobytes(), keys.tobytes(), count, float(radius ** 2) if count else 0.0)
            row_bytes = (count + 7) // 8
            self._adjacency_bits = [
                int.from_bytes(self._adjacency_packed[i * row_bytes:(i + 1) * row_bytes], "little")
                for i in range(count)]
            self._native_graph = _GRAPH_NATIVE.prepare_graph(self._adjacency_packed, count)
            labels, parents = _GRAPH_NATIVE.graph_forest(self._native_graph, count)
            self._forest_parents = np.asarray(parents, dtype=np.int64)
            return np.asarray(labels, dtype=np.int64)
        matrix = np.zeros((count, count), dtype=bool) if count <= 4000 else None
        left_parts, right_parts = [], []
        pending = 0

        def flush_pairs():
            nonlocal pending
            if not pending:
                return
            left, right = np.concatenate(left_parts), np.concatenate(right_parts)
            difference = self.anchor[left] - self.anchor[right]
            connected = np.sum(difference * difference, axis=1) <= radius ** 2
            matrix[left, right] = connected
            matrix[right, left] = connected
            left_parts.clear()
            right_parts.clear()
            pending = 0

        # Batch the bounded distance queries by small occupied cell, not one
        # Python/NumPy allocation per pixel. Visit each cell-pair just once.
        for key, members in buckets.items():
            candidates = list(members)
            for dx, dy in ((-1, -1), (-1, 0), (-1, 1), (0, -1)):
                candidates.extend(buckets.get((key[0] + dx, key[1] + dy), ()))
            members = np.asarray(members, dtype=np.int64)
            candidates = np.asarray(candidates, dtype=np.int64)
            if len(members) > 512 or len(candidates) > 2048:
                raise ValueError("normal connectivity bucket capacity exceeded")
            if matrix is not None:
                # Combine small cells without a dense floating-point N x N
                # distance array. Preserve every original pair and its order;
                # at most 65536 pairs occupy the temporary distance buffers.
                rows = max(1, 65536 // len(candidates))
                for start in range(0, len(members), rows):
                    chosen = members[start:start + rows]
                    size = len(chosen) * len(candidates)
                    if pending and pending + size > 65536:
                        flush_pairs()
                    left_parts.append(np.repeat(chosen, len(candidates)))
                    right_parts.append(np.tile(candidates, len(chosen)))
                    pending += size
                continue
            difference = self.anchor[members, None, :] - self.anchor[candidates][None, :, :]
            connected = np.sum(difference * difference, axis=2) <= radius ** 2
            rows, cols = np.nonzero(connected)
            for left, right in zip(members[rows], candidates[cols]):
                a, b = root(int(left)), root(int(right))
                if a != b:
                    parent[b] = a
                    self._forest_children[int(left)].append(int(right))
        if matrix is not None:
            flush_pairs()
            packed = np.packbits(matrix, axis=1, bitorder="little")
            self._adjacency_bits = [int.from_bytes(row.tobytes(), "little") for row in packed]
            self._adjacency_packed = packed.tobytes()
            if _GRAPH_NATIVE is not None:
                self._native_graph = _GRAPH_NATIVE.prepare_graph(self._adjacency_packed, count)
                labels, parents = _GRAPH_NATIVE.graph_forest(self._native_graph, count)
                self._forest_parents = np.asarray(parents, dtype=np.int64)
                return np.asarray(labels, dtype=np.int64)
            unseen = (1 << count) - 1
            byte_count = (count + 7) // 8
            while unseen:
                seed = unseen & -unseen
                reached = frontier = seed
                while frontier:
                    bit = frontier & -frontier
                    frontier ^= bit
                    added = self._adjacency_bits[bit.bit_length() - 1] & ~reached
                    children = added
                    while children:
                        child = children & -children
                        children ^= child
                        self._forest_parents[child.bit_length() - 1] = bit.bit_length() - 1
                    reached |= added
                    frontier |= added
                chosen = np.unpackbits(np.frombuffer(reached.to_bytes(byte_count, "little"), dtype=np.uint8),
                                       bitorder="little")[:count].astype(bool)
                parent[chosen] = seed.bit_length() - 1
                unseen &= ~reached
            return parent
        return np.asarray([root(index) for index in range(count)], dtype=np.int64)

    def _patch(self, key, radius):
        cache_key = (key, radius)
        cached = self._patch_cache.get(cache_key)
        if cached is not None:
            return cached
        reach = int(math.ceil(radius / self.cell))
        indices = []
        for dx in range(-reach, reach + 1):
            for dy in range(-reach, reach + 1):
                indices.extend(self.cells.get((key[0] + dx, key[1] + dy), ()))
        if len(indices) > self.cfg["max_patch_points"]:
            center = (np.asarray(key, dtype=np.float64) + 0.5) * self.cell
            distance = np.sum((self.anchor[indices] - center) ** 2, axis=1)
            order = np.argsort(distance, kind="stable")[:self.cfg["max_patch_points"]]
            indices = np.asarray(indices, dtype=np.int64)[order].tolist()
            self.patch_truncations += 1
        result = np.asarray(indices, dtype=np.int64)
        result.flags.writeable = False
        # Pure memoization of an immutable index, bounded even for long-lived
        # repeat sessions whose query centers keep moving through the map.
        if len(self._patch_cache) >= 512:
            self._patch_cache.clear()
        self._patch_cache[cache_key] = result
        return result

    def nearest(self, points, normals=None, valid_only=True):
        if (_GRAPH_NATIVE is not None and points.dtype == self.anchor.dtype == np.float64
                and points.ndim == self.anchor.ndim == 2
                and points.shape[1] == self.anchor.shape[1] == 2
                and len(points) <= 4000 and len(self.anchor) <= 4000
                and self.cfg["max_patch_points"] <= 2048
                and np.all(np.isfinite(points)) and np.all(np.isfinite(self.anchor))
                and (not len(points) or np.max(np.abs(points)) <= 1e150)
                and (not len(self.anchor) or np.max(np.abs(self.anchor)) <= 1e150)
                and np.geterr()["under"] == "ignore"):
            return self._nearest_grouped(points, normals, valid_only)
        indices = np.full(len(points), -1, dtype=np.int64)
        squared_distance = np.full(len(points), np.inf, dtype=np.float64)
        keys = np.floor(points / self.cell).astype(np.int64)
        unique, inverse, _ = _unique_cells(keys)
        for group, key in enumerate(unique):
            targets = np.flatnonzero(inverse == group)
            candidates = self._patch((int(key[0]), int(key[1])), self.cfg["match_distance_m"])
            if not len(candidates):
                continue
            if valid_only:
                candidates = candidates[self.normal_valid[candidates]]
            if not len(candidates):
                continue
            difference = points[targets, None, :] - self.anchor[candidates][None, :, :]
            distances = np.sum(difference * difference, axis=2)
            if normals is not None:
                compatible = np.abs(normals[targets] @ self.normals[candidates].T) >= math.cos(
                    self.cfg["normal_max_angle_rad"])
                distances[~compatible] = np.inf
            choices = np.argmin(distances, axis=1)
            indices[targets] = candidates[choices]
            squared_distance[targets] = distances[np.arange(len(targets)), choices]
        valid = squared_distance <= self.cfg["match_distance_m"] ** 2
        indices[~valid] = -1
        return indices

    def _nearest_grouped(self, points, normals, valid_only):
        """Batch only distance/argmin; keep original patch and NumPy dot calls."""
        keys = np.floor(points / self.cell).astype(np.int64)
        unique, inverse, _ = _unique_cells(keys)
        query_parts, candidate_parts, allowed_parts = [], [], []
        query_offsets, candidate_offsets = [0], [0]
        for group, key in enumerate(unique):
            targets = np.flatnonzero(inverse == group)
            candidates = self._patch((int(key[0]), int(key[1])), self.cfg["match_distance_m"])
            if valid_only and len(candidates):
                candidates = candidates[self.normal_valid[candidates]]
            query_parts.append(targets)
            candidate_parts.append(candidates)
            query_offsets.append(query_offsets[-1] + len(targets))
            candidate_offsets.append(candidate_offsets[-1] + len(candidates))
            if normals is not None and len(candidates):
                # Do not replace this BLAS operation with C multiply/add or a
                # different batch shape: threshold-adjacent rounding matters.
                compatible = np.abs(normals[targets] @ self.normals[candidates].T) >= math.cos(
                    self.cfg["normal_max_angle_rad"])
                allowed_parts.append(compatible.tobytes())
        # flatnonzero uses platform intp; the C byte protocol always uses int64.
        query_ids = (np.concatenate(query_parts).astype(np.int64, copy=False)
                     if query_parts else np.empty(0, dtype=np.int64))
        candidate_ids = np.concatenate(candidate_parts) if candidate_parts else np.empty(0, dtype=np.int64)
        result = _GRAPH_NATIVE.nearest_grouped(
            points.tobytes(), self.anchor.tobytes(), np.asarray(query_offsets, dtype=np.int64).tobytes(),
            query_ids.tobytes(), np.asarray(candidate_offsets, dtype=np.int64).tobytes(),
            candidate_ids.tobytes(), None if normals is None else b"".join(allowed_parts),
            float(self.cfg["match_distance_m"] ** 2))
        return np.frombuffer(result, dtype=np.int64).copy()


class LaneMapLocalizer:
    """One frozen anchor and bounded, causal correction/refinement state.

    ``correction_se2`` is T_map_from_VectorNav, so its global translation can be
    large when yaw is corrected far from the origin. Innovation gates instead
    measure the correction AT THE CURRENT VEHICLE. Initial acquisition has a
    larger bound; the caller must only acquire initially while held stationary.
    """

    def __init__(self, anchor_xy, config=None, anchor_support_xy=None, anchor_support_stamp_ns=None,
                 _reuse_index=None):
        self.config = _configuration(config)
        anchors = _points(anchor_xy, self.config["max_anchor_points"])
        self._anchor = _readonly(_voxel_means(anchors, self.config["anchor_voxel_m"]))
        support = None
        if anchor_support_xy is not None and anchor_support_stamp_ns is None:
            raise ValueError("past anchor direction requires its source stamp")
        if anchor_support_stamp_ns is not None and (
                anchor_support_xy is None or isinstance(anchor_support_stamp_ns, bool)
                or not isinstance(anchor_support_stamp_ns, Integral)
                or not 0 < anchor_support_stamp_ns < 2 ** 63):
            raise ValueError("invalid anchor direction source stamp")
        self.anchor_direction_source_stamp_ns = anchor_support_stamp_ns
        if anchor_support_xy is not None:
            support = _voxel_means(_points(anchor_support_xy, self.config["max_input_scan_points"]),
                                   self.config["scan_voxel_m"])
            if len(support) > self.config["max_scan_support_points"]:
                raise ValueError("past direction-support capacity exceeded")
        self._index = _FrozenIndex(self._anchor, self.config, direction_support=support, reuse_index=_reuse_index)
        self.reset()

    @property
    def anchor_points(self):
        return _readonly(self._anchor)

    @property
    def correction_se2(self):
        return _readonly(self._correction)

    def reset(self):
        self._scan_geometry_index = None
        self._correction = np.zeros(3, dtype=np.float64)
        self.last_seen_stamp_ns = None
        self.last_accepted_stamp_ns = None
        self.accepted_frames = 0
        self.rejected_frames = 0
        self.committed_frames = 0
        self._pending_result = None
        self._pending_data = None
        self._refined_votes = {}
        self._refinement_growth_stopped = False
        self._refinement_global_rejections = 0
        cfg = self.config
        self._refinement = RollingConsensusMap(
            cfg["refinement_resolution_m"], cfg["refinement_min_frame_votes"],
            minimum_reliable_frame_votes=1, window_s=cfg["refinement_window_s"],
            max_cells=cfg["refinement_max_cells"], max_frames=cfg["refinement_max_frames"])

    def correct(self, prior_xyyaw):
        prior = _pose(prior_xyyaw)
        corrected = np.empty(3, dtype=np.float64)
        corrected[:2] = _rotation(self._correction[2]) @ prior[:2] + self._correction[:2]
        corrected[2] = _wrap(prior[2] + self._correction[2])
        return corrected

    def fork_with_anchor(self, anchor_xy, anchor_support_xy=None, anchor_support_stamp_ns=None):
        """Prepare a same-frame keyframe without resetting tracking admission.

        The caller owns promotion of this independent candidate. Only committed
        correction and timestamp fences cross over; pending proposals and the
        anchor-specific refinement layer never do. Both anchors must be in the
        same map frame. This is not a coordinate-system reset or full clone.
        """
        state = self.capture_tracking_state()
        candidate = type(self)(anchor_xy, state.config, anchor_support_xy, anchor_support_stamp_ns,
                               _reuse_index=self._index)
        candidate._correction = np.array(state.correction_se2, dtype=np.float64)
        candidate.last_accepted_stamp_ns = state.last_accepted_stamp_ns
        candidate.last_seen_stamp_ns = state.last_seen_stamp_ns
        # Pure current-mask geometry memoization, not pose/map evidence. The
        # next index rechecks every dependency against its own current points.
        # Sharing this immutable source does not inherit a pending match.
        candidate._scan_geometry_index = self._scan_geometry_index
        return candidate

    def capture_tracking_state(self):
        """Copy only constant-size committed tracking state, with no index work."""
        return CommittedLocalizationState(
            self.config, tuple(self._correction),
            self.last_accepted_stamp_ns, self.last_seen_stamp_ns)

    @classmethod
    def from_tracking_state(cls, anchor_xy, state, anchor_support_xy=None, anchor_support_stamp_ns=None):
        """Build an independent index from an immutable same-frame handoff.

        The expensive anchor/index construction may run on a background worker.
        No pending proposal, counters or anchor-specific refinement is restored.
        """
        if not isinstance(state, CommittedLocalizationState):
            raise ValueError("immutable committed localization state required")
        candidate = cls(anchor_xy, state.config, anchor_support_xy, anchor_support_stamp_ns)
        candidate._correction = np.array(state.correction_se2, dtype=np.float64)
        candidate.last_accepted_stamp_ns = state.last_accepted_stamp_ns
        candidate.last_seen_stamp_ns = state.last_seen_stamp_ns
        return candidate

    def matches_anchor_source(self, anchor_xy):
        """Exactly compare source geometry using this class's own voxelization.

        This does not rebuild an index or permit approximate geometric matching.
        It verifies normalized geometry, not ownership of an external session.
        """
        try:
            points = _points(anchor_xy, self.config["max_anchor_points"])
            normalized = _voxel_means(points, self.config["anchor_voxel_m"])
        except (TypeError, ValueError, OverflowError):
            return False
        return bool(np.array_equal(self._anchor, normalized))

    def discard_pending(self):
        """Revoke an uncommitted proposal after a caller's deadline/guard fails."""
        self._pending_result = None
        self._pending_data = None

    def refined_points_and_votes(self):
        if not self._refined_votes:
            return np.empty((0, 2), dtype=np.float64), np.empty(0, dtype=np.float64)
        keys = np.asarray(sorted(self._refined_votes), dtype=np.uint64)
        x, y = unpack_cell_keys(keys)
        points = np.column_stack((x, y)) * self.config["refinement_resolution_m"]
        votes = np.asarray([self._refined_votes[int(key)] for key in keys], dtype=np.float64)
        return points, votes

    refinement_points_and_votes = refined_points_and_votes

    def refinement_metrics(self):
        return {"refinement_candidate_cells": len(self._refinement.candidate_votes),
                "refinement_confirmed_cells": len(self._refined_votes),
                "refinement_rolling_confirmed_cells": len(self._refinement.confirmed_keys),
                "refinement_retained_frames": self._refinement.retained_frame_count,
                "refinement_capacity_rejected_cells": self._refinement.capacity_rejected_cells,
                "refinement_frame_capacity_evictions": self._refinement.frame_capacity_evictions,
                "refinement_global_capacity_rejected_cells": self._refinement_global_rejections,
                "refinement_growth_stopped": self._refinement_growth_stopped}

    def commit(self, result):
        """Commit exactly the latest accepted proposal after external age checks.

        Failed, forged, superseded or already committed proposals return False.
        No state is changed for those cases, including the refinement evidence.
        """
        if (not isinstance(result, MatchResult) or result is not self._pending_result
                or not result.accepted or self._pending_data is None):
            return False
        stamp, correction, inlier_world = self._pending_data
        if self.last_accepted_stamp_ns is not None and stamp <= self.last_accepted_stamp_ns:
            return False
        indices = np.floor(inlier_world / self.config["refinement_resolution_m"] + 0.5).astype(np.int64)
        keys = np.unique(pack_cell_indices(indices[:, 0], indices[:, 1]))
        self._refinement.update(keys, keys, stamp_ns=stamp)
        eligible = [int(key) for key in keys if int(key) in self._refinement.confirmed_keys]
        new_keys = [key for key in eligible if key not in self._refined_votes]
        # The recent window proves multi-frame support; confirmed history then
        # persists around the entire lap. Capacity overflow freezes admissions
        # explicitly instead of silently deleting earlier track sections.
        if len(self._refined_votes) + len(new_keys) > self.config["refinement_max_cells"]:
            self._refinement_growth_stopped = True
        for key in eligible:
            if key in self._refined_votes:
                self._refined_votes[key] += 1
            elif self._refinement_growth_stopped:
                self._refinement_global_rejections += 1
            else:
                self._refined_votes[key] = self._refinement.candidate_votes[key]
        self._correction = correction.copy()
        self.last_accepted_stamp_ns = stamp
        self.committed_frames += 1
        self._pending_data = None
        self._pending_result = None
        return True

    def _correspondences(self, local, estimate, scan_normals=None):
        rotated = local @ _rotation(estimate[2]).T
        world = rotated + estimate[:2]
        world_normals = None if scan_normals is None else scan_normals @ _rotation(estimate[2]).T
        nearest = self._index.nearest(world, world_normals)
        selected = np.flatnonzero(nearest >= 0)
        anchor_indices = nearest[selected]
        normals = self._index.normals[anchor_indices]
        residual = np.sum((world[selected] - self._index.line_points[anchor_indices]) * normals, axis=1)
        return world, rotated, selected, normals, residual

    def match(self, local_xy, prior_xyyaw, stamp_ns, prior_stamp_ns=None, commit=True,
              scan_support_xy=None, scan_sensitivity=None):
        """Propose a point-to-line fit using only this mask and the frozen anchor.

        Strictly increasing mask stamps prevent frame-vote replay.  Optional
        ``prior_stamp_ns`` verifies the supplied prior is not from the future.
        ``commit=False`` is REQUIRED for a worker whose result may miss a deadline.
        A rejected fit never changes the last committed pose correction or map.
        ``scan_support_xy`` is optional SAME-current-mask geometry before the
        reliable-sensitivity cut. It defines local line directions only; those
        extra points are never fit, voted as inliers or used as old-map evidence.
        """
        started = time.perf_counter_ns()
        cfg = self.config
        self._pending_result = None
        self._pending_data = None
        metrics = {"graph_backend": graph_backend_info(), "anchor_points": len(self._anchor),
                   "anchor_line_points": int(np.sum(self._index.normal_valid)),
                   "anchor_multiscale_line_points": int(np.sum(
                       self._index.normal_radii > cfg["normal_radius_m"])),
                   "normal_max_radius_m": cfg["normal_max_radius_m"],
                   "anchor_line_thickness_p95_m": self._index.thickness_p95_m,
                   "anchor_past_direction_supported_points": getattr(self._index, "direction_supported_points", 0),
                   "anchor_reused_identical_geometry_points": self._index.reused_geometry_points,
                   "anchor_direction_source_stamp_ns": self.anchor_direction_source_stamp_ns,
                   "anchor_min_confirmed_position_support": (int(np.min(self._index.normal_position_support[
                       self._index.normal_valid])) if np.any(self._index.normal_valid) else 0),
                   "line_reference": "confirmed_position_with_optional_strictly_past_paint_direction",
                   "input_scan_points": 0, "scan_points": 0, "inliers": 0,
                   "scan_support_points": 0, "scan_geometric_rejections": 0,
                   "overlap_denominator": "connected_current_cross_section_density_not_sensitivity",
                   "inlier_ratio": 0.0, "overlap_ratio": 0.0,
                   "rmse_before_m": None, "rmse_after_m": None,
                   "observable_rank": 0, "longitudinal_observable": False,
                   "initial_acquisition": self.last_accepted_stamp_ns is None,
                   "iterations": 0, "runtime_budget_ms": cfg["runtime_budget_ms"],
                   "elapsed_ms": 0.0, "budget_exceeded": False,
                   "unobservable_translation_anchor": "raw_vectornav",
                   "max_scan_range_m": cfg["max_scan_range_m"],
                   "uses_frozen_anchor": True, "uses_refinement_for_matching": False}
        metrics.update(self.refinement_metrics())
        prior = None
        predicted = np.zeros(3, dtype=np.float64)
        source_stamp = int(stamp_ns) if isinstance(stamp_ns, Integral) and not isinstance(stamp_ns, bool) else 0

        def finish(reason, accepted=False, estimate=None, world=None):
            elapsed = (time.perf_counter_ns() - started) / 1e6
            metrics["elapsed_ms"] = float(elapsed)
            metrics["budget_exceeded"] = bool(elapsed > cfg["runtime_budget_ms"])
            if accepted and metrics["budget_exceeded"]:
                accepted, reason = False, "runtime_budget_exceeded"
            chosen = estimate if accepted else predicted
            correction = self._correction.copy()
            if accepted:
                correction[2] = _wrap(chosen[2] - prior[2])
                correction[:2] = chosen[:2] - _rotation(correction[2]) @ prior[:2]
            result = MatchResult(bool(accepted), _readonly(chosen), _readonly(correction),
                                 str(reason), source_stamp, dict(metrics))
            if accepted:
                self.accepted_frames += 1
                self._pending_result = result
                self._pending_data = (source_stamp, correction.copy(), world.copy())
                if commit:
                    self.commit(result)
            else:
                self.rejected_frames += 1
            return result

        if not 0 < source_stamp < 2 ** 63:
            return finish("invalid_mask_stamp")
        if (self.anchor_direction_source_stamp_ns is not None
                and self.anchor_direction_source_stamp_ns >= source_stamp):
            return finish("future_or_current_anchor_direction")
        if self.last_seen_stamp_ns is not None and source_stamp <= self.last_seen_stamp_ns:
            return finish("nonincreasing_mask_stamp")
        self.last_seen_stamp_ns = source_stamp
        try:
            prior = _pose(prior_xyyaw)
            predicted = self.correct(prior)
            scan = _points(local_xy, cfg["max_input_scan_points"])
            support = scan if scan_support_xy is None else _points(
                scan_support_xy, cfg["max_input_scan_points"])
            sensitivity = (None if scan_sensitivity is None else np.asarray(scan_sensitivity, dtype=float))
            if sensitivity is not None and (sensitivity.shape != (len(scan),)
                    or not np.all(np.isfinite(sensitivity)) or np.any(sensitivity < 0)):
                raise ValueError("scan sensitivity must be finite nonnegative aligned values")
        except (ValueError, TypeError, OverflowError):
            return finish("invalid_input")
        if prior_stamp_ns is not None:
            if (not isinstance(prior_stamp_ns, Integral) or isinstance(prior_stamp_ns, bool)
                    or not 0 < prior_stamp_ns <= source_stamp):
                return finish("future_or_invalid_prior_stamp")
        metrics["input_scan_points"] = len(scan)
        if not len(self._anchor):
            return finish("empty_anchor")
        if metrics["anchor_line_points"] < cfg["min_inliers"]:
            return finish("insufficient_anchor_line_support")
        in_range = np.linalg.norm(scan, axis=1) <= cfg["max_scan_range_m"]
        metrics["range_rejected_points"] = int(np.sum(~in_range))
        scan = scan[in_range]
        scan = _voxel_means(scan, cfg["scan_voxel_m"])
        if len(scan) > cfg["max_scan_points"]:
            indices = np.linspace(0, len(scan) - 1, cfg["max_scan_points"]).astype(np.int64)
            scan = scan[indices]
        metrics["scan_points"] = len(scan)
        if len(scan) < cfg["min_scan_points"]:
            return finish("insufficient_scan_points")
        # Compute features BEFORE consulting the old anchor. A filled stripe
        # clipped by the reliability window must not acquire a new transverse
        # normal merely because its currently reliable segment is very short.
        support = _voxel_means(support[np.linalg.norm(support, axis=1) <= cfg["max_scan_range_m"]],
                               cfg["scan_voxel_m"])
        metrics["scan_support_points"] = len(support)
        if len(support) > cfg["max_scan_support_points"]:
            return finish("scan_geometry_capacity")
        # Forks may share this memoization source. A bytes-backed view cannot
        # have its writeable flag re-enabled, unlike an owning read-only array.
        support = np.frombuffer(support.tobytes(), dtype=np.float64).reshape(support.shape)
        support_cells = {tuple(key): index for index, key in enumerate(
            np.floor(support / cfg["scan_voxel_m"]).astype(np.int64))}
        indices = np.asarray([support_cells.get(tuple(key), -1) for key in
                              np.floor(scan / cfg["scan_voxel_m"]).astype(np.int64)], dtype=np.int64)
        eligible = indices >= 0
        try:
            support_index = _FrozenIndex(support, cfg, normal_targets=indices[eligible],
                                         reuse_index=self._scan_geometry_index)
        except ValueError:
            return finish("scan_geometry_capacity")
        # A rejected fit may leave an exact geometry cache, never accepted pose
        # or map votes. Cache hits require current full-cell dependency equality;
        # stale/different observations cannot become current measurement support.
        self._scan_geometry_index = support_index
        metrics["scan_reused_identical_geometry_points"] = support_index.reused_geometry_points
        eligible[eligible] &= support_index.normal_valid[indices[eligible]]
        metrics["scan_geometric_rejections"] = int(np.sum(~eligible))
        scan_normals = support_index.normals[indices[eligible]]
        scan = scan[eligible]
        metrics["scan_eligible_pixels"] = len(scan)
        feature_weight = _line_weights(scan, scan_normals, support_index, indices[eligible], cfg)
        fit_weight = feature_weight
        # J4 screens projection stability in the caller; it is not a calibrated
        # point-to-line covariance. Inverse-J weighting suppressed useful heading
        # leverage and changed drift direction, so it is NOT a fitting weight.
        weight_sum = float(np.sum(feature_weight))
        metrics["scan_feature_weight_sum"] = weight_sum
        metrics["sensitivity_weighting"] = "screen_only_not_fit_covariance" if sensitivity is not None else "not_supplied"
        metrics["sensitivity_is_calibrated_covariance"] = False
        metrics["scan_line_points"] = len(scan)
        if len(scan) < cfg["min_scan_points"]:
            return finish("unobservable_geometry")
        estimate = predicted.copy()
        basis = None
        dominant_normal = None
        observable_rank = 0
        for iteration in range(cfg["max_iterations"]):
            metrics["iterations"] = iteration + 1
            world, rotated, selected, normals, residual = self._correspondences(scan, estimate, scan_normals)
            metrics["overlap_ratio"] = float(np.sum(feature_weight[selected]) / weight_sum)
            metrics["unweighted_overlap_ratio"] = float(len(selected) / len(scan))
            if len(selected) < cfg["min_inliers"] or metrics["overlap_ratio"] < cfg["min_overlap_ratio"]:
                return finish("insufficient_overlap")
            if iteration == 0:
                metrics["rmse_before_m"] = float(np.sqrt(np.mean(residual ** 2)))
                normal_eigenvalues, normal_eigenvectors = np.linalg.eigh(normals.T @ normals)
                diversity = float(normal_eigenvalues[0] / max(normal_eigenvalues[-1], 1e-12))
                metrics["normal_diversity_ratio"] = diversity
                longitudinal = (cfg["allow_longitudinal_correction"] and
                                diversity >= cfg["normal_diversity_ratio"])
                metrics["longitudinal_observable"] = bool(longitudinal)
                if longitudinal:
                    basis = np.eye(3, dtype=np.float64)
                else:
                    # A previous yaw correction changes the global SE2 translation
                    # as the vehicle moves. Reusing that translation as the null
                    # direction prior lets symmetric curves accumulate fictitious
                    # along-track drift despite a decreasing matching residual.
                    # Pin TOTAL forward displacement to the CURRENT RAW VN pose.
                    dominant_normal = np.array([-math.sin(prior[2]), math.cos(prior[2])])
                    basis = np.zeros((3, 2), dtype=np.float64)
                    basis[:2, 0] = dominant_normal
                    basis[2, 1] = 1.0
                    estimate[:2] = prior[:2] + dominant_normal * float(
                        (predicted[:2] - prior[:2]) @ dominant_normal)
                    world, rotated, selected, normals, residual = self._correspondences(scan, estimate, scan_normals)
                    if (len(selected) < cfg["min_inliers"] or
                            np.sum(feature_weight[selected]) / weight_sum < cfg["min_overlap_ratio"]):
                        return finish("insufficient_overlap_after_raw_prior_anchor")
            lever = rotated[selected]
            derivative = normals[:, 0] * -lever[:, 1] + normals[:, 1] * lever[:, 0]
            jacobian = np.column_stack((normals, derivative / cfg["rotation_scale_m"])) @ basis
            weight = fit_weight[selected] * np.minimum(
                1.0, cfg["huber_delta_m"] / np.maximum(np.abs(residual), 1e-12))
            hessian = jacobian.T @ (jacobian * weight[:, None])
            gradient = jacobian.T @ (residual * weight)
            eigenvalues, eigenvectors = np.linalg.eigh(hessian)
            observed = eigenvalues > max(1e-8, cfg["observable_eigen_ratio"] * eigenvalues[-1])
            observable_rank = int(np.sum(observed))
            metrics["observable_rank"] = observable_rank
            if observable_rank < 2:
                return finish("unobservable_geometry")
            # If a curved patch still has only two reliable modes, do not claim
            # full pose observability; solve only those modes with no ridge prior.
            if observable_rank < 3:
                metrics["longitudinal_observable"] = False
                if basis.shape[1] == 3:
                    dominant_normal = np.array([-math.sin(prior[2]), math.cos(prior[2])])
                    basis = np.zeros((3, 2), dtype=np.float64)
                    basis[:2, 0] = dominant_normal
                    basis[2, 1] = 1.0
                    estimate[:2] = prior[:2] + dominant_normal * float(
                        (predicted[:2] - prior[:2]) @ dominant_normal)
                    continue
            vectors = eigenvectors[:, observed]
            step = -(vectors @ ((vectors.T @ gradient) / eigenvalues[observed]))
            full_step = basis @ step
            full_step[2] /= cfg["rotation_scale_m"]
            estimate[:2] += full_step[:2]
            estimate[2] = _wrap(estimate[2] + full_step[2])
            if not np.all(np.isfinite(estimate)):
                return finish("nonfinite_fit")
            if (np.linalg.norm(full_step[:2]) < cfg["convergence_translation_m"]
                    and abs(full_step[2]) < cfg["convergence_yaw_rad"]):
                break
            if (time.perf_counter_ns() - started) / 1e6 > cfg["runtime_budget_ms"]:
                return finish("runtime_budget_exceeded")
        world, _, selected, normals, residual = self._correspondences(scan, estimate, scan_normals)
        inlier = np.abs(residual) <= cfg["inlier_distance_m"]
        count = int(np.sum(inlier))
        metrics["inliers"] = count
        metrics["inlier_ratio"] = float(np.sum(feature_weight[selected[inlier]]) / weight_sum)
        metrics["overlap_ratio"] = float(np.sum(feature_weight[selected]) / weight_sum)
        metrics["unweighted_inlier_ratio"] = float(count / len(scan))
        metrics["unweighted_overlap_ratio"] = float(len(selected) / len(scan))
        if count < cfg["min_inliers"] or metrics["inlier_ratio"] < cfg["min_overlap_ratio"]:
            return finish("insufficient_final_inliers")
        rmse = float(np.sqrt(np.mean(residual[inlier] ** 2)))
        metrics["rmse_after_m"] = rmse
        metrics["residual_p95_m"] = float(np.percentile(np.abs(residual[inlier]), 95))
        if rmse > cfg["max_rmse_m"]:
            return finish("residual_gate")
        if metrics["rmse_before_m"] is not None and rmse > metrics["rmse_before_m"] + 0.005:
            return finish("residual_worsened")
        innovation_xy = estimate[:2] - predicted[:2]
        innovation_yaw = _wrap(estimate[2] - predicted[2])
        total_xy = estimate[:2] - prior[:2]
        total_yaw = _wrap(estimate[2] - prior[2])
        metrics.update({"innovation_translation_m": float(np.linalg.norm(innovation_xy)),
                        "innovation_yaw_rad": float(innovation_yaw),
                        "total_translation_at_vehicle_m": float(np.linalg.norm(total_xy)),
                        "total_yaw_rad": float(total_yaw)})
        if dominant_normal is not None:
            tangent = np.array([-dominant_normal[1], dominant_normal[0]])
            metrics["innovation_along_lane_m"] = float(innovation_xy @ tangent)
            metrics["innovation_lateral_m"] = float(innovation_xy @ dominant_normal)
            metrics["total_along_prior_forward_m"] = float(total_xy @ -tangent)
        initial = self.last_accepted_stamp_ns is None
        translation_limit = cfg["initial_max_translation_m" if initial else "tracking_max_translation_m"]
        yaw_limit = cfg["initial_max_yaw_rad" if initial else "tracking_max_yaw_rad"]
        if np.linalg.norm(innovation_xy) > translation_limit or abs(innovation_yaw) > yaw_limit:
            return finish("initial_innovation_gate" if initial else "tracking_innovation_gate")
        if np.linalg.norm(total_xy) > cfg["max_total_translation_m"] or abs(total_yaw) > cfg["max_total_yaw_rad"]:
            return finish("total_correction_gate")
        return finish("matched_observable_subspace", True, estimate, world[selected[inlier]])
