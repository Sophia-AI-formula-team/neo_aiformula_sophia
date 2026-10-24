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

import numpy as np

from .mapping_core import RollingConsensusMap, pack_cell_indices, unpack_cell_keys


DEFAULT_CONFIG = {
    "max_anchor_points": 100000,
    "max_input_scan_points": 200000,
    "max_scan_points": 600,
    "min_scan_points": 30,
    "min_inliers": 24,
    "anchor_voxel_m": 0.08,
    "scan_voxel_m": 0.10,
    "hash_cell_m": 0.50,
    "normal_radius_m": 0.45,
    "normal_max_radius_m": 1.35,
    "normal_min_points": 6,
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
                    "min_scan_points", "min_inliers", "normal_min_points",
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


def _voxel_means(points, resolution):
    if not len(points):
        return np.empty((0, 2), dtype=np.float64)
    keys = np.floor(points / resolution).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    sums = np.zeros((len(counts), 2), dtype=np.float64)
    np.add.at(sums, inverse, points)
    return sums / counts[:, None]


@dataclass(frozen=True)
class MatchResult:
    accepted: bool
    pose_xyyaw: np.ndarray
    correction_se2: np.ndarray
    reason: str
    stamp_ns: int
    metrics: dict


class _FrozenIndex:
    """Spatial-hash batches: no full anchor-by-scan distance matrix per frame."""

    def __init__(self, anchor, cfg):
        self.anchor = anchor
        self.cfg = cfg
        self.cell = cfg["hash_cell_m"]
        self.cells = {}
        keys = np.floor(anchor / self.cell).astype(np.int64)
        for index, key in enumerate(keys):
            self.cells.setdefault((int(key[0]), int(key[1])), []).append(index)
        self.normals = np.zeros_like(anchor)
        self.line_points = anchor.copy()
        self.normal_valid = np.zeros(len(anchor), dtype=bool)
        self.normal_radii = np.zeros(len(anchor), dtype=np.float64)
        self.normal_thickness = np.zeros(len(anchor), dtype=np.float64)
        self.patch_truncations = 0
        radii = [cfg["normal_radius_m"]]
        while radii[-1] < cfg["normal_max_radius_m"]:
            radii.append(min(radii[-1] * 2.0, cfg["normal_max_radius_m"]))
        # Choose the SMALLEST scale passing the unchanged line-support test.
        # Thin lane marks retain small-scale geometry. A rasterized/accumulated
        # wide band may require a larger neighborhood to reveal its long axis.
        # Isotropic patches and intersections are not rescued by relaxing ratio.
        for radius in radii:
            for key, all_indices in self.cells.items():
                indices = np.asarray(all_indices, dtype=np.int64)
                indices = indices[~self.normal_valid[indices]]
                if not len(indices):
                    continue
                candidate_indices = self._patch(key, radius)
                if len(candidate_indices) < cfg["normal_min_points"]:
                    continue
                targets = anchor[indices]
                candidates = anchor[candidate_indices]
                difference = candidates[None, :, :] - targets[:, None, :]
                support = np.sum(difference * difference, axis=2) <= radius * radius
                weights = support.astype(np.float64)
                counts = np.sum(weights, axis=1)
                safe_count = np.maximum(counts, 1.0)
                means = np.sum(difference * weights[:, :, None], axis=1) / safe_count[:, None]
                centered = difference - means[:, None, :]
                covariance = np.einsum("nki,nkj,nk->nij", centered, centered, weights) / safe_count[:, None, None]
                eigenvalues, eigenvectors = np.linalg.eigh(covariance)
                valid = ((counts >= cfg["normal_min_points"]) &
                         (eigenvalues[:, 1] > 1e-5) &
                         (eigenvalues[:, 0] <= cfg["normal_max_thickness_m"] ** 2) &
                         (eigenvalues[:, 0] <= cfg["normal_max_variance_ratio"] * eigenvalues[:, 1]))
                chosen = indices[valid]
                normals = eigenvectors[valid, :, 0]
                self.normals[chosen] = normals
                self.normal_valid[chosen] = True
                self.normal_radii[chosen] = radius
                self.normal_thickness[chosen] = np.sqrt(np.maximum(eigenvalues[valid, 0], 0.0))
                # Match the local band center in its normal direction, not any
                # arbitrarily nearest pixel inside a thick, filled lane ribbon.
                offset = np.sum(means[valid] * normals, axis=1)
                self.line_points[chosen] = anchor[chosen] + normals * offset[:, None]
        self.normals.flags.writeable = False
        self.normal_valid.flags.writeable = False
        self.line_points.flags.writeable = False
        self.normal_radii.flags.writeable = False
        self.normal_thickness.flags.writeable = False
        self.thickness_p95_m = (float(np.percentile(self.normal_thickness[self.normal_valid], 95))
                                if np.any(self.normal_valid) else None)

    def _patch(self, key, radius):
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
        return np.asarray(indices, dtype=np.int64)

    def nearest(self, points):
        indices = np.full(len(points), -1, dtype=np.int64)
        squared_distance = np.full(len(points), np.inf, dtype=np.float64)
        keys = np.floor(points / self.cell).astype(np.int64)
        unique, inverse = np.unique(keys, axis=0, return_inverse=True)
        for group, key in enumerate(unique):
            targets = np.flatnonzero(inverse == group)
            candidates = self._patch((int(key[0]), int(key[1])), self.cfg["match_distance_m"])
            if not len(candidates):
                continue
            candidates = candidates[self.normal_valid[candidates]]
            if not len(candidates):
                continue
            difference = points[targets, None, :] - self.anchor[candidates][None, :, :]
            distances = np.sum(difference * difference, axis=2)
            choices = np.argmin(distances, axis=1)
            indices[targets] = candidates[choices]
            squared_distance[targets] = distances[np.arange(len(targets)), choices]
        valid = squared_distance <= self.cfg["match_distance_m"] ** 2
        indices[~valid] = -1
        return indices


class LaneMapLocalizer:
    """One frozen anchor and bounded, causal correction/refinement state.

    ``correction_se2`` is T_map_from_VectorNav, so its global translation can be
    large when yaw is corrected far from the origin. Innovation gates instead
    measure the correction AT THE CURRENT VEHICLE. Initial acquisition has a
    larger bound; the caller must only acquire initially while held stationary.
    """

    def __init__(self, anchor_xy, config=None):
        self.config = _configuration(config)
        anchors = _points(anchor_xy, self.config["max_anchor_points"])
        self._anchor = _readonly(_voxel_means(anchors, self.config["anchor_voxel_m"]))
        self._index = _FrozenIndex(self._anchor, self.config)
        self.reset()

    @property
    def anchor_points(self):
        return _readonly(self._anchor)

    @property
    def correction_se2(self):
        return _readonly(self._correction)

    def reset(self):
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

    def _correspondences(self, local, estimate):
        rotated = local @ _rotation(estimate[2]).T
        world = rotated + estimate[:2]
        nearest = self._index.nearest(world)
        selected = np.flatnonzero(nearest >= 0)
        anchor_indices = nearest[selected]
        normals = self._index.normals[anchor_indices]
        residual = np.sum((world[selected] - self._index.line_points[anchor_indices]) * normals, axis=1)
        return world, rotated, selected, normals, residual

    def match(self, local_xy, prior_xyyaw, stamp_ns, prior_stamp_ns=None, commit=True):
        """Propose a point-to-line fit using only this mask and the frozen anchor.

        Strictly increasing mask stamps prevent frame-vote replay.  Optional
        ``prior_stamp_ns`` verifies the supplied prior is not from the future.
        ``commit=False`` is REQUIRED for a worker whose result may miss a deadline.
        A rejected fit never changes the last committed pose correction or map.
        """
        started = time.perf_counter_ns()
        cfg = self.config
        self._pending_result = None
        self._pending_data = None
        metrics = {"anchor_points": len(self._anchor),
                   "anchor_line_points": int(np.sum(self._index.normal_valid)),
                   "anchor_multiscale_line_points": int(np.sum(
                       self._index.normal_radii > cfg["normal_radius_m"])),
                   "normal_max_radius_m": cfg["normal_max_radius_m"],
                   "anchor_line_thickness_p95_m": self._index.thickness_p95_m,
                   "line_reference": "frozen_multiscale_pca_band_center",
                   "input_scan_points": 0, "scan_points": 0, "inliers": 0,
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
        if self.last_seen_stamp_ns is not None and source_stamp <= self.last_seen_stamp_ns:
            return finish("nonincreasing_mask_stamp")
        self.last_seen_stamp_ns = source_stamp
        try:
            prior = _pose(prior_xyyaw)
            predicted = self.correct(prior)
            scan = _points(local_xy, cfg["max_input_scan_points"])
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
        scan = _voxel_means(scan[in_range], cfg["scan_voxel_m"])
        if len(scan) > cfg["max_scan_points"]:
            indices = np.linspace(0, len(scan) - 1, cfg["max_scan_points"]).astype(np.int64)
            scan = scan[indices]
        metrics["scan_points"] = len(scan)
        if len(scan) < cfg["min_scan_points"]:
            return finish("insufficient_scan_points")
        estimate = predicted.copy()
        basis = None
        dominant_normal = None
        observable_rank = 0
        for iteration in range(cfg["max_iterations"]):
            metrics["iterations"] = iteration + 1
            world, rotated, selected, normals, residual = self._correspondences(scan, estimate)
            metrics["overlap_ratio"] = float(len(selected) / len(scan))
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
                    world, rotated, selected, normals, residual = self._correspondences(scan, estimate)
                    if (len(selected) < cfg["min_inliers"] or
                            len(selected) / len(scan) < cfg["min_overlap_ratio"]):
                        return finish("insufficient_overlap_after_raw_prior_anchor")
            lever = rotated[selected]
            derivative = normals[:, 0] * -lever[:, 1] + normals[:, 1] * lever[:, 0]
            jacobian = np.column_stack((normals, derivative / cfg["rotation_scale_m"])) @ basis
            weight = np.minimum(1.0, cfg["huber_delta_m"] / np.maximum(np.abs(residual), 1e-12))
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
        world, _, selected, normals, residual = self._correspondences(scan, estimate)
        inlier = np.abs(residual) <= cfg["inlier_distance_m"]
        count = int(np.sum(inlier))
        metrics["inliers"] = count
        metrics["inlier_ratio"] = float(count / len(scan))
        metrics["overlap_ratio"] = float(len(selected) / len(scan))
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
