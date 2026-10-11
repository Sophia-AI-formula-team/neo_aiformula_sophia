"""Causal wheel/raw-gyro teach mapping; no ROS or GNSS input is accepted here.

One worker must serialize process/freeze. Projection and input freshness belong
to the ROS wrapper. A commit_guard factory may hold the wrapper's generation /
deadline lock through all persistent geometry mutations. A rejected match never
adds either a trace point or map evidence; there is no silent odometry fallback
after the bounded bootstrap phase.
"""

from collections import Counter
from contextlib import nullcontext
from dataclasses import dataclass
import json
import math
from numbers import Integral
import time
from types import MappingProxyType

import numpy as np

from lane_mapping_lya_reference.mapping_core import SparseConsensusMap, transform_local_points
from lane_mapping_lya_reference.mask_localization import (
    LaneMapLocalizer, MotionInformationState, _configuration, _motion_prediction, _voxel_means)
from lane_mapping_lya_reference.paint_geometry import (
    BoundaryObservation, BoundaryReference, BoundaryConsensus)
from lane_mapping_lya_reference.boundary_localization import BoundaryLaneLocalizer


DEFAULT_CONFIG = {
    "grid_resolution_m": 0.1, "minimum_frame_votes": 5,
    "minimum_reliable_votes": 1, "max_sensitivity": 0.5,
    "reliable_sensitivity": 0.1, "max_input_points": 200000,
    "max_candidate_cells": 200000, "max_confirmed_cells": 100000,
    "candidate_ttl_s": 30.0, "max_trace_points": 100000,
    "max_duration_s": 3600.0, "bootstrap_min_anchor_points": 100,
    "bootstrap_max_frames": 60, "bootstrap_max_duration_s": 5.0,
    "bootstrap_max_distance_m": 2.0, "keyframe_radius_m": 12.0,
    "keyframe_max_points": 3000, "keyframe_refresh_s": 1.0,
    "keyframe_refresh_distance_m": 1.0, "max_processing_ms": 80.0,
}

# A different valid, strictly past anchor may resolve these geometry failures.
# Invalid data, time faults, nonfinite fits and deadlines must never be retried
# through another reference. Only the one current active reference is eligible.
_REFERENCE_REJECTIONS = frozenset({
    "empty_anchor", "insufficient_anchor_line_support", "insufficient_overlap",
    "insufficient_overlap_after_raw_prior_anchor", "unobservable_geometry",
    "insufficient_final_inliers", "residual_gate", "residual_worsened",
    "tracking_innovation_gate", "total_correction_gate",
})


def _readonly(values, columns=None):
    array = np.asarray(values, dtype=np.float64)
    if columns is not None:
        array = array.reshape(-1, columns)
    if not np.all(np.isfinite(array)):
        raise ValueError("snapshot contains nonfinite geometry")
    # Backed by immutable bytes, not merely a writable array with its flag reset.
    return np.frombuffer(array.tobytes(), dtype=np.float64).reshape(array.shape)


@dataclass(frozen=True)
class MapSnapshot:
    trace: np.ndarray
    consensus_xy: np.ndarray
    votes: np.ndarray
    source_frontier_ns: int
    stats: object
    boundary_reference: object = None

    def __post_init__(self):
        object.__setattr__(self, "trace", _readonly(self.trace, 3))
        object.__setattr__(self, "consensus_xy", _readonly(self.consensus_xy, 2))
        object.__setattr__(self, "votes", _readonly(self.votes).reshape(-1))
        if len(self.votes) != len(self.consensus_xy) or np.any(self.votes < 1):
            raise ValueError("snapshot votes do not match consensus")
        if (isinstance(self.source_frontier_ns, bool) or not isinstance(self.source_frontier_ns, Integral)
                or not 0 < self.source_frontier_ns < 2 ** 63):
            raise ValueError("snapshot must have a positive source frontier")
        copied = json.loads(json.dumps(dict(self.stats), allow_nan=False))
        object.__setattr__(self, "stats", MappingProxyType(copied))
        if self.boundary_reference is not None and (
                not isinstance(self.boundary_reference, BoundaryReference)
                or self.boundary_reference.source_frontier_ns != self.source_frontier_ns):
            raise ValueError("snapshot boundary evidence must share its committed source frontier")


@dataclass(frozen=True)
class _PreparedKeyframe:
    matcher: object
    frontier_ns: int
    prepared_stamp_ns: int
    prior_xyyaw: np.ndarray
    tracking_owner: object
    tracking_stamp_ns: object
    direction_owner: object
    direction_source_stamp_ns: object
    bootstrap_motion_owner: object = None


class TeachMapEngine:
    def __init__(self, config=None, matcher_config=None, clock=None):
        self.config = dict(DEFAULT_CONFIG)
        if config:
            if set(config) - set(self.config):
                raise ValueError("unknown teach-map configuration")
            self.config.update(config)
        integer = {"minimum_frame_votes", "minimum_reliable_votes", "max_input_points",
                   "max_candidate_cells", "max_confirmed_cells", "max_trace_points",
                   "bootstrap_min_anchor_points", "bootstrap_max_frames", "keyframe_max_points"}
        for name, value in self.config.items():
            if isinstance(value, bool) or not isinstance(value, (int, float, np.number)) or not math.isfinite(float(value)) or value <= 0:
                raise ValueError("invalid teach-map setting: " + name)
            if name in integer:
                if int(value) != value:
                    raise ValueError("teach-map count must be integral: " + name)
                self.config[name] = int(value)
        cfg = self.config
        if (cfg["minimum_reliable_votes"] > cfg["minimum_frame_votes"]
                or cfg["reliable_sensitivity"] > cfg["max_sensitivity"]
                or cfg["keyframe_max_points"] > 100000
                or cfg["bootstrap_min_anchor_points"] > cfg["keyframe_max_points"]
                or cfg["max_confirmed_cells"] > 100000
                or cfg["max_input_points"] > 200000):
            raise ValueError("inconsistent or unbounded teach-map configuration")
        self.matcher_config = dict(matcher_config or {})
        self._geometry_config = _configuration(self.matcher_config)
        self._clock = clock or time.perf_counter
        self._map = SparseConsensusMap(cfg["grid_resolution_m"], cfg["minimum_frame_votes"],
                                       cfg["minimum_reliable_votes"], cfg["max_candidate_cells"],
                                       cfg["max_confirmed_cells"], cfg["candidate_ttl_s"],
                                       measured_geometry=True)
        self._trace = []
        self._matcher = None
        self._anchor_frontier = None
        self._keyframe_stamp = None
        self._keyframe_pose = None
        self._candidate = None
        self._first_stamp = None
        self._first_pose = None
        self._last_seen_stamp = None
        self._last_motion_source_stamp = None
        # Same-session raw bootstrap evidence, not a visual localization or a
        # general external-map seed. Advance only with the guarded map commit.
        self._bootstrap_motion_information = None
        self._boundary_map = None
        self._tracking_started = False
        self._snapshot = None
        self._past_direction_support = None
        self._past_direction_support_stamp_ns = None
        self._fault = None
        self.counts = Counter()

    @property
    def phase(self):
        return "invalid" if self._fault else "frozen" if self._snapshot is not None else "tracking" if self._tracking_started else "bootstrap"

    def _result(self, accepted, reason, pose, stamp, started, metrics=None):
        self.counts["accepted" if accepted else "rejected"] += 1
        self.counts[reason] += 1
        details = dict(metrics or {})
        attempts = details.get("reference_attempts", [])
        matched_frontier = details.get("selected_anchor_frontier_ns",
                                       attempts[-1]["frontier_ns"] if attempts else None)
        return {"accepted": bool(accepted), "reason": reason, "phase": self.phase,
                "pose_xyyaw": np.asarray(pose, dtype=float).copy(), "stamp_ns": int(stamp),
                "matched_against_frontier_ns": matched_frontier,
                "active_anchor_frontier_ns": self._anchor_frontier,
                "confirmed_cells": len(self._map.confirmed_keys), "trace_points": len(self._trace),
                "processing_ms": (self._clock() - started) * 1000.0,
                "metrics": details, "counts": dict(self.counts)}

    def _latch(self, reason):
        self._fault = str(reason)

    def _prepare_matcher(self, prior, stamp):
        frontier = self._map.last_stamp_ns
        if frontier is None or frontier >= stamp:
            raise ValueError("keyframe contains current or future evidence")
        if self._past_direction_support is not None and not (
                self._past_direction_support_stamp_ns is not None
                and self._past_direction_support_stamp_ns <= frontier):
            raise ValueError("keyframe direction contains uncommitted evidence")
        tracking_stamp = None if self._matcher is None else self._matcher.last_accepted_stamp_ns
        bootstrap_motion = self._bootstrap_motion_information if self._matcher is None else None
        if bootstrap_motion is not None and bootstrap_motion.source_stamp_ns > frontier:
            raise ValueError("bootstrap motion contains uncommitted or future map evidence")
        # Index preparation can exceed a deadline. Cache only an inactive
        # candidate, and never reuse one seeded from a different committed pose.
        if (self._candidate is not None and self._candidate.frontier_ns == frontier
                and self._candidate.tracking_owner is self._matcher
                and self._candidate.tracking_stamp_ns == tracking_stamp
                and self._candidate.direction_owner is self._past_direction_support
                and self._candidate.direction_source_stamp_ns == self._past_direction_support_stamp_ns
                and self._candidate.bootstrap_motion_owner is bootstrap_motion):
            return self._candidate
        if self._boundary_map is None:
            points = self._map.confirmed_points_and_votes()[0]
            distance = np.sum((points - prior[:2]) ** 2, axis=1)
            available = np.flatnonzero(distance <= self.config["keyframe_radius_m"] ** 2)
            if len(available) < self.config["bootstrap_min_anchor_points"]:
                return None
            order = np.argsort(distance[available], kind="stable")[:self.config["keyframe_max_points"]]
            anchor = points[available[order]].copy()
            matcher = (LaneMapLocalizer(anchor, self.matcher_config, self._past_direction_support,
                                       self._past_direction_support_stamp_ns) if self._matcher is None
                       else self._matcher.fork_with_anchor(anchor, self._past_direction_support,
                                                           self._past_direction_support_stamp_ns))
        else:
            boundary = self._boundary_map.snapshot_near(
                prior[:2], self.config["keyframe_radius_m"], self.config["keyframe_max_points"])
            if boundary.source_frontier_ns != frontier:
                raise ValueError("boundary and filled map frontiers differ")
            if len(boundary.points_xy) < self.config["bootstrap_min_anchor_points"]:
                return None
            anchor = boundary
            matcher = (BoundaryLaneLocalizer(anchor, self.matcher_config) if self._matcher is None
                       else self._matcher.fork_with_anchor(anchor))
        self._candidate = _PreparedKeyframe(matcher, int(frontier), stamp, prior.copy(), self._matcher,
                                            tracking_stamp, self._past_direction_support,
                                            self._past_direction_support_stamp_ns, bootstrap_motion)
        self.counts["keyframes_prepared"] += 1
        return self._candidate

    def _deadline(self, started):
        return (self._clock() - started) * 1000.0 > self.config["max_processing_ms"]

    def _match_reference(self, scan, prior, stamp, started, scan_support, scan_sensitivity,
                         motion_source_stamp_ns=None, boundary_observation=None):
        refresh = (self._matcher is None or (
            self._map.last_stamp_ns != self._anchor_frontier and (
                stamp - self._keyframe_stamp >= self.config["keyframe_refresh_s"] * 1e9
                or np.linalg.norm(prior[:2] - self._keyframe_pose[:2]) >= self.config["keyframe_refresh_distance_m"])))
        candidate = self._prepare_matcher(prior, stamp) if refresh else None
        attempts = []
        selected = candidate.matcher if candidate is not None else self._matcher
        frontier = candidate.frontier_ns if candidate is not None else self._anchor_frontier
        if selected is None:
            return None, None, None, {}, "insufficient_past_local_anchor"
        # Reuse the existing pre-match deadline read for diagnostics. This is
        # elapsed engine time (including keyframe preparation), not match time
        # and not proof that a reference was actually attempted.
        pre_match_ms = (self._clock() - started) * 1000.0
        timing = {"pre_match_engine_ms": pre_match_ms}
        if pre_match_ms > self.config["max_processing_ms"]:
            selected.discard_pending()
            return None, None, None, timing, "processing_deadline"
        motion_kwargs = ({} if motion_source_stamp_ns is None else
                         {"motion_source_stamp_ns": motion_source_stamp_ns})
        if boundary_observation is not None:
            motion_kwargs["boundary_observation"] = boundary_observation
        if self._matcher is None and motion_source_stamp_ns is not None:
            if (candidate is None or candidate.bootstrap_motion_owner is None
                    or candidate.bootstrap_motion_owner is not self._bootstrap_motion_information):
                return None, None, None, timing, "missing_bootstrap_motion_information"
            motion_kwargs["bootstrap_motion_information"] = candidate.bootstrap_motion_owner
        prior_stamp = stamp if motion_source_stamp_ns is None else motion_source_stamp_ns
        result = selected.match(scan, prior, stamp, prior_stamp_ns=prior_stamp, commit=False,
                                scan_support_xy=scan_support, scan_sensitivity=scan_sensitivity, **motion_kwargs)
        attempts.append({"reference": "candidate" if candidate is not None else "active",
                         "frontier_ns": frontier, "accepted": result.accepted,
                         "reason": result.reason, "metrics": dict(result.metrics)})
        if (not result.accepted and candidate is not None and self._matcher is not None
                and result.reason in _REFERENCE_REJECTIONS):
            if self._deadline(started):
                return None, None, None, dict(timing, reference_attempts=attempts), "processing_deadline"
            selected, frontier = self._matcher, self._anchor_frontier
            result = selected.match(scan, prior, stamp, prior_stamp_ns=prior_stamp, commit=False,
                                    scan_support_xy=scan_support, scan_sensitivity=scan_sensitivity, **motion_kwargs)
            attempts.append({"reference": "active", "frontier_ns": frontier,
                             "accepted": result.accepted, "reason": result.reason,
                             "metrics": dict(result.metrics)})
        metrics = dict(result.metrics, **timing, reference_attempts=attempts,
                       selected_anchor_frontier_ns=frontier,
                       selected_reference="candidate" if candidate is not None and selected is candidate.matcher else "active")
        failure = None if result.accepted else "match_" + result.reason
        promoted = candidate if candidate is not None and selected is candidate.matcher else None
        return selected, result, promoted, metrics, failure

    def _capacity_allows(self, keys, reliable):
        # Conservative preflight: expiry may free space, but never depend on it
        # to admit a transaction. A failed preflight changes no geometry.
        candidate = self._map.candidate_votes
        if len(candidate) + sum(int(k) not in candidate for k in keys) > self.config["max_candidate_cells"]:
            return False
        reliable_set = set(map(int, reliable))
        prospective = sum(int(k) not in self._map.confirmed_keys
                          and candidate.get(int(k), 0) + 1 >= self.config["minimum_frame_votes"]
                          and self._map.candidate_reliable_votes.get(int(k), 0) + (int(k) in reliable_set)
                          >= self.config["minimum_reliable_votes"] for k in keys)
        return len(self._map.confirmed_keys) + prospective <= self.config["max_confirmed_cells"]

    def process(self, local_xy, sensitivity, prior_xyyaw, stamp_ns, commit_guard=None,
                motion_source_stamp_ns=None, *, boundary_observation=None):
        # A proposal must not survive an exception during geometry preparation
        # or admission. Both possible owners remain reachable here, including
        # after candidate promotion. Successful commit already clears pending;
        # this is idempotent cleanup, not rollback of persistent map stores.
        try:
            return self._process_observation(
                local_xy, sensitivity, prior_xyyaw, stamp_ns, commit_guard,
                motion_source_stamp_ns, boundary_observation=boundary_observation)
        finally:
            if self._matcher is not None:
                self._matcher.discard_pending()
            if self._candidate is not None:
                self._candidate.matcher.discard_pending()

    def _process_observation(self, local_xy, sensitivity, prior_xyyaw, stamp_ns, commit_guard=None,
                             motion_source_stamp_ns=None, *, boundary_observation=None):
        started = self._clock()
        prior = np.asarray(prior_xyyaw, dtype=float)
        points, sensitivity = np.asarray(local_xy, dtype=float), np.asarray(sensitivity, dtype=float)
        if (prior.shape != (3,) or not np.all(np.isfinite(prior)) or np.max(np.abs(prior[:2])) > 1e6
                or points.ndim != 2 or points.shape[1] != 2 or len(points) > self.config["max_input_points"]
                or sensitivity.shape != (len(points),) or not np.all(np.isfinite(points))
                or np.any(np.abs(points) > 1000.)
                or not np.all(np.isfinite(sensitivity)) or np.any(sensitivity < 0)
                or isinstance(stamp_ns, bool) or int(stamp_ns) != stamp_ns or stamp_ns <= 0):
            raise ValueError("invalid teach-map observation")
        stamp = int(stamp_ns)
        if self._snapshot is not None:
            raise ValueError("teach map is frozen")
        if self._fault:
            return self._result(False, self._fault, prior, stamp, started)
        if self._last_seen_stamp is not None and stamp <= self._last_seen_stamp:
            self._latch("nonincreasing_mask_stamp")
            return self._result(False, self._fault, prior, stamp, started)
        self._last_seen_stamp = stamp
        if boundary_observation is not None and (
                not isinstance(boundary_observation, BoundaryObservation)
                or boundary_observation.source_stamp_ns != stamp):
            return self._result(False, "invalid_boundary_observation", prior, stamp, started)
        if self._trace and ((boundary_observation is None) != (self._boundary_map is None)):
            return self._result(False, "changed_feature_representation", prior, stamp, started)
        if motion_source_stamp_ns is None:
            if self._last_motion_source_stamp is not None:
                return self._result(False, "missing_motion_source_stamp", prior, stamp, started)
        elif (isinstance(motion_source_stamp_ns, bool) or not isinstance(motion_source_stamp_ns, Integral)
              or not 0 < motion_source_stamp_ns < 2 ** 63 or motion_source_stamp_ns > stamp
              or (self._last_motion_source_stamp is not None
                  and motion_source_stamp_ns <= self._last_motion_source_stamp)):
            return self._result(False, "invalid_or_nonincreasing_motion_source_stamp", prior, stamp, started)
        if motion_source_stamp_ns is not None:
            motion_source_stamp_ns = int(motion_source_stamp_ns)
            if self._trace and self._last_motion_source_stamp is None:
                # Already-voted legacy geometry has no committed motion prior.
                # Do not invent its starting covariance partway through TEACH.
                return self._result(False, "missing_bootstrap_motion_information", prior, stamp, started)
        if self._first_stamp is None:
            self._first_stamp, self._first_pose = stamp, prior.copy()
        if stamp - self._first_stamp > self.config["max_duration_s"] * 1e9:
            self._latch("teach_duration_capacity")
            return self._result(False, self._fault, prior, stamp, started)
        valid = sensitivity <= self.config["max_sensitivity"]
        points, sensitivity = points[valid], sensitivity[valid]
        if not len(points):
            return self._result(False, "empty_reliable_projection", prior, stamp, started)
        if len(self._trace) >= self.config["max_trace_points"]:
            self._latch("trace_capacity")
            return self._result(False, self._fault, prior, stamp, started)
        anchor_count = (len(self._map.confirmed_keys) if self._boundary_map is None
                        else self._boundary_map.confirmed_count)
        established = anchor_count >= self.config["bootstrap_min_anchor_points"]
        if established:
            self._tracking_started = True
        result = None
        selected = candidate = None
        metrics = {}
        pose, reason = prior, "bootstrap_odometry"
        if self._tracking_started:
            selected, result, candidate, metrics, failure = self._match_reference(
                points[sensitivity <= self.config["reliable_sensitivity"]], prior, stamp, started, points,
                sensitivity[sensitivity <= self.config["reliable_sensitivity"]], motion_source_stamp_ns,
                boundary_observation)
            if failure is not None:
                return self._result(False, failure, prior, stamp, started, metrics)
            pose, reason = result.pose_xyyaw, "matched_past_consensus"
        elif (self.counts["accepted"] >= self.config["bootstrap_max_frames"]
              or stamp - self._first_stamp > self.config["bootstrap_max_duration_s"] * 1e9
              or np.linalg.norm(prior[:2] - self._first_pose[:2]) > self.config["bootstrap_max_distance_m"]):
            self._latch("bootstrap_anchor_not_established")
            return self._result(False, self._fault, prior, stamp, started)
        prepared_bootstrap_motion = None
        if result is None and motion_source_stamp_ns is not None:
            try:
                if self._bootstrap_motion_information is None:
                    bootstrap_dt = None
                    covariance = np.diag([
                        self._geometry_config["motion_initial_lateral_sigma_m"] ** 2,
                        self._geometry_config["motion_initial_yaw_sigma_rad"] ** 2])
                else:
                    covariance, bootstrap_dt = _motion_prediction(
                        self._bootstrap_motion_information, prior, 0.,
                        motion_source_stamp_ns, self._geometry_config)
                prepared_bootstrap_motion = MotionInformationState(
                    motion_source_stamp_ns, tuple(prior), tuple(map(tuple, covariance)))
            except (ValueError, FloatingPointError, np.linalg.LinAlgError):
                return self._result(False, "invalid_bootstrap_motion_information", prior, stamp, started)
            metrics["bootstrap_motion_proposal"] = {
                "initialization": self._bootstrap_motion_information is None,
                "previous_source_stamp_ns": (None if self._bootstrap_motion_information is None else
                                              self._bootstrap_motion_information.source_stamp_ns),
                "source_stamp_ns": motion_source_stamp_ns,
                "interval_s": bootstrap_dt,
                "covariance": covariance.tolist(),
                "raw_prior_xyyaw": list(prepared_bootstrap_motion.raw_prior_xyyaw),
                "publication": "only_with_accepted_guarded_map_commit",
            }
        world = transform_local_points(points, *pose)
        # Prepare the bounded immutable context before admission. Commit only
        # swaps this pointer; never allocate/copy the full image under its lock.
        geometry = self._geometry_config
        support = _voxel_means(world[np.linalg.norm(points, axis=1) <= geometry["max_scan_range_m"]],
                               geometry["scan_voxel_m"])
        if len(support) > geometry["max_scan_support_points"]:
            if selected is not None:
                selected.discard_pending()
            return self._result(False, "past_direction_capacity", prior, stamp, started, metrics)
        prepared_support = _readonly(support, 2)
        prepared_geometry = self._map.prepare_geometry(world, stamp)
        keys = prepared_geometry.keys
        reliable = self._map.points_to_unique_keys(world[sensitivity <= self.config["reliable_sensitivity"]])
        if not self._capacity_allows(keys, reliable):
            if selected is not None:
                selected.discard_pending()
            self._latch("map_capacity")
            return self._result(False, self._fault, prior, stamp, started, metrics)
        boundary_map, prepared_boundary = self._boundary_map, None
        if boundary_observation is not None:
            if boundary_map is None:
                boundary_settings = {key: self.config[key] for key in (
                    "grid_resolution_m", "minimum_frame_votes", "minimum_reliable_votes",
                    "max_sensitivity", "reliable_sensitivity", "max_candidate_cells",
                    "max_confirmed_cells", "candidate_ttl_s")}
                boundary_settings["normal_max_angle_rad"] = geometry["normal_max_angle_rad"]
                boundary_map = BoundaryConsensus(boundary_settings)
            try:
                prepared_boundary = boundary_map.prepare(boundary_observation.transformed(pose))
            except ValueError as error:
                if selected is not None:
                    selected.discard_pending()
                metrics["boundary_preparation_error"] = str(error)
                return self._result(False, "boundary_preparation_rejected", prior, stamp, started, metrics)
            metrics["feature_representation"] = "paint_boundary_v1"
            metrics["boundary_reference_points_before_commit"] = boundary_map.confirmed_count
        with (commit_guard() if commit_guard is not None else nullcontext(True)) as admitted:
            if admitted is False:
                if selected is not None:
                    selected.discard_pending()
                return self._result(False, "commit_guard_rejected", prior, stamp, started, metrics)
            if self._deadline(started):
                if selected is not None:
                    selected.discard_pending()
                return self._result(False, "processing_deadline", prior, stamp, started, metrics)
            if prepared_boundary is not None and not boundary_map.can_commit(prepared_boundary):
                if selected is not None:
                    selected.discard_pending()
                return self._result(False, "boundary_commit_token_rejected", prior, stamp, started, metrics)
            if result is not None and not selected.commit(result):
                selected.discard_pending()
                return self._result(False, "matcher_commit_rejected", prior, stamp, started, metrics)
            self._map.update(keys, reliable, stamp_ns=stamp, geometry=prepared_geometry)
            if self._map.capacity_rejected_cells or self._map.confirmation_capacity_rejections:
                self._latch("unexpected_map_capacity")
                return self._result(False, self._fault, prior, stamp, started, metrics)
            if prepared_boundary is not None:
                # The single worker and the guard own both frontiers. The
                # prepared token was validated before any persistent mutation;
                # commit applies its already-built immutable records. This is
                # serialized admission, not exception-rollback across stores.
                if not boundary_map.commit(prepared_boundary):
                    self._latch("unexpected_boundary_commit_failure")
                    return self._result(False, self._fault, prior, stamp, started, metrics)
                self._boundary_map = boundary_map
            self._trace.append(np.asarray(pose, dtype=float).copy())
            self._last_motion_source_stamp = motion_source_stamp_ns
            if prepared_bootstrap_motion is not None:
                self._bootstrap_motion_information = prepared_bootstrap_motion
            # The entire preceding stable observation supplies paint direction
            # only, never an unconfirmed reference position. Publish it in the
            # same guarded transaction; a rejected/current frame cannot help
            # construct the anchor used to validate itself.
            self._past_direction_support = prepared_support
            self._past_direction_support_stamp_ns = stamp
            if candidate is not None:
                # Promotion shares the same generation/deadline guard as the
                # sole map vote and trace append. Failed candidates stay inert.
                self._matcher, self._anchor_frontier = candidate.matcher, candidate.frontier_ns
                self._keyframe_stamp, self._keyframe_pose = candidate.prepared_stamp_ns, candidate.prior_xyyaw
                self._candidate = None
                self.counts["keyframes_promoted"] += 1
        return self._result(True, reason, pose, stamp, started, metrics)

    def points_and_votes(self):
        """Return owned copies for logging/visualization, not a writable map view."""
        return self._map.confirmed_points_and_votes()

    consensus_points_and_votes = points_and_votes

    def freeze(self):
        if self._fault:
            raise ValueError("cannot finish invalid teach map: " + self._fault)
        if self._snapshot is None:
            if not self._trace or self._map.last_stamp_ns is None:
                raise ValueError("cannot freeze an empty teach map")
            points, votes = self._map.confirmed_points_and_votes()
            self._snapshot = MapSnapshot(np.asarray(self._trace), points, votes,
                                         int(self._map.last_stamp_ns), dict(self.counts,
                                             consensus_coordinate_semantics="measured_frame_equal_mean",
                                             consensus_grid_resolution_m=self.config["grid_resolution_m"],
                                             consensus_vote_semantics="one_vote_per_cell_per_committed_frame"),
                                         None if self._boundary_map is None else self._boundary_map.snapshot())
        return self._snapshot

    def capture_repeat_state(self):
        """Freeze and capture a small immutable continuous-session handoff.

        Call freeze first when even the one-time snapshot copy must be scheduled
        outside a control callback. No full-map localization index is built here.
        """
        self.freeze()
        if self._matcher is not None:
            return self._matcher.capture_tracking_state()
        # Bootstrap has no committed visual correction or timestamp fence to
        # transfer. An empty matcher preserves the original fresh semantics.
        return LaneMapLocalizer(np.empty((0, 2)), self.matcher_config).capture_tracking_state()

    def repeat_localizer(self):
        """Hand off this continuous session to an independent frozen-map matcher.

        The raw odometry frame does not reset between teach and repeat. Preserve
        only the active matcher's committed correction and timestamp fences;
        an unpromoted candidate, pending proposal or refinement is not evidence
        for the handoff. This is not an API for seeding another session's map.
        """
        snapshot = self.freeze()
        if snapshot.boundary_reference is not None:
            return BoundaryLaneLocalizer.from_tracking_state(
                snapshot.boundary_reference, self.capture_repeat_state())
        return LaneMapLocalizer.from_tracking_state(snapshot.consensus_xy, self.capture_repeat_state())
