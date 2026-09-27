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
from lane_mapping_lya_reference.mask_localization import LaneMapLocalizer


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
        self._clock = clock or time.perf_counter
        self._map = SparseConsensusMap(cfg["grid_resolution_m"], cfg["minimum_frame_votes"],
                                       cfg["minimum_reliable_votes"], cfg["max_candidate_cells"],
                                       cfg["max_confirmed_cells"], cfg["candidate_ttl_s"])
        self._trace = []
        self._matcher = None
        self._anchor_frontier = None
        self._keyframe_stamp = None
        self._keyframe_pose = None
        self._first_stamp = None
        self._first_pose = None
        self._last_seen_stamp = None
        self._tracking_started = False
        self._snapshot = None
        self._fault = None
        self.counts = Counter()

    @property
    def phase(self):
        return "invalid" if self._fault else "frozen" if self._snapshot is not None else "tracking" if self._tracking_started else "bootstrap"

    def _result(self, accepted, reason, pose, stamp, started, metrics=None):
        self.counts["accepted" if accepted else "rejected"] += 1
        self.counts[reason] += 1
        return {"accepted": bool(accepted), "reason": reason, "phase": self.phase,
                "pose_xyyaw": np.asarray(pose, dtype=float).copy(), "stamp_ns": int(stamp),
                "matched_against_frontier_ns": self._anchor_frontier,
                "confirmed_cells": len(self._map.confirmed_keys), "trace_points": len(self._trace),
                "processing_ms": (self._clock() - started) * 1000.0,
                "metrics": dict(metrics or {}), "counts": dict(self.counts)}

    def _latch(self, reason):
        self._fault = str(reason)

    def _refresh_matcher(self, prior, stamp):
        points, _ = self._map.confirmed_points_and_votes()
        distance = np.sum((points - prior[:2]) ** 2, axis=1)
        available = np.flatnonzero(distance <= self.config["keyframe_radius_m"] ** 2)
        if len(available) < self.config["bootstrap_min_anchor_points"]:
            return False
        order = np.argsort(distance[available], kind="stable")[:self.config["keyframe_max_points"]]
        anchor = points[available[order]].copy()
        frontier = self._map.last_stamp_ns
        if frontier is None or frontier >= stamp:
            raise ValueError("keyframe contains current or future evidence")
        self._matcher = LaneMapLocalizer(anchor, self.matcher_config)
        self._anchor_frontier = int(frontier)
        self._keyframe_stamp, self._keyframe_pose = stamp, prior.copy()
        self.counts["keyframes_prepared"] += 1
        return True

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

    def process(self, local_xy, sensitivity, prior_xyyaw, stamp_ns, commit_guard=None):
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
        established = len(self._map.confirmed_keys) >= self.config["bootstrap_min_anchor_points"]
        if established:
            self._tracking_started = True
        result = None
        pose, reason = prior, "bootstrap_odometry"
        if self._tracking_started:
            # A prepared index remains useful after a deadline-rejected frame.
            # Do not repeatedly rebuild the SAME unchanged map if preparation
            # itself exceeded one frame's budget; the next fresh mask can match
            # against it cheaply. Newly admitted geometry makes refresh eligible.
            refresh = (self._matcher is None or (
                self._map.last_stamp_ns != self._anchor_frontier and (
                    stamp - self._keyframe_stamp >= self.config["keyframe_refresh_s"] * 1e9
                    or np.linalg.norm(prior[:2] - self._keyframe_pose[:2]) >= self.config["keyframe_refresh_distance_m"])))
            if refresh and not self._refresh_matcher(prior, stamp):
                return self._result(False, "insufficient_past_local_anchor", prior, stamp, started)
            result = self._matcher.match(points[sensitivity <= self.config["reliable_sensitivity"]],
                                         prior, stamp, prior_stamp_ns=stamp, commit=False)
            if not result.accepted:
                return self._result(False, "match_" + result.reason, prior, stamp, started, result.metrics)
            pose, reason = result.pose_xyyaw, "matched_past_consensus"
        elif (self.counts["accepted"] >= self.config["bootstrap_max_frames"]
              or stamp - self._first_stamp > self.config["bootstrap_max_duration_s"] * 1e9
              or np.linalg.norm(prior[:2] - self._first_pose[:2]) > self.config["bootstrap_max_distance_m"]):
            self._latch("bootstrap_anchor_not_established")
            return self._result(False, self._fault, prior, stamp, started)
        world = transform_local_points(points, *pose)
        keys = self._map.points_to_unique_keys(world)
        reliable = self._map.points_to_unique_keys(world[sensitivity <= self.config["reliable_sensitivity"]])
        if not self._capacity_allows(keys, reliable):
            self._latch("map_capacity")
            return self._result(False, self._fault, prior, stamp, started)
        with (commit_guard() if commit_guard is not None else nullcontext(True)) as admitted:
            if admitted is False:
                return self._result(False, "commit_guard_rejected", prior, stamp, started)
            if (self._clock() - started) * 1000.0 > self.config["max_processing_ms"]:
                return self._result(False, "processing_deadline", prior, stamp, started)
            if result is not None and not self._matcher.commit(result):
                return self._result(False, "matcher_commit_rejected", prior, stamp, started)
            self._map.update(keys, reliable, stamp_ns=stamp)
            if self._map.capacity_rejected_cells or self._map.confirmation_capacity_rejections:
                self._latch("unexpected_map_capacity")
                return self._result(False, self._fault, prior, stamp, started)
            self._trace.append(np.asarray(pose, dtype=float).copy())
        return self._result(True, reason, pose, stamp, started, None if result is None else result.metrics)

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
                                         int(self._map.last_stamp_ns), dict(self.counts))
        return self._snapshot

    def repeat_localizer(self):
        """Create an independent frozen-map matcher; refinement cannot edit teach geometry."""
        snapshot = self.freeze()
        return LaneMapLocalizer(snapshot.consensus_xy.copy(), self.matcher_config)
