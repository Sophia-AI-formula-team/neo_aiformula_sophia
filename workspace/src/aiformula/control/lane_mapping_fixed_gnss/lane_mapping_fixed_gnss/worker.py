"""Single-owner map computation with one running and one latest pending job.

The ROS thread supplies a short locked admission context for actual commits.
Slow projection, matching, bundle creation and map export never run in the
command timer. Results are bounded; a full result queue stops further work.
"""
import queue
from dataclasses import replace
import threading
import time

import numpy as np

from lane_mapping_lya_reference.mapping_core import GroundLookup
from lane_mapping_lya_reference.tilted_projection import TiltedGroundLookup
from lane_mapping_lya_reference.paint_geometry import extract_boundary_observation
from .attitude import AttitudeSnapshot
from .mapping import TeachMapEngine
from .bundle import build_bundle, geometry_sha256, load_bundle


class MapWorker:
    def __init__(self, guard, mapping_config=None, matcher_config=None,
                 motion_attitude_mode="legacy_flat"):
        if motion_attitude_mode not in ("legacy_flat", "raw_imu_tilt"):
            raise ValueError("unknown motion attitude mode")
        self.motion_attitude_mode = motion_attitude_mode
        self.guard = guard
        self.engine = TeachMapEngine(config=mapping_config, matcher_config=matcher_config)
        self.matcher_config = matcher_config
        self.jobs = queue.Queue(1)
        self.results = queue.Queue(4)
        self.closing = threading.Event()
        self.lookup = None
        self.calibration = None
        self.matcher = None
        self.snapshot = None
        self.failure = None
        self.last_export_ns = 0
        self.thread = threading.Thread(target=self._run, daemon=True, name="endpoint-map")
        self.thread.start()

    def submit(self, job):
        """Only the latest queued mask survives. Phase changes invalidate old work."""
        try:
            self.jobs.put_nowait(job)
        except queue.Full:
            try:
                self.jobs.get_nowait()
            except queue.Empty:
                pass
            self.jobs.put_nowait(job)

    def take(self):
        try:
            return self.results.get_nowait()
        except queue.Empty:
            return None

    def _project(self, job):
        job.pop("boundary_observation", None)
        calibration = job["calibration"]
        if self.lookup is None:
            lookup_type = TiltedGroundLookup if self.motion_attitude_mode == "raw_imu_tilt" else GroundLookup
            self.lookup = lookup_type(calibration["width"], calibration["height"],
                calibration["camera_matrix"], np.asarray(calibration["base_from_camera"]), 0.5)
            self.calibration = calibration
        elif calibration != self.calibration:
            raise ValueError("calibration changed within session")
        if self.motion_attitude_mode == "raw_imu_tilt":
            attitude = job.get("attitude")
            if (not isinstance(attitude, AttitudeSnapshot)
                    or attitude.stamp_ns != job["stamp_ns"]
                    or attitude.source_stamp_ns != job.get("prior_gyro_stamp_ns")
                    or not attitude.source_stamp_ns <= job["prior_stamp_ns"] <= job["stamp_ns"]
                    or attitude.received_steady_ns > job.get("prior_receipt_ns", -1)
                    or attitude.wheel_receipt_ns != job.get("prior_receipt_ns")
                    or not attitude.wheel_receipt_ns <= job["receipt_ns"]):
                raise ValueError("raw-IMU projection requires the selected causal wheel attitude snapshot")
            local, sensitivity, _ = self.lookup.project_mask_with_sensitivity(
                job["mask"], 127, attitude.up_body)
            normal = attitude.up_body
        else:
            local, sensitivity, _ = self.lookup.project_mask_with_sensitivity(job["mask"], 127)
            normal = (0., 0., 1.)
        job["boundary_observation"] = extract_boundary_observation(
            job["mask"], self.lookup, normal, job["stamp_ns"], threshold=127)
        return local, sensitivity

    def _compute(self, job):
        kind = job["kind"]
        envelope = {"kind": kind, "job": job}
        started = time.monotonic_ns()
        job["compute_started_ns"] = started
        if kind == "freeze":
            self.snapshot = self.engine.freeze()
            envelope["snapshot"] = self.snapshot
        elif kind == "bundle":
            if self.snapshot is None:
                raise ValueError("map must be frozen before endpoint sampling")
            if self.snapshot is not self.engine.freeze():
                raise ValueError("bundle snapshot is not this continuous teach session")
            path = build_bundle(job["root"], self.snapshot, self.calibration,
                                route_config=job.get("route_config"))
            manifest, route, metadata, anchor, calibration = load_bundle(path)
            # Hash-valid files may still describe a different map. The live
            # correction belongs only to this exact frozen geometry/frontier,
            # never to an arbitrary loaded bundle or a restarted motion frame.
            if (not np.array_equal(anchor, self.snapshot.consensus_xy)
                    or metadata.get("snapshot_geometry_sha256") != geometry_sha256(self.snapshot)
                    or geometry_sha256(replace(self.snapshot,
                        boundary_reference=metadata.get("boundary_reference"))) != geometry_sha256(self.snapshot)
                    or metadata.get("source_frontier_ns") != self.snapshot.source_frontier_ns):
                raise ValueError("loaded bundle does not match this frozen teach snapshot")
            self.matcher = self.engine.repeat_localizer()
            envelope.update(path=str(path), route=route, manifest=manifest)
        elif kind in ("teach", "repeat"):
            local, sensitivity = self._project(job)
            # The production projection always produces this immutable object.
            # Old isolated XY test fixtures may replace _project; they remain
            # legacy rather than inventing black/white image boundaries.
            boundary = job.pop("boundary_observation", None)
            boundary_kwargs = {} if boundary is None else {"boundary_observation": boundary}
            if kind == "teach":
                envelope["result"] = self.engine.process(local, sensitivity, job["prior"],
                    job["stamp_ns"], commit_guard=lambda: self.guard(job),
                    motion_source_stamp_ns=job["prior_stamp_ns"], **boundary_kwargs)
                if time.monotonic_ns() - self.last_export_ns > 500000000:
                    # A copy from the worker's own consensus, not an optimistic
                    # per-frame overlay presented as a confirmed map.
                    envelope["map"] = self.engine.consensus_points_and_votes()
                    self.last_export_ns = time.monotonic_ns()
            else:
                if self.matcher is None:
                    raise ValueError("repeat map is not loaded")
                result = self.matcher.match(local[sensitivity <= 0.1], job["prior"],
                    job["stamp_ns"], prior_stamp_ns=job["prior_stamp_ns"], commit=False,
                    scan_support_xy=local, scan_sensitivity=sensitivity[sensitivity <= 0.1],
                    motion_source_stamp_ns=job["prior_stamp_ns"], **boundary_kwargs)
                job["proposal_correction"] = tuple(float(v) for v in result.correction_se2)
                committed = False
                try:
                    with self.guard(job) as admitted:
                        if admitted and result.accepted:
                            committed = bool(self.matcher.commit(result))
                finally:
                    if not committed:
                        self.matcher.discard_pending()
                envelope["compute_ms"] = (time.monotonic_ns() - started) * 1e-6
                envelope.update(result=result, committed=committed)
                if committed and time.monotonic_ns() - self.last_export_ns > 500000000:
                    envelope["map"] = self.matcher.refined_points_and_votes()
                    self.last_export_ns = time.monotonic_ns()
        else:
            raise ValueError("unknown worker operation")
        envelope.setdefault("compute_ms", (time.monotonic_ns() - started) * 1e-6)
        return envelope

    def _run(self):
        while not self.closing.is_set():
            try:
                job = self.jobs.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                result = self._compute(job)
            except Exception as error:
                result = {"kind": "error", "job": job, "error": str(error)}
            try:
                self.results.put(result, timeout=0.1)
            except queue.Full:
                self.failure = "worker result queue full; control consumer unavailable"
                return

    def close(self):
        self.closing.set()
        self.thread.join(timeout=2.0)
