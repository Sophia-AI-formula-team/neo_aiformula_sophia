"""Single-owner map computation with one running and one latest pending job.

The ROS thread supplies a short locked admission context for actual commits.
Slow projection, matching, bundle creation and map export never run in the
command timer. Results are bounded; a full result queue stops further work.
"""
import queue
import threading
import time

import numpy as np

from lane_mapping_lya_reference.mapping_core import GroundLookup
from lane_mapping_lya_reference.mask_localization import LaneMapLocalizer
from .mapping import TeachMapEngine
from .bundle import build_bundle, load_bundle


class MapWorker:
    def __init__(self, guard, mapping_config=None, matcher_config=None):
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
        calibration = job["calibration"]
        if self.lookup is None:
            self.lookup = GroundLookup(calibration["width"], calibration["height"],
                calibration["camera_matrix"], np.asarray(calibration["base_from_camera"]), 0.5)
            self.calibration = calibration
        elif calibration != self.calibration:
            raise ValueError("calibration changed within session")
        local, sensitivity, _ = self.lookup.project_mask_with_sensitivity(job["mask"], 127)
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
            path = build_bundle(job["root"], self.snapshot, self.calibration,
                                route_config=job.get("route_config"))
            manifest, route, metadata, anchor, calibration = load_bundle(path)
            self.matcher = LaneMapLocalizer(anchor, config=self.matcher_config)
            envelope.update(path=str(path), route=route, manifest=manifest)
        elif kind in ("teach", "repeat"):
            local, sensitivity = self._project(job)
            if kind == "teach":
                envelope["result"] = self.engine.process(local, sensitivity, job["prior"],
                    job["stamp_ns"], commit_guard=lambda: self.guard(job))
                if time.monotonic_ns() - self.last_export_ns > 500000000:
                    # A copy from the worker's own consensus, not an optimistic
                    # per-frame overlay presented as a confirmed map.
                    envelope["map"] = self.engine.consensus_points_and_votes()
                    self.last_export_ns = time.monotonic_ns()
            else:
                if self.matcher is None:
                    raise ValueError("repeat map is not loaded")
                result = self.matcher.match(local[sensitivity <= 0.1], job["prior"],
                    job["stamp_ns"], prior_stamp_ns=job["prior_stamp_ns"], commit=False)
                job["proposal_correction"] = tuple(float(v) for v in result.correction_se2)
                committed = False
                with self.guard(job) as admitted:
                    if admitted and result.accepted:
                        committed = bool(self.matcher.commit(result))
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
