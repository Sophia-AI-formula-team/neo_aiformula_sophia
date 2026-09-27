"""Exercise the actual bounded worker with real pure mapping/localization.

No ROS node, DDS, vehicle or future input is simulated as deployment proof.
The guard below models the ROS thread's short generation/deadline critical
section; production motion/admission checks are covered separately.
"""
import ast
from contextlib import contextmanager
from pathlib import Path
import sys
import threading
import time

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[2] / "lane_mapping_lya_reference"))

from lane_mapping_fixed_gnss.worker import MapWorker
from lane_mapping_lya_reference.mask_localization import LaneMapLocalizer


class Guard:
    def __init__(self):
        self.lock = threading.Lock()
        self.now = 10000000000
        self.generation = 1
        self.closed = False

    @contextmanager
    def __call__(self, job):
        with self.lock:
            yield (not self.closed and job["generation"] == self.generation
                   and self.now <= job["deadline_ns"])

    def revoke(self):
        with self.lock:
            self.generation += 1


def lanes():
    x = np.arange(.5, 8., .05)
    return np.vstack([np.column_stack((x, np.full(len(x), side))) for side in (-1.2, 1.2)])


def job(index=0, kind="teach"):
    stamp = 1000000000 + index * 50000000
    return dict(kind=kind, generation=1, stamp_ns=stamp, prior_stamp_ns=stamp - 1,
                receipt_ns=10000000000, deadline_ns=11000000000,
                prior=[0., 0., 0.], mask=np.zeros((2, 2), dtype=np.uint8), calibration={})


def wait_until(predicate, timeout=2):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.002)
    raise AssertionError("worker test timed out")


def take(worker):
    wait_until(lambda: not worker.results.empty() or worker.failure is not None)
    value = worker.take()
    assert value is not None, worker.failure
    return value


@pytest.fixture
def running():
    guard = Guard()
    worker = MapWorker(guard, mapping_config={"max_processing_ms": 2000.0},
                       matcher_config={"runtime_budget_ms": 2000.0})
    points = lanes()
    worker._project = lambda _: (points.copy(), np.full(len(points), .05))
    try:
        yield worker, guard
    finally:
        guard.closed = True
        worker.close()


def test_latest_only_queue_keeps_one_running_and_one_pending(running):
    worker, guard = running
    entered, release = threading.Event(), threading.Event()
    project = worker._project

    def pause(value):
        if value["stamp_ns"] == job()["stamp_ns"]:
            entered.set()
            assert release.wait(2)
        return project(value)

    worker._project = pause
    try:
        worker.submit(job())
        assert entered.wait(1)
        worker.submit(job(1))
        worker.submit(job(2))
        assert worker.jobs.qsize() == 1
        release.set()
        first, last = take(worker), take(worker)
        assert first["job"]["stamp_ns"] == job()["stamp_ns"]
        assert last["job"]["stamp_ns"] == job(2)["stamp_ns"]
        assert first["result"]["accepted"] and last["result"]["accepted"]
        assert len(worker.engine._trace) == 2
    finally:
        release.set()


@pytest.mark.parametrize("kind", ["teach", "repeat"])
@pytest.mark.parametrize("fault", ["revoked", "expired", "closed"])
def test_post_projection_revocation_or_expiration_cannot_mutate_geometry(running, kind, fault):
    worker, guard = running
    if kind == "repeat":
        worker.matcher = LaneMapLocalizer(lanes(), {"runtime_budget_ms": 2000.0})
        for stamp in (100, 200, 300):
            assert worker.matcher.match(lanes(), [0., .1, 0.], stamp).accepted
        old_correction = worker.matcher.correction_se2.copy()
        old_metrics = worker.matcher.refinement_metrics()
        old_points, old_votes = worker.matcher.refined_points_and_votes()
        old_commits = worker.matcher.committed_frames
    else:
        # Establish real consensus before revocation. The next frame performs
        # real scan matching, not merely an empty-map/bootstrap mock.
        for index in range(5):
            worker.submit(job(index))
            assert take(worker)["result"]["accepted"]
        old_points, old_votes = worker.engine.consensus_points_and_votes()
        old_candidates = dict(worker.engine._map.candidate_votes)
        old_trace = np.asarray(worker.engine._trace).copy()

    entered, release = threading.Event(), threading.Event()
    project = worker._project

    def pause(value):
        result = project(value)
        entered.set()
        assert release.wait(2)
        return result

    worker._project = pause
    try:
        value = job(6, kind)
        if kind == "repeat":
            value["prior"] = [0., .15, 0.]
        worker.submit(value)
        assert entered.wait(1)
        if fault == "revoked":
            guard.revoke()
        elif fault == "expired":
            guard.now = value["deadline_ns"] + 1
        else:
            guard.closed = True
        release.set()
        result = take(worker)
        assert result["kind"] == kind, result
        if kind == "repeat":
            assert result["result"].accepted and result["committed"] is False
            assert worker.matcher.committed_frames == old_commits
            assert worker.matcher.refinement_metrics() == old_metrics
            np.testing.assert_array_equal(worker.matcher.correction_se2, old_correction)
            new_points, new_votes = worker.matcher.refined_points_and_votes()
        else:
            assert result["result"]["reason"] == "commit_guard_rejected"
            assert not result["result"]["accepted"]
            assert worker.engine._map.candidate_votes == old_candidates
            np.testing.assert_array_equal(worker.engine._trace, old_trace)
            assert worker.engine._matcher.committed_frames == 0
            new_points, new_votes = worker.engine.consensus_points_and_votes()
        np.testing.assert_array_equal(new_points, old_points)
        np.testing.assert_array_equal(new_votes, old_votes)
    finally:
        release.set()


def test_valid_repeat_commits_and_refinement_is_a_separate_map(running):
    worker, _ = running
    worker.matcher = LaneMapLocalizer(lanes(), {"runtime_budget_ms": 2000.0})
    frozen = worker.matcher.anchor_points.copy()
    for index in range(3):
        value = job(index, "repeat")
        value["prior"] = [0., .1, 0.]
        worker.submit(value)
        result = take(worker)
        assert result["committed"] and result["result"].accepted
    assert worker.matcher.committed_frames == 3
    assert len(worker.matcher.refined_points_and_votes()[0]) > 0
    np.testing.assert_array_equal(worker.matcher.anchor_points, frozen)


def test_projection_failure_is_reported_and_never_fuses(running):
    worker, _ = running

    def broken(_):
        raise ValueError("invalid calibration")

    worker._project = broken
    worker.submit(job())
    result = take(worker)
    assert result["kind"] == "error" and "invalid calibration" in result["error"]
    assert not worker.engine._trace and not worker.engine._map.candidate_votes


def test_freeze_prevents_later_teach_mutation(running):
    worker, _ = running
    for index in range(5):
        worker.submit(job(index))
        assert take(worker)["result"]["accepted"]
    worker.submit(dict(kind="freeze", generation=1))
    frozen = take(worker)
    assert frozen["kind"] == "freeze"
    snapshot = frozen["snapshot"]
    before = snapshot.consensus_xy.copy()
    worker.submit(job(6))
    result = take(worker)
    assert result["kind"] == "error" and "frozen" in result["error"]
    np.testing.assert_array_equal(snapshot.consensus_xy, before)
    assert len(snapshot.trace) == 5


def test_full_results_queue_is_bounded_and_latches_worker_failure(running):
    worker, _ = running
    worker._compute = lambda value: {"kind": "test", "job": value}
    for index in range(4):
        worker.submit(job(index))
        wait_until(lambda: worker.results.qsize() == index + 1)
    worker.submit(job(4))
    wait_until(lambda: worker.failure is not None)
    assert "result queue full" in worker.failure
    assert worker.results.qsize() == 4
    worker.thread.join(timeout=1)
    assert not worker.thread.is_alive()


@pytest.mark.parametrize("name", ["worker.py", "node.py", "motion.py", "anchors.py", "mapping.py", "bundle.py"])
def test_runtime_sources_parse_as_python38(name):
    path = Path(__file__).parents[1] / "lane_mapping_fixed_gnss" / name
    ast.parse(path.read_text(encoding="utf-8"), feature_version=8)
