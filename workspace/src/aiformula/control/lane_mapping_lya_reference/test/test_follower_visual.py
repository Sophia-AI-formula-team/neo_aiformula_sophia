"""Actual follower method bodies with only ROS construction replaced.

This verifies causal/worker control flow, not DDS scheduling or hardware timing.
"""
import ast
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
import math
from pathlib import Path
import queue
import threading
import time
from types import SimpleNamespace as Obj
import uuid

import numpy as np
import pytest

from lane_mapping_lya_reference.controller import (ClosedRouteController, Command,
    Pose, SafetyArbiter, fresh, load_bundle)
from lane_mapping_lya_reference.mapping_core import CausalSampleBuffer, GroundLookup, VectorNavLocalizer, make_transform
from lane_mapping_lya_reference.mask_localization import LaneMapLocalizer


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def header(ns=0, frame="camera"):
    return Obj(stamp=Obj(sec=ns // 1000000000, nanosec=ns % 1000000000), frame_id=frame)


def pose_message():
    return Obj(header=header(), pose=Obj(position=Obj(x=0., y=0., z=0.),
                orientation=Obj(x=0., y=0., z=0., w=0.)))


def vn_message(ns):
    return Obj(header=header(ns), group_fields=0x10C8,
        position=Obj(x=35., y=139., z=20.), velocity=Obj(x=0., y=0., z=0.),
        yawpitchroll=Obj(x=90., y=0., z=0.),
        insstatus=Obj(mode=2, gps_fix=True, time_error=False, imu_error=False, gps_error=False))


def mask(ns, width=160, height=120, frame="camera"):
    return Obj(header=header(ns, frame), width=width, height=height, step=width,
               encoding="mono8", data=bytes([255]) * width * height)


class FakeWorker:
    def __init__(self):
        self.submitted, self.acknowledged, self.results = [], [], []
        self.latest_refined = None
        self.last_proposal = None
        self.auto_commit = True
        self.authorizations = []

    def submit(self, job):
        self.submitted.append(job)
        return False

    def acknowledge(self, identity, accepted, **authorization):
        self.acknowledged.append((identity, accepted))
        self.authorizations.append(authorization)
        if accepted and self.auto_commit:
            self.results.append(dict(kind="committed", job=self.last_proposal["job"],
                                     result=self.last_proposal["result"]))

    def take_result(self):
        result = self.results.pop(0) if self.results else None
        if result and result["kind"] == "proposal":
            self.last_proposal = result
        return result


@pytest.fixture
def module():
    source = Path(__file__).parents[1] / "lane_mapping_lya_reference" / "follower_node.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    tree.body = [item for item in tree.body if isinstance(item, (ast.ClassDef, ast.FunctionDef))]
    namespace = dict(Node=object, np=np, csv=csv, hashlib=hashlib, io=io,
        json=json, math=math, Path=Path, queue=queue, threading=threading, time=time,
        uuid=uuid, datetime=datetime, timezone=timezone, CausalSampleBuffer=CausalSampleBuffer,
        GroundLookup=GroundLookup, VectorNavLocalizer=VectorNavLocalizer,
        LaneMapLocalizer=LaneMapLocalizer, ClosedRouteController=ClosedRouteController,
        Command=Command, Pose=Pose, SafetyArbiter=SafetyArbiter, fresh=fresh,
        load_bundle=load_bundle, PoseStamped=pose_message,
        Header=header, PointCloud2=Obj, PointField=type("PointField", (), {
            "FLOAT32": 7, "__init__": lambda self, **kwargs: self.__dict__.update(kwargs)}),
        String=Obj, Bool=Obj, Twist=lambda: Obj(linear=Obj(x=0), angular=Obj(z=0)))
    exec(compile(tree, str(source), "exec"), namespace)
    return namespace


@pytest.fixture
def harness(module):
    cls = module["FixedFollower"]
    source = Path(__file__).parents[1] / "lane_mapping_lya_reference" / "follower_node.py"
    parsed = ast.parse(source.read_text(encoding="utf-8"))
    definition = next(item for item in parsed.body if isinstance(item, ast.ClassDef) and item.name == "FixedFollower")
    init = next(item for item in definition.body if isinstance(item, ast.FunctionDef) and item.name == "__init__")
    defaults = next(item.value for item in init.body if isinstance(item, ast.Assign)
                    and any(isinstance(target, ast.Name) and target.id == "defaults" for target in item.targets))
    node = cls.__new__(cls)
    node.p = ast.literal_eval(defaults)
    node.arbiter = SafetyArbiter()
    node.arbiter.repeat_selected, node.arbiter.state = True, "HOLD"
    node.clock = Obj(ros=1000000000, steady=1000000000)
    node._times = lambda: (node.clock.ros, node.clock.steady)
    node.manifest = {"frame_id": "lane_map"}
    node.repeat_context = {"camera": {"width": 160, "height": 120, "frame_id": "camera", "stamp_ns": 0}}
    node.visual_commit_fence = module["VisualCommitFence"]()
    node.visual_generation, node.visual_job_id = node.visual_commit_fence.advance(), 0
    node.visual_last_mask_stamp = node.visual_last_accepted_stamp = node.visual_last_arrival_steady_ns = None
    node.visual_streak, node.visual_pending_replacements = 0, 0
    node.visual_correction = np.zeros(3)
    node.visual_metrics, node.visual_reason = {}, "waiting"
    node.visual_last_rejection = node.visual_last_refined = None
    node.raw_pose = Pose(0, 0, 0, 0, node.clock.ros, node.clock.steady)
    node.pose_error = None
    node.pose_buffer = CausalSampleBuffer(512)
    node.arrival_seq = 10
    node.localizer = VectorNavLocalizer()
    node.mask_worker = FakeWorker()
    node.events, node.commands = [], []
    node.journal = Obj(write=lambda item: node.events.append(item))
    node._send = lambda command: node.commands.append(command)
    node._publish_state = lambda: None
    node.teacher_stopped = False
    node.teacher_state_source_ns = node.teacher_state_received_ns = 0
    node.repeat_prepare_ros_ns = node.repeat_prepare_ns = 0
    for name in ("pose_publisher", "raw_pose_publisher", "visual_publisher", "refined_publisher"):
        setattr(node, name, Publisher())
    return node


def proposal(node, source=None, accepted=True, correction=(0.2, -0.1, 0.03), identity=1):
    source = node.clock.ros if source is None else source
    job = dict(job_id=identity, generation=node.visual_generation, context=node.repeat_context,
               source_ns=source, received_steady_ns=node.clock.steady,
               prior_stamp_ns=source - 1, prior=np.zeros(3))
    result = Obj(accepted=accepted, reason="matched" if accepted else "low_overlap",
                 metrics={"residual_rmse_m": .02, "inliers": 60, "observable_rank": 2},
                 pose_xyyaw=np.array([.2, -.1, .03]), correction_se2=np.asarray(correction))
    return dict(kind="proposal", job=job, result=result, error=None, compute_ms=10,
                projected_pixels=100)


def trusted(node):
    for index in range(3):
        node.clock.ros = node.clock.steady = 1000000000 + index * 100000000
        node.raw_pose = Pose(0, 0, 0, 0, node.clock.ros, node.clock.steady)
        node.mask_worker.results.append(proposal(node, identity=index + 1))
        node._consume_visual_results(*node._times())


def test_bootstrap_requires_consecutive_masks_not_only_fresh_vn(harness):
    node = harness
    node._update_control_pose(*node._times())
    assert node.arbiter.pose is None
    node.mask_worker.results.append(proposal(node))
    node._consume_visual_results(*node._times())
    assert node.visual_streak == 1 and node.arbiter.pose is None
    assert node._visual_guard(*node._times())


def test_three_trusted_matches_correct_high_rate_vn(harness):
    node = harness
    trusted(node)
    assert node._visual_guard(*node._times()) is None
    assert node.arbiter.pose.x == pytest.approx(.2)
    assert node.arbiter.pose.y == pytest.approx(-.1)
    node.raw_pose = Pose(1, 0, .1, .5, 1210000000, 1210000000)
    node.clock.ros = node.clock.steady = 1210000000
    node._update_control_pose(*node._times())
    assert node.arbiter.pose.x == pytest.approx(math.cos(.03) + .2)
    assert node.arbiter.pose.y == pytest.approx(math.sin(.03) - .1)
    assert node.arbiter.pose.yaw == pytest.approx(.13)


def test_mask_pairing_never_uses_future_or_not_yet_arrived_pose(harness):
    node = harness
    localizer = VectorNavLocalizer()
    prior = localizer.accept(vn_message(990000000), 1000000000, 1)
    future = localizer.accept(vn_message(1010000000), 1010000000, 2)
    node.pose_buffer.append(prior)
    node.pose_buffer.append(future)
    node._mask(mask(1000000000))
    assert len(node.mask_worker.submitted) == 1
    assert node.mask_worker.submitted[0]["prior_stamp_ns"] == 990000000
    node.pose_buffer.clear()
    node.pose_buffer.append(localizer.last_sample)
    node.clock.ros = node.clock.steady = 1020000000
    node.arrival_seq = 0  # That pose has not arrived at this mask arrival frontier.
    node._mask(mask(1020000000))
    assert len(node.mask_worker.submitted) == 1
    assert "no_prior_vectornav" in node.visual_reason


def test_later_rejected_mask_invalidates_older_good_worker_result(harness):
    node = harness
    old = proposal(node)
    node.clock.ros = node.clock.steady = 1100000000
    node._mask(mask(1100000000, frame="wrong_frame"))
    node.mask_worker.results.append(old)
    node._consume_visual_results(*node._times())
    assert node.visual_streak == 0
    assert node.mask_worker.acknowledged == [(1, False)]
    assert node.arbiter.pose is None


@pytest.mark.parametrize("fault", ["expired", "generation", "pose_lost", "negative_clock", "moving"])
def test_unsafe_worker_result_is_never_committed(harness, fault):
    node = harness
    event = proposal(node)
    if fault == "expired":
        node.clock.ros = node.clock.steady = 1700000000
    elif fault == "generation":
        node.visual_generation += 1
    elif fault == "pose_lost":
        node.raw_pose = None
    elif fault == "moving":
        node.raw_pose = Pose(0, 0, 0, .5, node.clock.ros, node.clock.steady)
    else:
        node.clock.ros = 0
    node.mask_worker.results.append(event)
    node._consume_visual_results(*node._times())
    assert node.mask_worker.acknowledged == [(1, False)]
    assert node.visual_streak == 0


def test_low_confidence_holds_and_recovery_never_rearms(harness):
    node = harness
    trusted(node)
    node.arbiter.state = "REPEAT"
    node.mask_worker.results.append(proposal(node, accepted=False))
    node._consume_visual_results(*node._times())
    assert node.arbiter.state == "HOLD"
    assert node.commands[-1] == Command()
    trusted(node)
    assert node.arbiter.state == "HOLD"


def test_visual_steady_deadline_detects_ros_clock_pause(harness):
    node = harness
    trusted(node)
    node.clock.steady += 600000000
    assert "steady-clock" in node._visual_guard(*node._times())


def test_commit_failure_holds_after_ack(harness):
    node = harness
    trusted(node)
    node.arbiter.state = "REPEAT"
    node.mask_worker.results.append(dict(kind="commit_failed", generation=node.visual_generation,
                                         error="deliberate failure"))
    node._consume_visual_results(*node._times())
    assert node.arbiter.state == "HOLD"
    assert node.visual_streak == 0


def context_files(tmp_path):
    rows = "east_m,north_m,frame_votes\n" + "".join("{},1.2,5\n".format(i * .1) for i in range(100))
    raw = rows.encode()
    (tmp_path / "consensus.csv").write_bytes(raw)
    (tmp_path / "bundle.json").write_text("{}")
    transform = make_transform([0, 0, 1], [-.5, .5, -.5, .5]).tolist()
    metadata = dict(data_sha256={"consensus.csv": hashlib.sha256(raw).hexdigest()},
        camera_calibration=dict(width=160, height=120, frame_id="camera", stamp_ns=0,
            matrix=[[100, 0, 80], [0, 100, 60], [0, 0, 1]], distortion=[], distortion_model="plumb_bob"),
        static_extrinsic=dict(parent_frame="base", child_frame="camera", matrix=transform),
        parameters=dict(base_frame="base", mask_threshold=127,
            max_projection_sensitivity_m_per_px=.5, max_reliable_projection_sensitivity_m_per_px=.1))
    return tmp_path / "bundle.json", {"metadata_path": "metadata.json", "bundle_id": "test"}, metadata


def test_context_uses_hash_verified_frozen_original_calibration(module, harness, tmp_path):
    path, manifest, metadata = context_files(tmp_path)
    context = module["load_repeat_context"](path, manifest, metadata, harness.p)
    assert context["lookup"].width == 160
    assert context["lookup"].max_sensitivity_m_per_px == .5
    assert context["reliable_sensitivity"] == .1
    assert not context["anchor"].flags.writeable


@pytest.mark.parametrize("fault", ["hash", "size", "extrinsic", "camera", "missing", "csv"])
def test_context_rejects_bad_anchor_or_calibration(module, harness, tmp_path, fault):
    path, manifest, metadata = context_files(tmp_path)
    if fault == "hash":
        metadata["data_sha256"]["consensus.csv"] = "bad"
    elif fault == "size":
        metadata["camera_calibration"]["width"] = 1000000
    elif fault == "extrinsic":
        metadata["static_extrinsic"]["matrix"][0][0] = 2
    elif fault == "camera":
        metadata["camera_calibration"]["frame_id"] = "changed"
    elif fault == "csv":
        raw = b"east_m,north_m,frame_votes\nnan,1,5\n"
        (tmp_path / "consensus.csv").write_bytes(raw)
        metadata["data_sha256"]["consensus.csv"] = hashlib.sha256(raw).hexdigest()
    else:
        del metadata["static_extrinsic"]
    with pytest.raises((ValueError, KeyError)):
        module["load_repeat_context"](path, manifest, metadata, harness.p)


def wait_result(worker):
    end = time.monotonic() + 2
    while time.monotonic() < end:
        result = worker.take_result()
        if result is not None:
            return result
        time.sleep(.005)
    raise AssertionError("worker did not produce a result")


def test_worker_latest_only_and_requires_commit_ack(module):
    entered, release = threading.Event(), threading.Event()

    class Matcher:
        def __init__(self):
            self.calls, self.committed = [], []

        def match(self, points, prior, stamp, prior_stamp_ns=None, commit=True):
            assert commit is False
            self.calls.append(stamp)
            entered.set()
            release.wait(1)
            return Obj(stamp_ns=stamp, accepted=True)

        def commit(self, result):
            self.committed.append(result.stamp_ns)
            return True

        def refined_points_and_votes(self):
            return np.zeros((1, 2)), np.ones(1)

    matcher = Matcher()
    context = dict(matcher=matcher, threshold=127, reliable_sensitivity=.1,
                   last_refined_export_ns=0, refinement_publish_rate_hz=1.0,
                   lookup=Obj(project_mask_with_sensitivity=lambda *_:
                       (np.zeros((40, 2)), np.zeros(40), Obj(stable_pixels=40))))
    worker = module["LatestMaskWorker"]()
    worker.commit_fence.advance()
    try:
        def job(index):
            return dict(job_id=index, generation=1, context=context, message=mask(index, 2, 2),
                        source_ns=index, prior=np.zeros(3), prior_stamp_ns=index - 1)
        worker.submit(job(1))
        assert entered.wait(1)
        worker.submit(job(2))
        assert worker.submit(job(3))
        release.set()
        result = wait_result(worker)
        assert result["job"]["job_id"] == 1
        assert not matcher.committed
        worker.acknowledge(1, False)
        result = wait_result(worker)
        assert result["job"]["job_id"] == 3
        assert matcher.calls == [1, 3]
        now = time.monotonic_ns()
        worker.acknowledge(3, True, generation=1, issued_steady_ns=now,
                           deadline_steady_ns=now + 1000000000)
        wait_result(worker)
        assert matcher.committed == [3]
    finally:
        worker.close()


@pytest.mark.parametrize("limiting_clock", ["mask_source", "mask_receipt", "vn_source", "vn_receipt"])
def test_visual_ack_deadline_is_bounded_by_all_four_remaining_lifetimes(harness, limiting_clock):
    node = harness
    node.mask_worker.auto_commit = False
    now, steady = node._times()
    remaining = 30000000
    visual_limit = int(node.p["visual_max_age_s"] * 1e9)
    pose_limit = int(node.p["pose_timeout_s"] * 1e9)
    event = proposal(node)
    if limiting_clock == "mask_source":
        event["job"]["source_ns"] = now - visual_limit + remaining
    elif limiting_clock == "mask_receipt":
        event["job"]["received_steady_ns"] = steady - visual_limit + remaining
    elif limiting_clock == "vn_source":
        node.raw_pose = Pose(0, 0, 0, 0, now - pose_limit + remaining, steady)
    else:
        node.raw_pose = Pose(0, 0, 0, 0, now, steady - pose_limit + remaining)
    node.mask_worker.results.append(event)
    node._consume_visual_results(now, steady)
    assert node.mask_worker.acknowledged == [(1, True)]
    assert node.mask_worker.authorizations[-1] == dict(generation=node.visual_generation,
        issued_steady_ns=steady, deadline_steady_ns=steady + remaining)


@pytest.mark.parametrize("after_ack", ["invalidate", "moving", "deadline", "superseded", "close", "valid"])
def test_real_matcher_commit_fence_blocks_post_ack_fault_before_any_mutation(harness, module, after_ack):
    node = harness
    entered, release = threading.Event(), threading.Event()

    class PausedFence(module["VisualCommitFence"]):
        def commit(self, token, job, matcher, result):
            # Deterministically model a worker descheduled after receiving ACK,
            # before acquiring the actual production fence lock.
            entered.set()
            if not release.wait(2):
                raise AssertionError("test did not release the commit boundary")
            return super().commit(token, job, matcher, result)

    fence = PausedFence(clock=lambda: node.clock.steady)
    node.visual_commit_fence = fence
    node.visual_generation = fence.advance()
    anchor_x = np.arange(-5., 15.01, .05)
    scan_x = np.arange(.5, 8.01, .06)
    anchor = np.vstack([np.column_stack((anchor_x, np.full(len(anchor_x), side)))
                        for side in (-1.2, 1.2)])
    scan = np.vstack([np.column_stack((scan_x, np.full(len(scan_x), side)))
                      for side in (-1.2, 1.2)])
    matcher = LaneMapLocalizer(anchor, dict(runtime_budget_ms=2000.0))
    for source in (100, 200, 300):
        assert matcher.match(scan, [0., .2, 0.], source).accepted
    before_metrics = matcher.refinement_metrics()
    before_correction = matcher.correction_se2.copy()
    before_points, before_votes = matcher.refined_points_and_votes()
    assert len(before_points) > 0 and matcher.committed_frames == 3
    context = dict(matcher=matcher, threshold=127, reliable_sensitivity=.1,
                   last_refined_export_ns=0, refinement_publish_rate_hz=1.0,
                   lookup=Obj(project_mask_with_sensitivity=lambda *_:
                       (scan, np.zeros(len(scan)), Obj(stable_pixels=len(scan)))))
    node.repeat_context = context
    node.raw_pose = Pose(0., .25, 0., 0., node.clock.ros, node.clock.steady)
    worker = module["LatestMaskWorker"](fence)
    node.mask_worker = worker
    job = dict(job_id=1, generation=node.visual_generation, context=context,
               message=mask(node.clock.ros, 2, 2), source_ns=node.clock.ros,
               received_steady_ns=node.clock.steady, prior=np.array([0., .25, 0.]),
               prior_stamp_ns=node.clock.ros - 1)
    try:
        worker.submit(job)
        proposal_event = wait_result(worker)
        assert proposal_event["result"].accepted, proposal_event
        worker.results.put_nowait(proposal_event)
        node._consume_visual_results(*node._times())  # Actual production ACK path.
        assert entered.wait(1)
        assert fence.token is not None
        if after_ack == "invalidate":
            node._invalidate_visual("newer mask or VectorNav fault after ACK")
        elif after_ack == "moving":
            message = vn_message(node.clock.ros)
            message.velocity.x = node.p["stopped_speed_mps"] + .1
            node._vectornav(message)
            assert node.raw_pose is not None and node.pose_error is None
            assert node.raw_pose.speed > node.p["stopped_speed_mps"]
            assert "moved during visual acquisition" in node.visual_reason
            assert node.arbiter.pose is None and node.arbiter.state == "HOLD"
        elif after_ack == "deadline":
            node.clock.steady = fence.token["deadline_ns"] + 1
        elif after_ack == "superseded":
            fence.authorize(2, node.visual_generation, node.clock.steady, node.clock.steady + 100000000)
        elif after_ack == "close":
            fence.close()
        release.set()
        result = wait_result(worker)
        if after_ack == "valid":
            assert result["kind"] == "committed"
            assert matcher.committed_frames == 4
            assert matcher.last_accepted_stamp_ns == job["source_ns"]
            assert matcher.refinement_metrics()["refinement_retained_frames"] == 4
            assert not np.array_equal(matcher.correction_se2, before_correction)
        else:
            assert result["kind"] == "commit_failed"
            assert matcher.committed_frames == 3
            assert matcher.last_accepted_stamp_ns == 300
            assert matcher.refinement_metrics() == before_metrics
            np.testing.assert_array_equal(matcher.correction_se2, before_correction)
            points, votes = matcher.refined_points_and_votes()
            np.testing.assert_array_equal(points, before_points)
            np.testing.assert_array_equal(votes, before_votes)
            assert worker.last_committed_job is None and worker.latest_refined is None
    finally:
        release.set()
        worker.close()


def test_explicit_vn_invalid_state_cannot_be_hidden_by_visual_result(harness):
    node = harness
    trusted(node)
    old = proposal(node)
    node.arbiter.state = "REPEAT"
    msg = vn_message(node.clock.ros + 10000000)
    msg.insstatus.mode = 1  # ALIGNING is not TRACKING.
    node._vectornav(msg)
    node.mask_worker.results.append(old)
    node._consume_visual_results(*node._times())
    assert node.raw_pose is None and node.arbiter.pose is None
    assert node.arbiter.state == "HOLD"
    assert node.mask_worker.acknowledged[-1] == (1, False)


def prepare_tick(node):
    node.last_tick_ns = node.clock.steady - 50000000
    node.last_ros_ns = node.clock.ros - 50000000
    node.last_graph_check_ns = node.clock.steady
    node.journal.failed = None
    node._deployment_guard = lambda: None


@pytest.mark.parametrize("mode", ["fixed_only", "lya_reference"])
def test_mask_deadline_stops_both_variants_even_with_fresh_vn(harness, mode):
    node = harness
    trusted(node)
    node.arbiter.safety_mode, node.arbiter.state = mode, "REPEAT"
    node.clock.ros = node.clock.steady = 1800000000
    node.raw_pose = Pose(0, 0, 0, 0, node.clock.ros, node.clock.steady)
    prepare_tick(node)
    node._tick()
    assert node.arbiter.state == "HOLD"
    assert node.commands[-1] == Command()
    assert node.visual_streak == 0


def test_clock_regression_invalidates_inflight_before_consumption(harness):
    node = harness
    trusted(node)
    node.arbiter.state = "REPEAT"
    node.mask_worker.results.append(proposal(node))
    prepare_tick(node)
    node.clock.ros = 1000000000
    node._tick()
    assert node.mask_worker.acknowledged[-1] == (1, False)
    assert node.raw_pose is None and node.arbiter.pose is None
    assert node.arbiter.state == "HOLD"


def test_snapshot_is_separate_and_exclusive(harness, module, tmp_path):
    node = harness
    node.manifest.update(bundle_id="source", route_sha256="unchanged", origin_lla=[35, 139, 20])
    node.repeat_context["anchor_sha256"] = "anchor-hash"
    node.journal.directory = tmp_path
    node.mask_worker.latest_refined = dict(generation=node.visual_generation, source_ns=123,
        packed=module["pack_cloud"]([[1., 2.]], [3]))
    node._write_refinement_snapshot()
    saved = json.loads((tmp_path / "refinement_metadata.json").read_text())
    assert saved["anchor_changed"] is False and saved["route_changed"] is False
    assert saved["anchor_sha256"] == "anchor-hash"
    assert saved["historical_layer"] and not saved["trusted_at_shutdown"]
    assert "1.0,2.0,3" in (tmp_path / "refined_consensus.csv").read_text()
    with pytest.raises(FileExistsError):
        node._write_refinement_snapshot()


def test_snapshot_never_saves_another_loaded_bundles_worker_data(harness, module, tmp_path):
    node = harness
    node.journal.directory = tmp_path
    node.mask_worker.latest_refined = dict(generation=node.visual_generation - 1, source_ns=123,
        packed=module["pack_cloud"]([[1., 2.]], [3]))
    node._write_refinement_snapshot()
    assert not list(tmp_path.iterdir())


def test_ack_without_successful_commit_does_not_activate_correction(harness):
    node = harness
    node.mask_worker.auto_commit = False
    event = proposal(node)
    node.mask_worker.results.append(event)
    node._consume_visual_results(*node._times())
    assert node.mask_worker.acknowledged == [(1, True)]
    assert node.visual_streak == 0
    assert node.arbiter.pose is None
    assert np.array_equal(node.visual_correction, [0, 0, 0])


def test_later_fault_between_ack_and_commit_cannot_restore_confidence(harness):
    node = harness
    node.mask_worker.auto_commit = False
    event = proposal(node)
    node.mask_worker.results.append(event)
    node._consume_visual_results(*node._times())
    node._invalidate_visual("newer bad mask")
    node.mask_worker.results.append(dict(kind="committed", job=event["job"], result=event["result"]))
    node._consume_visual_results(*node._times())
    assert node.visual_streak == 0
    assert node.arbiter.pose is None


def test_delayed_commit_is_not_activated_after_mask_expiration(harness):
    node = harness
    node.mask_worker.auto_commit = False
    event = proposal(node)
    node.mask_worker.results.append(event)
    node._consume_visual_results(*node._times())
    node.clock.ros = node.clock.steady = 1700000000
    node.raw_pose = Pose(0, 0, 0, 0, node.clock.ros, node.clock.steady)
    node.mask_worker.results.append(dict(kind="committed", job=event["job"], result=event["result"]))
    node._consume_visual_results(*node._times())
    assert node.visual_streak == 0
    assert node.arbiter.pose is None


def stop_confirmation_harness(node):
    """Use actual callbacks, isolating the supervisor gate from route/mask gates."""
    node.clock.ros = node.clock.steady = 10000000000
    node.repeat_prepare_ros_ns = node.repeat_prepare_ns = 9000000000
    node._deployment_guard = lambda: None
    node._visual_guard = lambda now, steady: None
    node.arbiter.controller = Obj()
    node.arbiter.safety_mode = "fixed_only"
    node.started = []

    def start(now, steady):
        node.started.append((now, steady))
        return True, "started"

    node.arbiter.start_repeat = start
    return node


@pytest.mark.parametrize("stamp", [None, True, False, "10000000000", 10000000000.0,
                                  [], {}, 0, -1, 8999999999, 10020000001])
def test_teacher_stop_rejects_invalid_old_or_future_source_stamp(harness, stamp):
    node = stop_confirmation_harness(harness)
    node._teacher_state(Obj(data=json.dumps({"state": "STOPPED", "stamp_ns": node.clock.ros})))
    assert node.teacher_stopped
    node._teacher_state(Obj(data=json.dumps({"state": "STOPPED", "stamp_ns": stamp})))
    assert not node.teacher_stopped
    assert node.teacher_state_source_ns == node.teacher_state_received_ns == 0
    response = node._start_repeat(None, Obj())
    assert not response.success and "STOPPED" in response.message
    assert not node.started


@pytest.mark.parametrize("body", [{"state": "STOPPED"}, [], None])
def test_teacher_stop_missing_stamp_or_wrong_json_shape_clears_confirmation(harness, body):
    node = stop_confirmation_harness(harness)
    node._teacher_state(Obj(data=json.dumps({"state": "STOPPED", "stamp_ns": node.clock.ros})))
    node._teacher_state(Obj(data=json.dumps(body)))
    assert not node.teacher_stopped and node.teacher_state_source_ns == 0


@pytest.mark.parametrize("stamp", [9000000000, 10000000000, 10020000000])
def test_teacher_stop_accepts_exact_source_age_and_future_tolerance_boundaries(harness, stamp):
    node = stop_confirmation_harness(harness)
    node._teacher_state(Obj(data=json.dumps({"state": "STOPPED", "stamp_ns": stamp})))
    assert node.teacher_stopped
    assert node._start_repeat(None, Obj()).success
    assert node.started == [(node.clock.ros, node.clock.steady)]


def test_teacher_stop_rejects_retained_preprepare_source_even_when_received_now(harness):
    node = stop_confirmation_harness(harness)
    node.repeat_prepare_ros_ns = 9900000000
    node._teacher_state(Obj(data=json.dumps({"state": "STOPPED", "stamp_ns": 9800000000})))
    assert not node.teacher_stopped
    assert not node._start_repeat(None, Obj()).success
    assert not node.started


@pytest.mark.parametrize("frontier", ["repeat_prepare_ros_ns", "repeat_prepare_ns"])
def test_teacher_stop_service_rechecks_both_handover_frontiers(harness, frontier):
    node = stop_confirmation_harness(harness)
    node._teacher_state(Obj(data=json.dumps({"state": "STOPPED", "stamp_ns": node.clock.ros})))
    # A previously valid STOPPED must not certify a later handover request.
    setattr(node, frontier, node.clock.ros + 1)
    assert not node._start_repeat(None, Obj()).success
    assert not node.started


@pytest.mark.parametrize("ros_delta,steady_delta", [(1000000001, 0), (0, 1000000001),
                                                   (-20000001, 0), (0, -1)])
def test_teacher_stop_service_rechecks_source_and_receipt_deadlines(harness, ros_delta, steady_delta):
    node = stop_confirmation_harness(harness)
    node._teacher_state(Obj(data=json.dumps({"state": "STOPPED", "stamp_ns": node.clock.ros})))
    node.clock.ros += ros_delta
    node.clock.steady += steady_delta
    assert not node._start_repeat(None, Obj()).success
    assert not node.started
