"""Opt-in ROS callback policy and actual pure projection, not DDS/vehicle proof.

Reuse the AST callback harness only; run actual IMU/wheel/attitude classes. No
ROS runtime, orientation, GNSS, world pose or simulated dynamics are inputs.
"""
from collections import deque
from dataclasses import asdict, FrozenInstanceError, replace
from pathlib import Path
import math
import sys
from types import SimpleNamespace as Obj

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
for package in (ROOT, ROOT.parent / "lane_mapping_lya_reference",
                ROOT.parent / "trajectory_follower"):
    sys.path.insert(0, str(package))

from test_deployment_guard import harness as legacy_harness
from lane_mapping_fixed_gnss.control import Command, Pose
from lane_mapping_fixed_gnss.imu_motion import CausalImuWheelOdometry
from lane_mapping_fixed_gnss.motion import decode_honda_rpm
from lane_mapping_fixed_gnss.worker import MapWorker
from lane_mapping_lya_reference.mapping_core import GroundLookup, make_transform
from lane_mapping_lya_reference.tilted_projection import TiltedGroundLookup


G, DT = 9.80665, 50_000_000


@pytest.fixture
def raw(legacy_harness):
    node, graph, clock = legacy_harness
    node._gyro.__wrapped__.__globals__.update(asdict=asdict, Pose=Pose,
        make_transform=make_transform, decode_honda_rpm=decode_honda_rpm)
    node.p.update(motion_attitude_mode="raw_imu_tilt",
                  imu_specific_force_verified=True,
                  imu_source_stamp_semantics="driver_receipt_ros")
    node.motion = CausalImuWheelOdometry()
    node.held_output_since = node.last_output_receipt_ns = None
    node.gyro_rotation = node.gyro_frame = None
    node.last_gyro_ns = node.last_mask_ns = 0
    node.pose_buffer = deque(maxlen=4096)
    node.pose_pub = Obj(publish=lambda message: None)
    node._pose_message = lambda pose: pose
    node.start_reference_motion = None
    node.finish_pending = node.bundle_pending = node.closing = False
    node.calibration = None
    node.jobs = []
    node.worker.submit = node.jobs.append
    node._validate()
    return node, graph, clock


class RawImu:
    # Poison getters prove neither mode obtains attitude from INS orientation.
    @property
    def orientation(self):
        raise AssertionError("orientation must never be read")


def header(stamp, frame):
    sec, nanosec = divmod(stamp, 1_000_000_000)
    return Obj(stamp=Obj(sec=sec, nanosec=nanosec), frame_id=frame)


def imu(stamp, rates=(0., 0., 0.), force=(0., 0., G), frame="base_footprint"):
    result = RawImu()
    result.header = header(stamp, frame)
    result.angular_velocity = Obj(x=rates[0], y=rates[1], z=rates[2])
    result.linear_acceleration = Obj(x=force[0], y=force[1], z=force[2])
    result.angular_velocity_covariance = [0.] * 9
    result.linear_acceleration_covariance = [0.] * 9
    return result


def feed(raw, index, rates=(0., 0., 0.), force=(0., 0., G)):
    node, _, clock = raw
    clock.value = 1_000_000_000 + index * DT
    node._send(Command())
    node._gyro(imu(clock.value, rates, force))
    node._can(Obj(id=1809, header=header(clock.value, ""), data=bytes(8), dlc=8,
                  is_rtr=False, is_extended=False, is_error=False))
    assert not node.session_motion_invalid, node.events[-3:]
    return node.last_motion


def initialized(raw, up=(0., 0., 1.)):
    for index in range(22):
        sample = feed(raw, index, force=tuple(G * value for value in up))
    assert sample.attitude is not None and raw[0].motion.ready
    return sample


def camera(node):
    matrix = [[8., .23, 6.2], [0., 7., 2.1], [0., 0., 1.]]
    transform = np.eye(4)
    transform[:3, :3] = [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]
    transform[:3, 3] = [.23, -.12, .65]
    calibration = dict(width=13, height=9, camera_matrix=matrix,
                       frame_id="optical", base_from_camera=transform.tolist())
    node.camera = ({key: value for key, value in calibration.items()
                    if key != "base_from_camera"}, 1)
    node._static_transform = lambda frame: transform
    return calibration


def mask(stamp):
    return Obj(header=header(stamp, "optical"), width=13, height=9,
               step=13, encoding="mono8", data=bytes([255]) * 117)


@pytest.mark.parametrize("change", [dict(motion_attitude_mode="automatic"),
    dict(imu_specific_force_verified=False), dict(imu_specific_force_verified=1),
    dict(imu_source_stamp_semantics=""), dict(imu_source_stamp_semantics="guessed"),
    dict(gyro_bias_radps=.001)])
def test_raw_mode_requires_explicit_source_contract(raw, change):
    node, _, _ = raw
    node.p.update(change)
    with pytest.raises(ValueError):
        node._validate()


@pytest.mark.parametrize("semantics", ["hardware_sample", "driver_receipt_ros"])
def test_allowed_timestamp_declaration_does_not_change_topic_or_mount(raw, semantics):
    node, _, _ = raw
    node.p["imu_source_stamp_semantics"] = semantics
    node._validate()
    assert node.p["gyro_topic"] == "/aiformula_sensing/zed_node/imu/data_raw"
    assert node.p["gyro_mount_quaternion"] == [0., 0., 0., 0.]


def test_published_held_command_is_required_and_does_not_claim_physical_stillness(raw):
    node, _, clock = raw
    assert not node._imu_output_held(clock.value, clock.value)
    node._send(Command())
    assert node._imu_output_held(clock.value, clock.value)
    assert not node._imu_output_held(clock.value - 1, clock.value)
    clock.value += 200_000_001
    assert not node._imu_output_held(clock.value, clock.value)
    node._send(Command(.1, 0.))
    assert node.held_output_since is None
    assert not node._imu_output_held(clock.value, clock.value)


def test_real_callbacks_initialize_and_scalar_bias_is_not_double_applied(raw):
    node, _, clock = raw
    for index in range(21):
        feed(raw, index, rates=(.001, -.002, .003))
        assert not node._attitude_ready(clock.value, clock.value)
    sample = feed(raw, 21, rates=(.001, -.002, .003))
    assert node._attitude_ready(clock.value, clock.value)
    assert sample.attitude.gyro_bias == pytest.approx((.001, -.002, .003))
    assert sample.yaw_rate == pytest.approx(0.)
    assert node.events[-2][0] == "raw_imu"
    assert node.events[-2][1]["source_stamp_semantics"] == "driver_receipt_ros"


def test_initial_unpaired_wheel_waits_without_consuming_future_imu(raw):
    node, _, clock = raw
    node._send(Command())
    node._gyro(imu(clock.value))
    # Legitimate asynchronous startup: first wheel is older than the first IMU.
    node._can(Obj(id=1809, header=header(clock.value - 10_000_000, ""),
                  data=bytes(8), dlc=8, is_rtr=False, is_extended=False, is_error=False))
    assert not node.session_motion_invalid
    assert node.last_motion is None and node.motion.last_wheel_stamp_ns is None
    assert not node.pose_buffer and not node.motion.ready
    assert node.events[-1][0] == "raw_motion_waiting"
    # This genuinely new pair can start the still-held initialization window.
    sample = feed(raw, 1)
    assert sample is not None and sample.attitude is None
    assert sample.gyro_stamp_ns <= sample.stamp_ns


@pytest.mark.parametrize("method", ["_begin_teach", "_arm", "_prepare", "_repeat"])
def test_no_service_can_admit_raw_mode_before_selected_wheel_ready(raw, method):
    node, _, _ = raw
    feed(raw, 0)
    node._stopped = lambda *_: True  # Isolate the NEW admission, not physical stop.
    node.session_started = method != "_begin_teach"
    response = getattr(node, method)(None, Obj())
    assert not response.success and "initialized raw-IMU" in response.message
    assert not node.teach_enabled and not node.repeat_prepared


def test_mount_rotates_raw_force_and_rate_together_without_orientation(raw):
    node, _, _ = raw
    # Synthetic fixed mount: sensor z down maps to body z up.
    node.p["gyro_mount_quaternion"] = [1., 0., 0., 0.]
    for index in range(22):
        sample = feed(raw, index, rates=(.001, .002, -.003), force=(0., 0., -G))
    assert sample.attitude.up_body == pytest.approx((0., 0., 1.))
    assert sample.attitude.gyro_bias == pytest.approx((.001, -.002, .003))


@pytest.mark.parametrize("failure", ["missing_force", "unavailable", "frame", "empty_frame", "duplicate"])
def test_ready_sensor_failure_latches_and_cannot_resume(raw, failure):
    node, _, clock = raw
    initialized(raw)
    old = node.motion.attitude_tracker.last_sample
    clock.value += DT
    value = imu(clock.value)
    if failure == "missing_force":
        value.linear_acceleration.x = float("nan")
    elif failure == "unavailable":
        value.linear_acceleration_covariance[0] = -1.
    elif failure == "frame":
        value.header.frame_id = "different_imu"
        node.p["gyro_mount_quaternion"] = [0., 0., 0., 1.]
    elif failure == "empty_frame":
        value.header.frame_id = ""
        node.p["gyro_mount_quaternion"] = [0., 0., 0., 1.]
    else:
        value.header = header(old.source_stamp_ns, "base_footprint")
    node._gyro(value)
    assert node.session_motion_invalid and node.safety.state == "HOLD"
    assert node.motion.attitude_tracker.last_sample is old
    clock.value += DT
    node._gyro(imu(clock.value))
    assert node.motion.attitude_tracker.last_sample is old


def test_mask_owns_exact_past_gyro_snapshot_not_latest_and_copied_pixels(raw):
    node, _, clock = raw
    initialized(raw)
    selected = feed(raw, 22, rates=(.2, 0., 0.))
    feed(raw, 23, rates=(-.2, 0., 0.))
    camera(node)
    node.teach_enabled = True
    value = mask(selected.stamp_ns + DT // 2)
    node._mask(value)
    assert len(node.jobs) == 1, node.events[-1]
    job = node.jobs[0]
    snapshot = job["attitude"]
    assert snapshot.source is selected.attitude.source
    assert snapshot.source is not node.motion.attitude_tracker.last_sample
    assert snapshot.source_stamp_ns == selected.gyro_stamp_ns
    assert snapshot.up_body == pytest.approx((0., math.sin(.005), math.cos(.005)))
    assert job["prior_stamp_ns"] == selected.stamp_ns
    assert job["prior_receipt_ns"] == selected.received_steady_ns
    with pytest.raises(FrozenInstanceError):
        snapshot.up_body = (0., 0., 1.)
    assert not np.shares_memory(job["mask"], np.frombuffer(value.data, np.uint8))
    with node._commit_guard(job) as allowed:
        assert allowed
    # A new sensor source does not revoke a valid older snapshot; loss of the
    # current initialized wheel state DOES revoke it before map publication.
    node.last_motion = replace(node.last_motion, attitude=None)
    with node._commit_guard(job) as allowed:
        assert not allowed


@pytest.mark.parametrize("mutation", ["generation", "prior_source", "prior_receipt", "missing", "expired"])
def test_raw_commit_rejects_revoked_or_mismatched_snapshot(raw, mutation):
    node, _, clock = raw
    sample = initialized(raw)
    camera(node)
    node.teach_enabled = True
    node._mask(mask(sample.stamp_ns))
    job = node.jobs[0]
    if mutation == "generation":
        job["generation"] -= 1
    elif mutation == "prior_source":
        job["prior_gyro_stamp_ns"] -= 1
    elif mutation == "prior_receipt":
        job["prior_receipt_ns"] -= 1
    elif mutation == "missing":
        job.pop("attitude")
    else:
        clock.value += 150_000_001  # Original gyro limit, not relaxed to mask age.
    with node._commit_guard(job) as allowed:
        assert not allowed


def raw_job(raw, up=(0., 0., 1.)):
    node, _, _ = raw
    sample = initialized(raw, up)
    calibration = camera(node)
    return dict(stamp_ns=sample.stamp_ns, receipt_ns=sample.received_steady_ns,
        prior_stamp_ns=sample.stamp_ns, prior_gyro_stamp_ns=sample.gyro_stamp_ns,
        prior_receipt_ns=sample.received_steady_ns, attitude=sample.attitude,
        calibration=calibration, mask=np.full((9, 13), 255, np.uint8))


def bare_worker(mode):
    # _project uses no worker thread, live tracker, mapper or ROS callback.
    worker = MapWorker.__new__(MapWorker)
    worker.motion_attitude_mode, worker.lookup, worker.calibration = mode, None, None
    return worker


@pytest.mark.parametrize("tilt", [0., .02])
def test_actual_worker_projects_only_immutable_job_tilt(raw, tilt):
    job = raw_job(raw, (0., math.sin(tilt), math.cos(tilt)))
    worker = bare_worker("raw_imu_tilt")
    actual = worker._project(job)
    c = job["calibration"]
    expected = TiltedGroundLookup(13, 9, c["camera_matrix"],
        np.asarray(c["base_from_camera"]), .5).project_mask_with_sensitivity(
            job["mask"], 127, job["attitude"].up_body)
    for result, oracle in zip(actual, expected[:2]):
        assert result.tobytes() == oracle.tobytes()
    assert isinstance(worker.lookup, TiltedGroundLookup)
    if tilt == 0.:
        plain = bare_worker("legacy_flat")._project(job)
        assert all(a.tobytes() == b.tobytes() for a, b in zip(actual, plain))


@pytest.mark.parametrize("field", ["attitude", "prior_gyro_stamp_ns", "prior_stamp_ns", "prior_receipt_ns", "receipt_ns"])
def test_worker_cannot_project_unpaired_or_future_snapshot(raw, field):
    job = raw_job(raw)
    if field == "attitude":
        job[field] = None
    else:
        job[field] -= 1
    with pytest.raises(ValueError, match="selected causal wheel attitude"):
        bare_worker("raw_imu_tilt")._project(job)


def test_legacy_worker_does_not_require_imu_snapshot(raw):
    job = raw_job(raw)
    job.pop("attitude")
    worker = bare_worker("legacy_flat")
    points, sensitivity = worker._project(job)
    assert type(worker.lookup) is GroundLookup
    assert len(points) == len(sensitivity) > 0
