"""Real paired raw-IMU/wheel ownership; no ROS, INS, GNSS or truth inputs."""
from dataclasses import FrozenInstanceError
import math

import pytest

from lane_mapping_fixed_gnss.imu_motion import CausalImuWheelOdometry
from lane_mapping_fixed_gnss.motion import CausalWheelGyroOdometry


T, R, DT = 1_000_000_000, 9_000_000_000, 50_000_000
G = 9.80665


def feed(odom, index, gyro=(0., 0., 0.), force=(0., 0., G), held=True, rpm=0.):
    stamp, receipt = T + index * DT, R + index * DT
    odom.accept_imu(stamp, receipt, gyro, force, held, stamp, receipt)
    return odom.accept_wheels(stamp, receipt, rpm, rpm, stamp, receipt)


def initialized(up=(0., 0., 1.), bias=(0., 0., 0.), config=None):
    odom = CausalImuWheelOdometry(config)
    for i in range(22):
        sample = feed(odom, i, gyro=bias, force=tuple(G * x for x in up))
    assert odom.ready and sample.attitude is not None
    return odom


def test_warmup_is_raw_z_only_and_not_drive_ready():
    odom = CausalImuWheelOdometry()
    plain = CausalWheelGyroOdometry()
    for i in range(10):
        actual = feed(odom, i, gyro=(0., 0., .01))
        plain.accept_gyro(T+i*DT, R+i*DT, .01)
        expected = plain.accept_wheels(T+i*DT, R+i*DT, 0, 0, T+i*DT, R+i*DT)
        assert actual.attitude is None
        assert actual.yaw == expected.yaw
        assert actual.gyro_stamp_ns == expected.gyro_stamp_ns
    assert not odom.ready


def test_initialization_requires_earlier_wheel_and_sustained_hold():
    odom = CausalImuWheelOdometry()
    for i in range(30):
        feed(odom, i, held=False)
    assert not odom.ready
    for i in range(30, 52):
        result = feed(odom, i)
    assert odom.ready
    assert result.attitude.initialization.first_source_stamp_ns == T + 30*DT
    assert result.attitude.initialization.duration_s == 1.


def test_bias_and_tilt_derived_heading_are_not_raw_z():
    tilt = .35
    up = (0., math.sin(tilt), math.cos(tilt))
    bias = (.001, -.002, .003)
    odom = initialized(up, bias)
    rate = (0., .2, .3)
    result = feed(odom, 22, tuple(a+b for a, b in zip(rate, bias)))
    assert result.yaw_rate == pytest.approx(up[1]*rate[1] + up[2]*rate[2])
    assert result.yaw_rate != pytest.approx(rate[2] + bias[2])
    assert result.attitude.gyro_bias == pytest.approx(bias)
    assert result.attitude.source_stamp_ns == result.gyro_stamp_ns
    with pytest.raises(FrozenInstanceError):
        result.attitude = None
    with pytest.raises(FrozenInstanceError):
        result.attitude.source.up_body = (0, 0, 1)


def test_future_source_and_late_receipt_never_replace_selected_attitude():
    odom = initialized()
    first = odom.attitude_tracker.last_sample
    # Future source is already received but cannot be used by this older wheel.
    odom.accept_imu(T+23*DT, R+22*DT, (0., .2, .3), (0., 0., G), False,
                    T+23*DT, R+22*DT)
    result = odom.accept_wheels(T+22*DT, R+22*DT, 0, 0, T+23*DT, R+22*DT)
    assert result.attitude.source is first
    assert result.attitude.source is not odom.attitude_tracker.last_sample
    # Eligible source but a future receipt also cannot replace the selected pair.
    odom.accept_imu(T+24*DT, R+26*DT, (0., .2, .6), (0., 0., G), False,
                    T+24*DT, R+26*DT)
    result = odom.accept_wheels(T+24*DT, R+24*DT, 0, 0, T+24*DT, R+26*DT)
    assert result.gyro_stamp_ns == T+23*DT
    assert result.attitude.source_stamp_ns == result.gyro_stamp_ns
    assert result.attitude.received_steady_ns == R+22*DT


def test_wheel_snapshot_propagates_selected_held_rate_and_retains_source():
    odom = initialized()
    source = odom.accept_imu(T+22*DT, R+22*DT, (.2, 0., 0.), (0., 0., G), False,
                             T+22*DT, R+22*DT)
    sample = odom.accept_wheels(T+23*DT, R+23*DT, 0, 0, T+23*DT, R+23*DT)
    assert sample.attitude.source is source
    assert sample.attitude.stamp_ns == sample.stamp_ns
    assert sample.attitude.up_body == pytest.approx((0., math.sin(.01), math.cos(.01)))
    assert sample.attitude.source_stamp_ns == T+22*DT


def test_reset_origin_preserves_tilt_bias_frontiers_and_source_ownership():
    odom = initialized(bias=(.001, .002, .003))
    source = odom.attitude_tracker.last_sample
    frontier = odom.last_gyro_stamp_ns, odom.last_wheel_stamp_ns
    odom.reset_origin(stopped=True)
    assert odom.last_sample is None
    assert odom.attitude_tracker.last_sample is source
    assert odom.ready
    assert (odom.last_gyro_stamp_ns, odom.last_wheel_stamp_ns) == frontier
    sample = feed(odom, 22, gyro=(.001, .002, .003))
    assert (sample.x, sample.y, sample.yaw) == (0., 0., 0.)
    assert sample.attitude.initialization is source.initialization


def test_heading_admission_is_transactional_before_tracker_commit():
    odom = initialized()
    old = odom.attitude_tracker.last_sample
    gyro_count = len(odom._gyro)
    with pytest.raises(ValueError, match="yaw_rate_limit"):
        feed(odom, 22, gyro=(0., 0., 3.))
    assert odom.attitude_tracker.last_sample is old
    assert odom.attitude_tracker.last_source_stamp_ns == old.source_stamp_ns
    assert odom.last_gyro_stamp_ns == old.source_stamp_ns
    assert len(odom._gyro) == gyro_count
    with pytest.raises(ValueError, match="fault_latched"):
        feed(odom, 22)


@pytest.mark.parametrize("mutation", ["repeat", "receipt", "bool", "nan", "short", "force", "held", "source_future", "source_stale", "receipt_stale", "gap"])
def test_bad_imu_never_publishes_half_a_sensor_pair(mutation):
    odom = initialized()
    old = odom.attitude_tracker.last_sample
    stamp, receipt, now, steady = T+22*DT, R+22*DT, T+22*DT, R+22*DT
    gyro, force, held = (0., 0., 0.), (0., 0., G), True
    if mutation == "repeat": stamp = old.source_stamp_ns
    if mutation == "receipt": receipt = old.received_steady_ns-1
    if mutation == "bool": gyro = (0., True, 0.)
    if mutation == "nan": gyro = (0., 0., float("nan"))
    if mutation == "short": gyro = (0., 0.)
    if mutation == "force": force = (float("inf"), 0., G)
    if mutation == "held": held = 1
    if mutation == "source_future": now = stamp-1
    if mutation == "source_stale": now = stamp+150_000_001
    if mutation == "receipt_stale": steady = receipt+200_000_001
    if mutation == "gap": stamp = now = old.source_stamp_ns+200_000_001
    with pytest.raises(ValueError):
        odom.accept_imu(stamp, receipt, gyro, force, held, now, steady)
    assert odom.attitude_tracker.last_sample is old
    assert odom.last_gyro_stamp_ns == old.source_stamp_ns
    assert not odom.ready


@pytest.mark.parametrize("kind", ["speed", "acceleration", "stale", "receipt", "reverse", "gap"])
def test_original_wheel_gates_and_no_pose_commit(kind):
    odom = initialized()
    old = odom.last_sample
    stamp, receipt, now, steady, rpm = T+22*DT, R+22*DT, T+22*DT, R+22*DT, 0.
    if kind == "speed": rpm = 255.
    if kind == "acceleration": rpm = 60.
    if kind == "stale": now += 200_000_001
    if kind == "receipt": steady += 200_000_001
    if kind == "reverse": rpm = -1.
    if kind == "gap": stamp = now = old.stamp_ns+200_000_001
    with pytest.raises(ValueError):
        odom.accept_wheels(stamp, receipt, rpm, rpm, now, steady)
    assert odom.last_sample is old
    assert odom.last_wheel_stamp_ns == old.stamp_ns


def test_bounded_attitude_pair_buffer_and_legacy_bypass_rejected():
    odom = initialized(config={"gyro_buffer_size": 3})
    assert len(odom._attitudes) == len(odom._gyro) == 3
    assert [row[:2] for row in odom._attitudes] == [row[:2] for row in odom._gyro]
    with pytest.raises(ValueError, match="three_axis"):
        odom.accept_gyro(T+22*DT, R+22*DT, 0.)
    assert odom.ready


def test_mutable_input_vectors_are_copied_and_dynamic_accel_not_fused():
    odom = initialized()
    gyro, force = [0., .1, .2], [3., 5., 12.]
    sample = feed(odom, 22, gyro, force)
    gyro[:] = [9., 9., 9.]
    force[:] = [0., 0., 0.]
    assert sample.attitude.held_rate == (0., .1, .2)
    assert sample.attitude.up_body == (0., 0., 1.)
    snapshot = odom.snapshot()
    assert snapshot["gnss_used"] == snapshot["orientation_used"] == 0
    assert snapshot["raw_tilt_used"] == 1
    assert snapshot["raw_attitude"]["dynamic_acceleration_used"] is False
