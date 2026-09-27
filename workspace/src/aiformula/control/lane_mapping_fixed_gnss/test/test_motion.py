"""Independent timing, protocol and integration tests; no ROS construction."""
from dataclasses import FrozenInstanceError
import math

import pytest

from lane_mapping_fixed_gnss.motion import CausalWheelGyroOdometry, decode_honda_rpm


T = 1000000000
R = 9000000000


def sample(odom, stamp=T, receipt=R, rpm=60, rate=0.0):
    odom.accept_gyro(stamp, receipt, rate)
    return odom.accept_wheels(stamp, receipt, rpm, rpm, stamp, receipt)


def test_straight_si_units_and_immutable_sample():
    odom = CausalWheelGyroOdometry()
    first = sample(odom)
    second = sample(odom, T + 100000000, R + 100000000)
    assert first.x == first.y == first.yaw == 0
    assert second.speed == pytest.approx(math.pi * .254)
    assert second.x == pytest.approx(second.speed * .1)
    assert second.y == second.yaw == 0
    with pytest.raises(FrozenInstanceError):
        second.x = 99
    assert odom.snapshot()["gnss_used"] == odom.snapshot()["orientation_used"] == 0


def test_constant_turn_integrates_body_gyro_not_wheel_difference():
    odom = CausalWheelGyroOdometry()
    sample(odom, rate=1.0)
    result = sample(odom, T + 100000000, R + 100000000, rate=1.0)
    assert result.yaw == pytest.approx(.1)
    assert result.x == pytest.approx(result.speed * math.sin(.1))
    assert result.y == pytest.approx(result.speed * (1 - math.cos(.1)))


def test_no_future_gyro_interpolation_and_two_receipt_frontiers():
    odom = CausalWheelGyroOdometry()
    sample(odom, rate=.2)
    odom.accept_gyro(T + 50000000, R + 50000000, .4)
    odom.accept_gyro(T + 150000000, R + 60000000, 2.0)
    result = odom.accept_wheels(T + 100000000, R + 100000000, 60, 60,
                                T + 100000000, R + 100000000)
    assert result.gyro_stamp_ns == T + 50000000
    assert result.yaw == pytest.approx(.2 * .05 + .4 * .05)
    assert result.yaw_rate == .4
    other = CausalWheelGyroOdometry()
    other.accept_gyro(T, R + 1, .4)
    with pytest.raises(ValueError, match="no_past_gyro"):
        other.accept_wheels(T, R, 0, 0, T, R)


def test_gyro_buffer_is_bounded_and_future_only_is_not_usable():
    odom = CausalWheelGyroOdometry({"gyro_buffer_size": 3})
    for index in range(6):
        odom.accept_gyro(T + index, R + index, 0)
    assert odom.snapshot()["gyro_buffer_count"] == 3
    with pytest.raises(ValueError, match="no_past_gyro"):
        odom.accept_wheels(T + 2, R + 2, 0, 0, T + 2)


@pytest.mark.parametrize("stamp,receipt,rate", [(0, R, 0), (True, R, 0), (T, -1, 0),
    (T, R, float("nan")), (T, R, float("inf")), (T, R, True), (T, R, 2.51)])
def test_invalid_gyro_rejected_without_advancing_frontier(stamp, receipt, rate):
    odom = CausalWheelGyroOdometry()
    with pytest.raises(ValueError):
        odom.accept_gyro(stamp, receipt, rate)
    assert odom.last_gyro_stamp_ns is None


@pytest.mark.parametrize("stamp,receipt", [(T, R + 1), (T - 1, R + 1), (T + 1, R - 1)])
def test_gyro_restart_or_out_of_order_rejected(stamp, receipt):
    odom = CausalWheelGyroOdometry()
    odom.accept_gyro(T, R, 0)
    with pytest.raises(ValueError):
        odom.accept_gyro(stamp, receipt, 0)
    assert odom.last_gyro_stamp_ns == T


@pytest.mark.parametrize("fault", ["wheel_source_old", "wheel_source_future", "wheel_receipt_old",
    "wheel_receipt_future", "gyro_source_old", "gyro_receipt_old", "nan", "reverse", "speed"])
def test_invalid_wheel_or_stale_gyro_never_changes_pose(fault):
    odom = CausalWheelGyroOdometry()
    first = sample(odom, rpm=0)
    stamp, receipt, now, steady, rpm = T + 100000000, R + 100000000, T + 100000000, R + 100000000, 0
    if fault == "wheel_source_old":
        now += 200000001
    elif fault == "wheel_source_future":
        now = stamp - 1
    elif fault == "wheel_receipt_old":
        steady += 200000001
    elif fault == "wheel_receipt_future":
        steady = receipt - 1
    elif fault == "gyro_source_old":
        stamp = now = T + 160000000
    elif fault == "gyro_receipt_old":
        receipt = steady = R + 200000001
    elif fault == "nan":
        rpm = float("nan")
    elif fault == "reverse":
        rpm = -1
    elif fault == "speed":
        rpm = 255
    with pytest.raises(ValueError):
        odom.accept_wheels(stamp, receipt, rpm, rpm, now, steady)
    assert odom.last_sample is first


def test_wheel_time_gap_does_not_get_clamped_or_auto_reset():
    odom = CausalWheelGyroOdometry()
    first = sample(odom, rpm=0)
    odom.accept_gyro(T + 300000000, R + 300000000, 0)
    with pytest.raises(ValueError, match="wheel_dt_gap"):
        odom.accept_wheels(T + 300000000, R + 300000000, 0, 0, T + 300000000)
    assert odom.last_sample is first


@pytest.mark.parametrize("stamp,receipt", [(T, R + 1), (T - 1, R + 1), (T + 1, R - 1)])
def test_wheel_source_or_receipt_restart_rejected(stamp, receipt):
    odom = CausalWheelGyroOdometry()
    sample(odom, rpm=0)
    with pytest.raises(ValueError):
        odom.accept_wheels(stamp, receipt, 0, 0, T + 1, R + 1)


def test_acceleration_jump_rejects_even_below_speed_limit():
    odom = CausalWheelGyroOdometry()
    sample(odom, rpm=0)
    odom.accept_gyro(T + 10000000, R + 10000000, 0)
    with pytest.raises(ValueError, match="acceleration"):
        odom.accept_wheels(T + 10000000, R + 10000000, 100, 100, T + 10000000)


def test_explicit_origin_reset_preserves_sensor_times_and_does_not_relabel_stale_gyro():
    odom = CausalWheelGyroOdometry()
    sample(odom, rpm=0)
    with pytest.raises(ValueError):
        odom.reset_origin(stopped=False)
    odom.reset_origin(stopped=True)
    assert odom.last_sample is None and odom.last_gyro_stamp_ns == T
    assert odom.last_wheel_stamp_ns == T
    with pytest.raises(ValueError, match="wheel_stamp_not_increasing"):
        odom.accept_wheels(T, R + 1, 0, 0, T + 1)
    with pytest.raises(ValueError, match="gyro_source"):
        odom.accept_wheels(T + 300000000, R + 300000000, 0, 0, T + 300000000)
    sample(odom, T + 400000000, R + 400000000, rpm=0)
    assert odom.last_sample.x == 0


def test_origin_reset_rejected_while_last_wheel_speed_is_moving():
    odom = CausalWheelGyroOdometry()
    sample(odom)
    with pytest.raises(ValueError):
        odom.reset_origin(stopped=True)


def test_honda_decoder_exact_audited_bytes_and_order():
    assert decode_honda_rpm(1809, bytes([118, 0, 0, 0, 106, 0, 0, 0])) == (106, 118)


@pytest.mark.parametrize("kwargs", [dict(frame_id=1801), dict(frame_id=True), dict(dlc=7),
    dict(is_rtr=True), dict(is_extended=True), dict(is_error=True), dict(is_error=0),
    dict(data=[1, 1, 0, 0, 1, 0, 0, 0]), dict(data=[-1, 0, 0, 0, 1, 0, 0, 0]),
    dict(data=[256, 0, 0, 0, 1, 0, 0, 0]), dict(data=[1, 2]), dict(data=None)])
def test_honda_decoder_does_not_invent_unknown_protocol(kwargs):
    values = dict(frame_id=1809, data=[1, 0, 0, 0, 1, 0, 0, 0])
    values.update(kwargs)
    with pytest.raises(ValueError):
        decode_honda_rpm(**values)


@pytest.mark.parametrize("config", [{"future": 1}, {"max_dt_s": .3}, {"max_speed_mps": float("nan")},
    {"gyro_buffer_size": True}, {"gyro_buffer_size": 5000}, {"wheel_diameter_m": -1}])
def test_invalid_configuration(config):
    with pytest.raises(ValueError):
        CausalWheelGyroOdometry(config)
