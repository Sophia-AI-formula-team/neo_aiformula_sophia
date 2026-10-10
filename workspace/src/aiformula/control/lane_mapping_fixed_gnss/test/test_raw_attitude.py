"""Raw-only tilt oracle/causality tests. No ROS, Unity, INS or ground truth input."""
from dataclasses import FrozenInstanceError
import math

import numpy as np
import pytest

from lane_mapping_fixed_gnss.attitude import (
    CausalRawAttitude, DEFAULT_CONFIG, StationaryEvidence,
)


T = 1_000_000_000
R = 9_000_000_000
G = 9.80665


def still(stamp, receipt, speed=0., held=True):
    return StationaryEvidence(stamp, receipt, speed, held)


def feed(estimator, stamp, receipt, gyro=(0., 0., 0.), force=(0., 0., G),
         evidence=True, **kwargs):
    if evidence is True:
        evidence = still(stamp, receipt)
    kwargs.setdefault("now_source_ns", stamp)
    kwargs.setdefault("now_steady_ns", receipt)
    return estimator.accept_imu(stamp, receipt, gyro, force, evidence, **kwargs)


def initialized(gyro=(0., 0., 0.), force=(0., 0., G), config=None):
    estimator = CausalRawAttitude(config)
    for i in range(21):
        sample = feed(estimator, T + i * 50_000_000, R + i * 50_000_000, gyro, force)
    assert sample is not None
    return estimator, sample


def quaternion_matrix(axis, angle):
    axis = np.array(axis, dtype=float, copy=True)
    axis /= np.linalg.norm(axis)
    w = math.cos(angle / 2.)
    x, y, z = axis * math.sin(angle / 2.)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def test_defaults_are_engineering_admission_not_vehicle_or_timeout_changes():
    assert DEFAULT_CONFIG == {
        "initialization_duration_s": 1., "initialization_min_samples": 20,
        "initialization_max_samples": 512, "gravity_reference_mps2": G,
        "gravity_magnitude_tolerance_mps2": .5,
        "initialization_accel_max_deviation_mps2": .05,
        "initialization_gyro_max_norm_radps": .02,
        "initialization_gyro_max_deviation_radps": .002,
        "minimum_heading_denominator": .25,
    }


def test_initialization_only_valid_at_window_end_no_backfill_and_original_evidence():
    estimator = CausalRawAttitude()
    for i in range(20):
        assert feed(estimator, T + i*50_000_000, R + i*50_000_000) is None
        assert not estimator.ready
    sample = feed(estimator, T+1_000_000_000, R+1_000_000_000)
    assert estimator.ready
    assert sample.source_stamp_ns == T+1_000_000_000
    assert sample.initialization.first_source_stamp_ns == T
    assert sample.initialization.sample_count == 21
    assert sample.initialization.duration_s == 1.
    with pytest.raises(ValueError, match="gap_or_future_source"):
        sample.propagate_to(T, wheel_receipt_ns=R+1_000_000_000,
                            now_source_ns=sample.source_stamp_ns,
                            now_steady_ns=sample.received_steady_ns)


def test_initial_gravity_orientation_and_bias_are_frozen_not_reset_after_motion():
    up = quaternion_matrix([1, 2, 3], .2).T @ [0., 0., 1.]
    bias = (.003, -.004, .005)
    estimator, first = initialized(bias, up*G)
    assert first.up_body == pytest.approx(up, abs=2e-16)
    assert first.gyro_bias == pytest.approx(bias)
    assert first.held_rate == pytest.approx((0., 0., 0.), abs=1e-17)
    next_sample = feed(estimator, first.source_stamp_ns+10_000_000,
                       first.received_steady_ns+10_000_000, (.3, -.2, .1), evidence=None)
    assert next_sample.gyro_bias is first.gyro_bias
    assert next_sample.initialization is first.initialization


def test_known_three_dimensional_rotation_uses_previous_held_rate_not_current():
    estimator, first = initialized()
    rate = np.array([.3, -.2, .4])
    stamp, receipt = first.source_stamp_ns, first.received_steady_ns
    moving = feed(estimator, stamp+10_000_000, receipt+10_000_000, rate, evidence=None)
    assert moving.up_body == first.up_body  # rate just arrived; no retroactive action
    final = feed(estimator, stamp+110_000_000, receipt+110_000_000, (-1., 2., -3.), evidence=None)
    expected = quaternion_matrix(rate, np.linalg.norm(rate)*.1).T @ first.up_body
    assert final.up_body == pytest.approx(expected, abs=3e-16)
    assert final.held_rate == (-1., 2., -3.)
    assert np.linalg.norm(final.up_body) == pytest.approx(1., abs=2e-16)


def test_heading_rate_matches_derivative_of_horizontal_forward_not_gyro_z():
    rotation = quaternion_matrix([1, 2, 3], .4)
    up = rotation.T @ [0., 0., 1.]
    estimator, initial = initialized(force=up*G)
    rate = np.array([.3, -.2, .4])
    sample = feed(estimator, initial.source_stamp_ns+10_000_000,
                   initial.received_steady_ns+10_000_000, rate, evidence=None)
    epsilon = 1e-6
    f_plus = rotation @ quaternion_matrix(rate, np.linalg.norm(rate)*epsilon) @ [1., 0., 0.]
    f_minus = rotation @ quaternion_matrix(rate, -np.linalg.norm(rate)*epsilon) @ [1., 0., 0.]
    derivative = (math.atan2(f_plus[1], f_plus[0])-math.atan2(f_minus[1], f_minus[0]))/(2*epsilon)
    assert rate.tolist() == [.3, -.2, .4]
    assert sample.heading_rate() == pytest.approx(derivative, abs=2e-10)
    assert abs(sample.heading_rate()-rate[2]) > .001


def test_ready_acceleration_is_checked_but_never_changes_attitude_or_bias():
    a, first = initialized()
    b, other = initialized()
    for i in range(1, 40):
        stamp, receipt = first.source_stamp_ns+i*10_000_000, first.received_steady_ns+i*10_000_000
        x = feed(a, stamp, receipt, (.1, -.05, .2), (0., 0., G), evidence=None)
        y = feed(b, stamp, receipt, (.1, -.05, .2), (1e100, -25., 0.), evidence=None)
        assert x == y
    assert not a.snapshot()["dynamic_acceleration_used"]
    with pytest.raises(ValueError, match="specific_force"):
        feed(a, stamp+10_000_000, receipt+10_000_000, force=(0., float("nan"), G))
    assert not a.ready and a.last_sample is x


@pytest.mark.parametrize("bad", ["moving_wheel", "output_not_held", "wheel_future_source",
                                  "wheel_future_receipt", "wheel_stale", "missing_evidence",
                                  "gyro_norm", "gyro_spread", "accel_spread", "gravity_norm"])
def test_bad_startup_window_clears_candidates_not_source_frontiers_then_fresh_window(bad):
    estimator = CausalRawAttitude()
    for i in range(10):
        feed(estimator, T+i*50_000_000, R+i*50_000_000)
    stamp, receipt = T+500_000_000, R+500_000_000
    gyro, force, evidence = (0., 0., 0.), (0., 0., G), still(stamp, receipt)
    if bad == "moving_wheel": evidence = still(stamp, receipt, .051)
    if bad == "output_not_held": evidence = still(stamp, receipt, held=False)
    if bad == "wheel_future_source": evidence = still(stamp+1, receipt)
    if bad == "wheel_future_receipt": evidence = still(stamp, receipt+1)
    if bad == "wheel_stale": evidence = still(stamp-200_000_001, receipt)
    if bad == "missing_evidence": evidence = None
    if bad == "gyro_norm": gyro = (.0201, 0., 0.)
    if bad == "gyro_spread": gyro = (.004, 0., 0.)
    if bad == "accel_spread": force = (.06, 0., G)
    if bad == "gravity_norm": force = (0., 0., 20.)
    assert feed(estimator, stamp, receipt, gyro, force, evidence=evidence) is None
    assert estimator.snapshot()["retained_initialization_samples"] == 0
    assert estimator.last_source_stamp_ns == stamp
    assert estimator.fault_reason is None
    for i in range(1, 22):
        result = feed(estimator, stamp+i*50_000_000, receipt+i*50_000_000)
    assert result is not None
    assert result.initialization.first_source_stamp_ns == stamp+50_000_000


@pytest.mark.parametrize("fault", ["duplicate", "source_backwards", "receipt_backwards", "gap",
                                    "source_age", "future_source", "receipt_age", "future_receipt"])
def test_time_fault_latches_without_advancing_or_automatic_recovery(fault):
    estimator, initial = initialized()
    stamp, receipt = initial.source_stamp_ns+10_000_000, initial.received_steady_ns+10_000_000
    now, steady = stamp, receipt
    if fault == "duplicate": stamp = initial.source_stamp_ns
    if fault == "source_backwards": stamp = initial.source_stamp_ns-1
    if fault == "receipt_backwards": receipt = initial.received_steady_ns-1
    if fault == "gap": stamp = now = initial.source_stamp_ns+200_000_001
    if fault == "source_age": now = stamp+150_000_001
    if fault == "future_source": now = stamp-1
    if fault == "receipt_age": steady = receipt+200_000_001
    if fault == "future_receipt": steady = receipt-1
    with pytest.raises(ValueError):
        feed(estimator, stamp, receipt, evidence=None, now_source_ns=now, now_steady_ns=steady)
    assert estimator.last_sample is initial
    assert estimator.last_source_stamp_ns == initial.source_stamp_ns
    assert estimator.fault_reason is not None
    with pytest.raises(ValueError, match="fault_latched"):
        feed(estimator, initial.source_stamp_ns+20_000_000, initial.received_steady_ns+20_000_000)


def test_wheel_snapshot_is_immutable_past_held_and_cannot_renew_source_age():
    estimator, initial = initialized()
    sample = feed(estimator, initial.source_stamp_ns+10_000_000,
                   initial.received_steady_ns+10_000_000, (.1, .2, .3))
    stamp, receipt = sample.source_stamp_ns+100_000_000, sample.received_steady_ns+100_000_000
    snapshot = sample.propagate_to(stamp, wheel_receipt_ns=receipt,
                                  now_source_ns=stamp, now_steady_ns=receipt)
    expected = quaternion_matrix([.1, .2, .3], math.sqrt(.14)*.1).T @ sample.up_body
    assert snapshot.up_body == pytest.approx(expected, abs=2e-16)
    assert snapshot.source is sample
    assert snapshot.source_stamp_ns == sample.source_stamp_ns
    assert snapshot.received_steady_ns == sample.received_steady_ns
    chained = snapshot.propagate_to(stamp+50_000_000, wheel_receipt_ns=receipt+50_000_000,
                                   now_source_ns=stamp+50_000_000, now_steady_ns=receipt+50_000_000)
    direct = sample.propagate_to(stamp+50_000_000, wheel_receipt_ns=receipt+50_000_000,
                                 now_source_ns=stamp+50_000_000, now_steady_ns=receipt+50_000_000)
    assert chained == direct
    with pytest.raises(ValueError, match="source_stale"):
        chained.propagate_to(stamp+50_000_001, wheel_receipt_ns=receipt+50_000_001,
                            now_source_ns=stamp+50_000_001, now_steady_ns=receipt+50_000_001)
    with pytest.raises(FrozenInstanceError): snapshot.up_body = (0., 0., 1.)
    with pytest.raises(FrozenInstanceError): sample.gyro_bias = (1., 2., 3.)
    with pytest.raises(FrozenInstanceError): sample.initialization.sample_count = 1
    with pytest.raises(TypeError): sample.up_body[0] = 10.


@pytest.mark.parametrize("fault", ["not_arrived", "future_target", "future_source",
                                    "receipt_stale", "wheel_future_receipt"])
def test_wheel_pairing_rejects_missing_original_source_or_receipt_contract(fault):
    _, sample = initialized()
    target, receipt = sample.source_stamp_ns, sample.received_steady_ns
    now, steady = target, receipt
    if fault == "not_arrived": receipt -= 1
    if fault == "future_target": target += 1
    if fault == "future_source": target -= 1
    if fault == "receipt_stale": steady += 200_000_001
    if fault == "wheel_future_receipt": receipt += 1
    with pytest.raises(ValueError):
        sample.propagate_to(target, wheel_receipt_ns=receipt,
                            now_source_ns=now, now_steady_ns=steady)


def test_input_ownership_and_bounded_initialization_memory():
    estimator = CausalRawAttitude({"initialization_duration_s": 100.})
    gyro, force = [0., 0., 0.], [0., 0., G]
    for i in range(650):
        assert feed(estimator, T+i*1_000_000, R+i*1_000_000, gyro, force) is None
    assert estimator.snapshot()["retained_initialization_samples"] == 512
    assert estimator.snapshot()["window_evictions"] == 138
    assert estimator._window[-1][2] == (0., 0., 0.)
    gyro[0], force[2] = 99., 0.
    assert estimator._window[-1][2] == (0., 0., 0.)
    assert estimator._window[-1][3] == (0., 0., G)
    ready, first = initialized()
    assert ready.snapshot()["retained_initialization_samples"] == 0
    assert not hasattr(ready, "reset") and not hasattr(ready, "reset_origin")


def test_heading_singularity_and_external_admission_are_precommit_faults():
    estimator, initial = initialized()
    def reject(sample):
        assert sample.heading_rate() == pytest.approx(3.)
        raise RuntimeError("parent gyro rate limit")
    with pytest.raises(ValueError, match="parent gyro rate limit"):
        feed(estimator, initial.source_stamp_ns+10_000_000,
             initial.received_steady_ns+10_000_000, (0., 0., 3.), admission_check=reject)
    assert estimator.last_sample is initial
    assert estimator.last_source_stamp_ns == initial.source_stamp_ns
    assert not estimator.ready
    singular = CausalRawAttitude()
    with pytest.raises(ValueError, match="heading_projection_singular"):
        for i in range(21):
            feed(singular, T+i*50_000_000, R+i*50_000_000, force=(G, 0., 0.))
    assert singular.last_sample is None
    assert singular.last_source_stamp_ns == T+950_000_000


@pytest.mark.parametrize("kwargs", [dict(gyro_xyz=(True, 0, 0)), dict(gyro_xyz=(0, 0)),
    dict(gyro_xyz=(0, float("inf"), 0)), dict(specific_force_xyz=(0, 0, float("nan"))),
    dict(source_ns=True), dict(receipt_ns=0), dict(stationary_evidence=True)])
def test_malformed_sensor_input_is_not_a_recoverable_startup_window(kwargs):
    estimator = CausalRawAttitude()
    values = dict(source_ns=T, receipt_ns=R, gyro_xyz=(0., 0., 0.),
                  specific_force_xyz=(0., 0., G), stationary_evidence=still(T, R),
                  now_source_ns=T, now_steady_ns=R)
    values.update(kwargs)
    with pytest.raises(ValueError): estimator.accept_imu(**values)
    assert estimator.last_source_stamp_ns is None
    assert estimator.fault_reason is not None


def test_admission_check_skips_incomplete_window_and_rejects_initialization_before_commit():
    estimator = CausalRawAttitude()
    observed = []
    def deny(sample):
        observed.append(sample)
        return False
    for i in range(20):
        assert feed(estimator, T+i*50_000_000, R+i*50_000_000, admission_check=deny) is None
    assert not observed
    with pytest.raises(ValueError, match="external_admission_rejected"):
        feed(estimator, T+1_000_000_000, R+1_000_000_000, admission_check=deny)
    assert len(observed) == 1
    assert estimator.last_sample is None
    assert estimator.last_source_stamp_ns == T+950_000_000
    assert not estimator.ready


def test_returned_sample_never_aliases_mutable_input_arrays_or_configuration():
    options = {"initialization_min_samples": 20}
    estimator = CausalRawAttitude(options)
    gyro = np.array([.001, .002, .003])
    force = np.array([0., 0., G])
    for i in range(21):
        sample = feed(estimator, T+i*50_000_000, R+i*50_000_000, gyro, force)
    expected = sample.up_body, sample.gyro_bias, sample.held_rate
    gyro[:] = 5.
    force[:] = 0.
    options["initialization_min_samples"] = 1000
    assert (sample.up_body, sample.gyro_bias, sample.held_rate) == expected
    assert estimator.config["initialization_min_samples"] == 20
    with pytest.raises(TypeError): estimator.config["initialization_min_samples"] = 1000


@pytest.mark.parametrize("config", [True, {"new_gate": 1}, {"initialization_duration_s": 0},
    {"initialization_min_samples": True}, {"initialization_max_samples": 4097},
    {"initialization_min_samples": 513}, {"minimum_heading_denominator": 1.01},
    {"gravity_reference_mps2": float("nan")}])
def test_configuration_rejects_invalid_shape_and_ranges(config):
    with pytest.raises(ValueError): CausalRawAttitude(config)
