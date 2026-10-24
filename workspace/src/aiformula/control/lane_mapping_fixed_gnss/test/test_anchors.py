"""Optional GNSS gates repeat-start proximity, never map geometry or teaching."""
from dataclasses import replace, FrozenInstanceError
import inspect
import json
from types import SimpleNamespace as Obj

import pytest

from lane_mapping_fixed_gnss.anchors import EndpointAnchors, Fix, decode_vectornav_gps


T, R = 1000000000, 9000000000


def fix(stamp=T + 100000000, receipt=R + 100000000, **changes):
    return replace(Fix(stamp, receipt, 35.0, 139.0, 20.0, 0, 2,
                       (.25, 0, 0, 0, .36, 0, 0, 0, 1.0)), **changes)


def start(anchors):
    anchors.open_start(T, R, stopped=True)
    value = fix()
    anchors.observe_fix(value, value.stamp_ns, value.received_steady_ns, stopped=True)
    return value


def gps_message():
    return Obj(group_fields=0x230, fix=3, header=Obj(stamp=Obj(sec=1, nanosec=100000000)),
               poslla=Obj(x=35., y=139., z=20.), posu=Obj(x=.6, y=.5, z=1.))


def test_exact_two_fixes_and_metadata_ready_contract():
    anchors = EndpointAnchors()
    first = start(anchors)
    assert anchors.phase == "teach"
    anchors.open_end(T + 2000000000, R + 2000000000, stopped=True)
    end = fix(T + 2100000000, R + 2100000000, lat=35.000001)
    assert anchors.observe_fix(end, end.stamp_ns, end.received_steady_ns, stopped=True) is end
    report = anchors.snapshot()
    assert report["phase"] == "ready" and report["policy"] == "start_and_end_only"
    assert report["runtime_used"] == 0 and report["start_used"] == report["end_used"] == 1
    assert report["start"]["kind"] == "start" and report["start"]["accepted"]
    assert report["end"]["kind"] == "end" and report["end"]["accepted"]
    assert report["end"]["validation"]["passed"] is True
    assert report["end"]["validation"]["purpose"] == "repeat_start_proximity"
    assert report["ready_for_repeat"] is True
    assert report["mapping_requires_gnss"] is report["geometry_uses_gnss"] is False
    assert report["end"]["validation"]["distance_m"] == pytest.approx(.111195, abs=.001)
    assert report["end"]["applied_to_geometry"] is False
    assert report["geometry_changed_by_end"] is False
    json.dumps(report, allow_nan=False)
    report["start"]["covariance"][0] = 999
    assert anchors.snapshot()["start"]["covariance"][0] == first.covariance[0]


def test_teach_and_ready_gnss_flood_rejected_before_any_coordinate_access():
    class Poison:
        def __getattribute__(self, name):
            raise AssertionError("outside-window GNSS was inspected")

    anchors = EndpointAnchors()
    with pytest.raises(ValueError, match="outside"):
        anchors.observe_fix(Poison(), T, R)
    start(anchors)
    before = anchors.snapshot()["start"]
    for _ in range(100):
        with pytest.raises(ValueError, match="outside"):
            anchors.observe_fix(Poison(), T, R)
    assert anchors.phase == "teach" and anchors.snapshot()["start"] == before
    anchors.open_end(T + 1000000000, R + 1000000000, True)
    end = fix(T + 1100000000, R + 1100000000)
    anchors.observe_fix(end, end.stamp_ns, end.received_steady_ns, stopped=True)
    saved_end = anchors.snapshot()["end"]
    for _ in range(100):
        with pytest.raises(ValueError, match="outside"):
            anchors.observe_fix(Poison(), 0, 0)
    assert anchors.phase == "ready" and anchors.snapshot()["end"] == saved_end
    assert anchors.snapshot()["runtime_used"] == 0


@pytest.mark.parametrize("changes", [dict(lat=float("nan")), dict(lon=float("inf")), dict(lat=91),
    dict(lon=-181), dict(alt=True), dict(status=-1), dict(status=True), dict(covariance_type=0),
    dict(covariance=(0,) * 9), dict(covariance=(-1, 0, 0, 0, .25, 0, 0, 0, 1)),
    dict(covariance=(4, 0, 0, 0, 4, 0, 0, 0, 1)),
    dict(covariance=(1, 3, 0, 3, 1, 0, 0, 0, 1)),
    dict(covariance=(.25, .1, 0, 0, .25, 0, 0, 0, 1)),
    dict(covariance=(float("nan"),) * 9), dict(covariance=(1, 2)), dict(stamp_ns=True)])
def test_invalid_fix_faults_and_never_creates_anchor(changes):
    anchors = EndpointAnchors()
    anchors.open_start(T, R, True)
    with pytest.raises(ValueError):
        anchors.observe_fix(fix(**changes), T + 100000000, R + 100000000, stopped=True)
    assert anchors.phase == "fault" and anchors.snapshot()["start"] is None
    with pytest.raises(ValueError):
        anchors.open_start(T + 200000000, R + 200000000, True)


@pytest.mark.parametrize("which", ["predates_source", "predates_receipt", "source_stale", "source_future",
                                    "receipt_stale", "receipt_future", "movement", "no_stopped_proof"])
def test_window_fix_time_and_stationary_contract(which):
    anchors = EndpointAnchors()
    anchors.open_start(T, R, True)
    value, now, steady, stopped = fix(), T + 100000000, R + 100000000, True
    if which == "predates_source":
        value = replace(value, stamp_ns=T)
    elif which == "predates_receipt":
        value = replace(value, received_steady_ns=R - 1)
    elif which == "source_stale":
        now += 500000001
    elif which == "source_future":
        now -= 1
    elif which == "receipt_stale":
        steady += 500000001
    elif which == "receipt_future":
        steady -= 1
    else:
        stopped = False
    with pytest.raises(ValueError):
        if which == "no_stopped_proof":
            anchors.observe_fix(value, now, steady)
        else:
            anchors.observe_fix(value, now, steady, stopped=stopped)
    assert anchors.phase == "fault"


@pytest.mark.parametrize("now,steady,stopped", [(T + 2000000001, R, True),
    (T, R + 2000000001, True), (T - 1, R + 1, True), (T + 1, R - 1, True), (T, R, False)])
def test_watchdog_expires_or_faults_without_a_gnss_message(now, steady, stopped):
    anchors = EndpointAnchors()
    anchors.open_start(T, R, True)
    with pytest.raises(ValueError):
        anchors.check_window(now, steady, stopped)
    assert anchors.phase == "fault"


def test_end_distance_failure_records_check_but_never_adjusts_geometry():
    anchors = EndpointAnchors()
    start(anchors)
    anchors.open_end(T + 1000000000, R + 1000000000, True)
    value = fix(T + 1100000000, R + 1100000000, lat=35.0001)
    assert anchors.observe_fix(value, value.stamp_ns, value.received_steady_ns, stopped=True) is value
    report = anchors.snapshot()
    assert report["phase"] == "ready" and report["end_used"] == 1
    assert report["end"]["validation"]["passed"] is False
    assert report["end"]["validation"]["reason"] == "endpoint_distance_limit"
    assert report["end"]["validation"]["purpose"] == "repeat_start_proximity"
    assert report["end"]["applied_to_geometry"] is False
    assert report["ready_for_repeat"] is False
    assert report["mapping_requires_gnss"] is False


@pytest.mark.parametrize("start_state", ["unopened", "start_window", "failed_start"])
def test_teaching_does_not_require_gnss_and_end_without_start_never_grants_repeat(start_state):
    anchors = EndpointAnchors()
    if start_state != "unopened":
        anchors.open_start(T, R, True)
    if start_state == "failed_start":
        with pytest.raises(ValueError, match="no_valid_gnss_fix"):
            anchors.observe_fix(fix(status=-1), T + 100000000, R + 100000000, stopped=True)
    anchors.mark_teach_started()
    before = anchors.snapshot()
    assert before["phase"] == "teach_without_start" and before["teach_started"] is True
    assert before["start"] is None and before["start_used"] == before["runtime_used"] == 0
    assert before["ready_for_repeat"] is before["mapping_requires_gnss"] is False
    if start_state == "failed_start":
        assert before["start_unavailable_reason"] == "no_valid_gnss_fix"
    anchors.mark_teach_started()
    assert anchors.snapshot() == before
    with pytest.raises(ValueError, match="start_window_already"):
        anchors.open_start(T + 200000000, R + 200000000, True)
    for _ in range(3):
        with pytest.raises(ValueError, match="outside_endpoint_window"):
            anchors.observe_fix(fix(), T + 100000000, R + 100000000, stopped=True)
    anchors.open_end(T + 1000000000, R + 1000000000, True)
    value = fix(T + 1100000000, R + 1100000000)
    assert anchors.observe_fix(value, value.stamp_ns, value.received_steady_ns, stopped=True) is value
    report = anchors.snapshot()
    assert report["phase"] == "ready" and report["ready_for_repeat"] is False
    assert report["start"] is None and report["end"]["accepted"] is True
    assert report["end"]["validation"] == {
        "purpose": "repeat_start_proximity", "passed": False,
        "reason": "missing_start_reference", "distance_m": None, "max_distance_m": 3.0}
    assert report["geometry_changed_by_end"] is report["geometry_uses_gnss"] is False
    json.dumps(report, allow_nan=False)


def test_teach_boundary_preserves_valid_start_but_never_reopens_any_window():
    anchors = EndpointAnchors()
    start(anchors)
    reference = anchors.snapshot()["start"]
    anchors.mark_teach_started()
    assert anchors.phase == "teach" and anchors.snapshot()["start"] == reference
    anchors.open_end(T + 1000000000, R + 1000000000, True)
    with pytest.raises(ValueError, match="cannot_rewind"):
        anchors.mark_teach_started()
    anchors.cancel_window("end cancelled")
    with pytest.raises(ValueError, match="cannot_rewind"):
        anchors.mark_teach_started()
    with pytest.raises(ValueError):
        anchors.open_end(T + 2000000000, R + 2000000000, True)
    assert anchors.snapshot()["ready_for_repeat"] is False


def test_gnss_public_api_has_no_map_route_heading_or_geometry_argument():
    forbidden = {"map", "route", "trajectory", "pose", "yaw", "geometry", "origin", "transform"}
    for name in ("__init__", "open_start", "mark_teach_started", "open_end", "observe_fix"):
        parameters = set(inspect.signature(getattr(EndpointAnchors, name)).parameters)
        assert parameters.isdisjoint(forbidden)
    for field in forbidden:
        with pytest.raises(ValueError, match="unknown_configuration"):
            EndpointAnchors({field: 0})


def test_start_and_end_cannot_open_in_wrong_phase_or_while_moving():
    anchors = EndpointAnchors()
    with pytest.raises(ValueError):
        anchors.open_end(T, R, True)
    with pytest.raises(ValueError):
        anchors.open_start(T, R, False)
    assert anchors.phase == "fault"
    anchors = EndpointAnchors()
    start(anchors)
    with pytest.raises(ValueError):
        anchors.open_start(T + 1, R + 1, True)
    with pytest.raises(ValueError):
        anchors.open_end(T + 200000000, R + 200000000, False)
    assert anchors.phase == "fault"


def test_fix_copies_covariance_and_is_immutable():
    covariance = [.25, 0, 0, 0, .25, 0, 0, 0, 1]
    value = fix(covariance=covariance)
    covariance[0] = 999
    assert value.covariance[0] == .25
    with pytest.raises(FrozenInstanceError):
        value.lat = 0


@pytest.mark.parametrize("window", ["start", "end"])
def test_cancel_window_latches_fault_and_queued_fix_cannot_restore_it(window):
    anchors = EndpointAnchors()
    if window == "end":
        start(anchors)
        anchors.open_end(T + 1000000000, R + 1000000000, True)
    else:
        anchors.open_start(T, R, True)
    saved_start = anchors.snapshot()["start"]
    assert anchors.cancel_window("motion provenance lost") is True
    assert anchors.phase == "fault" and anchors.reason == "motion provenance lost"
    with pytest.raises(ValueError, match="outside_endpoint"):
        anchors.observe_fix(fix(), T + 100000000, R + 100000000, stopped=True)
    assert anchors.snapshot()["start"] == saved_start
    assert anchors.snapshot()["end"] is None
    assert anchors.cancel_window("later generic stop") is False
    assert anchors.reason == "motion provenance lost"


def test_cancel_outside_window_does_not_modify_accepted_anchors():
    anchors = EndpointAnchors()
    assert anchors.cancel_window("not opened") is False
    assert anchors.phase == "unopened"
    start(anchors)
    before = anchors.snapshot()
    assert anchors.cancel_window("teach hold") is False
    assert anchors.snapshot() == before


def test_raw_vectornav_gps_decoder_swaps_ned_sigmas_to_enu_variances():
    message = gps_message()
    decoded = decode_vectornav_gps(message, R)
    assert decoded.lat == 35 and decoded.lon == 139 and decoded.alt == 20
    assert decoded.covariance == (.25, 0, 0, 0, .36, 0, 0, 0, 1)
    assert decoded.status == 0 and decoded.covariance_type == 2  # 3D, not an RTK claim.


@pytest.mark.parametrize("fault", ["no_poslla", "no_posu", "no_fix_field", "2d", "no_fix",
    "unknown_fix_enum", "zero_sigma", "negative_sigma", "nan_sigma", "nan_lla", "bad_stamp"])
def test_raw_vectornav_gps_decoder_rejects_unsupported_or_unreliable_source(fault):
    message = gps_message()
    if fault == "no_poslla":
        message.group_fields = 0x210  # Actual January bag and audited driver default.
    elif fault == "no_posu":
        message.group_fields = 0x30
    elif fault == "no_fix_field":
        message.group_fields = 0x220
    elif fault == "2d":
        message.fix = 2
    elif fault == "no_fix":
        message.fix = 0
    elif fault == "unknown_fix_enum":
        message.fix = 5
    elif fault == "zero_sigma":
        message.posu.x = 0
    elif fault == "negative_sigma":
        message.posu.y = -1
    elif fault == "nan_sigma":
        message.posu.z = float("nan")
    elif fault == "nan_lla":
        message.poslla.x = float("nan")
    else:
        message.header.stamp.nanosec = 1000000000
    with pytest.raises(ValueError):
        decode_vectornav_gps(message, R)


@pytest.mark.parametrize("config", [{"unknown": 1}, {"window_s": 0}, {"max_fix_age_s": float("nan")},
    {"max_horizontal_sigma_m": True}, {"max_endpoint_distance_m": 100}])
def test_invalid_anchor_configuration(config):
    with pytest.raises(ValueError):
        EndpointAnchors(config)
