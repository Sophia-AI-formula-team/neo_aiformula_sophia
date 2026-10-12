import json
import pytest
from semantic_vehicle_wiring.capture import capture_from_json
from semantic_drive_bridge.policy import Rejected, SignalCapturePolicy

FRAME = "zed_left_camera_optical_frame"


def payload(state="red", **changes):
    data = dict(header=dict(stamp=dict(sec=10, nanosec=20), frame_id=FRAME), traffic_light_state=state)
    data.update(changes)
    return data


@pytest.mark.parametrize("state", ["red", "green", "unknown"])
def test_real_color_stamp_and_missing_range_reach_original_bridge(state):
    source = payload(state)
    capture = capture_from_json(json.dumps(source), FRAME)
    assert capture["header"] == source["header"]
    assert capture["sequence"] == 10000000020
    assert capture["state"] == state
    assert capture["distance_m"] is None and capture["uncertainty_m"] is None
    policy = SignalCapturePolicy(FRAME)
    result = policy.convert(json.dumps(capture), ros_ns=10100000020, steady=1.)
    assert result["state"] == state and result["distance_m"] is None
    with pytest.raises(Rejected, match="duplicate"):
        policy.convert(json.dumps(capture), ros_ns=10110000020, steady=1.1)


@pytest.mark.parametrize("data", [[], {}, payload(None), payload("yellow"),
    payload(header={"frame_id":"another_camera", "stamp":{"sec":10,"nanosec":20}}),
    payload(header={"frame_id":FRAME,"stamp":{"sec":True,"nanosec":20}}),
    payload(header={"frame_id":FRAME,"stamp":{"sec":0,"nanosec":0}}),
    payload(header={"frame_id":FRAME,"stamp":{"sec":10,"nanosec":1000000000}})])
def test_invalid_capture_is_rejected_without_fabricating_unknown(data):
    with pytest.raises(ValueError):
        capture_from_json(json.dumps(data), FRAME)


def test_unverified_range_is_never_reused():
    capture = capture_from_json(json.dumps(payload(distance_m=8.5, uncertainty_m=.1)), FRAME)
    assert capture["distance_m"] is None
