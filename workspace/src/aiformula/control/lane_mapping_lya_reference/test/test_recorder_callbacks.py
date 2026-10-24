"""Run the actual callback method bodies without requiring a ROS installation.

These are unit tests, not DDS/Foxy deployment evidence. Imports and ROS message
construction are replaced; callback control flow is compiled unchanged.
"""

import ast
import csv
import hashlib
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace as Obj
import uuid

import numpy as np
import pytest

from lane_mapping_lya_reference.mapping_core import CausalSampleBuffer, VectorNavLocalizer
from lane_mapping_lya_reference.route import RouteBuildError
from trajectory_follower.lya_profile import REFERENCE_SPEED_MPS


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def message(stamp, speed=0.0, yaw=90.0):
    return Obj(header=Obj(stamp=Obj(sec=stamp // 1000000000,
                                    nanosec=stamp % 1000000000)),
               group_fields=0x10C8, position=Obj(x=35.0, y=139.0, z=20.0),
               velocity=Obj(x=0.0, y=speed), yawpitchroll=Obj(x=yaw),
               insstatus=Obj(mode=2, gps_fix=True, time_error=False,
                             imu_error=False, gps_error=False))


@pytest.fixture
def harness(tmp_path):
    source = Path(__file__).parents[1] / "lane_mapping_lya_reference" / "recorder_node.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    tree.body = [item for item in tree.body if isinstance(item, (ast.ClassDef, ast.FunctionDef))]
    clock = Obj(value=1000000000)
    namespace = {"Node": object, "np": np, "math": math, "csv": csv,
                 "json": json, "hashlib": hashlib, "Path": Path, "uuid": uuid,
                 "time": Obj(monotonic_ns=lambda: clock.value),
                 "String": lambda **kwargs: Obj(**kwargs),
                 "RouteBuildError": RouteBuildError,
                 "REFERENCE_SPEED_MPS": REFERENCE_SPEED_MPS}
    exec(compile(tree, str(source), "exec"), namespace)
    cls = namespace["LaneLapRecorder"]
    node = cls.__new__(cls)
    node.params = dict(cls.DEFAULTS)
    node.phase, node.reason = "recording", "test"
    node.sequence = 0
    node.last_stopped_ns = None
    node.stopped_since_steady_ns = None
    node.last_vn_receive_steady_ns = None
    node.localizer = VectorNavLocalizer()
    node.pose_buffer = CausalSampleBuffer(128)
    node.pose_pub, node.bundle_pub = Publisher(), Publisher()
    node.counters = {"vectornav_accepted": 0, "vectornav_rejected": 0}
    node.trace, node.events_seen, node.rows = [], [], []
    node._now_ns = lambda: clock.value
    node._pose_message = lambda sample: sample
    node._publish_state = lambda: None
    node._event = lambda event, details=None: node.events_seen.append((event, details))
    node._row = lambda name, row: node.rows.append((name, row)) or True
    node.get_logger = lambda: Obj(error=lambda message: None)
    node.camera, node.extrinsic = {}, {}
    node.route_config = {}
    node.last_rejection = None
    node.map = Obj(confirmed_points_and_votes=lambda: (np.zeros((12, 2)), np.ones(12)))
    node._save = lambda finished=False: (tmp_path, np.zeros((12, 2)))
    return node, clock, namespace


def feed(node, clock, stamp, **kwargs):
    clock.value = stamp
    node._on_vectornav(message(stamp, **kwargs))


def test_pose_is_published_even_when_no_mask_has_arrived(harness):
    node, clock, _ = harness
    feed(node, clock, 1000000000)
    feed(node, clock, 1100000000)
    assert len(node.pose_pub.messages) == 2
    assert len(node.rows) == 2
    assert len(node.pose_buffer.samples) == 2


def test_a_localization_heading_jump_invalidates_lap(harness):
    node, clock, _ = harness
    feed(node, clock, 1000000000)
    feed(node, clock, 1005000000, yaw=40)
    assert node.phase == "invalid"
    assert len(node.pose_pub.messages) == 1


def test_stationary_evidence_does_not_bridge_a_sensor_gap(harness):
    node, clock, _ = harness
    feed(node, clock, 1000000000)
    feed(node, clock, 2000000000)
    assert node.last_stopped_ns == 2000000000
    assert node.stopped_since_steady_ns == 2000000000
    response = node._on_finish(None, Obj())
    assert not response.success
    assert node.phase == "recording"


def test_stale_stationary_pose_cannot_finish(harness):
    node, clock, _ = harness
    for stamp in range(1000000000, 1700000000, 100000000):
        feed(node, clock, stamp)
    clock.value = 3000000000
    assert not node._on_finish(None, Obj()).success


def test_finish_does_not_require_teacher_command_to_be_zero(harness):
    node, clock, namespace = harness
    for stamp in range(1000000000, 1700000000, 100000000):
        feed(node, clock, stamp)
    namespace["build_route"] = lambda *args: (_ for _ in ()).throw(RouteBuildError("route_is_unsafe"))
    response = node._on_finish(None, Obj())
    assert response.message == "route_is_unsafe"
    assert not response.success
    assert node.phase == "invalid"
    assert not node.bundle_pub.messages


def test_successful_finish_writes_hashed_relative_bundle(harness, tmp_path):
    node, clock, namespace = harness
    for stamp in range(1000000000, 1700000000, 100000000):
        feed(node, clock, stamp)
    namespace["write_json_exclusive"](tmp_path / "metadata.json", {"finished": True})
    namespace["build_route"] = lambda *args: {"schema_version": 1, "closed": True,
                                              "route_samples": [], "diagnostics": {"valid": True}}
    response = node._on_finish(None, Obj())
    assert response.success
    assert node.phase == "ready"
    bundle = json.loads(node.bundle_pub.messages[0].data)
    assert bundle["origin_lla"] == [35.0, 139.0, 20.0]
    assert bundle["route_path"] == "route.json"
    assert bundle["metadata_path"] == "metadata.json"
    assert bundle["route_sha256"] == hashlib.sha256((tmp_path / "route.json").read_bytes()).hexdigest()
    assert (tmp_path / "bundle.json").exists()


def test_json_writer_never_overwrites(harness, tmp_path):
    _, _, namespace = harness
    target = tmp_path / "preserved.json"
    namespace["write_json_exclusive"](target, {"original": True})
    with pytest.raises(FileExistsError):
        namespace["write_json_exclusive"](target, {"replacement": True})
    assert json.loads(target.read_text()) == {"original": True}
