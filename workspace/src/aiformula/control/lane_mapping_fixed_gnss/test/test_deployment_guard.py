"""Actual node callbacks with an isolated ROS-graph/message test double.

No ROS process or vehicle is started. These tests prove callback policy, not
DDS discovery latency, command arbitration or a physical emergency stop.
"""
import ast
from contextlib import contextmanager
from functools import wraps
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as Obj

import pytest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "lane_mapping_lya_reference"))
from lane_mapping_fixed_gnss.control import RuntimeSafety, Command, Pose
from lane_mapping_fixed_gnss.motion import DEFAULT_CONFIG as MOTION_DEFAULT_CONFIG
from lane_mapping_lya_reference.controller import fresh
from trajectory_follower.lya_profile import REFERENCE_SPEED_MPS, MAX_YAW_RATE_RPS


class Publisher:
    def __init__(self, topic):
        self.topic_name = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def harness():
    source = ROOT / "lane_mapping_fixed_gnss" / "node.py"
    tree = ast.parse(source.read_text(encoding="utf-8"), feature_version=8)
    tree.body = [item for item in tree.body if isinstance(item, (ast.ClassDef, ast.FunctionDef))]
    namespace = {"Node": object, "wraps": wraps, "contextmanager": contextmanager,
                 "math": math, "np": np, "json": json,
                 "REFERENCE_SPEED_MPS": REFERENCE_SPEED_MPS,
                 "MAX_YAW_RATE_RPS": MAX_YAW_RATE_RPS,
                 "MOTION_DEFAULT_CONFIG": MOTION_DEFAULT_CONFIG,
                 "Command": Command, "fresh": fresh,
                 "Twist": lambda: Obj(linear=Obj(x=0.0), angular=Obj(z=0.0))}
    exec(compile(tree, str(source), "exec"), namespace)
    cls = namespace["EndpointFollower"]
    node = cls.__new__(cls)
    node.p = dict(cls.DEFAULTS)
    if node.p["maximum_speed_mps"] == 0.0:
        node.p["maximum_speed_mps"] = node.p["reference_speed_mps"]
    node.command_pub = Publisher("/lane_learning_gnss/cmd_vel")
    node.deployment_issue = None
    node.lock = threading.RLock()
    node.safety = RuntimeSafety({"hardware_stop_verified": True})
    graph = Obj(count=1, error=None, queries=[])

    def count_publishers(topic):
        graph.queries.append(topic)
        if graph.error is not None:
            raise graph.error
        return graph.count

    node.count_publishers = count_publishers
    clock = Obj(value=1000000000)
    node._times = lambda: (clock.value, clock.value)
    node.session_started = False
    node.session_motion_invalid = False
    node.teach_enabled = node.repeat_prepared = False
    node.gnss_subscription = None
    node.camera = {}
    node.last_motion = None
    node.last_tick_ns = clock.value - 50000000
    node.last_state_ns = clock.value
    node.journal = Obj(failed=None)
    node.worker = Obj(failure=None, take=lambda: None)
    cancelled = []
    node.anchors = Obj(phase="teach", cancel_window=lambda reason: cancelled.append(reason))
    node.events = []
    node._event = lambda event, **values: node.events.append((event, values))
    return node, graph, clock


def enter_teach(node, clock):
    """Reach TEACH through the real safety API, not a forged nonzero output."""
    def stopped_step():
        clock.value += 100000000
        pose = Pose(0.0, 0.0, 0.0, 0.0, clock.value, clock.value)
        assert node.safety.update_pose(pose, clock.value, clock.value)
        assert node.safety.teacher_command((0.4, 0, 0, 0, 0, 0.04),
                                          clock.value, clock.value, clock.value)
        node.safety.tick(clock.value, clock.value, 0.1)
        node.last_motion = pose

    for _ in range(12):
        stopped_step()
    assert node.safety.begin_teach_session(clock.value, clock.value)[0]
    for _ in range(12):
        stopped_step()
    assert node.safety.arm_teach(clock.value, clock.value)[0]
    node.session_started = node.teach_enabled = True
    node.last_tick_ns = clock.value - 50000000
    node.last_state_ns = clock.value


def assert_zero_messages(messages):
    assert messages
    assert all(message.linear.x == 0.0 and message.angular.z == 0.0 for message in messages)


def test_default_private_output_and_disabled_vehicle_flags_unchanged(harness):
    node, graph, _ = harness
    assert node.p["command_output_topic"] == "/lane_learning_gnss/cmd_vel"
    for flag in ("enable_vehicle_output", "hardware_stop_verified", "motor_zero_passthrough_verified"):
        assert node.p[flag] is False
    assert node._deployment_guard() is None
    assert graph.queries == [node.command_pub.topic_name]


@pytest.mark.parametrize("missing", ["enable_vehicle_output", "hardware_stop_verified",
                                     "motor_zero_passthrough_verified"])
def test_remapped_output_requires_each_deployment_confirmation(harness, missing):
    node, graph, _ = harness
    node.command_pub.topic_name = "/vehicle/motor_command"
    # Synthetic approved-limit fixture lets this test reach its original flag
    # gate. This is not a proposed or approved real-vehicle value.
    node.p["accepted_teacher_max_speed_mps"] = 2.5
    for flag in ("enable_vehicle_output", "hardware_stop_verified", "motor_zero_passthrough_verified"):
        node.p[flag] = True
    node.p[missing] = False
    assert "all three" in node._check_deployment()
    assert node.safety.state == "HOLD"
    assert_zero_messages(node.command_pub.messages)
    assert not graph.queries


def test_graph_uses_resolved_remapped_name_not_parameter(harness):
    node, graph, _ = harness
    node.command_pub.topic_name = "/test/actual_resolved_motor_command"
    node.p["accepted_teacher_max_speed_mps"] = 2.5  # Synthetic approved-limit fixture.
    for flag in ("enable_vehicle_output", "hardware_stop_verified", "motor_zero_passthrough_verified"):
        node.p[flag] = True
    assert node.p["command_output_topic"] != node.command_pub.topic_name
    assert node._deployment_guard() is None
    assert graph.queries == [node.command_pub.topic_name]
    graph.count = 2
    assert "competing" in node._check_deployment()
    assert_zero_messages(node.command_pub.messages)


def test_remapped_output_without_explicit_teacher_ceiling_fails_closed(harness):
    node, graph, _ = harness
    node.command_pub.topic_name = "/test/unapproved_resolved_motor_command"
    for flag in ("enable_vehicle_output", "hardware_stop_verified", "motor_zero_passthrough_verified"):
        node.p[flag] = True
    assert node.p["accepted_teacher_max_speed_mps"] == 0.0
    assert "positive teacher speed ceiling" in node._check_deployment()
    assert node.safety.state == "HOLD"
    assert_zero_messages(node.command_pub.messages)
    assert not graph.queries


@pytest.mark.parametrize("ceiling", [float("nan"), float("inf"), -float("inf"), -1.0])
def test_invalid_teacher_ceiling_rejected_by_production_parameter_validation(harness, ceiling):
    node, _, _ = harness
    node.p["accepted_teacher_max_speed_mps"] = ceiling
    with pytest.raises(ValueError, match="ceiling must be finite and nonnegative"):
        node._validate()


def test_default_reference_and_normalized_speed_cap_come_from_shared_lya_profile(harness):
    node, _, _ = harness
    assert node.p["reference_speed_mps"] == REFERENCE_SPEED_MPS
    assert node.p["maximum_speed_mps"] == REFERENCE_SPEED_MPS
    node.p["maximum_speed_mps"] = 0.0  # Exercise production inheritance, not only the fixture.
    node._validate()
    assert node.p["maximum_speed_mps"] == REFERENCE_SPEED_MPS
    assert node.route_config["reference_speed_mps"] == REFERENCE_SPEED_MPS


def test_explicit_reference_override_drives_inherited_cap_and_route_metadata(harness):
    node, _, _ = harness
    node.p["reference_speed_mps"] = 0.2  # Explicit low-speed regression configuration.
    node.p["maximum_speed_mps"] = 0.0
    node._validate()
    assert node.p["maximum_speed_mps"] == 0.2
    assert node.route_config["reference_speed_mps"] == 0.2


def test_reference_above_existing_motion_limit_is_rejected_at_startup(harness):
    node, _, _ = harness
    node.p["reference_speed_mps"] = 4.0
    node.p["maximum_speed_mps"] = 0.0
    assert MOTION_DEFAULT_CONFIG["max_speed_mps"] == 3.0
    with pytest.raises(ValueError, match="motion"):
        node._validate()


@pytest.mark.parametrize("count", [0, -1, None, "1", True])
def test_unverifiable_graph_counts_fail_closed(harness, count):
    node, graph, _ = harness
    graph.count = count
    assert "cannot verify" in node._check_deployment()
    assert node.safety.state == "HOLD"
    assert_zero_messages(node.command_pub.messages)


@pytest.mark.parametrize("service", ["_arm", "_prepare", "_repeat"])
@pytest.mark.parametrize("failure", ["competitor", "query_error"])
def test_every_actuation_transition_checks_live_graph(harness, service, failure):
    node, graph, _ = harness
    if failure == "competitor":
        graph.count = 2
    else:
        graph.error = RuntimeError("discovery unavailable")
    response = getattr(node, service)(None, Obj())
    assert response.success is False
    assert "command" in response.message
    assert node.safety.state == "HOLD"
    assert graph.queries == [node.command_pub.topic_name]
    assert_zero_messages(node.command_pub.messages)


@pytest.mark.parametrize("failure", ["competitor", "query_error"])
def test_runtime_new_competitor_or_graph_error_stops_without_auto_resume(harness, failure):
    node, graph, clock = harness
    enter_teach(node, clock)
    node._tick()
    assert node.safety.state == "TEACH"
    assert node.command_pub.messages[-1].linear.x > 0.0
    before = len(node.command_pub.messages)
    if failure == "competitor":
        graph.count = 2
    else:
        graph.error = RuntimeError("runtime graph failure")
    generation = node.safety.generation
    node._tick()
    assert node.safety.state == "HOLD"
    assert node.safety.generation > generation
    assert node.deployment_issue is not None
    assert_zero_messages(node.command_pub.messages[before:])
    held_generation = node.safety.generation
    node._tick()
    assert node.safety.generation == held_generation  # Do not flood repeated fault records.
    graph.count, graph.error = 1, None
    node._tick()
    assert node.deployment_issue is None
    assert node.safety.state == "HOLD"
    assert_zero_messages(node.command_pub.messages[before:])
    assert len(graph.queries) == 4


def test_graph_fault_invalidates_pending_worker_result_before_consumption(harness):
    node, graph, clock = harness
    enter_teach(node, clock)
    node.safety.arbiter.state = "REPEAT"  # Test the fault boundary from the other active state.
    consumed = []
    node._consume = lambda *args: consumed.append(args)
    graph.count = 2
    node._tick()
    assert not consumed
    assert node.safety.state == "HOLD"
    assert_zero_messages(node.command_pub.messages)
