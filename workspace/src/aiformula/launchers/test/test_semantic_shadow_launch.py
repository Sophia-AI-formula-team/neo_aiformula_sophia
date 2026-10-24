"""Real-launch construction checks; these do not execute nodes or validate DDS.

Run with pytest on a sourced ROS host. Missing ROS imports, or missing installed
packages for the valid graph, are explicit skips rather than simulated passes.
"""

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("launch")
pytest.importorskip("launch_ros")
pytest.importorskip("ament_index_python")

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


ENTRY = Path(__file__).resolve().parents[1] / "launch" / "semantic_shadow.launch.py"
SPEC = importlib.util.spec_from_file_location("semantic_shadow_entry", str(ENTRY))
ENTRY_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ENTRY_MODULE)


@pytest.fixture
def context(tmp_path):
    # This fixture is only a parameter-file existence check. No camera, TF or
    # node is executed, so it must not be mistaken for calibration acceptance.
    calibration = tmp_path / "intrinsics.yaml"
    calibration.write_text("/**:\n  ros__parameters: {}\n", encoding="utf-8")
    result = LaunchContext()
    result.launch_configurations.update({
        "camera_params_file": str(calibration),
        "camera_frame_id": "test_camera_optical_frame",
        "expected_frame_id": "test_camera_optical_frame",
        "camera_name": "zedx",
        "lane_mask_topic": "/semantic_drive/lane_mask",
        "odom_topic": "/test/odom",
        "shadow_output_topic": "/semantic_drive/shadow_cmd_vel",
        "use_sim_time": "false",
    })
    return result


@pytest.mark.parametrize("value", ["", "/does/not/exist/intrinsics.yaml"])
def test_missing_calibration_fails_before_node_construction(context, value):
    context.launch_configurations["camera_params_file"] = value
    with pytest.raises(ValueError, match="camera_params_file"):
        ENTRY_MODULE._launch_nodes(context)


@pytest.mark.parametrize("key", ["camera_frame_id", "expected_frame_id", "camera_name"])
def test_blank_frame_or_camera_key_fails_before_node_construction(context, key):
    context.launch_configurations[key] = "   "
    with pytest.raises(ValueError, match="nonblank"):
        ENTRY_MODULE._launch_nodes(context)


@pytest.mark.parametrize("key,value", [
    ("lane_mask_topic", "relative/mask"),
    ("odom_topic", "/bad//odom"),
    ("shadow_output_topic", "/9invalid"),
])
def test_invalid_topic_fails_before_node_construction(context, key, value):
    context.launch_configurations[key] = value
    with pytest.raises(ValueError, match="absolute ROS topic"):
        ENTRY_MODULE._launch_nodes(context)


@pytest.mark.parametrize("topic", [
    "/aiformula_control/game_pad/cmd_vel",
    "/aiformula_control/another_motor_input",
])
def test_reserved_live_output_fails_before_node_construction(context, topic):
    context.launch_configurations["shadow_output_topic"] = topic
    with pytest.raises(ValueError, match="live motor/control"):
        ENTRY_MODULE._launch_nodes(context)


@pytest.mark.parametrize("topic", ["/cmd_vel", "/semantic_drive/gated_cmd_vel"])
def test_non_shadow_output_fails_before_node_construction(context, topic):
    context.launch_configurations["shadow_output_topic"] = topic
    with pytest.raises(ValueError, match="shadow_output_topic must remain"):
        ENTRY_MODULE._launch_nodes(context)


def test_bad_clock_boolean_fails_before_node_construction(context):
    context.launch_configurations["use_sim_time"] = "perhaps"
    with pytest.raises(ValueError, match="true or false"):
        ENTRY_MODULE._launch_nodes(context)


def _substitution_text(context, value):
    if isinstance(value, str):
        return value
    return perform_substitutions(context, value)


def test_installed_ros_graph_constructs_exact_seven_shadow_nodes(context):
    expected = [
        ("lane_line_publisher", "lane_line_publisher"),
        ("lane_points", "lane_0215"),
        ("kalman_filter", "withoutkalman_0312"),
        ("trajectory_follower", "lya_0221"),
        ("semantic_local_planner_ros", "map_assist"),
        ("semantic_drive_bridge", "signal_capture_bridge"),
        ("semantic_drive_bridge", "command_watchdog"),
    ]
    for package, _ in expected:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            pytest.skip("Install/source required ROS package: " + package)
    nodes = ENTRY_MODULE._launch_nodes(context)
    assert len(nodes) == 7
    assert all(isinstance(node, Node) for node in nodes)
    # Foxy exposes ExecuteProcess.cmd, not Node.node_package/node_executable.
    # Resolve the real ExecutableInPackage substitution without starting it.
    executables = [Path(_substitution_text(context, node.cmd[0])) for node in nodes]
    actual = [(path.parent.name, path.name) for path in executables]
    assert actual == expected
    # This entry constructs only the enumerated perception/control/bridge
    # nodes. No motor, teleop, CAN, lifecycle auto-arm or shell actions occur.
    assert not {package for package, _ in actual} & {
        "motor_controller", "teleop_twist_joy", "ros2_socketcan"}
    assert context.launch_configurations["shadow_output_topic"] == "/semantic_drive/shadow_cmd_vel"
