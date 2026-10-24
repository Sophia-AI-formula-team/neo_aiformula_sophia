"""Shared Foxy launch assembly; never starts sensors, CAN or motor drivers."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(context, name):
    text = LaunchConfiguration(name).perform(context).strip().lower()
    if text not in ("true", "false"):
        raise ValueError(name + " must be true or false")
    return text == "true"


def _nodes(context, mode):
    value = lambda name: LaunchConfiguration(name).perform(context)
    use_sim_time = _boolean(context, "use_sim_time")
    output = value("command_output_topic")
    if not output.startswith("/") or output == "/lane_learning/lya_cmd":
        raise ValueError("command_output_topic must be absolute and not alias the teacher")
    yaw_offset = float(value("yaw_offset_rad"))
    if not math.isfinite(yaw_offset):
        raise ValueError("yaw_offset_rad must be finite")
    log_directory = str(Path(value("log_directory")).expanduser().resolve())
    params_file = value("params_file")
    if not Path(params_file).is_file():
        raise ValueError("params_file does not exist: " + params_file)
    shared = {"use_sim_time": use_sim_time}
    # Empty means inherit the LYA source default or the user's params_file.
    # Never let a numeric launch default silently override that config file.
    if value("reference_speed_mps").strip():
        reference = float(value("reference_speed_mps"))
        if not math.isfinite(reference) or reference <= 0:
            raise ValueError("reference_speed_mps must be finite and positive")
        shared["reference_speed_mps"] = reference
    follower = {
        **shared,
        "safety_mode": mode,
        "command_output_topic": output,
        "teacher_command_topic": "/lane_learning/lya_cmd",
        "vectornav_topic": value("vectornav_topic"),
        "mask_topic": value("mask_topic"),
        "camera_frame_override": value("camera_frame_override"),
        "route_bundle_path": value("route_bundle_path"),
        "log_directory": log_directory,
        "yaw_offset_rad": yaw_offset,
        "enable_vehicle_output": _boolean(context, "enable_vehicle_output"),
        "motor_zero_passthrough_verified": _boolean(context, "motor_zero_passthrough_verified"),
        "hardware_stop_verified": _boolean(context, "hardware_stop_verified"),
    }
    package = "lane_mapping_fixed" if mode == "fixed_only" else "lane_mapping_lya_reference"
    executable = "fixed_follower" if mode == "fixed_only" else "reference_follower"
    actions = [Node(
        package=package, executable=executable, name="lane_fixed_follower",
        parameters=[params_file, follower],
        remappings=[("~/control_state", "/lane_learning/control_state")],
        output="screen",
    )]
    if _boolean(context, "record"):
        actions.append(Node(
            package="lane_mapping_lya_reference", executable="lap_recorder",
            name="lane_lap_recorder", output="screen",
            parameters=[params_file, {
                **shared,
                "mask_topic": value("mask_topic"),
                "vectornav_topic": value("vectornav_topic"),
                "camera_info_topic": value("camera_info_topic"),
                "camera_frame_override": value("camera_frame_override"),
                "base_frame": value("base_frame"),
                "lya_command_topic": "/lane_learning/lya_cmd",
                "yaw_offset_rad": yaw_offset,
                "output_directory": str(Path(log_directory) / "maps"),
            }],
        ))
    if _boolean(context, "manage_teacher"):
        actions.append(Node(
            package="lane_mapping_lya_reference", executable="teacher_supervisor",
            name="lane_teacher_supervisor", output="screen",
            parameters=[params_file, {
                **shared,
                "teacher_package": value("teacher_package"),
                "teacher_executable": value("teacher_executable"),
                "teacher_command_topic": "/lane_learning/lya_cmd",
                "control_state_topic": "/lane_learning/control_state",
                "log_directory": log_directory,
            }],
        ))
    # A failed recorder/supervisor must not leave the autonomous selector alive.
    # This is process coordination, not a replacement for a motor-side watchdog.
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason="lane-learning node exited"))],
    )) for node in actions]
    return handlers + actions


def generate_learning_launch(mode):
    if mode not in ("lya_reference", "fixed_only"):
        raise ValueError("Unknown lane learning safety mode")
    package = "lane_mapping_fixed" if mode == "fixed_only" else "lane_mapping_lya_reference"
    defaults = {
        "params_file": str(Path(get_package_share_directory(package)) / "config" / "learning.yaml"),
        "record": "true",
        "manage_teacher": "true",
        "use_sim_time": "false",
        "command_output_topic": "/lane_learning/cmd_vel",
        "enable_vehicle_output": "false",
        "motor_zero_passthrough_verified": "false",
        "hardware_stop_verified": "false",
        "route_bundle_path": "",
        "log_directory": "~/.ros/lane_learning",
        "mask_topic": "/aiformula_perception/road_detector/mask_image",
        "vectornav_topic": "/aiformula_sensing/vectornav/raw/common",
        "camera_info_topic": "/aiformula_sensing/zed_node/left/camera_info",
        "camera_frame_override": "",
        "base_frame": "base_footprint",
        "yaw_offset_rad": "0.0",
        "teacher_package": "trajectory_follower",
        "teacher_executable": "lya_0221",
        "reference_speed_mps": "",
    }
    descriptions = {
        "record": "Start the passive first-lap recorder; false loads an existing route only",
        "manage_teacher": "Start one privately remapped LYA process owned by this launch",
        "command_output_topic": "Private by default; real actuator topic requires all verification flags",
        "enable_vehicle_output": "Explicit request to permit a non-private command output",
        "motor_zero_passthrough_verified": "Operator has verified zero commands reach the motors unchanged",
        "hardware_stop_verified": "Operator has physically verified independent manual emergency stop",
        "route_bundle_path": "Absolute bundle.json path when reusing an existing map",
        "yaw_offset_rad": "Calibrated VectorNav yaw mounting correction, used by both recorder and follower",
        "reference_speed_mps": "Optional override for all nodes; empty inherits params_file or the current LYA source default",
    }
    declarations = [DeclareLaunchArgument(name, default_value=default,
                                          description=descriptions.get(name, name))
                    for name, default in defaults.items()]
    return LaunchDescription(declarations + [OpaqueFunction(function=_nodes, args=[mode])])
