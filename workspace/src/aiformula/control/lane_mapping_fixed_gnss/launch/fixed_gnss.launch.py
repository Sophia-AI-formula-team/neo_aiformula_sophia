"""Own a private LYA; do not start sensors, CAN, motors, or GNSS drivers."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    boolean = lambda name: value(name).lower() == "true"
    for name in ("manage_teacher", "use_sim_time", "enable_vehicle_output",
                 "hardware_stop_verified", "motor_zero_passthrough_verified"):
        if value(name).lower() not in ("true", "false"):
            raise ValueError(name + " must be true or false")
    follower = Node(package="lane_mapping_fixed_gnss", executable="endpoint_follower",
        name="lane_endpoint_follower", output="screen", parameters=[value("params_file"), {
            "use_sim_time": boolean("use_sim_time"), "log_directory": value("log_directory"),
            "command_output_topic": value("command_output_topic"),
            "enable_vehicle_output": boolean("enable_vehicle_output"),
            "hardware_stop_verified": boolean("hardware_stop_verified"),
            "motor_zero_passthrough_verified": boolean("motor_zero_passthrough_verified"),
        }])
    actions = [follower]
    if boolean("manage_teacher"):
        actions.append(Node(package="lane_mapping_lya_reference", executable="teacher_supervisor",
            name="lane_gnss_teacher_supervisor", output="screen", parameters=[{
                "use_sim_time": boolean("use_sim_time"),
                "teacher_package": "trajectory_follower",
                "teacher_executable": "lya_follower_connected_omegat_global",
                "teacher_command_topic": "/lane_learning/lya_cmd",
                "control_state_topic": "/lane_learning_gnss/control_state",
                "teacher_state_topic": "/lane_learning_gnss/teacher_state",
                "emergency_stop_topic": "/lane_learning_gnss/emergency_stop",
                "log_directory": value("log_directory"),
            }]))
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason="endpoint mapping process exited"))])) for node in actions]
    return handlers + actions


def generate_launch_description():
    defaults = {
        "params_file": str(Path(get_package_share_directory("lane_mapping_fixed_gnss")) / "config/runtime.yaml"),
        "manage_teacher": "true", "use_sim_time": "false",
        "command_output_topic": "/lane_learning_gnss/cmd_vel",
        "enable_vehicle_output": "false", "hardware_stop_verified": "false",
        "motor_zero_passthrough_verified": "false", "log_directory": "~/.ros/lane_learning_gnss",
    }
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=nodes)])
