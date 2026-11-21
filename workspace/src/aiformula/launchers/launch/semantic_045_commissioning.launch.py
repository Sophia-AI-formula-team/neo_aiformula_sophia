"""Exercise vehicle MapAssist, OA selection and LYA on diagnostic outputs."""
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from semantic_vehicle_wiring.manual_handoff import MOTOR_STATUS_TOPIC, OA_STATUS_TOPIC

PREFIX = "/semantic_drive/map_assist_045"
VEHICLE_COMMAND = "/aiformula_control/game_pad/cmd_vel"
PIXEL_STOP_TOPIC = "/flag_stop"


def pixel_stop_source(context, script):
    if LaunchConfiguration("traffic_stop_flag_topic").perform(context) != PIXEL_STOP_TOPIC:
        return []
    threshold = int(LaunchConfiguration("red_pixel_stop_threshold").perform(context))
    if threshold <= 0:
        raise ValueError("red_pixel_stop_threshold must be positive")
    return [ExecuteProcess(cmd=[sys.executable, str(script), str(threshold)], output="screen")]


def planning_processes(context, share, directory, filtered, lya_command):
    start_oa = LaunchConfiguration("start_oa2").perform(context) == "true"
    algorithm = LaunchConfiguration("oa_algorithm").perform(context)
    handoff = LaunchConfiguration("manual_handoff").perform(context)
    if handoff == "true" and start_oa and algorithm != "oa_2_2":
        raise ValueError("manual_handoff requires oa_algorithm:=oa_2_2 when OA is started")
    enabled = handoff == "true" or (handoff == "auto" and start_oa and algorithm == "oa_2_2")
    manual_topic = MOTOR_STATUS_TOPIC if enabled else ""
    manual_args = ["-p", f"manual_state_topic:={manual_topic}"] if manual_topic else []
    source = [(f"/processed_point_{label}", f"{PREFIX}/point_{label}") for label in "abc"]
    switch_args = [arg for old, new in source + filtered for arg in ("-r", f"{old}:={new}")]
    planner = GroupAction([
        SetRemap(src=VEHICLE_COMMAND, dst=lya_command),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "semantic_vehicle_045.launch.py")),
            launch_arguments={
                "lane": LaunchConfiguration("lane"),
                "lane_point_topic_prefix": LaunchConfiguration("lane_point_topic_prefix"),
                "oa_initially_idle": LaunchConfiguration("oa_initially_idle"),
                "traffic_stop_flag_topic": LaunchConfiguration("traffic_stop_flag_topic"),
                "traffic_stop_flag_timeout_sec": "0.5",
                "manual_state_topic": manual_topic,
                "manual_oa_status_topic": OA_STATUS_TOPIC if enabled and start_oa else "",
            }.items(),
        ),
    ])
    switch = ExecuteProcess(
        cmd=[sys.executable, str(directory / "fresh_oa_switch.py"), "--ros-args",
             *switch_args, *manual_args],
        output="screen",
    )
    actions = [planner, switch]
    if start_oa:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "obstacle_avoidance.launch.py")),
            launch_arguments={"algorithm": algorithm, "manual_state_topic": manual_topic}.items(),
        ))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("launchers"))
    directory = share / "shellscript" / "mapassist_045"
    lya_command = PREFIX + "/lya_cmd_vel"
    filtered = [(f"/{name}", f"{PREFIX}/{name}") for name in ("filtered_lane_pose", "filtered_omega_t")]
    follower = Node(
        package="trajectory_follower", executable="lya_follower_connected_omegat_global",
        condition=IfCondition(LaunchConfiguration("start_lya")),
        output="screen", remappings=filtered + [(VEHICLE_COMMAND, lya_command)],
    )
    def adapter(context):
        live = LaunchConfiguration("vehicle_output").perform(context) == "true"
        args = ["--vehicle-output"] if live else []
        return [ExecuteProcess(cmd=[sys.executable, str(directory / "guarded_stop_wait.py"), *args],
                               output="screen")]
    return LaunchDescription([
        DeclareLaunchArgument("start_lya", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_oa2", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("oa_algorithm", default_value="oa_2", choices=["oa_2", "oa_2_1", "oa_2_2"]),
        DeclareLaunchArgument("manual_handoff", default_value="auto", choices=["auto", "true", "false"],
                              description="Reset MapAssist and OA 2.2 on a motor joystick takeover."),
        DeclareLaunchArgument("lane_point_topic_prefix", default_value="/processed_point_",
                              choices=["/processed_point_", "/original_processed_point_"]),
        DeclareLaunchArgument("vehicle_output", default_value="false", choices=["true", "false"],
                              description="Connect legacy stop_wait and activate the motor session"),
        DeclareLaunchArgument("oa_initially_idle", default_value="true", choices=["true", "false"],
                              description="Start on the normal lane when standalone OA is off"),
        DeclareLaunchArgument("lane", choices=["inner", "outer"]),
        DeclareLaunchArgument("traffic_stop_flag_topic", default_value=PIXEL_STOP_TOPIC),
        DeclareLaunchArgument("red_pixel_stop_threshold", default_value="600"),
        OpaqueFunction(function=pixel_stop_source, args=[directory / "pixel_stop_flag.py"]),
        OpaqueFunction(function=planning_processes, args=[share, directory, filtered, lya_command]),
        follower, OpaqueFunction(function=adapter),
        Node(package="semantic_vehicle_wiring", executable="motor_gate_session", namespace="/semantic_drive",
             output="screen", condition=IfCondition(LaunchConfiguration("vehicle_output"))),
    ])
