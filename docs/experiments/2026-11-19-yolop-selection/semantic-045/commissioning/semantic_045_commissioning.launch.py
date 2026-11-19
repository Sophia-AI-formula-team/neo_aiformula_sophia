"""Exercise vehicle MapAssist, OA selection and LYA on diagnostic outputs."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

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


def generate_launch_description():
    directory = Path(__file__).resolve().parent
    planner_launch = directory.parent / "source/workspace/src/aiformula/launchers/launch/semantic_vehicle_045.launch.py"
    lya_command = PREFIX + "/lya_cmd_vel"
    filtered = [(f"/{name}", f"{PREFIX}/{name}") for name in ("filtered_lane_pose", "filtered_omega_t")]
    source = [(f"/processed_point_{label}", f"{PREFIX}/point_{label}") for label in "abc"]
    switch_args = [arg for old, new in source + filtered for arg in ("-r", f"{old}:={new}")]
    switch = ExecuteProcess(
        cmd=[sys.executable, str(directory / "fresh_oa_switch.py"), "--ros-args", *switch_args],
        output="screen",
    )
    planner = GroupAction([
        SetRemap(src=VEHICLE_COMMAND, dst=lya_command),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(planner_launch)),
            launch_arguments={
                "lane": LaunchConfiguration("lane"),
                "traffic_stop_flag_topic": LaunchConfiguration("traffic_stop_flag_topic"),
                "traffic_stop_flag_timeout_sec": "0.5",
            }.items(),
        ),
    ])
    follower = Node(
        package="trajectory_follower", executable="lya_follower_connected_omegat_global",
        output="screen", remappings=filtered + [(VEHICLE_COMMAND, lya_command)],
    )
    adapter = ExecuteProcess(cmd=[sys.executable, str(directory / "guarded_stop_wait.py")], output="screen")
    return LaunchDescription([
        DeclareLaunchArgument("lane", choices=["inner", "outer"]),
        DeclareLaunchArgument("traffic_stop_flag_topic", default_value=PIXEL_STOP_TOPIC),
        DeclareLaunchArgument("red_pixel_stop_threshold", default_value="600"),
        OpaqueFunction(function=pixel_stop_source, args=[directory / "pixel_stop_flag.py"]),
        planner, switch, follower, adapter,
    ])
