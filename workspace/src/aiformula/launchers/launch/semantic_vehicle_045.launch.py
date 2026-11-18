"""Connect MapAssist 0.4.5 to the vehicle topics for held-still commissioning."""

from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PREFIX = "/semantic_drive/map_assist_045"
LANE_RADII = {"inner": 6.0, "outer": 5.5}
AVOIDANCE_TOPIC = "/point_flag"
BRANCH_TOPIC = "/semantic_drive/branch_active"


def _map_assist_node(context):
    lane = LaunchConfiguration("lane").perform(context)
    stop_topic = LaunchConfiguration("traffic_stop_flag_topic").perform(context)
    if lane not in LANE_RADII:
        raise ValueError("lane must be inner or outer")
    if not re.fullmatch(r"(?:/[A-Za-z_][A-Za-z0-9_]*)+", stop_topic):
        raise ValueError("Set traffic_stop_flag_topic to the verified absolute std_msgs/msg/Bool topic")
    if stop_topic in {AVOIDANCE_TOPIC, BRANCH_TOPIC, "/lane_stop_flag", PREFIX + "/reset"}:
        raise ValueError("traffic_stop_flag_topic must be a separate distance flag topic")

    share = Path(get_package_share_directory("semantic_local_planner_ros"))
    overrides = {
        "use_sim_time": False,
        "enable_traffic_control": True,
        "enable_velocity_gate": True,
        "avoidance_initially_idle": False,
        "odom_topic": "/aiformula_sensing/gyro_odometry_publisher/odom",
        "traffic_light_topic": "/aiformula_perception/yolop_a3/traffic_light/state",
        "traffic_stop_flag_topic": stop_topic,
        "avoidance_flag_topic": AVOIDANCE_TOPIC,
        "branch_active_topic": BRANCH_TOPIC,
        "reset_topic": PREFIX + "/reset",
        "status_topic": PREFIX + "/status",
        "reference_path_topic": PREFIX + "/path",
        "input_cmd_vel_topic": "/aiformula_control/game_pad/cmd_vel",
        "output_cmd_vel_topic": PREFIX + "/diagnostic_cmd_vel",
        "detour_min_turn_radius_m": LANE_RADII[lane],
    }
    for label in "abc":
        overrides[f"input_point_{label}_topic"] = f"/processed_point_{label}"
        overrides[f"output_point_{label}_topic"] = f"{PREFIX}/point_{label}"

    # Keep the name used by the installed YAML. The separate status/command
    # topics keep the running vehicle gate and LYA independent of this node.
    return [Node(package="semantic_local_planner_ros", executable="map_assist", name="map_assist",
                 output="screen", parameters=[str(share / "config" / "map_assist.yaml"), overrides],
                 remappings=[("/map_assist/detour_path", PREFIX + "/detour_path")])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("lane", choices=["inner", "outer"],
                              description="Select the minimum radius for the commissioned lane."),
        DeclareLaunchArgument("traffic_stop_flag_topic",
                              description="Confirmed vehicle publisher of the continuous distance Bool."),
        OpaqueFunction(function=_map_assist_node),
    ])
