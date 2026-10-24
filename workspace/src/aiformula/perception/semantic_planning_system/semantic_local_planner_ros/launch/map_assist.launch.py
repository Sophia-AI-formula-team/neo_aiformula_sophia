"""One map-assistance node, gated directly by the existing avoidance flag."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("semantic_local_planner_ros")
    config = PathJoinSubstitution([share, "config", "map_assist.yaml"])
    official_map = PathJoinSubstitution([share, "config", "aiformula_topomap.osm"])
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=config),
        DeclareLaunchArgument("enable_correction", default_value="true"),
        DeclareLaunchArgument("avoidance_flag_topic", default_value="/lane_stop_flag"),
        DeclareLaunchArgument("route_node_ids", default_value=""),
        Node(
            package="semantic_local_planner_ros",
            executable="map_assist",
            name="map_assist",
            output="screen",
            parameters=[LaunchConfiguration("params_file"), {
                "topomap_path": official_map,
                "avoidance_flag_topic": ParameterValue(LaunchConfiguration("avoidance_flag_topic"), value_type=str),
                "route_node_ids": ParameterValue(LaunchConfiguration("route_node_ids"), value_type=str),
                "enable_correction": ParameterValue(LaunchConfiguration("enable_correction"), value_type=bool),
            }],
        ),
    ])
