"""Start the existing visual drive chain, optionally with map assistance.

Avoidance, camera, detection and odometry remain externally launched. The
disabled mode preserves the three original nodes' topic names. Empty route
arguments leave the route values in params_file untouched.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _boolean_argument(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off"):
        return False
    raise ValueError(f"{name} must be true or false, received {value!r}")


def _launch_nodes(context):
    enabled = _boolean_argument(context, "use_map_assist")
    lane_remappings = []
    controller_remappings = []
    if enabled:
        lane_remappings = [
            (f"/processed_point_{label}", f"/original_processed_point_{label}")
            for label in "abc"
        ]
        controller_remappings = [
            ("/aiformula_control/game_pad/cmd_vel", "/map_assist/controller_cmd_vel")
        ]

    nodes = [
        Node(
            package="lane_points",
            executable="lane_0529oa",
            output="screen",
            remappings=lane_remappings,
        ),
        Node(
            package="kalman_filter",
            executable="withoutkalman_0312",
            output="screen",
        ),
        Node(
            package="trajectory_follower",
            executable="lya_follower_connected_omegat_global",
            output="screen",
            remappings=controller_remappings,
        ),
    ]
    if not enabled:
        return nodes

    share = FindPackageShare("semantic_local_planner_ros")
    parameters = {
        "topomap_path": PathJoinSubstitution([share, "config", "aiformula_topomap.osm"]),
        "enable_correction": _boolean_argument(context, "enable_correction"),
        "avoidance_initially_idle": _boolean_argument(context, "avoidance_initially_idle"),
        "enable_traffic_control": True,
        "enable_velocity_gate": True,
        "input_cmd_vel_topic": "/map_assist/controller_cmd_vel",
        "output_cmd_vel_topic": "/aiformula_control/game_pad/cmd_vel",
        "traffic_light_topic": ParameterValue(
            LaunchConfiguration("traffic_light_topic").perform(context), value_type=str
        ),
    }
    for label in "abc":
        parameters[f"input_point_{label}_topic"] = f"/original_processed_point_{label}"
        parameters[f"output_point_{label}_topic"] = f"/processed_point_{label}"
    route_node_ids = LaunchConfiguration("route_node_ids").perform(context).strip()
    if route_node_ids:
        parameters["route_node_ids"] = ParameterValue(route_node_ids, value_type=str)
    nodes.append(Node(
        package="semantic_local_planner_ros",
        executable="map_assist",
        name="map_assist",
        output="screen",
        parameters=[LaunchConfiguration("params_file").perform(context), parameters],
    ))
    return nodes


def generate_launch_description():
    share = FindPackageShare("semantic_local_planner_ros")
    config = PathJoinSubstitution([share, "config", "map_assist.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=config,
                              description="Map-assistance parameter file, including the selected routes."),
        DeclareLaunchArgument("use_map_assist", default_value="true",
                              description="False starts the original three-node chain with its original topics."),
        DeclareLaunchArgument("enable_correction", default_value="true",
                              description="Enable bounded map correction of visual lane points."),
        DeclareLaunchArgument("avoidance_initially_idle", default_value="false",
                              description="Explicitly confirm OA is idle at cold start; otherwise wait for its False event."),
        DeclareLaunchArgument("traffic_light_topic", default_value="/map_assist/traffic_light_input",
                              description="String traffic-light input; connect the actual detector topic."),
        DeclareLaunchArgument("route_node_ids", default_value="",
                              description="Optional outer-route IDs; empty preserves params_file or uses the OSM outer loop."),
        OpaqueFunction(function=_launch_nodes),
    ])
