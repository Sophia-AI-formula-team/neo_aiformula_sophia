from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("semantic_bringup"), "config", "semantic_system.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="semantic_gate_ros",
                executable="traffic_light_gate_node",
                name="traffic_light_gate_node",
                parameters=[config],
            ),
            Node(
                package="semantic_costmap_ros",
                executable="local_costmap_node",
                name="local_costmap_node",
                parameters=[config],
            ),
            Node(
                package="semantic_local_planner_ros",
                executable="local_primitive_planner_node",
                name="local_primitive_planner_node",
                parameters=[config],
            ),
            Node(
                package="semantic_lane_recovery_ros",
                executable="lane_recovery_gate_node",
                name="lane_recovery_gate_node",
                parameters=[config],
            ),
            Node(
                package="semantic_safety_ros",
                executable="safety_supervisor_node",
                name="safety_supervisor_node",
                parameters=[config],
            ),
        ]
    )
