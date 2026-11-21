"""Start the preserved avoidance algorithm with the existing point-source switch."""

import os.path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def avoidance_process(context, runner):
    algorithm = LaunchConfiguration("algorithm").perform(context)
    manual_topic = LaunchConfiguration("manual_state_topic").perform(context)
    return [
        ExecuteProcess(
            name="obstacle_avoidance",
            cmd=[
                sys.executable,
                runner,
                algorithm,
                "--ros-args",
                "-r", "/processed_point_a:=/oa_point_a",
                "-r", "/processed_point_b:=/oa_point_b",
                "-r", "/processed_point_c:=/oa_point_c",
                "-r", "/lane_stop_flag:=/point_flag",
                *(
                    ["-p", f"manual_state_topic:={manual_topic}"]
                    if algorithm in ("oa_2_2", "oa_2_3") and manual_topic else []
                ),
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    runner = os.path.join(
        get_package_share_directory("launchers"),
        "shellscript",
        "run_obstacle_avoidance.py",
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "algorithm",
            default_value="oa_neo",
            choices=["oa_neo", "oa_neo_nurbs", "oa_neo_ubs", "oa_2", "oa_2_1", "oa_2_2", "oa_2_3"],
            description="Existing obstacle-avoidance executable; oa_neo uses a B-spline.",
        ),
        DeclareLaunchArgument("manual_state_topic", default_value=""),
        OpaqueFunction(function=avoidance_process, args=[runner]),
    ])
