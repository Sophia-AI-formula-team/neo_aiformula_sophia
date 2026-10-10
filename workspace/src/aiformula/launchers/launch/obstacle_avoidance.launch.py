"""Start the preserved avoidance algorithm with the existing point-source switch."""

import os.path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


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
            choices=["oa_neo", "oa_neo_nurbs", "oa_neo_ubs"],
            description="Existing obstacle-avoidance executable; oa_neo uses a B-spline.",
        ),
        ExecuteProcess(
            name="obstacle_avoidance",
            cmd=[
                sys.executable,
                runner,
                LaunchConfiguration("algorithm"),
                "--ros-args",
                "-r", "/processed_point_a:=/oa_point_a",
                "-r", "/processed_point_b:=/oa_point_b",
                "-r", "/processed_point_c:=/oa_point_c",
                "-r", "/lane_stop_flag:=/point_flag",
            ],
            output="screen",
        ),
    ])
