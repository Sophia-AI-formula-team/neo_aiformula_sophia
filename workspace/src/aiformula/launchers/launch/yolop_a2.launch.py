"""Run YOLOP A2 separately, publishing the existing lane and obstacle inputs.

Use allnodes with launch_legacy_road_detector:=false and stop other publishers
of the lane-mask or /cone_* topics before starting this launch. Traffic-light
outputs are isolated by default so they do not change the existing semantic
pipeline until that integration is explicitly selected and validated.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():
    runtime_dir = LaunchConfiguration("runtime_dir")
    runtime_python_path = PathJoinSubstitution([
        runtime_dir, "ros2", "yolop_a2_perception",
    ])

    arguments = [
        DeclareLaunchArgument(
            "runtime_dir",
            default_value=(
                "/home/workspace/src/aiformula/perception/road_detector/"
                "road_detector/YOLOP_A2_Deploy"
            ),
            description="Absolute directory containing the preserved A2 runtime and weights.",
        ),
        DeclareLaunchArgument(
            "python_executable",
            default_value=(
                "/home/control/aiformula_runtime/yolop_a2_20261005/venv/bin/python3"
            ),
            description="Isolated Python interpreter with compatible Jetson inference libraries.",
        ),
        DeclareLaunchArgument(
            "lane_weights",
            default_value=PathJoinSubstitution([runtime_dir, "weights", "lane_epoch_282.pth"]),
        ),
        DeclareLaunchArgument(
            "object_weights",
            default_value=PathJoinSubstitution([
                runtime_dir, "weights", "object_detector_usable.pth",
            ]),
        ),
        DeclareLaunchArgument("device", default_value="auto"),
        DeclareLaunchArgument("object_score", default_value="0.50"),
        DeclareLaunchArgument("cone_score", default_value="0.50"),
        DeclareLaunchArgument(
            "input_image_topic",
            default_value="/aiformula_sensing/zed_node/left_image/undistorted",
        ),
        DeclareLaunchArgument(
            "traffic_light_topic",
            default_value="/aiformula_perception/yolop_a2/traffic_light/state",
            description="Set /traffic_light/state only after selecting one validated light source.",
        ),
        DeclareLaunchArgument(
            "red_pixels_count_topic",
            default_value="/aiformula_perception/yolop_a2/red_pixels_count",
            description="A2 traffic-light red-pixel count; isolated from the existing adapter.",
        ),
    ]

    detector = ExecuteProcess(
        name="yolop_a2_perception",
        cmd=[
            LaunchConfiguration("python_executable"),
            "-m", "yolop_a2_perception.node",
            "--ros-args",
            "-r", "__node:=yolop_a2_perception",
            "-r", "__ns:=/aiformula_perception",
            "-p", ["lane_weights:=", LaunchConfiguration("lane_weights")],
            "-p", ["object_weights:=", LaunchConfiguration("object_weights")],
            "-p", ["device:=", LaunchConfiguration("device")],
            "-p", ["object_score:=", LaunchConfiguration("object_score")],
            "-p", ["cone_score:=", LaunchConfiguration("cone_score")],
            "-p", ["input_image_topic:=", LaunchConfiguration("input_image_topic")],
            "-r", "pub_mask_image:=/aiformula_perception/road_detector/mask_image",
            "-r", (
                "pub_annotated_mask_image:="
                "/aiformula_visualization/road_detector/annotated_mask_image"
            ),
            "-r", ["/traffic_light/state:=", LaunchConfiguration("traffic_light_topic")],
            "-r", ["/red_pixels_count:=", LaunchConfiguration("red_pixels_count_topic")],
        ],
        additional_env={
            "PYTHONPATH": [
                runtime_dir,
                os.pathsep,
                runtime_python_path,
                os.pathsep,
                EnvironmentVariable("PYTHONPATH", default_value=""),
            ],
            "PYTHONUNBUFFERED": "1",
            # Match the Jetson OpenMP preload already used by allnodes. Each
            # independently started launch needs its own process environment.
            "LD_PRELOAD": "/lib/aarch64-linux-gnu/libgomp.so.1",
        },
        output="screen",
    )

    return LaunchDescription(arguments + [
        LogInfo(msg=(
            "Starting A2 as the lane-mask and /cone_* publisher. "
            "The legacy road detector and separate cone detector must be stopped."
        )),
        detector,
    ])
