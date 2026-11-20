"""One optional MapAssist stage. Allnodes, YOLOP and LYA remain operator-owned."""
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    observer = ExecuteProcess(
        cmd=[sys.executable, "-m", "oa_1b.observer",
             "--threshold", LaunchConfiguration("red_pixel_stop_threshold")],
        output="screen",
    )
    commissioning = Path(get_package_share_directory("launchers")) / "launch/semantic_045_commissioning.launch.py"
    mapassist = GroupAction([
        SetRemap(src="/aiformula_perception/yolop_a3/traffic_light/state", dst="/oa_1b/traffic_light/state"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(commissioning)),
            launch_arguments={
                "lane": LaunchConfiguration("lane"),
                "start_oa2": "true",
                "oa_algorithm": "oa_2_1",
                "oa_initially_idle": "false",
                "start_lya": "false",
                "vehicle_output": LaunchConfiguration("vehicle_output"),
                "traffic_stop_flag_topic": "/oa_1b/flag_stop",
                "lane_point_topic_prefix": "/original_processed_point_",
            }.items(),
        ),
    ])
    ready = False
    previous = b""

    def on_stdout(event):
        nonlocal ready, previous
        if ready:
            return []
        previous += event.text
        lines = previous.split(b"\n")
        previous = lines.pop()
        if b"OA1B_CAMERA_READY" in lines:
            ready = True
            return [mapassist]
        return []

    def startup_timeout(_context):
        if ready:
            return []
        return [EmitEvent(event=Shutdown(reason="No fresh YOLOP detection and camera pair. Start yolop first."))]

    return LaunchDescription([
        DeclareLaunchArgument("lane", default_value="outer", choices=["inner", "outer"]),
        DeclareLaunchArgument("red_pixel_stop_threshold", default_value="50"),
        DeclareLaunchArgument("vehicle_output", default_value="false", choices=["true", "false"]),
        RegisterEventHandler(OnProcessIO(target_action=observer, on_stdout=on_stdout)),
        RegisterEventHandler(OnProcessExit(
            target_action=observer, on_exit=[EmitEvent(event=Shutdown(reason="OA 1B observer exited."))])),
        TimerAction(period=45.0, actions=[OpaqueFunction(function=startup_timeout)]),
        observer,
    ])
