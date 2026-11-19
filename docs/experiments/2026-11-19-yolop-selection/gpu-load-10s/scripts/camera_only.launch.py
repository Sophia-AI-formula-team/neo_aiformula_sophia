"""Start the experiment camera without the rest of the vehicle stack."""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def generate_launch_description():
    launchers = Path(get_package_share_directory("launchers"))
    spec = importlib.util.spec_from_file_location("bench_vehicle", launchers / "launch/allnodes.launch.py")
    vehicle = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(vehicle)
    config = Path(get_package_share_directory("vehicle")) / "config/zedx"
    parameters = {
        "config_common_path": str(config / "common.yaml"),
        "config_camera_path": str(config / "zedx.yaml"),
        "grab_resolution": "HD1080",
        "grab_frame_rate": "15",
        "pub_downscale_factor": "3.0",
        "use_sim_time": "false",
    }
    args = [DeclareLaunchArgument(name, default_value=value) for name, value in parameters.items()]
    return LaunchDescription(args + [OpaqueFunction(function=vehicle.create_zed_node)])
