"""Reuse the car configuration to start only the ZED driver."""
import runpy

from concept.bringup import share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def generate_launch_description():
    native = runpy.run_path(share('launchers', 'launch', 'allnodes.launch.py'))
    options = dict(use_sim_time='false', grab_resolution='HD1080',
                   grab_frame_rate='15', pub_downscale_factor='3.0',
                   config_common_path=share('vehicle', 'config', 'zedx', 'common.yaml'),
                   config_camera_path=share('vehicle', 'config', 'zedx', 'zedx.yaml'))
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value) for key, value in options.items()]
                             + [OpaqueFunction(function=lambda context: list(native['create_zed_node'](context)))])
