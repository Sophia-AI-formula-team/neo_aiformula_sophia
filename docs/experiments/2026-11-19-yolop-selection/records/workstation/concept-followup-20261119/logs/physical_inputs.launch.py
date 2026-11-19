"""Receive physical vehicle inputs and run the Concept proposal path in an isolated domain."""
import runpy
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from concept.bringup import include, share
from launch.actions import OpaqueFunction


def generate_launch_description():
    native = runpy.run_path(share('launchers', 'launch', 'allnodes.launch.py'))
    arguments = dict(can_interface='can0', receiver_interval_sec='0.1', enable_can_fd='false',
                     use_sim_time='false', grab_resolution='HD1080', grab_frame_rate='15',
                     pub_downscale_factor='3.0', config_common_path=share('vehicle', 'config', 'zedx', 'common.yaml'),
                     config_camera_path=share('vehicle', 'config', 'zedx', 'zedx.yaml'))
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in arguments.items()] + [
        *native['create_socket_can_receiver'](),
        include('launchers', 'vectornav.launch.py'),
        Node(package='concept_vehicle_state', executable='concept_vehicle_state'),
        OpaqueFunction(function=lambda context: list(native['create_zed_node'](context))),
        include('concept_yolop', 'concept_yolop.launch.py', calibration=str(Path.home()/'.config/concept/honda-ground.json')),
        Node(package='concept_drive', executable='drive', parameters=[{'target_speed_mps': 1.0}]),
    ])
