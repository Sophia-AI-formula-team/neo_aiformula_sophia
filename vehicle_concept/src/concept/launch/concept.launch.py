"""Start Concept in the vehicle or isolated replay mode."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from concept.bringup import build, share


def generate_launch_description():
    defaults = {
        'mode': 'vehicle', 'panel': 'true', 'rviz': 'true',
        'target_speed_mps': '1.0', 'max_auto_speed_mps': '1.0',
        'weights': '', 'calibration': '', 'device': 'cuda',
        'camera_info_topic': '/aiformula_sensing/zed_node/left/camera_info',
        'can_interface': 'can0', 'can_ready_timeout_sec': '10', 'enable_can_fd': 'false',
        'receiver_interval_sec': '0.1', 'sender_timeout_sec': '0.01',
        'use_sim_time': 'false', 'grab_resolution': 'HD1080', 'grab_frame_rate': '15',
        'pub_downscale_factor': '3.0',
        'config_common_path': share('vehicle', 'config', 'zedx', 'common.yaml'),
        'config_camera_path': share('vehicle', 'config', 'zedx', 'zedx.yaml'),
    }
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=build)])
