"""Start Concept in the vehicle or isolated replay mode."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from concept.bringup import build, share


def generate_launch_description():
    defaults = {
        'mode': 'vehicle', 'onboard_camera': 'auto', 'panel': 'true', 'rviz': 'true',
        'target_speed_mps': '', 'max_auto_speed_mps': '', 'planning_hz': '15.0',
        'weights': '', 'calibration': 'auto', 'device': 'cuda', 'cuda_fusion': 'auto',
        'camera_info_topic': '/aiformula_sensing/zed_node/left/camera_info',
        'can_interface': 'can0', 'can_ready_timeout_sec': '10', 'enable_can_fd': 'false',
        'receiver_interval_sec': '0.1', 'sender_timeout_sec': '0.01',
        'use_sim_time': 'false', 'grab_resolution': 'HD1080', 'grab_frame_rate': '60',
        'pub_downscale_factor': '3.0',
        'config_common_path': share('vehicle', 'config', 'zedx', 'common.yaml'),
        'config_camera_path': share('vehicle', 'config', 'zedx', 'zedx.yaml'),
    }
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [DeclareLaunchArgument('tracking_controller', default_value='auto',
                                                      description='auto shows a terminal menu; pure_pursuit or lya selects directly'),
                                OpaqueFunction(function=build)])
