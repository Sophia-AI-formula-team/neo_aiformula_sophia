from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'weights': '', 'device': 'cuda', 'cuda_fusion': 'auto',
        'calibration': '', 'camera_info_topic': '/aiformula_sensing/zed_node/left/camera_info',
        'image_topic': '/aiformula_sensing/zed_node/left_image/undistorted',
        'frame_topic': '/concept/perception/frame', 'overlay_topic': '/concept/perception/overlay',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        Node(package='concept_yolop', executable='concept_yolop_node', output='screen',
             parameters=[{k: ParameterValue(LaunchConfiguration(k), value_type=str) for k in defaults}]),
    ])
