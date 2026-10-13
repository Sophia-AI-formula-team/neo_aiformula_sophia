"""Observe the existing lane/LYA chain; motor_controller owns live arbitration.

Starts no motor, LYA, camera, or lane pipeline. Use with allnodes and ONE LYA.
MapAssist's gated output is diagnostic only; its real odometry lap is retained.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = {
        'use_sim_time': False,
        'checkpoint_path': '~/.ros/aiformula/topo_map/checkpoint.json',
        'traffic_policy': 'stop_wait',
        'enable_traffic_control': True,
        'enable_signal_approach': False,
        'enable_velocity_gate': True,
        'enable_correction': False,
        'avoidance_initially_idle': True,
        'avoidance_flag_topic': '/lane_stop_flag',
        'odom_topic': '/aiformula_sensing/gyro_odometry_publisher/odom',
        'traffic_light_topic': '/semantic_drive/unused_legacy_light',
        'signal_observation_topic': '/semantic_drive/map_signal',
        'input_cmd_vel_topic': '/aiformula_control/game_pad/cmd_vel',
        'output_cmd_vel_topic': '/semantic_drive/topo_diagnostic_cmd_vel',
        'status_topic': '/semantic_drive/map_status',
        'reference_path_topic': '/semantic_drive/reference_path',
        'reset_topic': '/semantic_drive/reset',
        'branch_active_topic': '/semantic_drive/branch_active',
    }
    for label in 'abc':
        params['input_point_' + label + '_topic'] = '/processed_point_' + label
        params['output_point_' + label + '_topic'] = '/semantic_drive/topo_point_' + label
    return LaunchDescription([
        Node(package='semantic_vehicle_wiring', executable='motor_gate_session',
             name='motor_gate_session', namespace='/semantic_drive', output='screen'),
        Node(package='semantic_vehicle_wiring', executable='a3_signal_capture',
             name='a3_signal_capture', namespace='/semantic_drive', output='screen',
             parameters=[{'expected_frame_id': 'zed_left_camera_optical_frame'}]),
        Node(package='semantic_drive_bridge', executable='signal_capture_bridge',
             name='signal_capture_bridge', namespace='/semantic_drive', output='screen',
             parameters=[{'input_topic': '/semantic_drive/signal_capture',
                          'output_topic': '/semantic_drive/map_signal',
                          'expected_frame_id': 'zed_left_camera_optical_frame', 'max_age_sec': 0.5}]),
        Node(package='semantic_local_planner_ros', executable='map_assist',
             name='map_assist', namespace='/semantic_drive', output='screen', parameters=[params]),
    ])
