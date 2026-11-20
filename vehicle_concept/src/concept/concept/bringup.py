"""Vehicle wiring from neo main d8277bd. Reuse native camera and lifecycle CAN actions."""
import math
import runpy
import tempfile
import time

import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import (EmitEvent, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, SetLaunchConfiguration)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

from concept.controller_selection import select_controller


def share(package, *parts):
    return str(Path(get_package_share_directory(package), *parts))


def include(package, filename, **arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share(package, 'launch', filename)),
        launch_arguments=arguments.items())


def check_ownership(context):
    """Refuse a second motor chain before opening a physical CAN sender."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor

    ros = Context()
    rclpy.init(args=[], context=ros)
    probe = rclpy.create_node('concept_launch_probe', context=ros)
    executor = SingleThreadedExecutor(context=ros)
    executor.add_node(probe)
    try:
        deadline = time.monotonic() + 1.8
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
            # The public motor/sender may be in an older namespace on installed vehicles.
            owners = {name for name, ns in probe.get_node_names_and_namespaces()
                      if name in ('socket_can_sender', 'socket_can_receiver', 'motor_controller', 'zed_node', 'joy_node')}
            if owners or probe.count_publishers('/aiformula_control/motor_controller/reference_signal'):
                raise RuntimeError('Original vehicle stack already active: ' + ', '.join(sorted(owners))
                                   + '. Stop that launch before starting Concept.')
    finally:
        executor.shutdown()
        probe.destroy_node()
        rclpy.shutdown(context=ros)
    return []


def zed_actions(context, native, directory):
    # Keep sensor acquisition independent of the image traffic sent to ROS.
    grab_hz = float(LC('grab_frame_rate').perform(context))
    publish_hz = float(LC('pub_frame_rate').perform(context))
    if not all(math.isfinite(rate) and rate > 0 for rate in (grab_hz, publish_hz)):
        raise ValueError('ZED acquisition and publication rates must be positive and finite')
    original = Path(LC('config_common_path').perform(context))
    config = yaml.safe_load(original.read_text(encoding='utf-8'))
    general = config['/**']['ros__parameters']['general']
    general['pub_frame_rate'] = min(grab_hz, publish_hz)
    overlay = Path(directory, 'zed-common.yaml')
    overlay.write_text(yaml.safe_dump(config), encoding='utf-8')
    return GroupAction(actions=[SetLaunchConfiguration('config_common_path', str(overlay)),
                                *native['create_zed_node'](context)])


def sensor_actions(context):
    import xacro

    native = runpy.run_path(share('launchers', 'launch', 'allnodes.launch.py'))
    native['wait_for_can_interface'](context)
    joy_config = share('launchers', 'config', 'joy.yaml')
    remap = '/aiformula_control/joy_node/joy_l2_throttle'
    # The original launcher generates a URDF inside the installed vehicle share.
    # Render the same chassis into a private temporary file for this launch.
    description = xacro.process_file(share('vehicle', 'xacro', 'ai_car1.xacro')).toxml()
    temporary = tempfile.TemporaryDirectory(prefix='concept-chassis-')
    urdf = Path(temporary.name, 'ai_car1.urdf')
    urdf.write_text(description, encoding='utf-8')
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda _: temporary.cleanup())])),
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='/aiformula_sensing',
             parameters=[{'robot_description': ParameterValue(description, value_type=str)}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher', namespace='/aiformula_sensing',
             arguments=[str(urdf)]),
        zed_actions(context, native, temporary.name),
        include('launchers', 'vectornav.launch.py'),
        include('odometry_publisher', 'gyro_odometry_publisher.launch.py', use_rviz='false',
                imu_topic='/aiformula_sensing/vectornav/imu'),
        include('rear_potentiometer', 'rear_potentiometer.launch.py'),
        Node(package='joy', executable='joy_node', name='joy_node', namespace='/aiformula_control',
             parameters=[joy_config], remappings=[('joy', '/aiformula_control/joy_node/joy')]),
        Node(package='launchers', executable='l2_throttle_joy_remap.py', name='l2_throttle_joy_remap',
             namespace='/aiformula_control',
             remappings=[('joy_in', '/aiformula_control/joy_node/joy'), ('joy_out', remap)]),
        *native['create_socket_can_receiver'](), *native['create_socket_can_sender'](),
    ]


def build(context):
    mode = LC('mode').perform(context)
    if mode not in ('vehicle', 'replay'):
        raise ValueError('Select mode:=vehicle or mode:=replay')
    vehicle = mode == 'vehicle'
    tracking_controller = select_controller(LC('tracking_controller').perform(context))
    calibration = LC('calibration').perform(context)
    if calibration == 'auto':
        config_home = context.environment.get('XDG_CONFIG_HOME') or Path.home() / '.config'
        receipt = Path(config_home) / 'concept' / 'camera.json'
        if not receipt.is_file():
            receipt = receipt.with_name('honda-ground.json')
        calibration = str(receipt) if vehicle and receipt.is_file() else ''
    simulated_clock = LC('use_sim_time').perform(context).lower() == 'true'
    if vehicle and simulated_clock:
        raise ValueError('Vehicle mode requires the vehicle clock.')
    actions = [LogInfo(msg='Concept tracking controller: ' + tracking_controller)]
    if not context.environment.get('FASTRTPS_DEFAULT_PROFILES_FILE'):
        profile = 'vehicle-fastdds.xml' if vehicle else 'foxy-fastdds.xml'
        actions.append(SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                                              share('concept', 'config', profile)))
    actions.append(SetParameter(name='use_sim_time', value=simulated_clock))
    if vehicle:
        actions.extend([OpaqueFunction(function=check_ownership), OpaqueFunction(function=sensor_actions)])
    else:
        actions.append(Node(package='concept', executable='replay_sink'))
    physical = '/aiformula_control/motor_controller/reference_signal'
    buttons = share('launchers', 'config', 'gamepad', 'dualshock4.yaml')
    motor_remaps = [
        ('sub_speed_command', '/concept/internal/motor_cmd'),
        ('pub_can', '/concept/internal/motor_can'),
        ('/aiformula_control/joy_node/joy_l2_throttle', '/concept/internal/unused_motor_joy'),
        ('/semantic_drive/motor_session', '/concept/internal/unused_motor_session'),
        ('/semantic_drive/map_status', '/concept/internal/unused_motor_map_status'),
    ]
    direct = LC('onboard_camera').perform(context)
    if direct not in ('auto', 'true', 'false'):
        raise ValueError('onboard_camera must be auto, true or false')
    transport = LC('local_frame_transport', default='auto').perform(context).lower()
    if transport not in ('auto', 'true', 'false'):
        raise ValueError('local_frame_transport must be auto, true or false')
    local_transport = (vehicle and direct == 'auto') if transport == 'auto' else transport == 'true'
    direct = (vehicle and not local_transport) if direct == 'auto' else direct == 'true'
    if local_transport and direct:
        raise ValueError('local_frame_transport requires onboard_camera:=false or auto')
    if vehicle and local_transport and 'CONCEPT_CUDA_GRAPH' not in context.environment:
        board = Path('/proc/device-tree/model')
        if board.is_file() and b'Jetson AGX Orin Developer Kit' in board.read_bytes():
            actions.append(SetEnvironmentVariable('CONCEPT_CUDA_GRAPH', '1'))
    vision = dict(calibration=calibration, device=LC('device'), weights=LC('weights'), cuda_fusion=LC('cuda_fusion'),
                  camera_info_topic=LC('camera_info_topic'))
    driving = dict(onboard_camera=direct, local_frame_transport=local_transport,
                   planning_hz=float(LC('planning_hz').perform(context)),
                   min_nonzero_speed_mps=float(LC('min_nonzero_speed_mps').perform(context)),
                   lya_v_t_mps=float(LC('lya_v_t_mps').perform(context)),
                   profile_timing=LC('profile_timing').perform(context).lower() in ('true', '1'),
                   tracking_controller=tracking_controller)
    gate = dict(can_sender_node='/aiformula_control/socket_can_sender' if vehicle else '/concept/replay/virtual_receiver')
    for params, key in ((driving, 'target_speed_mps'), (gate, 'max_auto_speed_mps')):
        if vehicle:
            config_home = context.environment.get('XDG_CONFIG_HOME') or Path.home() / '.config'
            params['speed_settings_directory'] = str(Path(config_home) / 'concept' / 'speed')
        explicit = LC(key).perform(context)
        if explicit:
            params[key] = float(explicit)
    if direct:
        driving.update({key: ParameterValue(value, value_type=str) for key, value in vision.items()})
    else:
        actions.append(include('concept_yolop', 'concept_yolop.launch.py', **vision,
                               local_frame_transport='true' if local_transport else 'false',
                               profile_timing='true' if local_transport and driving['profile_timing'] else 'false'))
    actions.extend([
        Node(package='motor_controller', executable='motor_controller', name='motor_controller', namespace='/concept/hardware',
             parameters=[share('vehicle','config','wheel.yaml'), share('motor_controller','config','motor_controller.yaml'), buttons],
             remappings=motor_remaps),
        Node(package='concept_vehicle_state', executable='concept_vehicle_state', output='screen'),
        Node(package='concept_control_gate', executable='concept_control_gate', output='screen',
             parameters=[buttons, gate],
             remappings=[] if vehicle else [(physical, '/concept/replay/reference_signal')]),
        Node(package='concept_drive', executable='drive', output='screen', respawn=True, respawn_delay=2.,
             additional_env={'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'},
             parameters=[driving]),
        Node(package='concept_visualization', executable='visualization', output='screen'),
    ])
    if LC('panel').perform(context).lower() in ('true', '1'):
        # The panel alone owns a bag process. Allow startup (15 s), graceful
        # finalization (15 s), group cleanup and readback of a long recording.
        panel = Node(package='concept_panel', executable='panel', output='screen',
                     parameters=[{'disable_output_on_close': True}],
                     sigterm_timeout='45', sigkill_timeout='75')
        actions.extend([
            RegisterEventHandler(OnProcessExit(target_action=panel, on_exit=[
                EmitEvent(event=Shutdown(reason='Concept panel exited'))])),
            panel,
        ])
    rviz = LC('rviz').perform(context).lower()
    if rviz in ('true', '1') or (rviz == 'auto' and not vehicle):
        actions.append(Node(package='rviz2', executable='rviz2', name='concept_rviz',
                            arguments=['-d', share('concept_visualization', 'rviz', 'concept.rviz')]))
    return actions
