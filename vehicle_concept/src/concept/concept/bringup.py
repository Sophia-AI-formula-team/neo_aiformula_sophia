"""Vehicle wiring from neo main d8277bd. Reuse native camera and lifecycle CAN actions."""
import runpy
import tempfile
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


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


def sensor_actions(context):
    import xacro

    native = runpy.run_path(share('launchers', 'launch', 'allnodes.launch.py'))
    native['wait_for_can_interface'](context)
    joy_config = share('launchers', 'config', 'joy.yaml')
    buttons = share('launchers', 'config', 'gamepad', 'dualshock4.yaml')
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
        *native['create_zed_node'](context),
        include('launchers', 'vectornav.launch.py'),
        include('odometry_publisher', 'gyro_odometry_publisher.launch.py', use_rviz='false',
                imu_topic='/aiformula_sensing/vectornav/imu'),
        include('rear_potentiometer', 'rear_potentiometer.launch.py'),
        Node(package='joy', executable='joy_node', name='joy_node', namespace='/aiformula_control',
             parameters=[joy_config], remappings=[('joy', '/aiformula_control/joy_node/joy')]),
        Node(package='launchers', executable='l2_throttle_joy_remap.py', name='l2_throttle_joy_remap',
             namespace='/aiformula_control',
             remappings=[('joy_in', '/aiformula_control/joy_node/joy'), ('joy_out', remap)]),
        Node(package='teleop_twist_joy', executable='teleop_node', namespace='/aiformula_control',
             parameters=[buttons], remappings=[('joy', remap), ('cmd_vel', '/concept/manual_preview')]),
        *native['create_socket_can_receiver'](), *native['create_socket_can_sender'](),
    ]


def build(context):
    mode = LC('mode').perform(context)
    if mode not in ('vehicle', 'replay'):
        raise ValueError('Select mode:=vehicle or mode:=replay')
    vehicle = mode == 'vehicle'
    simulated_clock = LC('use_sim_time').perform(context).lower() == 'true'
    if vehicle and simulated_clock:
        raise ValueError('Vehicle mode requires the vehicle clock.')
    actions = []
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
    actions.extend([
        Node(package='motor_controller', executable='motor_controller', name='motor_controller', namespace='/concept/hardware',
             parameters=[share('vehicle','config','wheel.yaml'), share('motor_controller','config','motor_controller.yaml'), buttons],
             remappings=motor_remaps),
        Node(package='concept_vehicle_state', executable='concept_vehicle_state', output='screen'),
        Node(package='concept_control_gate', executable='concept_control_gate', output='screen',
             parameters=[buttons, {'max_auto_speed_mps': float(LC('max_auto_speed_mps').perform(context)),
                                  'can_sender_node': '/aiformula_control/socket_can_sender' if vehicle else '/concept/replay/virtual_receiver'}],
             remappings=[] if vehicle else [(physical, '/concept/replay/reference_signal')]),
        include('concept_yolop', 'concept_yolop.launch.py', weights=LC('weights'),
                calibration=LC('calibration'), device=LC('device'),
                camera_info_topic=LC('camera_info_topic')),
        Node(package='concept_drive', executable='drive', output='screen', respawn=True, respawn_delay=2.,
             additional_env={'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'},
             parameters=[{'target_speed_mps': float(LC('target_speed_mps').perform(context)),
                          'planning_hz': float(LC('planning_hz').perform(context))}]),
        Node(package='concept_visualization', executable='visualization', output='screen'),
    ])
    if LC('panel').perform(context).lower() in ('true', '1'):
        actions.append(Node(package='concept_panel', executable='panel', output='screen'))
    if LC('rviz').perform(context).lower() in ('true', '1'):
        actions.append(Node(package='rviz2', executable='rviz2', name='concept_rviz',
                            arguments=['-d', share('concept_visualization', 'rviz', 'concept.rviz')]))
    return actions
