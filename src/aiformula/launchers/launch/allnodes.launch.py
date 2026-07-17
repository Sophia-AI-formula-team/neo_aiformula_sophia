import os.path as osp
import time
from pathlib import Path
from typing import Tuple

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

from common_python.launch_util import check_zedx_available_fps, get_frame_ids_and_topic_names
from vehicle.vehicle_util import get_zed_intrinsic_param_path


def wait_for_can_interface(context: LaunchContext) -> Tuple:
    interface = LaunchConfiguration("can_interface").perform(context)
    timeout_sec = float(LaunchConfiguration("can_ready_timeout_sec").perform(context))
    operstate_path = Path("/sys/class/net") / interface / "operstate"
    carrier_path = Path("/sys/class/net") / interface / "carrier"
    logger = get_logger("allnodes.can_ready")
    deadline = time.monotonic() + timeout_sec

    logger.info(f"Waiting up to {timeout_sec:.1f}s for CAN interface '{interface}'")
    while time.monotonic() < deadline:
        try:
            operstate = operstate_path.read_text(encoding="ascii").strip()
            carrier = carrier_path.read_text(encoding="ascii").strip()
        except OSError:
            operstate = "missing"
            carrier = "0"

        if operstate == "up" and carrier == "1":
            logger.info(f"CAN interface '{interface}' is ready")
            return ()
        time.sleep(0.5)

    raise RuntimeError(
        f"CAN interface '{interface}' did not become ready within {timeout_sec:.1f}s "
        f"(operstate={operstate}, carrier={carrier})"
    )


def create_zed_node(context: LaunchContext) -> Tuple[Node]:
    _, topic_names = get_frame_ids_and_topic_names()
    grab_resolution = LaunchConfiguration("grab_resolution").perform(context)
    grab_frame_rate = LaunchConfiguration("grab_frame_rate").perform(context)
    pub_downscale_factor = LaunchConfiguration("pub_downscale_factor").perform(context)
    is_valid_fps = check_zedx_available_fps(grab_resolution, grab_frame_rate)

    return (
        Node(
            package="zed_wrapper",
            executable="zed_wrapper",
            name="zed_node",
            namespace="/aiformula_sensing",
            output="screen",
            condition=IfCondition("true" if is_valid_fps else "false"),
            parameters=[
                LaunchConfiguration("config_common_path"),
                LaunchConfiguration("config_camera_path"),
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "general.grab_resolution": LaunchConfiguration("grab_resolution"),
                    "general.grab_frame_rate": int(grab_frame_rate),
                    "general.pub_downscale_factor": float(pub_downscale_factor),
                },
            ],
            remappings=[
                ("~/left/image_rect_color", topic_names["sensing"]["zedx"]["left_image"]["undistorted"]),
                ("~/right/image_rect_color", topic_names["sensing"]["zedx"]["right_image"]["undistorted"]),
                ("~/imu/data", topic_names["sensing"]["zedx"]["imu"]),
            ],
        ),
    )


def create_road_detector_node(context: LaunchContext) -> Tuple[Node]:
    _, topic_names = get_frame_ids_and_topic_names()
    road_detector_dir = get_package_share_directory("road_detector")

    return (
        Node(
            package="road_detector",
            executable="road_detector",
            name="road_detector",
            namespace="/aiformula_perception",
            output="screen",
            parameters=[
                osp.join(road_detector_dir, "config", "normalization.yaml"),
                {
                    "weight_path": LaunchConfiguration("weight_path"),
                    "use_architecture": LaunchConfiguration("use_architecture"),
                },
            ],
            remappings=[
                ("sub_image", topic_names["sensing"]["zedx"]["left_image"]["undistorted"]),
                ("pub_mask_image", topic_names["perception"]["mask_image"]),
                ("pub_annotated_mask_image", topic_names["visualization"]["annotated_mask_image"]),
            ],
        ),
    )


def create_lane_line_publisher_node(context: LaunchContext) -> Tuple[Node]:
    frame_ids, topic_names = get_frame_ids_and_topic_names()
    lane_line_dir = get_package_share_directory("lane_line_publisher")
    camera_sn = LaunchConfiguration("camera_sn").perform(context)
    camera_resolution = LaunchConfiguration("camera_resolution").perform(context)

    return (
        Node(
            package="lane_line_publisher",
            executable="lane_line_publisher",
            name="lane_line_publisher",
            output="screen",
            emulate_tty=True,
            parameters=[
                osp.join(lane_line_dir, "config", "lane_line_publisher.yaml"),
                get_zed_intrinsic_param_path(camera_sn, camera_resolution),
                {
                    "vehicle_frame_id": frame_ids["base_footprint"],
                    "camera_frame_id": LaunchConfiguration("camera_frame_id"),
                    "camera_name": "zedx",
                    "debug": LaunchConfiguration("lane_debug"),
                },
            ],
            remappings=[
                ("mask_image", topic_names["perception"]["mask_image"]),
                ("annotated_mask_image", topic_names["perception"]["annotated_mask_image"]),
                ("dynamic_roi_image", "/aiformula_perception/lane_line_publisher/dynamic_roi_image"),
                ("linear_fit_image", "/aiformula_perception/lane_line_publisher/linear_fit_image"),
                ("vehicle_fit_image", "/aiformula_perception/lane_line_publisher/vehicle_fit_image"),
                ("lane_lines/left", topic_names["perception"]["lane_lines"]["left"]),
                ("lane_lines/right", topic_names["perception"]["lane_lines"]["right"]),
                ("lane_lines/center", topic_names["perception"]["lane_lines"]["center"]),
                ("tf", "/tf"),
                ("tf_static", "/tf_static"),
            ],
        ),
    )


def create_socket_can_receiver():
    node = LifecycleNode(
        package="ros2_socketcan",
        executable="socket_can_receiver_node_exe",
        name="socket_can_receiver",
        namespace="/aiformula_sensing",
        output="screen",
        parameters=[
            {
                "interface": LaunchConfiguration("can_interface"),
                "enable_can_fd": LaunchConfiguration("enable_can_fd"),
                "interval_sec": LaunchConfiguration("receiver_interval_sec"),
                "filters": "0:0",
                "use_bus_time": False,
            }
        ],
        remappings=[("from_can_bus", "/aiformula_sensing/vehicle_info")],
    )
    configure = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        )
    )
    activate = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        )
    )
    return node, configure, activate


def create_socket_can_sender():
    node = LifecycleNode(
        package="ros2_socketcan",
        executable="socket_can_sender_node_exe",
        name="socket_can_sender",
        namespace="/aiformula_control",
        output="screen",
        parameters=[
            {
                "interface": LaunchConfiguration("can_interface"),
                "enable_can_fd": LaunchConfiguration("enable_can_fd"),
                "timeout_sec": LaunchConfiguration("sender_timeout_sec"),
            }
        ],
        remappings=[("to_can_bus", "/aiformula_control/motor_controller/reference_signal")],
    )
    configure = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        )
    )
    activate = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        )
    )
    return node, configure, activate


def generate_launch_description():
    launchers_dir = get_package_share_directory("launchers")
    vehicle_dir = get_package_share_directory("vehicle")
    motor_controller_dir = get_package_share_directory("motor_controller")
    odometry_publisher_dir = get_package_share_directory("odometry_publisher")
    rear_potentiometer_dir = get_package_share_directory("rear_potentiometer")

    launch_args = (
        DeclareLaunchArgument(
            "weight_path",
            default_value="/home/workspace/src/aiformula/perception/road_detector/weights/123.pth",
        ),
        DeclareLaunchArgument("use_architecture", default_value="0"),
        DeclareLaunchArgument("camera_sn", default_value="SN48442725"),
        DeclareLaunchArgument("camera_resolution", default_value="nHD"),
        DeclareLaunchArgument("camera_frame_id", default_value="zed_left_camera_optical_frame"),
        DeclareLaunchArgument("grab_resolution", default_value="HD1080"),
        DeclareLaunchArgument("grab_frame_rate", default_value="15"),
        DeclareLaunchArgument("pub_downscale_factor", default_value="3.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "odometry_imu_topic",
            default_value="/aiformula_sensing/vectornav/imu",
        ),
        DeclareLaunchArgument("vehicle_name", default_value="ai_car1"),
        DeclareLaunchArgument("use_joint_state_publisher", default_value="true"),
        DeclareLaunchArgument("lane_debug", default_value="true"),
        DeclareLaunchArgument("game_pad", default_value="dualshock4"),
        DeclareLaunchArgument(
            "button_layout_config",
            default_value=[
                osp.join(launchers_dir, "config", "gamepad", ""),
                LaunchConfiguration("game_pad"),
                ".yaml",
            ],
        ),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("can_ready_timeout_sec", default_value="120.0"),
        DeclareLaunchArgument("receiver_interval_sec", default_value="0.1"),
        DeclareLaunchArgument("sender_timeout_sec", default_value="0.01"),
        DeclareLaunchArgument("enable_can_fd", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("launch_image_view", default_value="true"),
        DeclareLaunchArgument(
            "image_view_topic",
            default_value="/aiformula_perception/lane_line_publisher/vehicle_fit_image",
        ),
        DeclareLaunchArgument(
            "config_common_path",
            default_value=osp.join(vehicle_dir, "config", "zedx", "common.yaml"),
        ),
        DeclareLaunchArgument(
            "config_camera_path",
            default_value=osp.join(vehicle_dir, "config", "zedx", "zedx.yaml"),
        ),
    )

    rviz_on_ready = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    "/home/workspace/src/aiformula/perception/lane_line_publisher/rviz/2x2.rviz",
                ],
                output="both",
                additional_env={
                    "DISPLAY": ":0",
                    "XAUTHORITY": "/home/control/.Xauthority",
                },
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
            ),
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                arguments=[LaunchConfiguration("image_view_topic")],
                output="both",
                additional_env={
                    "DISPLAY": ":0",
                    "XAUTHORITY": "/home/control/.Xauthority",
                },
                condition=IfCondition(LaunchConfiguration("launch_image_view")),
            ),
        ],
    )

    vehicle_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(vehicle_dir, "launch", "extrinsic_tfstatic_broadcaster.launch.py")
        ),
        launch_arguments={
            "vehicle_name": LaunchConfiguration("vehicle_name"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_joint_state_publisher": LaunchConfiguration("use_joint_state_publisher"),
            "use_gui": "false",
        }.items(),
    )

    vectornav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(launchers_dir, "launch", "vectornav.launch.py")
        ),
    )

    gyro_odometry_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(odometry_publisher_dir, "launch", "gyro_odometry_publisher.launch.py")
        ),
        launch_arguments={
            "use_rviz": "false",
            "imu_topic": LaunchConfiguration("odometry_imu_topic"),
        }.items(),
    )

    rear_potentiometer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(rear_potentiometer_dir, "launch", "rear_potentiometer.launch.py")
        ),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        namespace="/aiformula_control",
        parameters=[osp.join(launchers_dir, "config", "joy.yaml")],
        remappings=[("joy", "/aiformula_control/joy_node/joy")],
    )

    joy_remap_node = Node(
        package="launchers",
        executable="l2_throttle_joy_remap.py",
        name="l2_throttle_joy_remap",
        namespace="/aiformula_control",
        parameters=[
            {
                "forward_axis": 2,
                "reverse_axis": 5,
                "target_axis": 1,
                "enable_button": 6,
                "mode_switch_axis": 7,
                "mode_switch_threshold": 0.5,
                "deadband": 0.01,
            }
        ],
        remappings=[
            ("joy_in", "/aiformula_control/joy_node/joy"),
            ("joy_out", "/aiformula_control/joy_node/joy_l2_throttle"),
        ],
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        namespace="/aiformula_control",
        parameters=[LaunchConfiguration("button_layout_config")],
        remappings=[
            ("joy", "/aiformula_control/joy_node/joy_l2_throttle"),
            ("cmd_vel", "/aiformula_control/game_pad/cmd_vel"),
        ],
    )

    motor_controller = Node(
        package="motor_controller",
        executable="motor_controller",
        name="motor_controller",
        namespace="/aiformula_control",
        output="screen",
        emulate_tty=True,
        parameters=[
            osp.join(vehicle_dir, "config", "wheel.yaml"),
            osp.join(motor_controller_dir, "config", "motor_controller.yaml"),
        ],
        remappings=[
            ("sub_speed_command", "/aiformula_control/game_pad/cmd_vel"),
            ("pub_can", "/aiformula_control/motor_controller/reference_signal"),
        ],
    )

    can_receiver, can_receiver_configure, can_receiver_activate = create_socket_can_receiver()
    can_sender, can_sender_configure, can_sender_activate = create_socket_can_sender()

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
            SetEnvironmentVariable(name="LD_PRELOAD", value="/lib/aarch64-linux-gnu/libgomp.so.1"),
            *launch_args,
            OpaqueFunction(function=wait_for_can_interface),
            vehicle_description,
            OpaqueFunction(function=create_zed_node),
            vectornav,
            OpaqueFunction(function=create_road_detector_node),
            OpaqueFunction(function=create_lane_line_publisher_node),
            Node(
                package="lane_points",
                executable="lane_0215",
                output="screen",
            ),
            Node(
                package="kalman_filter",
                executable="withoutkalman_0312",
                output="screen",
            ),
            joy_node,
            joy_remap_node,
            teleop_node,
            motor_controller,
            can_receiver,
            can_receiver_configure,
            can_receiver_activate,
            can_sender,
            can_sender_configure,
            can_sender_activate,
            gyro_odometry_publisher,
            rear_potentiometer,
            rviz_on_ready,
        ]
    )
