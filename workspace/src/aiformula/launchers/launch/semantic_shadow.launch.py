"""Run the native neo lane/LYA/MapAssist chain on private shadow topics only.

Calibrated masks, odometry, camera TF and atomic metric signal captures are
external inputs. This entry deliberately has no motor, teleop or live mode.
Pose2D/Twist source provenance is still incomplete; see SEMANTIC_DEPLOYMENT.md.
"""

from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PREFIX = "/semantic_drive"
LIVE_MOTOR_TOPIC = "/aiformula_control/game_pad/cmd_vel"


def _value(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _boolean(context, name):
    value = _value(context, name).lower()
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off"):
        return False
    raise ValueError("{} must be true or false".format(name))


def _absolute_topic(value, name):
    if not re.fullmatch(r"/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*", value):
        raise ValueError("{} must be an absolute ROS topic name".format(name))
    return value


def _launch_nodes(context):
    camera_file = _value(context, "camera_params_file")
    if not camera_file or not Path(camera_file).is_file():
        raise ValueError("camera_params_file must name an existing calibrated ROS parameter YAML")
    camera_frame = _value(context, "camera_frame_id")
    signal_frame = _value(context, "expected_frame_id")
    camera_name = _value(context, "camera_name")
    if not camera_frame or not signal_frame or not camera_name:
        raise ValueError("camera_frame_id, expected_frame_id and camera_name must be nonblank")
    mask_topic = _absolute_topic(_value(context, "lane_mask_topic"), "lane_mask_topic")
    odom_topic = _absolute_topic(_value(context, "odom_topic"), "odom_topic")
    output_topic = _absolute_topic(_value(context, "shadow_output_topic"), "shadow_output_topic")
    if output_topic == LIVE_MOTOR_TOPIC or output_topic.startswith("/aiformula_control/"):
        raise ValueError("semantic_shadow rejects live motor/control command topics")
    if not output_topic.startswith(PREFIX + "/shadow_"):
        raise ValueError("shadow_output_topic must remain under /semantic_drive/shadow_*")
    use_sim_time = _boolean(context, "use_sim_time")
    common_parameters = {"use_sim_time": use_sim_time}
    lane_config = str(Path(get_package_share_directory("lane_line_publisher")) /
                      "config" / "lane_line_publisher.yaml")

    lane_remaps = [
        ("/aiformula_perception/lane_line_publisher/lane_lines/" + side,
         PREFIX + "/lane_lines/" + side)
        for side in ("left", "right", "center")
    ]
    lane_remaps += [("/processed_point_" + label, PREFIX + "/original_point_" + label)
                    for label in "abc"]
    filter_remaps = [("/processed_point_" + label, PREFIX + "/processed_point_" + label)
                     for label in "abc"]
    # The shadow chain does not start OA or subscribe to the live OA flags.
    filter_remaps += [("/oa_point_" + label, PREFIX + "/unused_oa_point_" + label)
                      for label in "abc"]
    filter_remaps += [
        ("/point_flag", PREFIX + "/unused_point_flag"),
        ("filtered_lane_pose", PREFIX + "/filtered_lane_pose"),
        ("filtered_omega_t", PREFIX + "/filtered_omega_t"),
    ]
    map_parameters = dict(common_parameters)
    map_parameters.update({
        "odom_topic": odom_topic,
        "avoidance_initially_idle": True,
        "avoidance_flag_topic": PREFIX + "/unused_avoidance_flag",
        "branch_active_topic": PREFIX + "/branch_active",
        "reset_topic": PREFIX + "/reset",
        "status_topic": PREFIX + "/map_status",
        "reference_path_topic": PREFIX + "/reference_path",
        "enable_traffic_control": True,
        "enable_signal_approach": False,
        "traffic_policy": "stop_wait",
        "enable_velocity_gate": True,
        "traffic_light_topic": PREFIX + "/unused_legacy_light",
        "signal_observation_topic": PREFIX + "/map_signal",
        "input_cmd_vel_topic": PREFIX + "/controller_cmd_vel",
        "output_cmd_vel_topic": PREFIX + "/gated_cmd_vel",
    })
    for label in "abc":
        map_parameters["input_point_" + label + "_topic"] = PREFIX + "/original_point_" + label
        map_parameters["output_point_" + label + "_topic"] = PREFIX + "/processed_point_" + label

    return [
        Node(
            package="lane_line_publisher", executable="lane_line_publisher",
            name="lane_line_publisher", namespace=PREFIX, output="screen",
            parameters=[lane_config, camera_file, dict(common_parameters,
                        vehicle_frame_id="base_link", camera_frame_id=camera_frame,
                        camera_name=camera_name, debug=True)],
            remappings=[("mask_image", mask_topic), ("tf", "/tf"), ("tf_static", "/tf_static")],
        ),
        Node(
            package="lane_points", executable="lane_0215",
            name="lane_points", namespace=PREFIX, output="screen",
            parameters=[common_parameters], remappings=lane_remaps,
        ),
        Node(
            package="kalman_filter", executable="withoutkalman_0312",
            name="lane_target", namespace=PREFIX, output="screen",
            parameters=[common_parameters], remappings=filter_remaps,
        ),
        Node(
            package="trajectory_follower", executable="lya_0221",
            name="trajectory_follower", namespace=PREFIX, output="screen",
            parameters=[common_parameters],
            remappings=[
                ("/aiformula_sensing/gyro_odometry_publisher/odom", odom_topic),
                (LIVE_MOTOR_TOPIC, PREFIX + "/controller_cmd_vel"),
                ("/filtered_lane_pose", PREFIX + "/filtered_lane_pose"),
                ("/filtered_omega_t", PREFIX + "/filtered_omega_t"),
                # lya_0221 broadcasts odom->base_link itself. Its private TF
                # buffer receives that transform without duplicating global TF.
                ("/tf", PREFIX + "/tf"), ("/tf_static", PREFIX + "/tf_static"),
            ],
        ),
        Node(
            package="semantic_local_planner_ros", executable="map_assist",
            name="map_assist", namespace=PREFIX, output="screen",
            parameters=[map_parameters],
        ),
        Node(
            package="semantic_drive_bridge", executable="signal_capture_bridge",
            name="signal_capture_bridge", namespace=PREFIX, output="screen",
            parameters=[dict(common_parameters,
                             input_topic=PREFIX + "/signal_capture",
                             output_topic=PREFIX + "/map_signal",
                             expected_frame_id=signal_frame, max_age_sec=0.5)],
        ),
        Node(
            package="semantic_drive_bridge", executable="command_watchdog",
            name="command_watchdog", namespace=PREFIX, output="screen",
            parameters=[dict(common_parameters,
                             input_topic=PREFIX + "/gated_cmd_vel",
                             output_topic=output_topic,
                             status_topic=PREFIX + "/map_status",
                             diagnostics_topic=PREFIX + "/watchdog_status",
                             timeout_sec=0.25)],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_params_file", default_value="",
                              description="Required: calibrated LLP ROS YAML for the actual mask resolution."),
        DeclareLaunchArgument("camera_frame_id", default_value="",
                              description="Required: calibrated optical frame with external TF to base_link."),
        DeclareLaunchArgument("expected_frame_id", default_value="",
                              description="Required: exact source camera header.frame_id in signal captures."),
        DeclareLaunchArgument("camera_name", default_value="zedx",
                              description="Intrinsics parameter key in camera_params_file."),
        DeclareLaunchArgument("lane_mask_topic", default_value=PREFIX + "/lane_mask",
                              description="External calibrated sensor_msgs/Image mask with its capture header."),
        DeclareLaunchArgument("odom_topic", default_value="/aiformula_sensing/gyro_odometry_publisher/odom",
                              description="External nav_msgs/Odometry: pose in odom, child base_link, real body velocity."),
        DeclareLaunchArgument("shadow_output_topic", default_value=PREFIX + "/shadow_cmd_vel",
                              description="Private shadow_* Twist output only; live motor topics are rejected."),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use the same ROS capture clock on every producer and consumer."),
        OpaqueFunction(function=_launch_nodes),
    ])
