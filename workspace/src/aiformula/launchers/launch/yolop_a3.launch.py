"""One-command camera and A3 V14 perception, with the existing vehicle topics."""
import importlib.util
import os
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def prepare(context):
    # Reuse the vehicle's actual camera/calibration and lane consumer wiring.
    path = Path(get_package_share_directory("launchers")) / "launch/allnodes.launch.py"
    spec = importlib.util.spec_from_file_location("a3_vehicle_launch", str(path))
    vehicle = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(vehicle)
    enabled = lambda name: LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")
    shadow = enabled("shadow")
    if shadow and enabled("start_lane_publisher"):
        raise RuntimeError("shadow:=true requires start_lane_publisher:=false")

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    ros_context = Context()
    rclpy.init(args=[], context=ros_context)
    observer = rclpy.create_node("a3_startup_guard", context=ros_context,
                                 enable_rosout=False, start_parameter_services=False)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(observer)
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        targets = ["/a3_shadow/lane_mask", "/a3_shadow/obstacle_mask", "/a3_shadow/object_overlay"] if shadow else [
            "/aiformula_perception/road_detector/mask_image", "/cone_mask", "/cone_bbox_image"]
        if enabled("start_camera"):
            targets.append(LaunchConfiguration("input_image_topic").perform(context))
        if enabled("start_lane_publisher"):
            targets.append("/aiformula_perception/lane_line_publisher/lane_lines/center")
        duplicates = [topic for topic in targets if observer.count_publishers(topic)]
        if duplicates:
            raise RuntimeError("Existing publishers: " + ", ".join(duplicates) +
                               ". Stop the previous perception or use start_camera:=false/start_lane_publisher:=false.")
    finally:
        executor.shutdown()
        observer.destroy_node()
        rclpy.shutdown(context=ros_context)

    actions = []
    if enabled("start_camera"):
        actions.extend(vehicle.create_zed_node(context))
    if enabled("start_lane_publisher"):
        actions.extend(vehicle.create_lane_line_publisher_node(context))

    root = LaunchConfiguration("deployment_root")
    lane_topic = "/a3_shadow/lane_mask" if shadow else "/aiformula_perception/road_detector/mask_image"
    overlay_topic = "/a3_shadow/lane_overlay" if shadow else "/aiformula_visualization/road_detector/annotated_mask_image"
    obstacle_topic = "/a3_shadow/obstacle_mask" if shadow else "/cone_mask"
    object_topic = "/a3_shadow/object_overlay" if shadow else "/cone_bbox_image"
    detections_topic = "/a3_shadow/detections" if shadow else "/aiformula_perception/yolop_a3/detections"
    light_topic = "/a3_shadow/traffic_light_state" if shadow else "/aiformula_perception/yolop_a3/traffic_light/state"
    red_topic = "/a3_shadow/red_pixels_count" if shadow else "/aiformula_perception/yolop_a3/red_pixels_count"
    actions.append(ExecuteProcess(
        name="yolop_a3_v14",
        cmd=[PathJoinSubstitution([root, "venv/bin/python3"]),
             PathJoinSubstitution([root, "run_a3.py"]),
             "--ros-args", "-r", "__ns:=/aiformula_perception",
             "-p", ["weights:=", PathJoinSubstitution([root, "upstream/YOLOP_A3_Deploy/weights/shared_best.pth"])],
             "-p", ["device:=", LaunchConfiguration("device")],
             "-p", ["score_threshold:=", LaunchConfiguration("score_threshold")],
             "-p", ["use_sim_time:=", LaunchConfiguration("use_sim_time")],
             "-p", ["input_image_topic:=", LaunchConfiguration("input_image_topic")],
             "-r", "pub_mask_image:=" + lane_topic,
             "-r", "pub_annotated_mask_image:=" + overlay_topic,
             "-r", "/cone_mask:=" + obstacle_topic,
             "-r", "/cone_bbox_image:=" + object_topic,
             "-r", "/aiformula_perception/yolop_a3/detections:=" + detections_topic,
             "-r", "/traffic_light/state:=" + light_topic,
             "-r", "/red_pixels_count:=" + red_topic],
        additional_env={
            "PYTHONPATH": [PathJoinSubstitution([root, "ros_install/yolop_a3_perception/lib/python3.8/site-packages"]),
                           os.pathsep, EnvironmentVariable("PYTHONPATH", default_value="")],
            "PYTHONNOUSERSITE": "1", "PYTHONUNBUFFERED": "1",
            "LD_PRELOAD": "/lib/aarch64-linux-gnu/libgomp.so.1",
        }, output="screen"))
    return actions


def generate_launch_description():
    vehicle_dir = get_package_share_directory("vehicle")
    args = {
        "deployment_root": "/home/control/aiformula_runtime/yolop_a3_v14_20261012",
        "start_camera": "true", "start_lane_publisher": "true", "shadow": "false",
        "device": "cuda", "score_threshold": "0.5",
        "input_image_topic": "/aiformula_sensing/zed_node/left_image/undistorted",
        "camera_sn": "SN48797506", "camera_resolution": "nHD",
        "camera_frame_id": "zed_left_camera_optical_frame",
        "grab_resolution": "HD1080", "grab_frame_rate": "15", "pub_downscale_factor": "3.0",
        "use_sim_time": "false", "vehicle_name": "ai_car1", "lane_debug": "true",
        "config_common_path": os.path.join(vehicle_dir, "config/zedx/common.yaml"),
        "config_camera_path": os.path.join(vehicle_dir, "config/zedx/zedx.yaml"),
    }
    extrinsics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(vehicle_dir, "launch/extrinsic_tfstatic_broadcaster.launch.py")),
        launch_arguments={"vehicle_name": LaunchConfiguration("vehicle_name"),
                          "use_sim_time": LaunchConfiguration("use_sim_time"),
                          "use_joint_state_publisher": "true", "use_gui": "false"}.items(),
        condition=IfCondition(LaunchConfiguration("start_lane_publisher")))
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value) for name, value in args.items()] +
                             [OpaqueFunction(function=prepare), extrinsics])
