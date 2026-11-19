"""Start the vehicle and live MapAssist with LYA and OA available separately."""
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

MOTOR_COMMAND = "/semantic_drive/map_assist_045/diagnostic_cmd_vel"
FINAL_CAN = "/aiformula_control/motor_controller/reference_signal"
OWNED_NODES = {"socket_can_sender", "socket_can_receiver", "motor_controller",
               "zed_node", "map_assist", "motor_gate_session"}


def check_owners(_context):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor

    ros = Context()
    rclpy.init(args=[], context=ros)
    node = rclpy.create_node("mapassist_formal_start_check", context=ros, enable_rosout=False,
                             start_parameter_services=False)
    executor = SingleThreadedExecutor(context=ros)
    executor.add_node(node)
    try:
        deadline = time.monotonic() + 1.8
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.10)
            owners = sorted({name for name, _ in node.get_node_names_and_namespaces() if name in OWNED_NODES})
            if owners or node.count_publishers(FINAL_CAN):
                detail = ", ".join(owners) if owners else FINAL_CAN
                raise RuntimeError("Stop the existing vehicle or MapAssist launch before starting this shortcut: " + detail)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=ros)
    return []


def generate_launch_description():
    launchers = Path(get_package_share_directory("launchers"), "launch")
    commissioning = Path(__file__).resolve().with_name("semantic_045_commissioning.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("lane", default_value="outer", choices=["inner", "outer"],
                              description="outer = 5.5 m; inner = 6.0 m minimum detour radius"),
        OpaqueFunction(function=check_owners),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launchers / "allnodes.launch.py")),
            launch_arguments={"motor_cmd_topic": MOTOR_COMMAND, "launch_yolop_a3": "false"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(commissioning)),
            launch_arguments={"lane": LaunchConfiguration("lane"),
                              "vehicle_output": "true", "start_lya": "false"}.items(),
        ),
    ])
