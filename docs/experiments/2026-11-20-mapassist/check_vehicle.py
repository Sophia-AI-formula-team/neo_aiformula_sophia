"""Vehicle ownership check from the 11/19 formal launch."""
import time

FINAL_CAN = "/aiformula_control/motor_controller/reference_signal"
OWNED_NODES = {"socket_can_sender", "socket_can_receiver", "motor_controller",
               "zed_node", "map_assist", "motor_gate_session"}


def check_owners():
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


if __name__ == "__main__":
    check_owners()
