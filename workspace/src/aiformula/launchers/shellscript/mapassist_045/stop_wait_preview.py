"""Preview the legacy motor status contract without publishing on vehicle topics."""
import json
import math

INPUT = "/semantic_drive/map_assist_045/status"
OUTPUT = "/semantic_drive/map_assist_045/stop_wait_preview"
MAX_AGE_NS = 600_000_000


def _invalid_constant(value):
    raise ValueError(value)


def convert_status(text, now_ns):
    try:
        source = json.loads(text, parse_constant=_invalid_constant)
        stamp = source["ros_time_sec"]
        if (type(stamp) not in (float, int) or not math.isfinite(stamp)
                or stamp < 0 or not -1000 <= now_ns - round(stamp * 1e9) <= MAX_AGE_NS):
            return None
    except (TypeError, ValueError, KeyError, OverflowError):
        return None

    mode = source.get("traffic_mode")
    reason = source.get("traffic_reason")
    can_resume = (
        source.get("traffic_planner_version") == "ubs_auto_turn_v1"
        and source.get("use_sim_time") is False
        and mode == "normal"
        and isinstance(reason, str)
        and source.get("traffic_light") == "green"
        and source.get("traffic_light_raw") == "green"
        and source.get("traffic_stop_committed") is False
        and source.get("auto_stop_latched") is False
        and source.get("avoidance_active") is False
        and source.get("branch_active") is False
        and source.get("path_output_allowed") is True
    )
    if not can_resume:
        if mode in ("prepare", "detour"):
            reason = "unsupported_new_mode_" + mode
        elif mode != "wait" or not isinstance(reason, str):
            reason = "waiting_for_confirmed_light_release_and_ready_inputs"

    return {
        "traffic_policy": "stop_wait",
        "traffic_mode": "normal" if can_resume else "wait",
        "traffic_stop_required": not can_resume,
        "traffic_reason": reason,
        "ros_time_sec": stamp,
    }


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String

    class Preview(Node):
        def __init__(self):
            super().__init__("map_assist_stop_wait_preview")
            self.output = self.create_publisher(String, OUTPUT, 1)
            if self.output.topic_name != OUTPUT:
                raise ValueError("Stop-wait preview must remain on its diagnostic topic")
            self.subscription = self.create_subscription(String, INPUT, self.receive, 1)
            self.last_stamp = None

        def receive(self, message):
            publishers = self.get_publishers_info_by_topic(INPUT)
            if (len(publishers) != 1 or publishers[0].node_name != "map_assist"
                    or publishers[0].node_namespace != "/"):
                return
            status = convert_status(message.data, self.get_clock().now().nanoseconds)
            if status is None:
                return
            stamp = status["ros_time_sec"]
            if self.last_stamp is not None and stamp <= self.last_stamp:
                return
            self.last_stamp = stamp
            self.output.publish(String(data=json.dumps(status, allow_nan=False)))

    rclpy.init()
    node = None
    try:
        node = Preview()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
