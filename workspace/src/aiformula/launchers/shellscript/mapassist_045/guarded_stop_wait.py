"""Encode the commissioned MapAssist chain's stop status for a legacy motor gate."""
import argparse
import json
import math
import time

from stop_wait_preview import INPUT, convert_status

PREFIX = "/semantic_drive/map_assist_045"
GUARDED_COMMAND = PREFIX + "/diagnostic_cmd_vel"
OUTPUT = PREFIX + "/stop_wait_candidate"
LEGACY_OUTPUT = "/semantic_drive/map_status"


def translate(text, now_ns, has_fresh_command):
    status = convert_status(text, now_ns)
    if status is None:
        return None

    source = json.loads(text)
    mode = source.get("traffic_mode")
    route_ready = source.get("path_output_allowed") is True
    if mode == "detour":
        route_ready = (
            route_ready
            and source.get("smooth_plan_available") is True
            and source.get("route_matched") is True
        )
    if source.get("avoidance_active") is True:
        route_ready = True

    allowed = (
        has_fresh_command and route_ready
        and mode in ("normal", "prepare", "detour")
        and source.get("traffic_planner_version") == "ubs_auto_turn_v1"
        and source.get("use_sim_time") is False
        and type(source.get("avoidance_active")) is bool
        and source.get("branch_active") is False
        and (mode == "detour" or (
            source.get("traffic_stop_committed") is False
            and source.get("auto_stop_latched") is False
        ))
        and source.get("velocity_gate_reason") == "forwarding_controller"
    )
    status["traffic_mode"] = "normal" if allowed else "wait"
    status["traffic_stop_required"] = not allowed
    if allowed:
        status["traffic_reason"] = str(source.get("traffic_reason", "map_assist_ready"))
    elif mode != "wait":
        status["traffic_reason"] = "semantic_045_route_or_command_unready"
    return status


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle-output", action="store_true")
    args = parser.parse_args()
    output_topic = LEGACY_OUTPUT if args.vehicle_output else OUTPUT

    class Candidate(Node):
        def __init__(self):
            super().__init__("map_assist_stop_wait_candidate")
            self.last_command = -math.inf
            self.last_stamp = None
            self.output = self.create_publisher(String, output_topic, 1)
            if self.output.topic_name != output_topic:
                raise ValueError("Unexpected legacy stop status topic")
            self.create_subscription(Twist, GUARDED_COMMAND, self.on_command, 1)
            self.create_subscription(String, INPUT, self.on_status, 1)

        def on_command(self, command):
            values = (command.linear.x, command.angular.z)
            valid = all(math.isfinite(value) for value in values) and any(values)
            self.last_command = time.monotonic() if valid else -math.inf

        def on_status(self, message):
            for topic in (INPUT, GUARDED_COMMAND):
                publishers = self.get_publishers_info_by_topic(topic)
                if len(publishers) != 1 or publishers[0].node_name != "map_assist" or publishers[0].node_namespace != "/":
                    return
            if self.count_publishers(output_topic) != 1:
                return
            status = translate(message.data, self.get_clock().now().nanoseconds,
                               time.monotonic() - self.last_command <= 0.25)
            if status is not None:
                stamp = status["ros_time_sec"]
                if self.last_stamp is None or stamp > self.last_stamp:
                    self.last_stamp = stamp
                    self.output.publish(String(data=json.dumps(status, allow_nan=False)))

    rclpy.init()
    node = None
    try:
        node = Candidate()
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
