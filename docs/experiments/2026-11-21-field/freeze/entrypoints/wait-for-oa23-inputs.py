"""Check the dynamic ROI and stamped motor handoff inputs before starting MapAssist."""
from time import monotonic

import rclpy
from std_msgs.msg import String
from semantic_vehicle_wiring.manual_handoff import MOTOR_STATUS_TOPIC, read_state

roi_topic = "/aiformula_perception/lane_line_publisher/dynamic_roi_mask"
rclpy.init(args=[])
node = rclpy.create_node("mapassist_oa23_start_check", enable_rosout=False,
                         start_parameter_services=False)
ready = False


def on_motor_state(message):
    global ready
    ready = read_state(message.data, node.get_clock().now().nanoseconds, "manual_held") is not None


try:
    node.create_subscription(String, MOTOR_STATUS_TOPIC, on_motor_state, 10)
    deadline = monotonic() + 4.0
    while monotonic() < deadline:
        publishers = node.get_publishers_info_by_topic(roi_topic)
        if (ready and len(publishers) == 1
                and publishers[0].node_name == "lane_line_publisher"
                and publishers[0].topic_type == "sensor_msgs/msg/Image"):
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    else:
        raise SystemExit("MapAssist OA 2.3 needs dynamic ROI and the stamped motor status. "
                         "Restart Allnodes via the new shortcut.")
finally:
    node.destroy_node()
    rclpy.shutdown()
