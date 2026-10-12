"""Preserve A3's actual source stamp and color; explicitly provide no metric range."""
import json


def _reject_constant(value):
    raise ValueError("nonfinite JSON constant: " + value)


def capture_from_json(text, expected_frame):
    data = json.loads(text, parse_constant=_reject_constant)
    if not isinstance(data, dict):
        raise ValueError("A3 detections must be a JSON object")
    header = data.get("header")
    if not isinstance(header, dict) or header.get("frame_id") != expected_frame:
        raise ValueError("A3 source frame does not match the configured camera")
    stamp = header.get("stamp")
    if not isinstance(stamp, dict):
        raise ValueError("A3 source stamp is missing")
    sec, nanosec = stamp.get("sec"), stamp.get("nanosec")
    if (type(sec) is not int or type(nanosec) is not int or not 0 <= sec <= 2147483647
            or not 0 <= nanosec < 1000000000):
        raise ValueError("A3 source stamp is invalid")
    sequence = sec * 1000000000 + nanosec
    if sequence <= 0:
        raise ValueError("A3 source stamp is zero")
    state = data.get("traffic_light_state")
    if state not in ("red", "green", "unknown"):
        raise ValueError("A3 traffic-light state is missing or invalid")
    return dict(header=dict(stamp=dict(sec=sec, nanosec=nanosec), frame_id=expected_frame),
                sequence=sequence, state=state, distance_m=None, uncertainty_m=None)


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class CaptureNode(Node):
        def __init__(self):
            super().__init__("a3_signal_capture")
            topic = str(self.declare_parameter("input_topic", "/aiformula_perception/yolop_a3/detections").value)
            frame = str(self.declare_parameter("expected_frame_id", "zed_left_camera_optical_frame").value)
            if not frame or frame.strip() != frame:
                raise ValueError("expected_frame_id must be explicitly configured")
            self.frame = frame
            self.last_sequence = 0
            self.publisher = self.create_publisher(String, "/semantic_drive/signal_capture", 1)
            if self.publisher.topic_name != "/semantic_drive/signal_capture":
                self.destroy_publisher(self.publisher)
                raise ValueError("A3 capture output must remain /semantic_drive/signal_capture")
            self.subscription = self.create_subscription(String, topic, self.receive, 1)
            self.get_logger().info("A3 capture wiring ready; metric distance remains unavailable")

        def receive(self, message):
            try:
                payload = capture_from_json(message.data, self.frame)
                if payload["sequence"] <= self.last_sequence:
                    raise ValueError("duplicate or rolled-back capture; restart the full chain after a clock reset")
                self.last_sequence = payload["sequence"]
                self.publisher.publish(String(data=json.dumps(payload, allow_nan=False)))
            except (ValueError, TypeError, RecursionError) as error:
                self.get_logger().warning("Rejected A3 capture: " + str(error))

    rclpy.init(args=args)
    node = None
    try:
        node = CaptureNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
