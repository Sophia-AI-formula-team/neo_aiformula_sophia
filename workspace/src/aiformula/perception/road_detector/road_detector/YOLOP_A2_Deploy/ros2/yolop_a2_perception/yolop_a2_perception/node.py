from __future__ import annotations

import json
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String

from yolop_a2_runtime import DualPerceptionModel, ROS_TOPICS


class YolopA2Node(Node):
    def __init__(self) -> None:
        super().__init__("yolop_a2_perception")
        lane_weights = Path(
            self.declare_parameter("lane_weights", "weights/lane_epoch_282.pth").value
        ).expanduser()
        object_weights = Path(
            self.declare_parameter(
                "object_weights",
                "weights/object_detector_usable.pth",
            ).value
        ).expanduser()
        device = str(self.declare_parameter("device", "auto").value)
        object_score = float(self.declare_parameter("object_score", 0.50).value)
        cone_score = float(self.declare_parameter("cone_score", 0.50).value)
        input_topic = str(
            self.declare_parameter("input_image_topic", ROS_TOPICS.input_image).value
        )

        self.bridge = CvBridge()
        self.model = DualPerceptionModel(
            lane_weights,
            object_weights,
            device=device,
            object_score_threshold=object_score,
            cone_score_threshold=cone_score,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(depth=1)
        self.lane_overlay_publisher = self.create_publisher(
            Image,
            ROS_TOPICS.lane_overlay,
            output_qos,
        )
        self.lane_mask_publisher = self.create_publisher(
            Image,
            ROS_TOPICS.lane_mask,
            output_qos,
        )
        self.object_overlay_publisher = self.create_publisher(
            Image,
            ROS_TOPICS.object_overlay,
            output_qos,
        )
        self.obstacle_mask_publisher = self.create_publisher(
            Image,
            ROS_TOPICS.obstacle_mask,
            output_qos,
        )
        self.red_pixels_publisher = self.create_publisher(
            Int32,
            ROS_TOPICS.red_pixels_count,
            output_qos,
        )
        self.traffic_light_publisher = self.create_publisher(
            String,
            ROS_TOPICS.traffic_light_state,
            output_qos,
        )
        self.detections_publisher = self.create_publisher(
            String,
            ROS_TOPICS.detections_json,
            output_qos,
        )
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            sensor_qos,
        )
        self.get_logger().info(
            f"YOLOP_A2 ready on {input_topic}; lane={lane_weights}, objects={object_weights}"
        )

    def image_callback(self, message: Image) -> None:
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            output = self.model.predict(image_bgr)
            self._publish_image(
                self.lane_overlay_publisher,
                output.lane_overlay,
                "bgr8",
                message,
            )
            self._publish_image(
                self.lane_mask_publisher,
                output.lane_mask,
                "mono8",
                message,
            )
            self._publish_image(
                self.object_overlay_publisher,
                output.object_overlay,
                "bgr8",
                message,
            )
            self._publish_image(
                self.obstacle_mask_publisher,
                output.obstacle_mask,
                "mono8",
                message,
            )
            self.red_pixels_publisher.publish(Int32(data=output.red_pixels_count))
            self.traffic_light_publisher.publish(String(data=output.traffic_light_state))

            payload = output.detections_payload()
            payload["header"] = {
                "stamp": {
                    "sec": int(message.header.stamp.sec),
                    "nanosec": int(message.header.stamp.nanosec),
                },
                "frame_id": message.header.frame_id,
            }
            self.detections_publisher.publish(
                String(data=json.dumps(payload, separators=(",", ":")))
            )
        except Exception as error:
            self.get_logger().error(f"perception callback failed: {error}")

    def _publish_image(
        self,
        publisher: object,
        image: object,
        encoding: str,
        source_message: Image,
    ) -> None:
        output_message = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        output_message.header = source_message.header
        publisher.publish(output_message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = YolopA2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
