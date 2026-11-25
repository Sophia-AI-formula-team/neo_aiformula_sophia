"""ROS 2 camera subscriber and independent semantic publisher."""
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .common import as_json, camera_dict, image_bgr, ns, transform_matrix
from .semantic import SemanticStage, Yolop


class SemanticNode(Node):
    def __init__(self):
        super().__init__("semantic")
        model = self.declare_parameter("model", "").value
        width = self.declare_parameter("analysis_width", 800).value
        self.width = int(width)
        self.roi = float(self.declare_parameter("roi_top_ratio", .5).value)
        self.body = self.declare_parameter("body_frame", "base_link").value
        self.static_info = self.declare_parameter("camera_info_is_static", False).value
        image = self.declare_parameter("image_topic", "/CAM_FRONT/image_rect_compressed").value
        info = self.declare_parameter("info_topic", "/CAM_FRONT/camera_info").value
        image_kind = self.declare_parameter("image_kind", "compressed").value
        if not model or self.width < 160 or image_kind not in ("raw", "compressed"):
            raise ValueError("Set model, select raw/compressed, and use analysis_width >= 160")
        cv2.setNumThreads(2)
        yolop = Yolop(model, self.declare_parameter("score", .30).value,
                      self.declare_parameter("inference_threads", 3).value,
                      provider=self.declare_parameter("inference_provider", "auto").value)
        self.get_logger().info(f"YOLOP backend: {yolop.provider}")
        self.worker = SemanticStage(yolop,
                                    self.declare_parameter("scan_near_m", 2.).value,
                                    self.declare_parameter("scan_far_m", 16.).value,
                                    self.declare_parameter("lane_width_m", 3.35).value,
                                    self.declare_parameter("orange_cone_candidates", False).value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.info = None
        self.output = self.create_publisher(String, "/front_demo/semantic/observation", 5)
        self.status = self.create_publisher(String, "/front_demo/semantic/status", 5)
        self.create_subscription(CameraInfo, info, self.on_info, qos_profile_sensor_data)
        self.create_subscription(CompressedImage if image_kind == "compressed" else Image, image,
                                 self.on_image, qos_profile_sensor_data)

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        stamp = ns(msg.header.stamp)
        try:
            if not self.info or self.info.header.frame_id != msg.header.frame_id:
                raise ValueError("waiting_for_camera_info")
            info_stamp = ns(self.info.header.stamp)
            if not self.static_info and info_stamp and abs(info_stamp-stamp) > 500_000_000:
                raise ValueError("camera_info_timestamp_mismatch")
            rigid = self.buffer.lookup_transform(self.body, msg.header.frame_id, Time())
            raw = image_bgr(msg)
            h = round(self.width*raw.shape[0]/raw.shape[1])
            frame = cv2.resize(raw, (self.width, h), interpolation=cv2.INTER_AREA)
            model = camera_dict(self.info, transform_matrix(rigid.transform), self.width, h, self.roi)
            obs = self.worker.step(frame, stamp, msg.header.frame_id, self.body, model)
            self.output.publish(String(data=as_json(obs)))
            self.status.publish(String(data=as_json(dict(stamp_ns=stamp, status=obs["status"]))))
        except (ValueError, TransformException, cv2.error) as error:
            self.status.publish(String(data=as_json(dict(stamp_ns=stamp, status=str(error)))))


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
