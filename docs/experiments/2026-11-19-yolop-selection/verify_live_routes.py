"""Exercise the vehicle topic names with synthetic frames in isolated DDS."""
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String

TOPICS = {
    "/aiformula_perception/road_detector/mask_image": Image,
    "/aiformula_visualization/road_detector/annotated_mask_image": Image,
    "/cone_mask": Image,
    "/cone_bbox_image": Image,
    "/aiformula_perception/yolop_a3/detections": String,
    "/aiformula_perception/yolop_a3/traffic_light/state": String,
    "/aiformula_perception/yolop_a3/red_pixels_count": Int32,
}
CAMERA = "/aiformula_sensing/zed_node/left_image/undistorted"


def main():
    if sys.argv[1:] != ["--isolated"]:
        raise SystemExit("Run only through the isolated localhost smoke test.")
    rclpy.init()
    node = rclpy.create_node("yolop_experiment_probe")
    try:
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        camera = node.create_publisher(Image, CAMERA, qos)
        received = {}

        def callback(topic):
            return lambda message: received.setdefault(topic, message)

        for topic, kind in TOPICS.items():
            node.create_subscription(kind, topic, callback(topic), qos)
        msg = Image()
        msg.header.frame_id = "isolated_experiment_camera"
        msg.header.stamp.sec = 123
        msg.width, msg.height, msg.encoding = 640, 360, "bgr8"
        msg.step = msg.width * 3
        msg.data = bytes(msg.height * msg.step)
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline and len(received) < len(TOPICS):
            if camera.get_subscription_count():
                camera.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.2)
        assert received.keys() == TOPICS.keys(), sorted(TOPICS.keys() - received.keys())
        for topic, data in received.items():
            assert node.count_publishers(topic) == 1, topic
            if isinstance(data, Image):
                assert data.header == msg.header, topic
                assert (data.width, data.height) == (640, 360), topic
                expected = "mono8" if topic in ("/cone_mask", "/aiformula_perception/road_detector/mask_image") else "bgr8"
                assert data.encoding == expected, (topic, data.encoding)
            print(topic, type(data).__name__)
        assert not any(name.startswith("/a3_shadow/") for name, _ in node.get_topic_names_and_types())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
