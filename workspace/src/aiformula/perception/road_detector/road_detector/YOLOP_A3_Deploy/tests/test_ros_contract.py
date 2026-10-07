"""ROS callback contract tests with stubs; not live DDS/ROS acceptance."""
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from yolop_a3_runtime import PerceptionOutput


def load_node(monkeypatch, result):
    class Node:
        def __init__(self, name):
            self.publishers, self.errors = {}, []

        def declare_parameter(self, name, default):
            return SimpleNamespace(value=default)

        def create_publisher(self, kind, topic, depth):
            messages = []
            self.publishers[topic] = messages
            return SimpleNamespace(publish=messages.append)

        def create_subscription(self, kind, topic, callback, qos):
            return SimpleNamespace(topic=topic, callback=callback, qos=qos)

        def get_logger(self):
            return SimpleNamespace(info=lambda text: None, error=self.errors.append)

    class Bridge:
        def imgmsg_to_cv2(self, message, desired_encoding):
            assert desired_encoding == "bgr8"
            return message.pixels

        def cv2_to_imgmsg(self, pixels, encoding):
            return SimpleNamespace(pixels=pixels, encoding=encoding, header=None)

    definitions = {
        "rclpy": {},
        "rclpy.node": {"Node": Node},
        "rclpy.qos": {"HistoryPolicy": SimpleNamespace(KEEP_LAST=1),
                       "ReliabilityPolicy": SimpleNamespace(BEST_EFFORT=2),
                       "QoSProfile": SimpleNamespace},
        "cv_bridge": {"CvBridge": Bridge},
        "sensor_msgs": {},
        "sensor_msgs.msg": {"Image": SimpleNamespace},
        "std_msgs": {},
        "std_msgs.msg": {"Int32": SimpleNamespace, "String": SimpleNamespace},
    }
    for name, values in definitions.items():
        module = ModuleType(name)
        module.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, module)
    path = ROOT / "ros2/yolop_a3_perception/yolop_a3_perception/node.py"
    spec = importlib.util.spec_from_file_location("a3_ros_contract_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    requested = []

    def factory(weights, **options):
        requested.append((weights, options))
        return SimpleNamespace(predict=lambda image: result)

    monkeypatch.setattr(module, "PerceptionModel", factory)
    return module.YolopA3Node(), requested


def test_callback_preserves_headers_encodings_and_all_seven_outputs(monkeypatch):
    image = np.zeros((20, 30, 3), np.uint8)
    mask = np.zeros((20, 30), np.uint8)
    output = PerceptionOutput(mask, image, mask, image, image, (), "unknown", 0, {"total": 1.})
    node, requested = load_node(monkeypatch, output)
    assert requested == [(Path("weights/shared_best.pth"), dict(device="auto", score_threshold=.5))]
    assert node.subscription.topic == "/aiformula_sensing/zed_node/left_image/undistorted"
    assert node.subscription.qos.depth == 1
    header = SimpleNamespace(stamp=SimpleNamespace(sec=123, nanosec=456), frame_id="zed_left")
    node.receive(SimpleNamespace(pixels=image, header=header))
    encodings = {"pub_mask_image": "mono8", "pub_annotated_mask_image": "bgr8",
                 "/cone_mask": "mono8", "/cone_bbox_image": "bgr8"}
    assert len(node.publishers) == 7
    assert all(len(messages) == 1 for messages in node.publishers.values())
    for topic, encoding in encodings.items():
        message = node.publishers[topic][0]
        assert message.header is header and message.encoding == encoding
    payload = json.loads(node.publishers["/aiformula_perception/yolop_a3/detections"][0].data)
    assert payload["header"] == dict(stamp=dict(sec=123, nanosec=456), frame_id="zed_left")
    assert node.publishers["/traffic_light/state"][0].data == "unknown"
    assert node.publishers["/red_pixels_count"][0].data == 0
    assert not node.errors


def test_callback_failure_is_reported_without_fabricating_new_detections(monkeypatch):
    node, _ = load_node(monkeypatch, None)
    node.receive(SimpleNamespace(pixels=np.zeros((20, 30, 3), np.uint8), header=None))
    assert len(node.errors) == 1 and "callback failed" in node.errors[0]
    assert all(not messages for messages in node.publishers.values())
