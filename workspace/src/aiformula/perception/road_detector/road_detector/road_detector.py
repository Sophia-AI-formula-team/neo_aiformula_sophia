import sys
from copy import deepcopy
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import torch

from pathlib import Path

yolop_module_path = Path(__file__).resolve().parent / 'yolop'
if str(yolop_module_path) not in sys.path:
    sys.path.insert(0, str(yolop_module_path))

from lib.config import cfg
from lib.utils import letterbox_for_img
from lib.utils.utils import select_device
from lib.models import get_net


class RoadDetector(Node):

    def __init__(self):
        super().__init__('road_detector')
        buffer_size = 10
        self.cv_bridge = CvBridge()
        architecture, weights, mean, std = self.get_params()
        self.architecture = select_device(device=architecture)
        self.use_half_precision = (self.architecture.type != 'cpu')  # half precision only supported on CUDA

        # Load model
        self.model = get_net(cfg)
        checkpoint = torch.load(weights, map_location=self.architecture)
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model = self.model.to(self.architecture)
        self.model.eval()

        if self.use_half_precision:
            self.model.half()  # to FP16

        # Match torchvision ToTensor + Normalize without requiring torchvision on Jetson.
        self.mean = torch.as_tensor(mean, dtype=torch.float32).view(3, 1, 1)
        self.std = torch.as_tensor(std, dtype=torch.float32).view(3, 1, 1)

        self.annotated_mask_image_pub = self.create_publisher(Image, 'pub_annotated_mask_image', buffer_size)
        self.lane_mask_image_pub = self.create_publisher(Image, 'pub_mask_image', buffer_size)
        self.image_sub = self.create_subscription(
            Image, 'sub_image', self.image_callback, buffer_size
        )

    def get_params(self):
        self.declare_parameter('use_architecture')
        self.declare_parameter('weight_path')
        self.declare_parameter('mean')
        self.declare_parameter('standard_deviation')
        architecture = self.get_parameter('use_architecture').get_parameter_value().string_value
        weights = self.get_parameter('weight_path').get_parameter_value().string_value
        mean = np.array(self.get_parameter('mean').get_parameter_value().double_array_value)
        std = np.array(self.get_parameter('standard_deviation').get_parameter_value().double_array_value)
        return architecture, weights, mean, std

    def padding_image(self, image):
        # Padded resize
        padding_image, (ratio_to_padding, _), (dw, dh) = letterbox_for_img(
            image, new_shape=640, auto=True
        )  # ratio_to_padding (width, height)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        return np.ascontiguousarray(padding_image), ratio_to_padding, top, bottom, left, right

    def image_to_tensor(self, image):
        tensor = torch.from_numpy(image).permute(2, 0, 1).float().div(255.0)
        return (tensor - self.mean).div(self.std)

    def image_callback(self, msg):
        try:
            undistorted_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        padding_image, ratio_to_padding, top, bottom, left, right = self.padding_image(undistorted_image)
        normalize_image = self.image_to_tensor(padding_image).to(self.architecture)
        input_image = normalize_image.half() if self.use_half_precision else normalize_image.float()

        if input_image.ndimension() == 3:
            input_image = input_image.unsqueeze(0)

        # Inference for lane-only model
        with torch.no_grad():
            output = self.model(input_image)
            ll_seg_out = output[0] if isinstance(output, (tuple, list)) else output
            det_out = None
            da_seg_out = None

        _, _, height, width = input_image.shape
        ll_predict = ll_seg_out[:, :, top:(height - bottom), left:(width - right)]

        ll_seg_mask_raw = torch.nn.functional.interpolate(
            ll_predict,
            scale_factor=int(1 / ratio_to_padding),
            mode='bilinear',
            align_corners=False
        )

        # Handle both 1-channel and multi-channel outputs
        if ll_seg_mask_raw.shape[1] == 1:
            ll_seg_mask = (torch.sigmoid(ll_seg_mask_raw) > 0.5).int().squeeze().cpu().numpy()
        else:
            _, ll_seg_points = torch.max(ll_seg_mask_raw, 1)
            ll_seg_mask = ll_seg_points.int().squeeze().cpu().numpy()

        self.publish_result(undistorted_image, ll_seg_mask, msg.header)

    def publish_result(self, image, ll_seg_mask, source_header):
        ll_seg_mask = np.array(ll_seg_mask, dtype=np.uint8)
        ll_seg_mask_bin = (ll_seg_mask > 0).astype(np.uint8)

        # Publish annotated image
        color_mask = np.zeros_like(image, dtype=np.uint8)
        color_mask[ll_seg_mask_bin > 0] = (0, 255, 0)  # green in BGR
        annotated_image = cv2.addWeighted(image, 1.0, color_mask, 0.5, 0)

        annotated_msg = self.cv_bridge.cv2_to_imgmsg(annotated_image, "bgr8")
        # Both products retain the camera frame and acquisition time, without
        # sharing mutable Header/Time objects with the input or each other.
        annotated_msg.header = deepcopy(source_header)
        self.annotated_mask_image_pub.publish(annotated_msg)

        # Publish mask image
        mask_image = (ll_seg_mask_bin * 255).astype(np.uint8)
        ll_seg_mask_msg = self.cv_bridge.cv2_to_imgmsg(mask_image, "mono8")
        ll_seg_mask_msg.header = deepcopy(source_header)
        self.lane_mask_image_pub.publish(ll_seg_mask_msg)


def main():
    with torch.no_grad():
        rclpy.init()
        road_detector = RoadDetector()
        rclpy.spin(road_detector)
        road_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
