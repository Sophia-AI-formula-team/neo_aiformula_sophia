#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
from pathlib import Path

# ====== 禁用 YOLOv5 自动 pip 安装依赖（避免运行时乱装包）======
os.environ["YOLOv5_AUTOINSTALL"] = "0"
os.environ["YOLOV5_AUTOINSTALL"] = "0"
os.environ["AUTOINSTALL"] = "0"

import torch

# ====== 修补 ultralytics 的 torch_load：移除 weights_only（torch1.13 不支持）======
try:
    import ultralytics.utils.patches as up

    def torch_load_safe(*args, **kwargs):
        kwargs.pop("weights_only", None)
        return torch.load(*args, **kwargs)

    up.torch_load = torch_load_safe
except Exception:
    pass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import cv2

from ament_index_python.packages import get_package_prefix


def add_ros_python_site_packages(pkg_name: str):
    prefix = Path(get_package_prefix(pkg_name)).resolve()
    py_ver = f"python{sys.version_info.major}.{sys.version_info.minor}"
    site_pkgs = prefix / "lib" / py_ver / "site-packages"
    if not site_pkgs.exists():
        raise RuntimeError(f"site-packages not found: {site_pkgs}")
    sys.path.insert(0, str(site_pkgs))


class ConeDetector(Node):
    def __init__(self):
        super().__init__('cone_detector')
        self.bridge = CvBridge()

        # ---------- Sub ----------
        self.sub = self.create_subscription(
            Image,
            '/aiformula_sensing/zed_node/left_image/undistorted',
            self.image_callback,
            10
        )

        # ---------- Pub ----------
        self.mask_pub = self.create_publisher(Image, '/cone_mask', 10)         # mono8
        self.bbox_pub = self.create_publisher(Image, '/cone_bbox_image', 10)  # bgr8 (with bbox)

        # ---------- Params ----------
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf_thres", 0.45)
        self.declare_parameter("iou_thres", 0.45)

        # ===== NEW: 不在顶部做推理（裁掉顶部比例，仅用于推理）=====
        # ignore_top_ratio=0.30 => 推理只用 [0.30H : H] 区域
        self.declare_parameter("ignore_top_ratio", 0.50)

        self.imgsz = int(self.get_parameter("imgsz").value)
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.iou_thres = float(self.get_parameter("iou_thres").value)
        self.ignore_top_ratio = float(self.get_parameter("ignore_top_ratio").value)

        # ---------- Locate vendor repo ----------
        add_ros_python_site_packages("traffic_cones_detection_ros")
        import traffic_cones_detection_ros

        pkg_root = Path(traffic_cones_detection_ros.__file__).resolve().parent
        tp = pkg_root / "vendor" / "traffic_cones_detection"
        if not tp.exists():
            raise RuntimeError(f"vendor repo not found: {tp}")
        sys.path.insert(0, str(tp))

        weights = tp / "models" / "best.pt"
        if not weights.exists():
            raise RuntimeError(f"weights not found: {weights}")
        self.weights = weights

        # ---------- Device ----------
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"device={self.device}, weights={self.weights}")

        # ---------- Load YOLOv5 backend ----------
        from models.common import DetectMultiBackend
        self.model = DetectMultiBackend(str(self.weights), device=self.device, dnn=False, data=None, fp16=False)

        # warm-up
        dummy = torch.zeros((1, 3, self.imgsz, self.imgsz), device=self.device)
        _ = self.model(dummy)

        self.get_logger().info(
            f"Loaded. imgsz={self.imgsz}, conf_thres={self.conf_thres}, "
            f"iou_thres={self.iou_thres}, ignore_top_ratio={self.ignore_top_ratio}"
        )

    @torch.no_grad()
    def image_callback(self, msg: Image):
        # 1) ROS -> cv2
        try:
            img0 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        h0, w0 = img0.shape[:2]
        mask = np.zeros((h0, w0), dtype=np.uint8)
        vis = img0.copy()

        # ===== 推理只用下部区域 =====
        y_ignore = int(round(h0 * self.ignore_top_ratio))
        y_ignore = max(0, min(y_ignore, h0))  # [0, h0]

        # 如果忽略比例太大导致没有有效区域，直接发布空
        if y_ignore >= h0 - 1:
            self.publish_all(mask, vis, msg.header)
            return

        # 推理用图：裁掉顶部（仅用于推理，不影响输出坐标系）
        img_inf = img0[y_ignore:, :]
        h1, w1 = img_inf.shape[:2]

        # 2) preprocess + infer + nms（在裁剪图上做）
        from utils.augmentations import letterbox
        from utils.general import non_max_suppression, scale_boxes

        img, _, _ = letterbox(img_inf, new_shape=self.imgsz, auto=False)
        img = img.transpose((2, 0, 1))  # HWC -> CHW
        img = np.ascontiguousarray(img)

        im = torch.from_numpy(img).to(self.device)
        im = im.float() / 255.0
        if im.ndimension() == 3:
            im = im.unsqueeze(0)

        pred = self.model(im)
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres)

        if len(pred) == 0 or pred[0] is None or len(pred[0]) == 0:
            self.publish_all(mask, vis, msg.header)
            return

        det = pred[0]

        # 注意：scale_boxes 的目标 shape 必须是“裁剪图 img_inf”的 shape
        det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], img_inf.shape).round()

        # 3) bbox loop -> mask + overlay（映射回原图：y 加回 y_ignore）
        for *xyxy, conf, cls in det:
            x1, y1, x2, y2 = [int(v.item()) for v in xyxy]

            # clamp 到裁剪图范围
            x1 = max(0, min(x1, w1 - 1))
            x2 = max(0, min(x2, w1 - 1))
            y1 = max(0, min(y1, h1 - 1))
            y2 = max(0, min(y2, h1 - 1))
            if x2 <= x1 or y2 <= y1:
                continue

            # ===== 映射回原图坐标系 =====
            y1_full = y1 + y_ignore
            y2_full = y2 + y_ignore
            x1_full = x1
            x2_full = x2

            # clamp 到原图范围（保险）
            x1_full = max(0, min(x1_full, w0 - 1))
            x2_full = max(0, min(x2_full, w0 - 1))
            y1_full = max(0, min(y1_full, h0 - 1))
            y2_full = max(0, min(y2_full, h0 - 1))
            if x2_full <= x1_full or y2_full <= y1_full:
                continue

            # mask（只会落在 y_ignore 以下）
            mask[y1_full:y2_full, x1_full:x2_full] = 255

            # bbox overlay
            cv2.rectangle(vis, (x1_full, y1_full), (x2_full, y2_full), (0, 255, 0), 2)
            cv2.putText(
                vis,
                f"cone {float(conf):.2f}",
                (x1_full, max(0, y1_full - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        # 可选：画出忽略线（紫色）
        # cv2.line(vis, (0, y_ignore), (w0 - 1, y_ignore), (255, 0, 255), 2)

        self.publish_all(mask, vis, msg.header)

    def publish_all(self, mask: np.ndarray, vis_bgr: np.ndarray, header):
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = header
        self.mask_pub.publish(mask_msg)

        vis_msg = self.bridge.cv2_to_imgmsg(vis_bgr, encoding='bgr8')
        vis_msg.header = header
        self.bbox_pub.publish(vis_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
