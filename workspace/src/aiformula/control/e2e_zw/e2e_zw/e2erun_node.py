#!/usr/bin/env python3
import os
import time
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

import torch
from torchvision import transforms
from PIL import Image as PILImage
import sys
sys.path.insert(0, '/home/nvidia/e2e_ws/src')
from e2e_zw.models.simplemodel import SimpleDrivingModel




class E2ERunNode(Node):
    def __init__(self):
        super().__init__('e2e_inference_node')

        # === 可配置参数（也支持通过 ROS2 参数覆盖） ===
        self.declare_parameter('image_topic', '/aiformula_sensing/zed_node/left_image/undistorted')#换成车的话题了
        self.declare_parameter('cmd_vel_topic', '/aiformula_control/game_pad/cmd_vel')#换成车的话题了
        self.declare_parameter('weights_path', '/home/nvidia/e2e_ws/src/e2e_zw/weights/driving_model.pth')
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('v_max', 3.0)     # 线速度限幅（m/s）
        self.declare_parameter('w_max', 8.0)     # 角速度限幅（rad/s）
        self.declare_parameter('ema_alpha', 0) # 平滑系数，0~1，设 0 关闭
        self.declare_parameter('log_every', 10)  # 每 N 帧打印一次日志

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.weights_path = self.get_parameter('weights_path').get_parameter_value().string_value
        self.use_gpu = self.get_parameter('use_gpu').get_parameter_value().bool_value
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.log_every = int(self.get_parameter('log_every').value)

        # === 设备选择 ===
        self.device = torch.device('cuda' if (self.use_gpu and torch.cuda.is_available()) else 'cpu')
        torch.set_num_threads(1)  # Jetson/CPU 上通常更稳

        # === 加载模型 ===
        self.model = SimpleDrivingModel().to(self.device)
        if not Path(self.weights_path).exists():
            raise FileNotFoundError(f'找不到权重文件：{self.weights_path}')
        self.model.load_state_dict(torch.load(self.weights_path, map_location=self.device))
        self.model.eval()
        self.get_logger().info(f'✅ 模型已加载：{self.weights_path}，设备：{self.device}')

        # === 预处理（与训练保持一致：Resize→ToTensor，不做 mean/std 归一化） ===
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
        ])

        # === 订阅图像（相机数据 QoS 用 BEST_EFFORT 更常见） ===
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos)

        # === 发布控制指令 ===
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # === 状态记录 ===
        self.frame_count = 0
        self.last_v = None
        self.last_w = None

        self.get_logger().info(
            f'📷 订阅图像: {self.image_topic}  →  🚗 发布速度: {self.cmd_vel_topic}\n'
            f'参数：v_max={self.v_max}, w_max={self.w_max}, ema_alpha={self.ema_alpha}'
        )

    def preprocess_image(self, cv_bgr):
        """
        1) 取下半部分（裁剪一半高度）
        2) BGR → RGB
        3) Resize 到 (224,224)
        4) ToTensor (C,H,W), [0,1]
        """
        h, w, _ = cv_bgr.shape
        bottom = cv_bgr[h // 2:, :, :]  # 下半部分
        # BGR -> RGB
        cv_rgb = bottom[:, :, ::-1]
        pil_img = PILImage.fromarray(cv_rgb)
        tensor = self.transform(pil_img)  # [3,224,224], float32
        tensor = tensor.unsqueeze(0).to(self.device)  # [1,3,224,224]
        return tensor

    def postprocess_control(self, v, w):
        """
        限幅 + EMA 平滑（可选）
        """
        # 限幅
        v = float(np.clip(v, 0.0, self.v_max))   # 一般线速度不允许负，按需调整
        w = float(np.clip(w, -self.w_max, self.w_max))

        # EMA 平滑
        if self.ema_alpha > 0 and self.last_v is not None and self.last_w is not None:
            a = self.ema_alpha
            v = (1 - a) * self.last_v + a * v
            w = (1 - a) * self.last_w + a * w

        self.last_v, self.last_w = v, w
        return v, w

    @torch.no_grad()
    def image_callback(self, msg: Image):
        try:
            cv_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        # 预处理
        x = self.preprocess_image(cv_bgr)

        # 前向
        out = self.model(x)[0]  # tensor([v, w])
        v_pred = out[0].item()
        w_pred = out[1].item()

        # 后处理（限幅+平滑）
        v, w = self.postprocess_control(v_pred, w_pred)

        # 发布
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

        # 日志
        self.frame_count += 1
        if self.frame_count % self.log_every == 0:
            self.get_logger().info(f'#{self.frame_count} → v={v:.3f}, w={w:.3f} (raw v={v_pred:.3f}, w={w_pred:.3f})')


def main(args=None):
    rclpy.init(args=args)
    node = E2ERunNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
