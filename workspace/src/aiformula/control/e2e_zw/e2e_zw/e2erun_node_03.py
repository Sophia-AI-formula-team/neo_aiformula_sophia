#!/usr/bin/env python3
import os
import time
import queue
import threading
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

        # === 参数设置 ===
        self.declare_parameter('image_topic', '/aiformula_sensing/zed_node/left_image/undistorted')
        self.declare_parameter('cmd_vel_topic', '/aiformula_control/game_pad/cmd_vel')
        self.declare_parameter('weights_path', '/home/nvidia/e2e_ws/src/e2e_zw/weights/driving_model.pth')
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('v_max', 3.0)
        self.declare_parameter('w_max', 5.0)
        self.declare_parameter('ema_alpha', 0)
        self.declare_parameter('log_every', 10)
        self.declare_parameter('save_every', 1)

        # 固定保存根目录
        self.save_root = Path.home() / "/media/nvidia/OrinSSD2000/zw01/logs"
        self.save_root.mkdir(parents=True, exist_ok=True)

        # 生成新日志文件夹
        run_name = time.strftime("run_%Y%m%d_%H%M%S")
        self.run_dir = self.save_root / run_name
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.save_img_dir = self.run_dir / "images"
        self.save_img_dir.mkdir(parents=True, exist_ok=True)
        self.save_csv_path = self.run_dir / "cmdvel.csv"

        # === 参数读取 ===
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.weights_path = self.get_parameter('weights_path').get_parameter_value().string_value
        self.use_gpu = self.get_parameter('use_gpu').get_parameter_value().bool_value
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.log_every = int(self.get_parameter('log_every').value)
        self.save_every = int(self.get_parameter('save_every').value)

        # === 设备选择 ===
        self.device = torch.device('cuda' if (self.use_gpu and torch.cuda.is_available()) else 'cpu')
        torch.set_num_threads(1)

        # === 加载模型 ===
        self.model = SimpleDrivingModel().to(self.device)
        if not Path(self.weights_path).exists():
            raise FileNotFoundError(f'找不到权重文件：{self.weights_path}')
        self.model.load_state_dict(torch.load(self.weights_path, map_location=self.device))
        self.model.eval()
        self.get_logger().info(f'✅ 模型已加载：{self.weights_path}  → 设备：{self.device}')

        # === 图像预处理 ===
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
        ])

        # === ROS2 通信配置 ===
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # === 状态变量 ===
        self.frame_count = 0
        self.last_v = None
        self.last_w = None

        # === 异步保存线程 ===
        self.save_queue = queue.Queue()
        self.saver_thread = threading.Thread(target=self.save_worker, daemon=True)
        self.saver_thread.start()

        # 初始化 CSV 文件
        with open(self.save_csv_path, "w") as f:
            f.write("filename,timestamp,linear_x,angular_z\n")

        self.get_logger().info(
            f'📷 图像: {self.image_topic}\n'
            f'🚗 控制: {self.cmd_vel_topic}\n'
            f'💾 保存目录: {self.run_dir}\n'
            f'参数: v_max={self.v_max}, w_max={self.w_max}, save_every={self.save_every}'
        )

    def preprocess_image(self, cv_bgr):
        """
        裁剪下半部分并转换为张量
        """
        h, w, _ = cv_bgr.shape
        bottom = cv_bgr[h // 2:, :, :]
        cv_rgb = bottom[:, :, ::-1]
        pil_img = PILImage.fromarray(cv_rgb)
        tensor = self.transform(pil_img)
        tensor = tensor.unsqueeze(0).to(self.device)
        return tensor, pil_img

    def postprocess_control(self, v, w):
        """
        限幅与EMA平滑
        """
        v = float(np.clip(v, 0.0, self.v_max))
        w = float(np.clip(w, -self.w_max, self.w_max))

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

        # === 预处理 ===
        x, pil_img = self.preprocess_image(cv_bgr)

        # === 模型推理 ===
        out = self.model(x)[0]
        v_pred, w_pred = out[0].item(), out[1].item()
        v, w = self.postprocess_control(v_pred, w_pred)

        # === 发布控制指令 ===
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

        # === 异步保存 ===
        self.frame_count += 1
        if self.frame_count % self.save_every == 0:
            timestamp = time.time()
            img_name = f"{self.frame_count:06d}.png"
            img_path = self.save_img_dir / img_name
            self.save_queue.put((pil_img.copy(), img_path, timestamp, v, w))

        # === 日志输出 ===
        if self.frame_count % self.log_every == 0:
            self.get_logger().info(
                f'#{self.frame_count} → v={v:.3f}, w={w:.3f}  '
                f'(raw v={v_pred:.3f}, w={w_pred:.3f})'
            )

    def save_worker(self):
        """
        异步保存线程
        """
        while True:
            try:
                pil_img, img_path, timestamp, v, w = self.save_queue.get()
                pil_img.save(img_path)
                with open(self.save_csv_path, "a") as f:
                    f.write(f"{img_path.name},{timestamp:.6f},{v:.6f},{w:.6f}\n")
            except Exception as e:
                self.get_logger().error(f"保存线程错误: {e}")


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
