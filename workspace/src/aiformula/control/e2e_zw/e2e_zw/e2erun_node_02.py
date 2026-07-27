#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
e2e_run_node.py

功能：
 - 订阅摄像头图像，在线裁剪下半部分、预处理、送模型推理
 - 发布模型输出的 cmd_vel（linear.x, angular.z）
 - 支持按键一次性触发动作（按一次执行固定 v/w 持续若干秒），动作期间覆盖模型输出
 - 按键：w (左变道), s (右变道), a (左转弯), d (右转弯)
"""
import os
import sys
import time
import select
import termios
import tty
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

# 导入你的模型类（调整路径以匹配你的项目）
try:
    from e2e_zw.models.simplemodel import SimpleDrivingModel
except Exception:
    # 尝试更简单的导入（如果模型在顶层 models/simplemodel.py）
    try:
        from models.simplemodel import SimpleDrivingModel
    except Exception as e:
        raise ImportError("无法导入 SimpleDrivingModel，请确认模型文件路径。错误: " + str(e))


class E2ERunNode(Node):
    def __init__(self):
        super().__init__('e2e_run_node')

        # ---------------- 参数 ----------------
        self.declare_parameter('image_topic', '/aiformula_sensing/zed_node/left_image/undistorted')
        self.declare_parameter('cmd_vel_topic', '/aiformula_control/game_pad/cmd_vel')
        # 默认权重路径，可在启动时通过 -p weights_path:=... 覆盖
        self.declare_parameter('weights_path', os.path.expanduser('~/e2e_ws/src/e2e_zw/weights/driving_model.pth'))
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('v_max', 3.0)
        self.declare_parameter('w_max', 5.0)
        self.declare_parameter('ema_alpha', 0.15)
        self.declare_parameter('log_every', 20)

        # 获取参数
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.weights_path = self.get_parameter('weights_path').get_parameter_value().string_value
        self.use_gpu = self.get_parameter('use_gpu').get_parameter_value().bool_value
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.log_every = int(self.get_parameter('log_every').value)

        # ---------------- 设备与模型 ----------------
        self.device = torch.device('cuda' if (self.use_gpu and torch.cuda.is_available()) else 'cpu')
        torch.set_num_threads(1)

        self.model = SimpleDrivingModel().to(self.device)
        if not Path(self.weights_path).exists():
            self.get_logger().error(f'找不到权重文件：{self.weights_path}')
            raise FileNotFoundError(self.weights_path)
        state = torch.load(self.weights_path, map_location=self.device)
        # 兼容单/多 GPU 保存的 state_dict
        if isinstance(state, dict) and 'state_dict' in state:
            state = state['state_dict']
        # 如果是完整模型对象，try load_state_dict 会出错，直接 assign
        try:
            self.model.load_state_dict(state)
        except Exception:
            try:
                # 可能保存的是 model.state_dict()
                self.model.load_state_dict(state)
            except Exception:
                # 最后尝试直接 torch.load 返回的模型
                self.model = state
        self.model.eval()
        self.get_logger().info(f'模型已加载：{self.weights_path}，设备：{self.device}')

        # ---------------- 预处理（与训练保持一致） ----------------
        # 注意：确保训练和推理的预处理严格一致（下方示例：裁下半部分 + Resize + ToTensor）
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
        ])

        # ---------------- 订阅图像 & 发布 cmd_vel ----------------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # ---------------- 控制 / 状态 ----------------
        self.frame_count = 0
        self.last_v = None
        self.last_w = None

        # ---------------- 按键触发动作任务（一次按键触发一段时间） ----------------
        # override_task 为 (v, w, end_time) 或 None
        self.override_task = None

        # 默认按键-动作映射（你可以根据实测改这个字典）
        # 格式： key: (v, w, duration_seconds)
        self.action_map = {
            'i': (0.35,  0.25, 2.0),   # 左变道：前进0.35m/s，角速度0.25 rad/s，持续2s
            'j': (0.35, -0.25, 2.0),   # 右变道
            'k': (0.25,  0.9,  1.6),   # 左转弯，较大角速度
            'l': (0.25, -0.9,  1.6),   # 右转弯
        }

        # 用于读取终端键盘输入的设置
        self.stdin_fd = sys.stdin.fileno()
        try:
            self.stdin_attr = termios.tcgetattr(self.stdin_fd)
            self.terminal_ok = True
        except Exception:
            self.terminal_ok = False
            self.get_logger().warning("无法获取 stdin 终端属性，键盘触发可能不可用（非交互式会话）")

        # 创建键盘轮询定时器（非阻塞）
        if self.terminal_ok:
            self.kb_timer = self.create_timer(0.05, self.keyboard_poll)

        self.get_logger().info(f"订阅图像: {self.image_topic} -> 发布: {self.cmd_vel_topic}")
        self.get_logger().info(f"按键触发映射 (press once): {self.action_map}")

    # ========== 非阻塞读键 ==========
    def _get_key_once(self, timeout=0.01):
        """
        非阻塞读取一个键（单字符），若超时返回 ''。
        注意：调用前要保存并在合适时 restored termios 设置。
        """
        if not self.terminal_ok:
            return ''
        tty.setraw(self.stdin_fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
        else:
            ch = ''
        termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.stdin_attr)
        return ch

    def keyboard_poll(self):
        key = self._get_key_once(timeout=0.01)
        if not key:
            return
        key = key.lower()
        # q 键用于紧急停止脚本（只发 0 指令并退出）
        if key == 'q':
            self.get_logger().warning("检测到 q，发布急停并退出节点（按 Ctrl-C 可直接终止）")
            self.cmd_pub.publish(Twist())  # 全 0
            # 不直接 exit node，保留由外部中断或 Ctrl-C 停止
            return
        # 如果 key 在动作表里，创建/替换 override_task
        if key in self.action_map:
            v, w, dur = self.action_map[key]
            end_time = time.time() + float(dur)
            self.override_task = (float(v), float(w), end_time)
            self.get_logger().info(f"按键触发: '{key}' -> v={v}, w={w}, duration={dur}s")

    # ========== 图像预处理 ==========
    def preprocess_image(self, cv_bgr):
        """
        裁剪下半部分 -> BGR->RGB -> PIL -> transform -> tensor([1,3,H,W])
        """
        h, w, c = cv_bgr.shape
        bottom = cv_bgr[h // 2:, :, :]  # 下半部分
        # BGR -> RGB
        cv_rgb = bottom[:, :, ::-1]
        pil_img = PILImage.fromarray(cv_rgb)
        tensor = self.transform(pil_img).unsqueeze(0).to(self.device)
        return tensor

    # ========== 控制后处理（限幅 + EMA 平滑） ==========
    def postprocess_control(self, v, w):
        # 限幅
        v = float(np.clip(v, 0.0, self.v_max))
        w = float(np.clip(w, -self.w_max, self.w_max))
        # EMA 平滑（仅对模型输出连续性有帮助）
        if self.ema_alpha > 0 and self.last_v is not None and self.last_w is not None:
            a = self.ema_alpha
            v = (1 - a) * self.last_v + a * v
            w = (1 - a) * self.last_w + a * w
        # 更新历史（无论模型还是 override 都更新，保证切换平滑）
        self.last_v, self.last_w = v, w
        return v, w

    # ========== 回调：接收相机图像 ==========
    @torch.no_grad()
    def image_callback(self, msg: Image):
        # 转 CV 图像
        try:
            cv_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        # 预处理并推理（模型）
        try:
            x = self.preprocess_image(cv_bgr)  # [1,3,224,224]
            out = self.model(x)[0]             # tensor([v, w])
            v_pred = float(out[0].item())
            w_pred = float(out[1].item())
        except Exception as e:
            self.get_logger().error(f"模型推理失败: {e}")
            # 失败时保持之前指令或发送 0
            v_pred, w_pred = 0.0, 0.0

        # 判断是否有正在执行的 override_task
        now = time.time()
        if self.override_task is not None:
            v_o, w_o, end_time = self.override_task
            if now < end_time:
                # 动作期间使用 override 指令（不做 EMA 来平滑模型到 override 的突变）
                v, w = float(np.clip(v_o, 0.0, self.v_max)), float(np.clip(w_o, -self.w_max, self.w_max))
                # 将 last_v/last_w 更新成 override 的值，保证动作结束后模型输出平滑过渡
                self.last_v, self.last_w = v, w
                remaining = end_time - now
                if self.frame_count % self.log_every == 0:
                    self.get_logger().info(f"执行动作 override -> v={v:.3f}, w={w:.3f}, 剩余 {remaining:.2f}s")
            else:
                # 动作结束，清除
                self.override_task = None
                v, w = self.postprocess_control(v_pred, w_pred)
        else:
            # 正常走模型输出（加后处理）
            v, w = self.postprocess_control(v_pred, w_pred)
            if self.frame_count % self.log_every == 0:
                self.get_logger().info(f"AI -> v={v:.3f}, w={w:.3f} (raw v={v_pred:.3f}, w={w_pred:.3f})")

        # 发布 cmd_vel
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

        self.frame_count += 1

    # ========== 在节点销毁/退出时恢复终端属性 ==========
    def destroy_node(self):
        try:
            if self.terminal_ok:
                termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.stdin_attr)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = E2ERunNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
