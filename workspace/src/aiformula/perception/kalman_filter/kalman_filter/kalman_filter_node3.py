import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose2D


class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # 订阅和发布 Pose2D 数据
        self.subscription = self.create_subscription(
            Pose2D, '/processed_points_with_angle', self.observation_callback, 10)
        self.publisher = self.create_publisher(
            Pose2D, 'filtered_lane_pose', 10)

        # 初始化卡尔曼滤波器参数
        self.dt = 0.1  # 时间步长
        self.x = np.zeros(3)  # [x, y, theta]
        self.P = np.eye(3) * 1000  # 初始协方差矩阵
        self.F = np.eye(3)  # 状态转移矩阵，直接对状态值保持
        self.Q = np.eye(3) * 0.1  # 过程噪声协方差
        self.H = np.eye(3)  # 测量矩阵
        self.R = np.eye(3) * 1.0  # 测量噪声协方差

    def observation_callback(self, msg):
        # 从 Pose2D 中提取测量值
        z = np.array([msg.x, msg.y, msg.theta])  # 测量值

        # 预测步骤
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q

        # 更新步骤
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))
        y = z - np.dot(self.H, self.x)  # 残差
        self.x = self.x + np.dot(K, y)
        self.P = np.dot(np.eye(3) - np.dot(K, self.H), self.P)

        # 限制输出范围
        self.x[0] = np.clip(self.x[0], -3.5, 3.5)  # 限制 x 范围
        self.x[1] = np.clip(self.x[1], -1.0, 1.0)  # 限制 y 范围
        self.x[2] = np.clip(self.x[2], -0.5, 0.5)  # 限制 theta 范围

        # 发布滤波后的结果
        filtered_pose = Pose2D(x=self.x[0], y=self.x[1], theta=self.x[2])
        self.publisher.publish(filtered_pose)

        self.get_logger().info(
            f"Filtered Pose2D: x={filtered_pose.x}, y={filtered_pose.y}, theta={filtered_pose.theta}")

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
