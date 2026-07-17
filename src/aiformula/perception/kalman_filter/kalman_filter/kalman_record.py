#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
np.float = float
from geometry_msgs.msg import Pose2D
import pandas as pd
from datetime import datetime
import os

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # 1) 订阅和发布 Pose2D 数据
        self.pose_subscription_a = self.create_subscription(
            Pose2D, '/processed_point_a',
            lambda msg: self.observation_callback(msg, 'A'),
            10
        )
        self.pose_subscription_b = self.create_subscription(
            Pose2D, '/processed_point_b',
            lambda msg: self.observation_callback(msg, 'B'),
            10
        )
        self.pose_subscription_c = self.create_subscription(
            Pose2D, '/processed_point_c',
            lambda msg: self.observation_callback(msg, 'C'),
            10
        )
        self.publisher_point = self.create_publisher(Pose2D, 'filtered_lane_pose', 10)
        self.publisher_omega_t = self.create_publisher(Pose2D, 'filtered_omega_t', 10)

        # 2) 初始化卡尔曼滤波器参数
        self.dt = 0.1  # 时间步长
        # 状态向量: [A_x, A_y, B_x, B_y, C_x, C_y]
        self.x = np.zeros(6)
        self.P = np.eye(6) * 1000
        self.F = np.eye(6)
        self.Q = np.eye(6) * 0.1

        # 针对 A/B/C 定义不同的测量矩阵 H 和噪声矩阵 R
        self.H_a = np.array([[1, 0, 0, 0, 0, 0],
                             [0, 1, 0, 0, 0, 0]])
        self.R_a = np.eye(2) * 1.0

        self.H_b = np.array([[0, 0, 1, 0, 0, 0],
                             [0, 0, 0, 1, 0, 0]])
        self.R_b = np.eye(2) * 1.0

        self.H_c = np.array([[0, 0, 0, 0, 1, 0],
                             [0, 0, 0, 0, 0, 1]])
        self.R_c = np.eye(2) * 1.0

        # 3) 初始化 pandas DataFrame 用于记录数据
        self.data = pd.DataFrame(columns=[
            "Timestamp", "PointLabel",
            "A_x", "A_y", "B_x", "B_y", "C_x", "C_y",
            "theta_1", "theta_2", "omega_t",
            "filtered_pose_x", "filtered_pose_y", "filtered_pose_theta"
        ])

        # 4) 文件夹路径和Excel文件名
        self.folder_path = "./kalman_filter_data"
        if not os.path.exists(self.folder_path):
            os.makedirs(self.folder_path)
        self.excel_file_path = os.path.join(self.folder_path, "kalman_data.xlsx")

    def observation_callback(self, msg, point_label):
        """
        根据传入的 point_label ('A'/'B'/'C') 选择对应的测量矩阵 H 和 R，
        并完成卡尔曼滤波的预测、更新、数据发布以及记录到 DataFrame 中。
        """
        # 提取测量值 (x, y)
        z = np.array([msg.x, msg.y], dtype=float)

        # 预测步骤
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q

        # 根据 point_label 选择对应的 H, R
        if point_label == 'A':
            H_current = self.H_a
            R_current = self.R_a
        elif point_label == 'B':
            H_current = self.H_b
            R_current = self.R_b
        else:  # 'C'
            H_current = self.H_c
            R_current = self.R_c

        # 更新步骤
        S = np.dot(H_current, np.dot(self.P, H_current.T)) + R_current
        K = np.dot(self.P, np.dot(H_current.T, np.linalg.inv(S)))
        y = z - np.dot(H_current, self.x)
        self.x = self.x + np.dot(K, y)
        I = np.eye(6)
        self.P = np.dot((I - np.dot(K, H_current)), self.P)

        # 限制输出范围
        self.x[0] = np.clip(self.x[0], -3.5, 3.5)  # A_x
        self.x[1] = np.clip(self.x[1], -1.0, 1.0)  # A_y
        self.x[2] = np.clip(self.x[2], -3.5, 3.5)  # B_x
        self.x[3] = np.clip(self.x[3], -1.0, 1.0)  # B_y
        self.x[4] = np.clip(self.x[4], -3.5, 3.5)  # C_x
        self.x[5] = np.clip(self.x[5], -1.0, 1.0)  # C_y

        Ax, Ay, Bx, By, Cx, Cy = self.x
        theta_1 = np.arctan2(By - Ay, Bx - Ax)
        theta_2 = np.arctan2(Cy - By, Cx - Bx)

        # 发布计算结果
        filtered_omega_t = Pose2D(
            x=theta_1,
            y=theta_2,
            theta=(theta_2 - theta_1) / 0.25
        )
        self.publisher_omega_t.publish(filtered_omega_t)

        filtered_pose = Pose2D(
            x=Ax,
            y=Ay,
            theta=theta_1
        )
        self.publisher_point.publish(filtered_pose)

        self.get_logger().info(
            f"Filtered => A({Ax:.2f},{Ay:.2f}), B({Bx:.2f},{By:.2f}), C({Cx:.2f},{Cy:.2f}); "
            f"theta1={theta_1:.3f}, theta2={theta_2:.3f}, omega_t={filtered_omega_t.theta:.3f}; "
            f"Filtered Pose => A({filtered_pose.x:.2f},{filtered_pose.y:.2f}), theta({filtered_pose.theta:.3f})"
        )

        # 记录数据到 DataFrame
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        new_row = pd.DataFrame([[
            timestamp, point_label,
            Ax, Ay, Bx, By, Cx, Cy,
            theta_1, theta_2, filtered_omega_t.theta,
            filtered_pose.x, filtered_pose.y, filtered_pose.theta
        ]], columns=self.data.columns)
        self.data = pd.concat([self.data, new_row], ignore_index=True)

    def save_data_to_excel(self):
        """
        将累积的 DataFrame 数据保存到 Excel 文件中
        """
        try:
            self.data.to_excel(self.excel_file_path, index=False)
            self.get_logger().info(f"Kalman filter data saved to {self.excel_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save data to Excel: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data_to_excel()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
