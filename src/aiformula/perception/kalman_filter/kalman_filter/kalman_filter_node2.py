import rclpy
from rclpy.node import Node
import numpy as np
import pandas as pd
from datetime import datetime
from geometry_msgs.msg import Pose2D
import math
import os

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # 1. 订阅/发布 Pose2D 数据
        self.subscription = self.create_subscription(
            Pose2D, '/processed_points_with_angle', self.observation_callback, 10)
        self.publisher = self.create_publisher(
            Pose2D, 'filtered_lane_pose', 10)

        # 2. 卡尔曼滤波器初始化
        self.dt = 0.1              # 时间步长
        self.x = np.zeros(3)       # [x, y, theta]
        self.P = np.eye(3) * 1000  # 初始协方差矩阵
        self.F = np.eye(3)         # 状态转移矩阵
        self.Q = np.eye(3) * 0.1   # 过程噪声协方差
        self.H = np.eye(3)         # 测量矩阵
        self.R = np.eye(3) * 1.0   # 测量噪声协方差

        # 3. 准备一个 DataFrame 来存储后续要导出的数据
        #   包含 ["Time", "Y", "Theta"] 三列
        self.data = pd.DataFrame(columns=["Time", "Y", "Theta"])

    def observation_callback(self, msg):
        # ------------------------
        # (A) 提取观测值 z
        # ------------------------
        z = np.array([msg.x, msg.y, msg.theta])  # 测量值

        # ------------------------
        # (B) 预测步骤
        # ------------------------
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q

        # ------------------------
        # (C) 更新步骤
        # ------------------------
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))
        y = z - np.dot(self.H, self.x)    # 观测残差
        self.x = self.x + np.dot(K, y)
        self.P = np.dot(np.eye(3) - np.dot(K, self.H), self.P)

        # ------------------------
        # (D) 发布滤波后的结果
        # ------------------------
        filtered_pose = Pose2D(x=self.x[0], y=self.x[1], theta=self.x[2])
        self.publisher.publish(filtered_pose)

        # ------------------------
        # (E) 记录当前时刻信息到 self.data
        # ------------------------
        current_time_str = datetime.now().strftime("%H:%M:%S")  # 24 小时制字符串
        new_row = pd.DataFrame([[current_time_str, self.x[1], self.x[2]]],
                               columns=["Time", "Y", "Theta"])
        self.data = pd.concat([self.data, new_row], ignore_index=True)

        # ------------------------
        # (F) 打印日志
        # ------------------------
        self.get_logger().info(
            f"Filtered Pose2D: x={filtered_pose.x}, y={filtered_pose.y}, theta={filtered_pose.theta}"
        )

    def save_data_to_excel(self):
        """保存数据到 Excel 文件"""
        folder_path = "./lane_analysis_data"  
        y1_file_path = f"{folder_path}/y_data.xlsx"

        # 如果文件夹不存在就创建
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        try:
            # 将 self.data 写入到 Excel 文件
            # self.data 里包含了三列: Time, Y, Theta
            self.data.to_excel(y1_file_path, index=False)
            self.get_logger().info(f"Data saved to {y1_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save data to Excel: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.save_data_to_excel()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
