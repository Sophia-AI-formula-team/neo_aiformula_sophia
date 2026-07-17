import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose2D


class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # === 1) 订阅和发布 Pose2D 数据 ===
        # 保持原有命名方式，但给每个订阅添加一个 lambda，传入区分符 'A'/'B'/'C'
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
        self.publisher_point = self.create_publisher(
            Pose2D, 'filtered_lane_pose', 10)
        
        self.publisher_omega_t = self.create_publisher(
            Pose2D, 'filtered_omega_t', 10)

        # === 2) 初始化卡尔曼滤波器参数 ===
        self.dt = 0.1               # 时间步长 (如需运动模型可使用)
        # 状态从 3 维扩展为 6 维: [A_x, A_y, B_x, B_y, C_x, C_y]
        self.x = np.zeros(6)        # 初始状态
        self.P = np.eye(6) * 1000   # 初始协方差矩阵
        self.F = np.eye(6)          # 状态转移矩阵(简单假设静止, 仅用于 predict)
        self.Q = np.eye(6) * 0.1    # 过程噪声协方差(可根据实际情况调)
        
        # 对测量做局部更新，需要针对 A/B/C 定义不同的 H、R
        # A 点只观测 state[0], state[1]，即 A_x, A_y
        self.H_a = np.array([
            [1, 0, 0, 0, 0, 0],  # 对应 A_x
            [0, 1, 0, 0, 0, 0],  # 对应 A_y
        ])
        self.R_a = np.eye(2) * 1.0
        
        # B 点观测 state[2], state[3]
        self.H_b = np.array([
            [0, 0, 1, 0, 0, 0],  # B_x
            [0, 0, 0, 1, 0, 0],  # B_y
        ])
        self.R_b = np.eye(2) * 1.0
        
        # C 点观测 state[4], state[5]
        self.H_c = np.array([
            [0, 0, 0, 0, 1, 0],  # C_x
            [0, 0, 0, 0, 0, 1],  # C_y
        ])
        self.R_c = np.eye(2) * 1.0

    def observation_callback(self, msg, point_label):
        """
        保持原函数签名风格:
          - 只是在回调里根据 point_label ('A'/'B'/'C') 
            来选择不同的测量矩阵 H_x 和 R_x,做局部更新
          - msg 里只用 x,y 参与滤波
        """
        # === 1) 从 Pose2D 中提取测量值(仅 x,y) ===
        z = np.array([msg.x, msg.y], dtype=float)  # 只滤除 (x,y)

        # === 2) 预测步骤 ===
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q

        # === 3) 更新步骤: 根据 label 选择对应的 H, R ===
        if point_label == 'A':
            H_current = self.H_a
            R_current = self.R_a
        elif point_label == 'B':
            H_current = self.H_b
            R_current = self.R_b
        else:  # 'C'
            H_current = self.H_c
            R_current = self.R_c

        # S = H P H^T + R
        S = np.dot(H_current, np.dot(self.P, H_current.T)) + R_current
        # K = P H^T S^-1
        K = np.dot(self.P, np.dot(H_current.T, np.linalg.inv(S)))
        # y = z - H x
        y = z - np.dot(H_current, self.x)
        # x = x + K y
        self.x = self.x + np.dot(K, y)
        # P = (I - K H) P
        I = np.eye(6)
        self.P = np.dot((I - np.dot(K, H_current)), self.P)

        # === 4) 限制输出范围 ===
        # 这里示例裁剪到 [-3.5, 3.5]，你可根据实际需求修改
        self.x[0] = np.clip(self.x[0], -3.5, 3.5)  # A_x
        self.x[1] = np.clip(self.x[1], -1.0, 1.0)  # A_y
        self.x[2] = np.clip(self.x[2], -3.5, 3.5)  # B_x
        self.x[3] = np.clip(self.x[3], -1.0, 1.0)  # B_y
        self.x[4] = np.clip(self.x[4], -3.5, 3.5)  # C_x
        self.x[5] = np.clip(self.x[5], -1.0, 1.0)  # C_y

        # === 5) 基于滤波后的 (A_x,A_y,B_x,B_y,C_x,C_y) 计算两段夹角 ===
        Ax, Ay, Bx, By, Cx, Cy = self.x
        theta_1 = np.arctan2(By - Ay, Bx - Ax)  # A->B
        theta_2 = np.arctan2(Cy - By, Cx - Bx)  # B->C

        # === 6) 将两个夹角打包发布(用 Pose2D，x=theta_1, y=theta_2, theta=0) ===
        filtered_omega_t = Pose2D(
            x=theta_1,
            y=theta_2,
            theta=(theta_2-theta_1)/0.25
        )
        self.publisher_omega_t.publish(filtered_omega_t)

        filtered_pose = Pose2D(
            x=Ax,
            y=Ay,
            theta=theta_1
        )
        self.publisher_point.publish(filtered_pose)


        # === 7) 输出日志(和原来类似的风格) ===
        self.get_logger().info(
            f"Filtered => A({Ax:.2f},{Ay:.2f}), B({Bx:.2f},{By:.2f}), C({Cx:.2f},{Cy:.2f}); "
            f"theta1={theta_1:.3f}, theta2={theta_2:.3f},omega_t={filtered_omega_t.theta:.3f}; "
            f"Filtered => A({filtered_pose.x:.2f},{filtered_pose.y:.2f}), omega({filtered_pose.theta:.3f}) "
        )

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
