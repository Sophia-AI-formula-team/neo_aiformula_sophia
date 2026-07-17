import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # 订阅和发布(还未修改）
        self.subscription = self.create_subscription(
            Point, 'raw_lane_point', self.observation_callback, 10)
        self.publisher = self.create_publisher(
            Point, 'filtered_lane_point', 10)
        self.theta_publisher = self.create_publisher(
            Float32, 'filtered_theta', 10)

        # 初始化滤波器参数，增加Q则更敏感，增加R则对实际值偏重更高
        self.dt = 0.1
        self.x = np.zeros(6)  # [x, y, theta, vx, vy, v_theta]
        self.P = np.eye(6) * 1000
        self.F = np.array([
            [1, 0, 0, self.dt, 0, 0],
            [0, 1, 0, 0, self.dt, 0],
            [0, 0, 1, 0, 0, self.dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        self.Q = np.eye(6) * 0.1
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
        self.R = np.eye(3) * 1.0

    def observation_callback(self, msg):
        z = np.array([msg.x, msg.y, msg.z])  # 测量值

        # 预测
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q

        # 更新
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(np.dot(self.H, np.dot(self.P, self.H.T)) + self.R)))
        y = z - np.dot(self.H, self.x)  # 残差
        self.x = self.x + np.dot(K, y)
        self.P = np.dot(np.eye(6) - np.dot(K, self.H), self.P)

        # 发布滤波结果
        filtered_point = Point(x=self.x[0], y=self.x[1], z=0.0)
        filtered_theta = Float32(data=self.x[2])

        self.publisher.publish(filtered_point)
        self.theta_publisher.publish(filtered_theta)
        self.get_logger().info(f"Filtered: Point({filtered_point.x}, {filtered_point.y}), Theta: {filtered_theta.data}")

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
