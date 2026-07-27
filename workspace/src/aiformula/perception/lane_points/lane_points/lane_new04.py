import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose2D
import time
import math

class LaneLineSubscriber(Node):
    def __init__(self, a, b):
        super().__init__('lane_line_subscriber')

        # 创建订阅者，订阅话题
        self.subscription = self.create_subscription(
            PointCloud2,
            '/aiformula_perception/lane_line_publisher/lane_lines/center',  
            self.pointcloud_callback,
            10)

        # 创建发布者，发布到 '/processed_points_with_angle' 话题
        self.publisher = self.create_publisher(Pose2D, '/processed_points_with_angle', 10)

        # 初始化订阅的时间控制
        self.last_received_time = 1.0
        self.target_rate = 5.0  # 设定为每秒处理一次

        # 点的索引，指定接收的点
        self.a = a
        self.b = b

    def pointcloud_callback(self, msg):
        """处理接收到的点云消息"""
        # 控制订阅频率，避免过于频繁处理
        current_time = time.time()
        if current_time - self.last_received_time < 1.0 / self.target_rate:
            return  # 跳过当前帧，等待下次

        self.last_received_time = current_time

        # 解析 PointCloud2 消息
        points = self.parse_pointcloud2(msg)

        # 检查点云数量是否足够
        if len(points) <= max(self.a, self.b):
            self.get_logger().warning("Insufficient points in PointCloud2 message.")
            return

        # 获取指定的两个点
        point_a = points[self.a]
        point_b = points[self.b]

        # 计算角度并发布
        self.publish_angle_between_points(point_a, point_b)

    def parse_pointcloud2(self, msg):
        """解析 PointCloud2 消息"""
        points = []
        # 使用 sensor_msgs_py.point_cloud2 来读取点云数据
        try:
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append(p)
        except Exception as e:
            self.get_logger().error(f"Error while parsing PointCloud2 message: {e}")
        return points

    def publish_angle_between_points(self, point_a, point_b):
        """计算两个点之间的角度并发布"""
        x1, y1, _ = point_a
        x2, y2, _ = point_b

        # 使用 atan2 计算角度
        angle = math.atan2(y2 - y1, x2 - x1)
        self.get_logger().info(f"Angle between point {self.a} and point {self.b}: {angle:.2f} radians")

        # 创建 Pose2D 消息并设置
        pose_msg = Pose2D()
        pose_msg.x = x1
        pose_msg.y = y1
        pose_msg.theta = angle  # 设置角度

        # 发布 Pose2D 消息
        self.publisher.publish(pose_msg)
        self.get_logger().info(f"Publishing Pose2D: x={x1}, y={y1}, theta={angle:.2f} radians")


def main(args=None):
    rclpy.init(args=args)

    # 设置 a 和 b 的索引值
    a = 7  # 修改为需要的第一个点的索引
    b = 8  # 修改为需要的第二个点的索引

    # 创建节点并运行
    lane_line_subscriber = LaneLineSubscriber(a, b)
    rclpy.spin(lane_line_subscriber)

    # 销毁节点
    lane_line_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
