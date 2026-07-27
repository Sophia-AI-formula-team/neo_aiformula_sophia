import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
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

   
        # 点的索引，指定接收的点
        self.a = a
        self.b = b

    def pointcloud_callback(self, msg):
        """处理接收到的点云消息"""


        # 解析 PointCloud2 消息
        # points = self.parse_pointcloud2(msg)
        points = list(read_points(msg, field_names=("x", "y"), skip_nans=True))

        self.get_logger().info(f"Received {len(points)} points.")

        # 检查点云数量是否足够
        # if len(points) <= max(self.a, self.b):
        #     self.get_logger().warning("Insufficient points in PointCloud2 message.")
        #     return

        # 获取指定的两个点
        # point_a = points[self.a]
        # point_b = points[self.b]
        # 确保点云数据有足够的点，至少有 5 个点才能访问第 4 和第 5 个点
        if len(points) >= 5:
            # 提取第 4 和第 5 个点
            point_4 = points[3]  # 第 4 个点
            point_5 = points[4]  # 第 5 个点

            # 输出第 4 和第 5 个点的值
            self.get_logger().info(f"4th point: {point_4}")
            self.get_logger().info(f"5th point: {point_5}")

        # 计算角度并发布
        self.publish_angle_between_points(point_4, point_5)

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

    # 设置 a 和 b 的索引值v
    # a = 5  # 修改为需要的第一个点的索引
    # b = 6  # 修改为需要的第二个点的索引

    # 创建节点并运行
    lane_line_subscriber = LaneLineSubscriber(4, 5)
    rclpy.spin(lane_line_subscriber)

    # 销毁节点
    lane_line_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
