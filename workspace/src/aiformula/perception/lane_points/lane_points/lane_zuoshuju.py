import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose2D
import time
import math
import pandas as pd  # 用于保存数据到 Excel

class LaneLineSubscriber(Node):
    def __init__(self, a, b):
        super().__init__('lane_line_subscriber')

        # 创建订阅者，订阅 '/lane_points' 话题
        self.subscription = self.create_subscription(
            PointCloud2,
            '/aiformula_perception/lane_line_publisher/lane_lines/center',  # 请根据实际话题修改
            self.pointcloud_callback,
            10)

        # 创建发布者，发布到 '/processed_points_with_angle' 话题
        self.publisher = self.create_publisher(Pose2D, '/processed_points_with_angle', 10)

        # 初始化订阅的时间控制
        self.last_received_time = 0.0
        self.target_rate = 1.0  # 设定为每秒处理一次

        # 点的索引，指定接收的点
        self.a = a
        self.b = b

        # 数据存储
        self.data_a = []  # 存储点 A 的 y 坐标
        self.data_b = []  # 存储点 B 的 y 坐标
        self.data_angle = []  # 存储角度

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
        self.publish_angle_between_points(point_a, point_b, current_time)

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

    def publish_angle_between_points(self, point_a, point_b, current_time):
        """计算两个点之间的角度并发布"""
        x1, y1, _ = point_a
        x2, y2, _ = point_b

        # 使用 atan2 计算角度
        angle = math.atan2(y2 - y1, x2 - x1)
        self.get_logger().info(f"Angle between point {self.a} and point {self.b}: {angle:.2f} radians")

        # 创建 Pose2D 消息并设置
        pose_msg = Pose2D()
        pose_msg.x = x2
        pose_msg.y = y2
        pose_msg.theta = angle  # 设置角度

        # 发布 Pose2D 消息
        self.publisher.publish(pose_msg)
        self.get_logger().info(f"Publishing Pose2D: x={x2}, y={y2}, theta={angle:.2f} radians")

        # 记录数据
        timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))  # 24 小时制
        self.data_a.append((timestamp, y1))
        self.data_b.append((timestamp, y2))
        self.data_angle.append((timestamp, angle))

    def save_data_to_excel(self):
        """保存数据到 Excel 文件"""
        folder_path = "./lane_analysis_data"  # 文件夹路径
        a_file_path = f"{folder_path}/point_a_data.xlsx"
        b_file_path = f"{folder_path}/point_b_data.xlsx"
        angle_file_path = f"{folder_path}/angle_data.xlsx"

        # 创建数据文件夹
        import os
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        # 保存数据到 Excel
        pd.DataFrame(self.data_a, columns=["Time", "Point A Y"]).to_excel(a_file_path, index=False)
        pd.DataFrame(self.data_b, columns=["Time", "Point B Y"]).to_excel(b_file_path, index=False)
        pd.DataFrame(self.data_angle, columns=["Time", "Angle"]).to_excel(angle_file_path, index=False)

        self.get_logger().info(f"Data saved to {folder_path}")


def main(args=None):
    rclpy.init(args=args)

    # 设置 a 和 b 的索引值
    a = 7  # 修改为需要的第一个点的索引
    b = 8  # 修改为需要的第二个点的索引

    # 创建节点并运行
    lane_line_subscriber = LaneLineSubscriber(a, b)

    try:
        rclpy.spin(lane_line_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # 在关闭节点时保存数据
        lane_line_subscriber.save_data_to_excel()
        lane_line_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
