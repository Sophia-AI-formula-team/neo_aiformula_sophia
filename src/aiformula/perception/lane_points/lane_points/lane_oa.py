#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose2D
import pandas as pd
import time
import os

class LaneLineSubscriber(Node):
    def __init__(self, a, b, c):
        super().__init__('lane_line_subscriber')

        # ==== 1) 订阅 左、右、中心 三条车道线的话题 ====
        self.center_subscription = self.create_subscription(
            PointCloud2,
            'lane_line_center',  
            self.center_callback,
            10
        )
        self.left_subscription = self.create_subscription(
            PointCloud2,
            'lane_line_left',
            self.left_callback,
            10
        )
        self.right_subscription = self.create_subscription(
            PointCloud2,
            'lane_line_right',
            self.right_callback,
            10
        )

        # ==== 2) 三个发布者：分别发布 A、B、C 三个点的 Pose2D ====
        self.pose_publisher_a = self.create_publisher(Pose2D, '/processed_point_a', 10)
        self.pose_publisher_b = self.create_publisher(Pose2D, '/processed_point_b', 10)
        self.pose_publisher_c = self.create_publisher(Pose2D, '/processed_point_c', 10)

        # ==== 3) 用于存储各条线的点云数据 ====
        self.left_points = []    
        self.right_points = []
        self.center_points = []

        # ==== 4) 其他参数 ====
        self.a = a  # 第一个点索引
        self.b = b  # 第二个点索引
        self.c = c  # 第三个点索引

        # ==== 5) 初始化数据记录 DataFrame ====
        self.data = pd.DataFrame(columns=["Timestamp", "A_x", "A_y", "B_x", "B_y", "C_x", "C_y"])

        # ==== 6) 文件夹路径和Excel文件保存路径 ====
        self.folder_path = "./lane_analysis_data"
        if not os.path.exists(self.folder_path):
            os.makedirs(self.folder_path)
        self.excel_file_path = os.path.join(self.folder_path, "lane_line_data.xlsx")

        # ==== 7) 车道宽度 (米) ====
        self.lane_width_m = 2.0

    # -------------------------------------------------------------
    #   回调函数： 左车道线
    # -------------------------------------------------------------
    def left_callback(self, msg):
        self.left_points = self.parse_pointcloud2(msg)
        self.get_logger().debug(f"Received LEFT lane with {len(self.left_points)} points")

    # -------------------------------------------------------------
    #   回调函数： 右车道线
    # -------------------------------------------------------------
    def right_callback(self, msg):
        self.right_points = self.parse_pointcloud2(msg)
        self.get_logger().debug(f"Received RIGHT lane with {len(self.right_points)} points")

    # -------------------------------------------------------------
    #   回调函数： 中心线
    # -------------------------------------------------------------
    def center_callback(self, msg):
        current_time = time.time()

        self.center_points = self.parse_pointcloud2(msg)
        center_count = len(self.center_points)
        left_count = len(self.left_points)
        right_count = len(self.right_points)

        # 如果左右线都足够，则直接用 center
        if left_count >= max(self.a, self.b, self.c)+1 and right_count >= max(self.a, self.b, self.c)+1:
            if center_count <= max(self.a, self.b, self.c):
                self.get_logger().warning("Center line not enough, but left & right are enough => using center anyway.")
            self.do_publish_points_by_center(current_time)
        # 如果仅左线足够
        elif left_count >= max(self.a, self.b, self.c)+1 and right_count < max(self.a, self.b, self.c)+1:
            self.get_logger().info("Right lane not enough, using Left + lane_width to simulate center.")
            self.do_publish_points_by_one_side(
                side_points=self.left_points,
                side='left',
                current_time=current_time
            )
        # 如果仅右线足够
        elif right_count >= max(self.a, self.b, self.c)+1 and left_count < max(self.a, self.b, self.c)+1:
            self.get_logger().info("Left lane not enough, using Right + lane_width to simulate center.")
            self.do_publish_points_by_one_side(
                side_points=self.right_points,
                side='right',
                current_time=current_time
            )
        else:
            # 两侧都不足 => 默认直行（这里只发布一个 Pose2D）
            self.get_logger().warning("Both lanes not enough => assume straight (default values)")
            pose_msg_a = Pose2D()
            pose_msg_a.x = 1.5
            pose_msg_a.y = 0.0
            pose_msg_a.theta = 0.0
            pose_msg_b = Pose2D()
            pose_msg_b.x = 2.0
            pose_msg_b.y = 0.0
            pose_msg_b.theta = 0.0
            pose_msg_c = Pose2D()
            pose_msg_c.x = 2.3
            pose_msg_c.y = 0.0
            pose_msg_c.theta = 0.0
            self.pose_publisher_a.publish(pose_msg_a)
            self.pose_publisher_b.publish(pose_msg_b)
            self.pose_publisher_c.publish(pose_msg_c)
            self.get_logger().info("Published default Pose2D (straight)")
            # 记录默认数据
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S.%f", time.localtime(current_time))
            row = {"Timestamp": timestamp,
                   "A_x": pose_msg_a.x, "A_y": pose_msg_a.y,
                   "B_x": pose_msg_b.x, "B_y": pose_msg_b.y,
                   "C_x": pose_msg_c.x, "C_y": pose_msg_c.y}
            self.data = pd.concat([self.data, pd.DataFrame([row])], ignore_index=True)

    # -------------------------------------------------------------
    #   使用中心线（已存到 self.center_points），取第 a, b, c 点
    # -------------------------------------------------------------
    def do_publish_points_by_center(self, current_time):
        if len(self.center_points) <= max(self.a, self.b, self.c):
            self.get_logger().warning("Insufficient points in center line.")
            return

        point_a = self.center_points[self.a]
        point_b = self.center_points[self.b]
        point_c = self.center_points[self.c]

        self.publish_points_abc(point_a, point_b, point_c, current_time)

    # -------------------------------------------------------------
    #   仅一侧可用时，通过平移来模拟 center，然后发布
    # -------------------------------------------------------------
    def do_publish_points_by_one_side(self, side_points, side, current_time):
        if len(side_points) <= max(self.a, self.b, self.c):
            self.get_logger().warning(f"Side {side} not enough points => skip.")
            return

        pa = side_points[self.a]
        pb = side_points[self.b]
        pc = side_points[self.c]

        # 根据左右线决定往 Y 轴正/负方向平移
        if side == 'left':
            pa_center = (pa[0], pa[1] - self.lane_width_m/2, pa[2])
            pb_center = (pb[0], pb[1] - self.lane_width_m/2, pb[2])
            pc_center = (pc[0], pc[1] - self.lane_width_m/2, pc[2])
        else:  # side == 'right'
            pa_center = (pa[0], pa[1] + self.lane_width_m/2, pa[2])
            pb_center = (pb[0], pb[1] + self.lane_width_m/2, pb[2])
            pc_center = (pc[0], pc[1] + self.lane_width_m/2, pc[2])

        self.publish_points_abc(pa_center, pb_center, pc_center, current_time)

    # -------------------------------------------------------------
    #   分别用三个发布者发布 (A, B, C) 三个 Pose2D（theta=0），并记录数据
    # -------------------------------------------------------------
    def publish_points_abc(self, point_a, point_b, point_c, current_time):
        # A 点
        pose_a = Pose2D()
        pose_a.x, pose_a.y, _ = point_a
        pose_a.theta = 0.0
        self.pose_publisher_a.publish(pose_a)
        self.get_logger().info(f"Publish Pose2D(A): x={pose_a.x:.2f}, y={pose_a.y:.2f}, theta=0")

        # B 点
        pose_b = Pose2D()
        pose_b.x, pose_b.y, _ = point_b
        pose_b.theta = 0.0
        self.pose_publisher_b.publish(pose_b)
        self.get_logger().info(f"Publish Pose2D(B): x={pose_b.x:.2f}, y={pose_b.y:.2f}, theta=0")

        # C 点
        pose_c = Pose2D()
        pose_c.x, pose_c.y, _ = point_c
        pose_c.theta = 0.0
        self.pose_publisher_c.publish(pose_c)
        self.get_logger().info(f"Publish Pose2D(C): x={pose_c.x:.2f}, y={pose_c.y:.2f}, theta=0")

        # 记录数据到 DataFrame
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S.%f", time.localtime(current_time))
        row = {"Timestamp": timestamp,
               "A_x": pose_a.x, "A_y": pose_a.y,
               "B_x": pose_b.x, "B_y": pose_b.y,
               "C_x": pose_c.x, "C_y": pose_c.y}
        self.data = pd.concat([self.data, pd.DataFrame([row])], ignore_index=True)

    # -------------------------------------------------------------
    #   解析点云数据
    # -------------------------------------------------------------
    def parse_pointcloud2(self, msg):
        points = []
        try:
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append(p)
        except Exception as e:
            self.get_logger().error(f"Error parsing PointCloud2: {e}")
        return points

    def save_data_to_excel(self):
        try:
            self.data.to_excel(self.excel_file_path, index=False)
            self.get_logger().info(f"Data saved to {self.excel_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save data to Excel: {e}")

def main(args=None):
    rclpy.init(args=args)

    # 示例中取索引 a=6, b=7, c=8
    a = 6  
    b = 7  
    c = 8  

    lane_line_subscriber = LaneLineSubscriber(a, b, c)
    try:
        rclpy.spin(lane_line_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        lane_line_subscriber.save_data_to_excel()
        lane_line_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
