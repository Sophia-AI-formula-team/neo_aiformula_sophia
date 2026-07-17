# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs_py.point_cloud2 as pc2
# from geometry_msgs.msg import Pose2D
# import numpy as np
# import pandas as pd
# import time
# import math

# class LaneLineSubscriber(Node):
#     def __init__(self, a, b):
#         super().__init__('lane_line_subscriber')

#         # 创建订阅者，订阅话题
#         self.subscription = self.create_subscription(
#             PointCloud2,
#             '/aiformula_perception/lane_line_publisher/lane_lines/center',  
#             self.pointcloud_callback,
#             10)

#         # 创建发布者，发布到 '/processed_points_with_angle' 话题
#         self.publisher = self.create_publisher(Pose2D, '/processed_points_with_angle', 10)

#         self.last_received_time = 0.0
#         # 点的索引，指定接收的点
#         self.a = a
#         self.b = b

#         self.data_b = []

#     def pointcloud_callback(self, msg):
#         """处理接收到的点云消息"""
#         current_time = time.time()
        

#         self.last_received_time = current_time

#         # 解析 PointCloud2 消息
#         points = self.parse_pointcloud2(msg)

#         # 检查点云数量是否足够
#         if len(points) <= max(self.a, self.b):
#             self.get_logger().warning("Insufficient points in PointCloud2 message.")
#             return

#         # 获取指定的两个点
#         point_a = points[self.a]
#         point_b = points[self.b]

#         # 计算角度并发布
#         self.publish_angle_between_points(point_a, point_b, current_time)

#     def parse_pointcloud2(self, msg):
#         """解析 PointCloud2 消息"""
#         points = []
#         # 使用 sensor_msgs_py.point_cloud2 来读取点云数据
#         try:
#             for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
#                 points.append(p)
#         except Exception as e:
#             self.get_logger().error(f"Error while parsing PointCloud2 message: {e}")
#         return points

#     def publish_angle_between_points(self, point_a, point_b, current_time):
#         """计算两个点之间的角度并发布"""
#         x1, y1, _ = point_a
#         x2, y2, _ = point_b

#         # 使用 atan2 计算角度
#         angle = math.atan2(y2 - y1, x2 - x1)
#         self.get_logger().info(f"Angle between point {self.a} and point {self.b}: {angle:.2f} radians")

#         y_1 = np.clip(y1, -2, 2) 

#         # 创建 Pose2D 消息并设置
#         pose_msg = Pose2D()
#         pose_msg.x = x1
#         pose_msg.y = y_1
#         pose_msg.theta = angle  # 设置角度

#         # 发布 Pose2D 消息
#         self.publisher.publish(pose_msg)
#         self.get_logger().info(f"Publishing Pose2D: x={x1}, y={y_1}, theta={angle:.2f} radians")

#         # 记录数据
#         timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))  # 24 小时制
#         # self.data_a.append((timestamp, y_1))
#         self.data_b.append((timestamp, y_1))
#         # self.data_angle.append((timestamp, angle))

#     def save_data_to_excel(self):
#         """保存数据到 Excel 文件"""
#         folder_path = "./lane_analysis_data"  # 文件夹路径
#         # a_file_path = f"{folder_path}/point_a_data.xlsx"
#         b_file_path = f"{folder_path}/point_b_data1.xlsx"
#         # angle_file_path = f"{folder_path}/angle_data.xlsx"

#         # 创建数据文件夹
#         import os
#         if not os.path.exists(folder_path):
#             os.makedirs(folder_path)

#         # 保存数据到 Excel
#         # pd.DataFrame(self.x1, columns=["Time", "Point A Y"]).to_excel(a_file_path, index=False)
#         pd.DataFrame(self.data_b, columns=["Time", "Point B Y"]).to_excel(b_file_path, index=False)
#         # pd.DataFrame(self.theta, columns=["Time", "Angle"]).to_excel(angle_file_path, index=False)

#         self.get_logger().info(f"Data saved to {folder_path}")

# def main(args=None):
#     rclpy.init(args=args)

#     # 设置 a 和 b 的索引值
#     a = 6  # 修改为需要的第一个点的索引
#     b = 7  # 修改为需要的第二个点的索引

#     # 创建节点并运行
#     lane_line_subscriber = LaneLineSubscriber(a, b)
    

#     try:
#         rclpy.spin(lane_line_subscriber)
#     except KeyboardInterrupt:
#         pass
#     finally:
#     # 销毁节点
#         lane_line_subscriber.save_data_to_excel()
#         lane_line_subscriber.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


















import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose2D
import numpy as np
import pandas as pd
import time
import math

class LaneLineSubscriber(Node):
    def __init__(self, a, b):
        super().__init__('lane_line_subscriber')

        # ==== 1) 订阅 左、右、中心 三条车道线的话题 ====
        # 原先的中心线订阅改个名字更清晰(也可以不改)
        self.center_subscription = self.create_subscription(
            PointCloud2,
            '/aiformula_perception/lane_line_publisher/lane_lines/center',  
            self.center_callback,  # 回调函数改个名字
            10
        )
        self.left_subscription = self.create_subscription(
            PointCloud2,
            '/aiformula_perception/lane_line_publisher/lane_lines/left',
            self.left_callback,
            10
        )
        self.right_subscription = self.create_subscription(
            PointCloud2,
            '/aiformula_perception/lane_line_publisher/lane_lines/right',
            self.right_callback,
            10
        )

        # ==== 2) 发布者 ====
        self.publisher = self.create_publisher(Pose2D, '/processed_points_with_angle', 10)

        # ==== 3) 用于存储各条线的点云数据 ====
        self.left_points = []    # [(x,y,z), (x,y,z), ...]
        self.right_points = []
        self.center_points = []

        # ==== 4) 其他参数 ====
        self.a = a
        self.b = b

        self.data_b = []
        self.last_received_time = 0.0

        # ==== 5) 车道宽度 (米) ====
        self.lane_width_m = 3.4  # 这里写死，或改为参数

    # -------------------------------------------------------------
    #   回调函数： 左车道线
    # -------------------------------------------------------------
    def left_callback(self, msg):
        self.left_points = self.parse_pointcloud2(msg)
        # 这里只是存储，不做额外处理
        self.get_logger().debug(f"Received LEFT lane with {len(self.left_points)} points")

    # -------------------------------------------------------------
    #   回调函数： 右车道线
    # -------------------------------------------------------------
    def right_callback(self, msg):
        self.right_points = self.parse_pointcloud2(msg)
        self.get_logger().debug(f"Received RIGHT lane with {len(self.right_points)} points")

    # -------------------------------------------------------------
    #   回调函数： 中心线 (原先的 pointcloud_callback)
    # -------------------------------------------------------------
    def center_callback(self, msg):
        current_time = time.time()
        self.last_received_time = current_time

        # 1) 解析中心线点云
        self.center_points = self.parse_pointcloud2(msg)
        center_count = len(self.center_points)

        # 2) 判断左右线是否“足够”
        left_count = len(self.left_points)
        right_count = len(self.right_points)

        # 3) 分情况处理
        # case A: 如果左右线都够，则直接用 center
        #   这里 “够不够” 的判断逻辑你可自行调整, 这里只示例 >= max(a,b)+1
        if left_count >= max(self.a, self.b)+1 and right_count >= max(self.a, self.b)+1:
            if center_count <= max(self.a, self.b):
                self.get_logger().warning("Center line not enough, but left & right are enough => using center anyway.")
            # 使用中心线
            self.do_angle_publish_by_center(current_time)

        # case B: 如果仅左线足够 (而右线不足)
        elif left_count >= max(self.a, self.b)+1 and right_count < max(self.a, self.b)+1:
            self.get_logger().info("Right lane not enough, using Left + lane_width to simulate center.")
            # 用左线推断中心：简单地将 左线 点集 y 坐标 平移一半车道宽(assuming Y左)
            #   => center = left.y - lane_width_m/2
            self.do_angle_publish_by_one_side(
                side_points=self.left_points,
                side='left',
                current_time=current_time
            )

        # case C: 如果仅右线足够 (而左线不足)
        elif right_count >= max(self.a, self.b)+1 and left_count < max(self.a, self.b)+1:
            self.get_logger().info("Left lane not enough, using Right + lane_width to simulate center.")
            # 类似地 => center = right.y + lane_width_m/2
            self.do_angle_publish_by_one_side(
                side_points=self.right_points,
                side='right',
                current_time=current_time
            )

        # case D: 如果左右都不足 => 直行
        else:
            self.get_logger().warning("Both lanes not enough => assume straight (x=2.5,y=0,theta=0)")
            pose_msg = Pose2D()
            pose_msg.x = 2.5
            pose_msg.y = 0.0
            pose_msg.theta = 0.0
            self.publisher.publish(pose_msg)
            self.get_logger().info("Published default Pose2D (straight)")

    # -------------------------------------------------------------
    #   使用中心线 (已存到 self.center_points)，取第 a,b 点算角度
    # -------------------------------------------------------------
    def do_angle_publish_by_center(self, current_time):
        if len(self.center_points) <= max(self.a, self.b):
            self.get_logger().warning("Insufficient points in center line.")
            return

        # 取 a,b 索引
        point_a = self.center_points[self.a]
        point_b = self.center_points[self.b]
        # 计算 & 发布
        self.publish_angle_between_points(point_a, point_b, current_time)

    # -------------------------------------------------------------
    #   仅一侧可用时，通过平移来模拟 center
    #   side='left' => center = left.y - lane_width_m/2
    #   side='right'=> center = right.y + lane_width_m/2
    # -------------------------------------------------------------
    def do_angle_publish_by_one_side(self, side_points, side, current_time):
        if len(side_points) <= max(self.a, self.b):
            self.get_logger().warning(f"Side {side} not enough points => skip.")
            return

        # 拿到 a,b 两点
        pa = side_points[self.a]
        pb = side_points[self.b]

        # pa_center & pb_center
        if side=='left':
            # 车辆坐标系: X前, Y左 => 中心线应比左线在 Y 小 half_lane
            #   center.y = left.y - lane_width_m/2
            pa_center = (pa[0], pa[1] - self.lane_width_m/2, pa[2])
            pb_center = (pb[0], pb[1] - self.lane_width_m/2, pb[2])
        else:
            # side=='right'
            pa_center = (pa[0], pa[1] + self.lane_width_m/2, pa[2])
            pb_center = (pb[0], pb[1] + self.lane_width_m/2, pb[2])

        # 计算角度并发布
        self.publish_angle_between_points(pa_center, pb_center, current_time)

    # -------------------------------------------------------------
    #   共用解析函数
    # -------------------------------------------------------------
    def parse_pointcloud2(self, msg):
        points = []
        try:
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append(p)
        except Exception as e:
            self.get_logger().error(f"Error parsing PointCloud2: {e}")
        return points

    # -------------------------------------------------------------
    #   计算两个点的角度并发布 Pose2D
    # -------------------------------------------------------------
    def publish_angle_between_points(self, point_a, point_b, current_time):
        x1, y1, _ = point_a
        x2, y2, _ = point_b

        # 使用 atan2 计算角度
        angle = math.atan2(y2 - y1, x2 - x1)
        self.get_logger().info(f"Angle between point {self.a} and {self.b}: {angle:.2f} rad")

        y_1 = np.clip(y1, -2, 2)

        # 创建 Pose2D 消息并设置
        pose_msg = Pose2D()
        pose_msg.x = x1
        pose_msg.y = y_1
        pose_msg.theta = angle

        # 发布 Pose2D 消息
        self.publisher.publish(pose_msg)
        self.get_logger().info(f"Publish Pose2D: x={x1:.2f}, y={y_1:.2f}, theta={angle:.2f} rad")

        # 记录数据
        timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))
        self.data_b.append((timestamp, y_1))

    # -------------------------------------------------------------
    #   退出时保存数据
    # -------------------------------------------------------------
    def save_data_to_excel(self):
        folder_path = "./lane_analysis_data"
        b_file_path = f"{folder_path}/point_b_data1.xlsx"

        import os
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        pd.DataFrame(self.data_b, columns=["Time", "Point B Y"]).to_excel(b_file_path, index=False)
        self.get_logger().info(f"Data saved to {folder_path}")

def main(args=None):
    rclpy.init(args=args)

    a = 6  # 修改为需要的第一个点的索引
    b = 7  # 修改为需要的第二个点的索引

    lane_line_subscriber = LaneLineSubscriber(a, b)
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
