import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose2D
import pandas as pd
import time
from std_msgs.msg import Bool

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

        self.stop_flag = False
        self.flag_sub = self.create_subscription(
            Bool,
            '/lane_stop_flag',
            self.flag_cb,
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

        # ==== 4) 选点索引 ====
        self.a = a
        self.b = b
        self.c = c

        self.data_b = []
        self.last_received_time = 0.0

        # ==== 5) 车道宽度 (米) ====
        self.lane_width_m = 2.6

        # ==== 6) 你要的“手动坐标调整”：发布前缩放系数 ====
        self.xy_scale = 0.5   # x,y 都缩小一半

    def flag_cb(self, msg: Bool):
        self.stop_flag = msg.data
        self.get_logger().info(f"Receive stop flag: {self.stop_flag}")

    def left_callback(self, msg):
        self.left_points = self.parse_pointcloud2(msg)
        self.get_logger().debug(f"Received LEFT lane with {len(self.left_points)} points")

    def right_callback(self, msg):
        self.right_points = self.parse_pointcloud2(msg)
        self.get_logger().debug(f"Received RIGHT lane with {len(self.right_points)} points")

    def center_callback(self, msg):
        if self.stop_flag:
            return

        current_time = time.time()
        self.last_received_time = current_time

        self.center_points = self.parse_pointcloud2(msg)
        center_count = len(self.center_points)

        left_count = len(self.left_points)
        right_count = len(self.right_points)

        need_n = max(self.a, self.b, self.c) + 1

        # 如果左右线都够，则直接用 center
        if left_count >= need_n and right_count >= need_n:
            if center_count < need_n:
                self.get_logger().warning("Center line not enough, but left & right are enough => using center anyway.")
            self.do_publish_points_by_center(current_time)

        # 如果仅左线足够
        elif left_count >= need_n and right_count < need_n:
            self.get_logger().info("Right lane not enough, using Left + lane_width to simulate center.")
            self.do_publish_points_by_one_side(
                side_points=self.left_points,
                side='left',
                current_time=current_time
            )

        # 如果仅右线足够
        elif right_count >= need_n and left_count < need_n:
            self.get_logger().info("Left lane not enough, using Right + lane_width to simulate center.")
            self.do_publish_points_by_one_side(
                side_points=self.right_points,
                side='right',
                current_time=current_time
            )

        else:
            # 两侧都不足 => 默认直行
            self.get_logger().warning("Both lanes not enough => assume straight (x=2.5,y=0,theta=0)")
            pose_msg_a = Pose2D(); pose_msg_a.x = 2.5; pose_msg_a.y = 0.0; pose_msg_a.theta = 0.0
            pose_msg_b = Pose2D(); pose_msg_b.x = 3.0; pose_msg_b.y = 0.0; pose_msg_b.theta = 0.0
            pose_msg_c = Pose2D(); pose_msg_c.x = 3.5; pose_msg_c.y = 0.0; pose_msg_c.theta = 0.0

            # 默认直行也做同样缩放（如果你不想缩放默认直行，就把下面三行删掉）
            pose_msg_a.x *= self.xy_scale; pose_msg_a.y *= self.xy_scale
            pose_msg_b.x *= self.xy_scale; pose_msg_b.y *= self.xy_scale
            pose_msg_c.x *= self.xy_scale; pose_msg_c.y *= self.xy_scale

            self.pose_publisher_a.publish(pose_msg_a)
            self.pose_publisher_b.publish(pose_msg_b)
            self.pose_publisher_c.publish(pose_msg_c)
            self.get_logger().info("Published default Pose2D (straight)")

    # ---------- 核心：统一缩放函数 ----------
    def apply_xy_scale(self, p):
        # p = (x,y,z) or (x,y,anything)
        return (p[0] * self.xy_scale, p[1] * self.xy_scale, p[2])

    def do_publish_points_by_center(self, current_time):
        if len(self.center_points) <= max(self.a, self.b, self.c):
            self.get_logger().warning("Insufficient points in center line.")
            return

        point_a = self.center_points[self.a]
        point_b = self.center_points[self.b]
        point_c = self.center_points[self.c]

        self.publish_points_abc(point_a, point_b, point_c, current_time)

    def do_publish_points_by_one_side(self, side_points, side, current_time):
        if len(side_points) <= max(self.a, self.b, self.c):
            self.get_logger().warning(f"Side {side} not enough points => skip.")
            return

        pa = side_points[self.a]
        pb = side_points[self.b]
        pc = side_points[self.c]

        if side == 'left':
            pa_center = (pa[0], pa[1] - self.lane_width_m/1.4, pa[2])
            pb_center = (pb[0], pb[1] - self.lane_width_m/1.2, pb[2])
            pc_center = (pc[0], pc[1] - self.lane_width_m/1.2, pc[2])
        else:
            pa_center = (pa[0], pa[1] + self.lane_width_m/2, pa[2])
            pb_center = (pb[0], pb[1] + self.lane_width_m/1.8, pb[2])
            pc_center = (pc[0], pc[1] + self.lane_width_m/1.8, pc[2])

        self.publish_points_abc(pa_center, pb_center, pc_center, current_time)

    def publish_points_abc(self, point_a, point_b, point_c, current_time):
        # ===== 你要的“发布前缩小一半”：只在这里做，保证所有分支一致 =====
        point_a = self.apply_xy_scale(point_a)
        point_b = self.apply_xy_scale(point_b)
        point_c = self.apply_xy_scale(point_c)

        pose_a = Pose2D()
        pose_a.x, pose_a.y, _ = point_a
        pose_a.theta = 0.0
        self.pose_publisher_a.publish(pose_a)
        self.get_logger().info(f"Publish Pose2D(A): x={pose_a.x:.2f}, y={pose_a.y:.2f}, theta=0")

        pose_b = Pose2D()
        pose_b.x, pose_b.y, _ = point_b
        pose_b.theta = 0.0
        self.pose_publisher_b.publish(pose_b)
        self.get_logger().info(f"Publish Pose2D(B): x={pose_b.x:.2f}, y={pose_b.y:.2f}, theta=0")

        pose_c = Pose2D()
        pose_c.x, pose_c.y, _ = point_c
        pose_c.theta = 0.0
        self.pose_publisher_c.publish(pose_c)
        self.get_logger().info(f"Publish Pose2D(C): x={pose_c.x:.2f}, y={pose_c.y:.2f}, theta=0")

    def parse_pointcloud2(self, msg):
        points = []
        try:
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append(p)
        except Exception as e:
            self.get_logger().error(f"Error parsing PointCloud2: {e}")
        return points

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

    # 你要的 “468点” => 索引 4, 6, 8
    a = 4
    b = 6
    c = 8

    lane_line_subscriber = LaneLineSubscriber(a, b, c)
    try:
        rclpy.spin(lane_line_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        lane_line_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()