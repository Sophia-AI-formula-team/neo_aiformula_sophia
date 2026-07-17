import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
import time
import math
import tf_transformations

class LaneLineSubscriber(Node):
    def __init__(self, a, b, c):
        super().__init__('lane_line_subscriber')

        # ---------------------------------------------------
        #  (1) 订阅三条车道线
        # ---------------------------------------------------
        self.center_subscription = self.create_subscription(
            PointCloud2,
            '/aiformula_perception/lane_line_publisher/lane_lines/center',  
            self.center_callback,
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

        # ---------------------------------------------------
        #  (2) 发布者
        # ---------------------------------------------------
        self.pose_publisher_a = self.create_publisher(Pose2D, '/processed_point_a', 10)
        self.pose_publisher_b = self.create_publisher(Pose2D, '/processed_point_b', 10)
        self.pose_publisher_c = self.create_publisher(Pose2D, '/processed_point_c', 10)
        self.pose_publisher   = self.create_publisher(Pose2D, '/processed_pose', 10)

        # ---------------------------------------------------
        #  (3) 用于存储各条线的点云数据
        # ---------------------------------------------------
        self.left_points = []
        self.right_points = []
        self.center_points = []

        # ---------------------------------------------------
        #  (4) 其他参数
        # ---------------------------------------------------
        self.a = a
        self.b = b
        self.c = c

        self.last_received_time = 0.0

        # ---------------------------------------------------
        #  (5) 车道宽度 (米)
        # ---------------------------------------------------
        self.lane_width_m = 2.0

        # ---------------------------------------------------
        #  (6) 订阅像素点数量
        # ---------------------------------------------------
        self.create_subscription(
            Int32,
            '/red_pixels_count', 
            self.pixel_count_callback,
            10
        )

        # ---------------------------------------------------
        #  (7) 订阅 /odom 用于位姿判断(顺序播放时检测到达)
        # ---------------------------------------------------
        self.create_subscription(
            Odometry,
            '/aiformula_sensing/gyro_odometry_publisher/odom',
            self.odom_callback,
            10
        )
        self.current_pose = None

        # ---------------------------------------------------
        #  (8) 定义两组新的路径点
        # ---------------------------------------------------
        self.new_path_L = [
            Pose2D(x=0.7, y=0.72),
            Pose2D(x=1.4, y=1.44),
            Pose2D(x=2.1, y=2.16),
            Pose2D(x=2.8, y=2.88),
            Pose2D(x=3.5, y=3.6),
            Pose2D(x=4.2, y=3.6)
        ]
        self.new_path_R = [
            Pose2D(x=0.7, y=-0.72),
            Pose2D(x=1.4, y=-1.44),
            Pose2D(x=2.1, y=-2.16),
            Pose2D(x=2.8, y=-2.88),
            Pose2D(x=3.5, y=-3.6),
            Pose2D(x=4.2, y=-3.6)
        ]
        self.use_left = True

        # ---------------------------------------------------
        #  (9) 顺序行走相关的标志变量
        # ---------------------------------------------------
        self.is_executing_new_path = False  
        self.sequential_waypoints = []      
        self.current_waypoint_index = 0

        # ---------------------------------------------------
        #  (10) 其他初始化
        # ---------------------------------------------------
        self.previous_time = self.get_clock().now()

    # ============================================================
    #   A) 里程计回调：获取当前位姿；若在执行新路径 => 检测到达
    # ============================================================
    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        # 如果正在执行“红点触发”新路径 => 检查是否到达当前 waypoint
        if self.is_executing_new_path:
            self.check_if_arrived_and_publish_next()

    def check_if_arrived_and_publish_next(self):
        """若已到达当前轨迹点 => 切换下一点；若所有点完成 => 结束并切换use_left"""
        if self.current_waypoint_index < len(self.sequential_waypoints):
            # 当前目标
            target = self.sequential_waypoints[self.current_waypoint_index]
            tx, ty, ttheta = target.x, target.y, target.theta

            # 当前位姿
            cx = self.current_pose.position.x
            cy = self.current_pose.position.y
            qx = self.current_pose.orientation.x
            qy = self.current_pose.orientation.y
            qz = self.current_pose.orientation.z
            qw = self.current_pose.orientation.w
            roll, pitch, yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])

            # 计算误差
            dist = math.sqrt((tx - cx)**2 + (ty - cy)**2)
            dtheta = abs(ttheta - yaw)
            dtheta = abs(math.atan2(math.sin(dtheta), math.cos(dtheta)))

            dist_thr = 0.2
            yaw_thr  = 0.3

            if dist < dist_thr and dtheta < yaw_thr:
                self.get_logger().info(
                    f"[Waypoint #{self.current_waypoint_index}] Arrived! dist={dist:.2f}, yaw_err={dtheta:.2f}"
                )
                # 下一点
                self.current_waypoint_index += 1
                if self.current_waypoint_index < len(self.sequential_waypoints):
                    self.publish_current_waypoint()
                else:
                    # 全部播放完
                    self.is_executing_new_path = False
                    # 这里才做 “左->右 / 右->左” 的切换
                    self.use_left = not self.use_left
                    self.get_logger().info("All path done => Switch use_left and resume lane logic.")
            else:
                # 未到达 => 等待下一次 odom 回调
                pass

    def publish_current_waypoint(self):
        """发布当前索引的Waypoint到 /processed_pose"""
        if self.current_waypoint_index < len(self.sequential_waypoints):
            wp = self.sequential_waypoints[self.current_waypoint_index]
            self.pose_publisher.publish(wp)
            self.get_logger().info(
                f"Publish seq waypoint #{self.current_waypoint_index}: x={wp.x:.2f}, y={wp.y:.2f}, theta={wp.theta:.2f}"
            )

    # ============================================================
    #   B) pixel_count 回调：若 >20000 => 触发新路径 (一次)
    # ============================================================
    def pixel_count_callback(self, msg: Int32):
        pixel_count = msg.data
        self.get_logger().info(f"Received pixel_count: {pixel_count}")

        if pixel_count > 800:
            # 如果当前已经在执行新路径，就不重复触发
            if self.is_executing_new_path:
                return

            # 开始执行新的路径
            self.is_executing_new_path = True
            # 根据当前 self.use_left 来决定走左还是右
            if self.use_left:
                self.get_logger().warn("Go LEFT path...")
                self.sequential_waypoints = self.new_path_L
            else:
                self.get_logger().warn("Go RIGHT path...")
                self.sequential_waypoints = self.new_path_R

            # 重置并发布第0个点
            self.current_waypoint_index = 0
            self.publish_current_waypoint()
            # 注意：不在这里 self.use_left = not self.use_left
            # 而是要在路径播放"结束"后再切换

    # ============================================================
    #   C) 车道线逻辑 (若未执行新路径)
    # ============================================================
    def center_callback(self, msg):
        if self.is_executing_new_path:
            # 执行新路径时，跳过车道逻辑
            return

        current_time = time.time()
        self.last_received_time = current_time

        self.center_points = self.parse_pointcloud2(msg)
        center_count = len(self.center_points)

        left_count = len(self.left_points)
        right_count = len(self.right_points)

        if left_count >= max(self.a, self.b, self.c)+1 and right_count >= max(self.a, self.b, self.c)+1:
            if center_count <= max(self.a, self.b, self.c):
                self.get_logger().warning("Center line not enough, but left & right are enough => using center anyway.")
            self.do_publish_points_by_center(current_time)
        elif left_count >= max(self.a, self.b, self.c)+1 and right_count < max(self.a, self.b, self.c)+1:
            self.get_logger().info("Right lane not enough, using Left + lane_width to simulate center.")
            self.do_publish_points_by_one_side(
                side_points=self.left_points,
                side='left',
                current_time=current_time
            )
        elif right_count >= max(self.a, self.b, self.c)+1 and left_count < max(self.a, self.b, self.c)+1:
            self.get_logger().info("Left lane not enough, using Right + lane_width to simulate center.")
            self.do_publish_points_by_one_side(
                side_points=self.right_points,
                side='right',
                current_time=current_time
            )
        else:
            self.get_logger().warning("Both lanes not enough => assume straight (x=2.5,y=0,theta=0)")
            pose_msg = Pose2D()
            pose_msg.x = 2.5
            pose_msg.y = 0.0
            pose_msg.theta = 0.0
            self.pose_publisher_a.publish(pose_msg)
            self.pose_publisher_b.publish(pose_msg)
            self.pose_publisher_c.publish(pose_msg)
            self.get_logger().info("Published default Pose2D (straight)")

    def left_callback(self, msg):
        if not self.is_executing_new_path:
            self.left_points = self.parse_pointcloud2(msg)

    def right_callback(self, msg):
        if not self.is_executing_new_path:
            self.right_points = self.parse_pointcloud2(msg)

    # ============================================================
    #   D) 封装
    # ============================================================
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
            self.get_logger().warning(f"Side {side} not enough => skip.")
            return
        pa = side_points[self.a]
        pb = side_points[self.b]
        pc = side_points[self.c]

        if side == 'left':
            pa_center = (pa[0], pa[1] - self.lane_width_m/2, pa[2])
            pb_center = (pb[0], pb[1] - self.lane_width_m/2, pb[2])
            pc_center = (pc[0], pc[1] - self.lane_width_m/2, pc[2])
        else:
            pa_center = (pa[0], pa[1] + self.lane_width_m/2, pa[2])
            pb_center = (pb[0], pb[1] + self.lane_width_m/2, pb[2])
            pc_center = (pc[0], pc[1] + self.lane_width_m/2, pc[2])

        self.publish_points_abc(pa_center, pb_center, pc_center, current_time)

    def publish_points_abc(self, point_a, point_b, point_c, current_time):
        pose_a = Pose2D()
        pose_a.x, pose_a.y, _ = point_a
        pose_a.theta = 0.0
        self.pose_publisher_a.publish(pose_a)

        pose_b = Pose2D()
        pose_b.x, pose_b.y, _ = point_b
        pose_b.theta = 0.0
        self.pose_publisher_b.publish(pose_b)

        pose_c = Pose2D()
        pose_c.x, pose_c.y, _ = point_c
        pose_c.theta = 0.0
        self.pose_publisher_c.publish(pose_c)

    def parse_pointcloud2(self, msg):
        points = []
        try:
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append(p)
        except Exception as e:
            self.get_logger().error(f"Error parsing PointCloud2: {e}")
        return points

def main(args=None):
    rclpy.init(args=args)
    lane_line_subscriber = LaneLineSubscriber(a=6, b=7, c=8)
    try:
        rclpy.spin(lane_line_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        lane_line_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
