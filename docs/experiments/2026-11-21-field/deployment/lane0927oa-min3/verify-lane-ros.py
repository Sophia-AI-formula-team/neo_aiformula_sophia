"""Check the exact candidate over local-only ROS topics."""

from datetime import datetime
import sys
import time
from types import ModuleType

from geometry_msgs.msg import Pose2D
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Bool, Header

# The uncalled Excel export is the module's sole pandas consumer.
sys.modules.setdefault('pandas', ModuleType('pandas'))
from lane_0927oa import LaneLineSubscriber


def main():
    rclpy.init(args=['--ros-args', '-r', '/lane_stop_flag:=/point_flag'])
    lane = LaneLineSubscriber(0, 2, 4)
    io = rclpy.create_node('lane_0927oa_static_check')
    executor = SingleThreadedExecutor()
    executor.add_node(io)
    executor.add_node(lane)

    clouds = {key: io.create_publisher(PointCloud2, 'lane_line_' + key, 10)
              for key in ('left', 'right', 'center')}
    ownership = io.create_publisher(Bool, '/point_flag', 10)
    outputs = {label: [] for label in 'abc'}
    for label, log in outputs.items():
        io.create_subscription(Pose2D, '/original_processed_point_' + label, log.append, 10)

    def wait_for(condition):
        end = time.monotonic() + 4.0
        while not condition() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        assert condition()

    def cloud(key, count):
        y = {'left': 1.30, 'right': -1.40, 'center': 0.5}[key]
        msg = create_cloud_xyz32(Header(frame_id='base_link'), [(float(j + 1), y, 0.0) for j in range(count)])
        clouds[key].publish(msg)

    def sides(left, right):
        cloud('left', left)
        cloud('right', right)
        wait_for(lambda: len(lane.left_points) == left and len(lane.right_points) == right)

    def test(left, right, center, x, y):
        sizes = {label: len(log) for label, log in outputs.items()}
        sides(left, right)
        cloud('center', center)
        wait_for(lambda: all(len(log) == sizes[label] + 1 for label, log in outputs.items()))
        actual = [(log[-1].x, round(log[-1].y, 3)) for log in outputs.values()]
        assert actual == list(zip(x, [y] * 3)), actual
        print(datetime.now().astimezone().isoformat(), f'{left}/{right}/{center} -> {actual}', flush=True)

    try:
        wait_for(lambda: all(p.get_subscription_count() for p in (*clouds.values(), ownership))
                 and all(p.get_subscription_count() for p in (
                     lane.pose_publisher_a, lane.pose_publisher_b, lane.pose_publisher_c)))
        test(3, 0, 0, (1.0, 2.0, 3.0), 0.2)
        test(0, 4, 0, (1.0, 3.0, 4.0), -0.3)
        test(3, 3, 3, (1.0, 2.0, 3.0), 0.5)
        test(5, 5, 6, (1.0, 3.0, 5.0), 0.5)
        test(2, 2, 0, (1.5, 2.0, 3.0), 0.0)
        ownership.publish(Bool(data=True))
        wait_for(lambda: lane.stop_flag)
        old_sizes = [len(log) for log in outputs.values()]
        cloud('center', 3)
        until = time.monotonic() + 0.15
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=0.01)
        assert old_sizes == [len(log) for log in outputs.values()]
        print(datetime.now().astimezone().isoformat(), 'OA ownership suppressed lane ABC', flush=True)
    finally:
        executor.shutdown()
        lane.destroy_node()
        io.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
