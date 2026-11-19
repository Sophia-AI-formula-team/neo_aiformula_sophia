"""Use the original Neo OA selector and discard cached points on a source switch."""
import math
from geometry_msgs.msg import Pose2D, PoseArray

import rclpy
from kalman_filter.withoutkalman_0312 import DataProcessingNode
from std_msgs.msg import Bool

OA_READY_TOPIC = "/semantic_drive/map_assist_045/oa_ready"
GROUP_TOPIC = "/semantic_drive/map_assist_045/oa_triplet"
MAX_TRANSIT_NS = 150_000_000


class FreshOaSwitch(DataProcessingNode):
    def __init__(self):
        super().__init__()
        self.oa_ready = self.create_publisher(Bool, OA_READY_TOPIC, 10)
        self.last_stamp = -1
        self.source_changed_at = -1
        self.create_subscription(PoseArray, GROUP_TOPIC, self.group_callback, 10)

    def flag_callback(self, message):
        previous_source = self.current_source
        super().flag_callback(message)
        if self.current_source != previous_source:
            self.A = self.B = self.C = None
            self.source_changed_at = self.get_clock().now().nanoseconds
            self.oa_ready.publish(Bool(data=False))

    def observation_callback_a(self, msg):
        if self.current_source == "processed":
            super().observation_callback_a(msg)

    def observation_callback_b(self, msg):
        if self.current_source == "processed":
            super().observation_callback_b(msg)

    def observation_callback_c(self, msg):
        if self.current_source == "processed":
            super().observation_callback_c(msg)

    def group_callback(self, msg):
        if self.current_source != "oa":
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        now = self.get_clock().now().nanoseconds
        if stamp <= max(self.source_changed_at, self.last_stamp):
            return
        if (msg.header.frame_id != "oa_window" or len(msg.poses) != 3
                or not 0 <= now - stamp <= MAX_TRANSIT_NS
                or not all(math.isfinite(v) for p in msg.poses for v in (p.position.x, p.position.y))):
            self.oa_ready.publish(Bool(data=False))
            return
        self.last_stamp = stamp
        a, b, c = [Pose2D(x=p.position.x, y=p.position.y, theta=0.0) for p in msg.poses]
        # One ROS message supplies every coordinate for this solver invocation.
        super().observation_callback_a(a)
        super().observation_callback_b(b)
        self.oa_ready.publish(Bool(data=True))
        super().observation_callback_c(c)


def main():
    rclpy.init()
    node = FreshOaSwitch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
