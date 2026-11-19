"""Use the original Neo OA selector and discard cached points on a source switch."""
import math
import time

import rclpy
from kalman_filter.withoutkalman_0312 import DataProcessingNode
from std_msgs.msg import Bool

OA_READY_TOPIC = "/semantic_drive/map_assist_045/oa_ready"
MAX_OA_POINT_SKEW_SEC = 0.15


class FreshOaSwitch(DataProcessingNode):
    def __init__(self):
        super().__init__()
        self.oa_ready = self.create_publisher(Bool, OA_READY_TOPIC, 10)
        self.oa_point_times = {}

    def flag_callback(self, message):
        previous_source = self.current_source
        super().flag_callback(message)
        if self.current_source != previous_source:
            self.A = self.B = self.C = None
            self.oa_point_times.clear()
            self.oa_ready.publish(Bool(data=False))

    def _accept_point(self, name, message):
        if self.current_source != "oa":
            return True
        if not all(math.isfinite(value) for value in (message.x, message.y, message.theta)):
            self.A = self.B = self.C = None
            self.oa_point_times.clear()
            self.oa_ready.publish(Bool(data=False))
            return False
        self.oa_point_times[name] = time.monotonic()
        return True

    def observation_callback_a(self, message):
        if self._accept_point("a", message):
            super().observation_callback_a(message)

    def observation_callback_b(self, message):
        if self._accept_point("b", message):
            super().observation_callback_b(message)

    def observation_callback_c(self, message):
        if not self._accept_point("c", message):
            return
        if self.current_source == "oa":
            if len(self.oa_point_times) != 3 or self.A is None or self.B is None:
                return
            times = self.oa_point_times.values()
            if max(times) - min(times) > MAX_OA_POINT_SKEW_SEC:
                self.oa_ready.publish(Bool(data=False))
                return
            self.oa_point_times.clear()
            self.oa_ready.publish(Bool(data=True))
        super().observation_callback_c(message)


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
