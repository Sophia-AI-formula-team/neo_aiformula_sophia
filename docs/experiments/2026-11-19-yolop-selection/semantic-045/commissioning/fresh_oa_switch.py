"""Use the original Neo OA selector and discard cached points on a source switch."""
import rclpy
from kalman_filter.withoutkalman_0312 import DataProcessingNode


class FreshOaSwitch(DataProcessingNode):
    def flag_callback(self, message):
        previous_source = self.current_source
        super().flag_callback(message)
        if self.current_source != previous_source:
            self.A = self.B = self.C = None


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
