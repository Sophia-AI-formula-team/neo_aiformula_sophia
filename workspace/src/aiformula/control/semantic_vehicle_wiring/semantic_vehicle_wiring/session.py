"""Explicit launch lifetime: healthy exit disables; an unexpected loss keeps a hold."""
import json
import os
import signal
import time
import uuid


def semantic_main():
    os.execvp('ros2', ['ros2', 'launch', 'launchers', 'semantic_vehicle.launch.py'])


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init(args=args)
    node = Node('motor_gate_session')
    publisher = node.create_publisher(String, '/semantic_drive/motor_session', 10)
    identity, sequence = uuid.uuid4().hex, 0
    stopping = [False]
    def stop(signum, frame):
        stopping[0] = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    def publish(active):
        nonlocal sequence
        sequence += 1
        publisher.publish(String(data=json.dumps(dict(session=identity, sequence=sequence,
            stamp_ns=node.get_clock().now().nanoseconds, active=active))))
    try:
        while not stopping[0] and rclpy.ok():
            publish(True)
            until = time.monotonic() + 0.1
            while not stopping[0] and rclpy.ok() and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        if rclpy.ok():
            # Several fresh explicit releases cover discovery/queue scheduling.
            # A missing release is a hold, never an implicit bypass of a red.
            for _ in range(5):
                publish(False)
                rclpy.spin_once(node, timeout_sec=0.04)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
