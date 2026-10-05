#!/usr/bin/env python3
"""Run an unchanged OA executable, restoring the lane source after it exits.

The existing CMake directory install includes this file. The launch invokes it
with the current Python interpreter, so an executable install rule is not needed.
This bounded best-effort cleanup cannot run after SIGKILL, power loss, or a host
failure. It does not add a watchdog or change the existing three-point algorithm.
"""

import argparse
import os
import signal
import subprocess
import sys
import time


ALGORITHMS = ("oa_neo", "oa_neo_nurbs", "oa_neo_ubs")


def reset_lane_source():
    """Publish False repeatedly during DDS discovery, for at most two seconds."""
    import rclpy
    from rclpy.context import Context
    from std_msgs.msg import Bool

    context = Context()
    node = None
    rclpy.init(args=[], context=context)
    try:
        node = rclpy.create_node("obstacle_avoidance_source_reset", context=context)
        publisher = node.create_publisher(Bool, "/point_flag", 10)
        message = Bool(data=False)
        deadline = time.monotonic() + 2.0
        matched = False
        while time.monotonic() < deadline and rclpy.ok(context=context):
            publisher.publish(message)
            matched = matched or publisher.get_subscription_count() > 0
            rclpy.spin_once(node, timeout_sec=0.05)
        if matched:
            print("OA exited: sent /point_flag=false to restore the lane source.", flush=True)
        else:
            print(
                "OA exited: reset published, but no /point_flag subscriber was discovered; "
                "source restoration is unconfirmed.",
                file=sys.stderr,
                flush=True,
            )
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok(context=context):
            rclpy.shutdown(context=context)


def run_child(command):
    """Keep cleanup outside the algorithm's process and forward launch signals."""
    child = subprocess.Popen(command, start_new_session=True)
    previous_handlers = {}

    def forward_signal(signum, _frame):
        if child.poll() is None:
            try:
                os.killpg(child.pid, signum)
            except ProcessLookupError:
                pass

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, forward_signal)
        returncode = child.wait()
        # The algorithm is gone before False is sent, so it cannot race the reset.
        # A second soft stop may interrupt cleanup; forced termination cannot be
        # made reliable by a process-local wrapper.
        try:
            reset_lane_source()
        except Exception as error:
            print(
                "OA source reset failed: {}. Publish /point_flag=false before resuming "
                "lane processing.".format(error),
                file=sys.stderr,
                flush=True,
            )
            return 1
        return returncode if returncode >= 0 else 128 - returncode
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("algorithm", choices=ALGORITHMS)
    parser.add_argument("ros_args", nargs=argparse.REMAINDER)
    arguments = parser.parse_args(argv)

    from ament_index_python.packages import get_package_prefix

    executable = os.path.join(
        get_package_prefix("obsticle_avoidence"),
        "lib",
        "obsticle_avoidence",
        arguments.algorithm,
    )
    if not os.path.isfile(executable) or not os.access(executable, os.X_OK):
        parser.error("Installed avoidance executable not found: {}".format(executable))
    return run_child([executable] + arguments.ros_args)


if __name__ == "__main__":
    sys.exit(main())
