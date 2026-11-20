#!/usr/bin/env python3
"""Run an unchanged OA executable, restoring the lane source after it exits.

The existing CMake directory install includes this file. The launch invokes it
with the current Python interpreter, so an executable install rule is not needed.
On Linux, parent-death signals also stop OA if the launcher or runner vanishes.
The lane-source reset is best-effort when the runner itself is forcibly killed.
"""

import argparse
import ctypes
import os
import signal
import subprocess
import sys
import time


ALGORITHMS = ("oa_neo", "oa_neo_nurbs", "oa_neo_ubs", "oa_2")
PR_SET_PDEATHSIG = 1


def watch_parent(parent_pid, signum):
    """Arm the Linux parent-death signal and check whether the parent already exited."""
    if sys.platform.startswith("linux"):
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(PR_SET_PDEATHSIG, int(signum), 0, 0, 0) != 0:
            error = ctypes.get_errno()
            raise OSError(error, os.strerror(error))
    return parent_pid != 1 and os.getppid() == parent_pid


def exec_child(parent_pid, command):
    """Arm shutdown before replacing this process with the installed OA executable."""
    if not watch_parent(parent_pid, signal.SIGTERM):
        return 128 + signal.SIGTERM
    os.execv(command[0], command)


def reset_lane_source():
    """Publish False repeatedly during DDS discovery, for at most two seconds."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Bool

    context = Context()
    node = None
    executor = None
    rclpy.init(args=[], context=context)
    try:
        node = rclpy.create_node("obstacle_avoidance_source_reset", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        publisher = node.create_publisher(Bool, "/point_flag", 10)
        message = Bool(data=False)
        deadline = time.monotonic() + 2.0
        matched = False
        while time.monotonic() < deadline and rclpy.ok(context=context):
            publisher.publish(message)
            matched = matched or publisher.get_subscription_count() > 0
            executor.spin_once(timeout_sec=0.05)
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
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok(context=context):
            rclpy.shutdown(context=context)


def run_child(command):
    """Keep cleanup outside the algorithm's process and forward launch signals."""
    launcher_pid = os.getppid()
    child = None
    pending_signal = None
    previous_handlers = {}

    def forward_signal(signum, _frame):
        nonlocal pending_signal
        stop_signal = signal.SIGINT if signum == signal.SIGHUP else signum
        if child is None:
            pending_signal = stop_signal
        elif child.poll() is None:
            try:
                os.killpg(child.pid, stop_signal)
            except ProcessLookupError:
                pass

    try:
        for signum in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, forward_signal)
        if not watch_parent(launcher_pid, signal.SIGHUP):
            return 128 + signal.SIGHUP
        if pending_signal is not None:
            return 128 + pending_signal
        # Register the child's parent-death signal before exec, without a preexec
        # callback between fork and exec in the runner.
        child = subprocess.Popen(
            [sys.executable, os.path.abspath(__file__), "--oa-child", str(os.getpid()), *command],
            start_new_session=True,
        )
        if pending_signal is not None:
            forward_signal(pending_signal, None)
        returncode = child.wait()
        # The algorithm is gone before False is sent, so it cannot race the reset.
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
    if argv is None:
        argv = sys.argv[1:]
    if argv and argv[0] == "--oa-child":
        return exec_child(int(argv[1]), argv[2:])

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
