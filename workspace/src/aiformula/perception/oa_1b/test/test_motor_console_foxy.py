"""Exercise the deployed Foxy motor console inside a disposable, device-free container."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


NS = "/aiformula_control"
MOTOR = NS + "/motor_controller"
JOY = NS + "/joy_node/joy_l2_throttle"


def check(args, private):
    site = args.vehicle / "lib/python3.8/site-packages"
    env = os.environ.copy()
    paths = [str(site), env["PYTHONPATH"]]
    if private:
        paths.insert(0, str(args.control / "python"))
    env["PYTHONPATH"] = os.pathsep.join(paths)
    executable = args.vehicle / "lib/motor_controller/motor_controller"
    command = [sys.executable, str(executable), "--ros-args", "-r", "__ns:=" + NS,
               "-r", "pub_can:=/oa1b_bench/can_only", "--params-file", str(args.wheel),
               "--params-file", str(args.vehicle / "share/motor_controller/config/motor_controller.yaml"),
               "--params-file", str(args.gamepad)]
    motor = subprocess.Popen(command, env=env)
    rclpy.init()
    probe = Node("oa1b_console_test")
    events, gate = [], []
    joy = probe.create_publisher(Joy, JOY, 10)
    probe.create_subscription(Time, MOTOR + "/manual_motion_started", events.append, 10)
    probe.create_subscription(String, MOTOR + "/gate_status", lambda msg: gate.append(json.loads(msg.data)), 10)

    def listen(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if motor.poll() is not None:
                raise RuntimeError("Installed vehicle console exited early")
            rclpy.spin_once(probe, timeout_sec=0.025)

    def publish(held, throttle):
        msg = Joy()
        msg.axes = [0.0, throttle, 0.0, 0.0]
        msg.buttons = [0] * 8
        msg.buttons[6] = int(held)
        for _ in range(5):
            joy.publish(msg)
            listen(0.04)

    try:
        deadline = time.monotonic() + 10
        while (not gate or not probe.count_subscribers(JOY)) and time.monotonic() < deadline:
            listen(0.05)
        assert gate and probe.count_subscribers(JOY) == 1
        assert probe.count_publishers(MOTOR + "/manual_motion_started") == int(private)
        assert probe.count_publishers("/oa1b_bench/can_only") == 1
        publish(True, 0.0)
        assert len(events) == 0
        publish(True, 0.3)
        publish(True, 0.3)
        assert len(events) == int(private)
        assert any(v["manual_held"] and abs(v["command"][0] - 1.05) < .0001 for v in gate)
        publish(False, 0.0)
        publish(True, 0.3)
        assert len(events) == 2 * int(private)
        publish(False, 0.0)
        print(json.dumps(dict(private=private, motion_edges=len(events), manual_mps=1.05,
                              basename=executable.name, can_topic="/oa1b_bench/can_only")))
    finally:
        if motor.poll() is None:
            motor.send_signal(signal.SIGINT)
        motor.wait(timeout=5)
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ["vehicle", "control", "wheel", "gamepad"]:
        parser.add_argument("--"+name, required=True, type=Path)
    args = parser.parse_args()
    check(args, True)
    check(args, False)
