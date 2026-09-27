"""Own exactly one remapped LYA process; never search for or kill other nodes."""

from datetime import datetime, timezone
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String


class TeacherSupervisor(Node):
    def __init__(self):
        super().__init__("lane_teacher_supervisor")
        for key, value in {
            "teacher_package": "trajectory_follower",
            "teacher_executable": "lya_follower_connected_omegat_global",
            "teacher_command_topic": "/lane_learning/lya_cmd",
            "control_state_topic": "/lane_learning/control_state",
            "teacher_state_topic": "/lane_learning/teacher_state",
            "emergency_stop_topic": "/lane_learning/emergency_stop",
            "log_directory": "~/.ros/lane_learning",
        }.items():
            self.declare_parameter(key, value)
        value = lambda key: self.get_parameter(key).value
        if value("teacher_command_topic") != "/lane_learning/lya_cmd":
            raise ValueError("managed LYA must publish only to /lane_learning/lya_cmd")
        directory = Path(str(value("log_directory"))).expanduser() / (
            "teacher_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            + "_" + uuid.uuid4().hex[:8]
        )
        directory.mkdir(parents=True, exist_ok=False)
        self._logger = logging.getLogger("lane_teacher_" + directory.name)
        self._logger.setLevel(logging.INFO)
        self._logger.propagate = False
        self._handler = RotatingFileHandler(
            str(directory / "teacher.log"), maxBytes=5_000_000, backupCount=4,
            encoding="utf-8",
        )
        self._logger.addHandler(self._handler)
        self._event("starting", log_directory=str(directory))
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status = self.create_publisher(String, str(value("teacher_state_topic")), qos)
        self._estop = self.create_publisher(Bool, str(value("emergency_stop_topic")), qos)
        self.create_subscription(String, str(value("control_state_topic")), self._control, qos)
        package = str(value("teacher_package"))
        executable = str(value("teacher_executable"))
        if Path(executable).name != executable:
            raise ValueError("teacher_executable must be an installed executable name")
        # Start the installed executable itself, not the ros2 run wrapper. The
        # PID we supervise is the LYA process and its owned process group.
        binary = Path(get_package_prefix(package)) / "lib" / package / executable
        if not binary.is_file():
            raise ValueError("LYA executable is not installed: " + str(binary))
        command = [str(binary),
            "--ros-args", "-r", "/aiformula_control/game_pad/cmd_vel:="
            + str(value("teacher_command_topic")),
        ]
        self._expected_stop = False
        self._stop_started = None
        self._reported_exit = False
        self._control_seen = None
        options = {"start_new_session": True} if os.name != "nt" else {
            "creationflags": subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW
        }
        self._process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, encoding="utf-8", errors="replace", bufsize=1, **options
        )
        self._reader = threading.Thread(target=self._read_output, daemon=True)
        self._reader.start()
        self._started = time.monotonic()
        self._timer = self.create_timer(0.05, self._poll, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._event("spawned", pid=self._process.pid, command=command)
        self.get_logger().info("Managed LYA PID {}; logs {}".format(self._process.pid, directory))

    def _event(self, event, **values):
        self._logger.info(json.dumps({"wall_time": time.time(), "event": event, **values}))

    def _read_output(self):
        try:
            for line in self._process.stdout:
                self._logger.info("LYA %s", line.rstrip())
        except Exception as error:
            self._event("stdout_error", error=repr(error))

    def _control(self, message):
        try:
            state = json.loads(message.data)
            if not isinstance(state, dict):
                raise ValueError("state must be an object")
            age = (self.get_clock().now().nanoseconds - int(state["stamp_ns"])) * 1e-9
            if not 0 <= age <= 1.0:
                raise ValueError("stale controller state")
            if state.get("safety_mode") not in ("fixed_only", "lya_reference"):
                raise ValueError("unknown safety mode")
            self._control_seen = time.monotonic()
            if state.get("safety_mode") == "fixed_only" and state.get("teacher_enabled") is False:
                self._request_stop("fixed_route_handoff")
        except (ValueError, TypeError, KeyError):
            self._event("malformed_control_state")

    def _group_alive(self):
        # Reap the direct child first. A surviving descendant still prevents a
        # STOPPED acknowledgement, even after the parent executable exits.
        code = self._process.poll()
        if os.name == "nt":
            return code is None
        try:
            os.killpg(self._process.pid, 0)
            return True
        except ProcessLookupError:
            return False

    def _signal_group(self, force=False):
        try:
            if os.name == "nt":
                if self._process.poll() is None:
                    self._process.kill() if force else self._process.terminate()
            else:
                os.killpg(self._process.pid, signal.SIGKILL if force else signal.SIGINT)
        except ProcessLookupError:
            pass  # Child/group may have exited between observation and signal.

    def _request_stop(self, reason):
        if self._expected_stop:
            return
        self._expected_stop = True
        self._stop_started = time.monotonic()
        self._event("stop_requested", reason=reason)
        self._signal_group()

    def _poll(self):
        now = time.monotonic()
        if self._control_seen is not None and now - self._control_seen > 2.0:
            self._estop.publish(Bool(data=True))
            self._request_stop("control_heartbeat_lost")
        elif self._control_seen is None and now - self._started > 10.0:
            self._request_stop("controller_not_available")
        code = self._process.poll()
        alive = self._group_alive()
        if alive and self._stop_started is not None and now - self._stop_started > 3.0:
            self._signal_group(force=True)
            self._event("owned_process_forced_stop")
        state = ("STOPPING" if self._expected_stop else "RUNNING") if alive else (
            "STOPPED" if self._expected_stop else "FAILED")
        if code is not None and not self._reported_exit:
            self._reported_exit = True
            self._event("exited", expected=self._expected_stop, returncode=code)
            if not self._expected_stop:
                self._estop.publish(Bool(data=True))
                self._request_stop("unexpected_parent_exit_cleanup")
        self._status.publish(String(data=json.dumps({
            "state": state, "status": state, "stopped": state == "STOPPED",
            "running": alive, "pid": self._process.pid,
            "stamp_ns": self.get_clock().now().nanoseconds,
        })))

    def close(self):
        self._request_stop("supervisor_shutdown")
        try:
            self._process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self._signal_group(force=True)
            self._process.wait(timeout=2.0)
        if self._group_alive():
            self._signal_group(force=True)
        self._reader.join(timeout=1.0)
        self._handler.close()
        self._logger.removeHandler(self._handler)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TeacherSupervisor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
