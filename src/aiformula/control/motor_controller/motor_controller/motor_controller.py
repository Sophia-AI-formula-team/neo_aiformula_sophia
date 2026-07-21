#!/usr/bin/env python
from __future__ import annotations

import csv
import io
import math
import sys
from collections import deque
from pathlib import Path
from typing import List

import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter as RclpyParameter
try:
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
except ImportError:
    from rclpy.qos import QoSDurabilityPolicy as DurabilityPolicy
    from rclpy.qos import QoSProfile
    from rclpy.qos import QoSReliabilityPolicy as ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import ByteMultiArray, Float64MultiArray
from torch import nn


class CorrectionControlModel(nn.Module):
    def __init__(
        self,
        history_dim: int,
        hidden_size: int,
        gru_layers: int,
        dropout: float,
        gain_span: float,
        max_bias_v: float,
        max_bias_omega: float,
    ) -> None:
        super().__init__()
        self.gain_span = float(gain_span)
        self.max_bias_v = float(max_bias_v)
        self.max_bias_omega = float(max_bias_omega)
        self.gru = nn.GRU(
            input_size=history_dim,
            hidden_size=hidden_size,
            num_layers=gru_layers,
            batch_first=True,
            dropout=dropout if gru_layers > 1 else 0.0,
        )
        self.head = nn.Sequential(
            nn.Linear(hidden_size + 2, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 4),
        )

    def forward(self, history_norm: torch.Tensor, command_norm: torch.Tensor) -> torch.Tensor:
        _, hidden = self.gru(history_norm)
        context = hidden[-1]
        raw = self.head(torch.cat([context, command_norm], dim=-1))
        a_v = 1.0 + self.gain_span * torch.tanh(raw[:, 0])
        a_omega = 1.0 + self.gain_span * torch.tanh(raw[:, 1])
        b_v = self.max_bias_v * torch.tanh(raw[:, 2])
        b_omega = self.max_bias_omega * torch.tanh(raw[:, 3])
        return torch.stack([a_v, a_omega, b_v, b_omega], dim=-1)


class RpmAffineModel(nn.Module):
    def __init__(
        self,
        history_dim: int,
        command_dim: int,
        hidden_size: int,
        gru_layers: int,
        dropout: float,
        gain_span: float,
        max_bias_rpm: float,
    ) -> None:
        super().__init__()
        self.gain_span = float(gain_span)
        self.max_bias_rpm = float(max_bias_rpm)
        self.gru = nn.GRU(
            input_size=history_dim,
            hidden_size=hidden_size,
            num_layers=gru_layers,
            batch_first=True,
            dropout=dropout if gru_layers > 1 else 0.0,
        )
        self.head = nn.Sequential(
            nn.Linear(hidden_size + command_dim, 96),
            nn.ReLU(),
            nn.Linear(96, 48),
            nn.ReLU(),
            nn.Linear(48, 4),
        )

    def forward(self, history_norm: torch.Tensor, command_norm: torch.Tensor) -> torch.Tensor:
        _, hidden = self.gru(history_norm)
        context = hidden[-1]
        raw = self.head(torch.cat([context, command_norm], dim=-1))
        a_right = 1.0 + self.gain_span * torch.tanh(raw[:, 0])
        a_left = 1.0 + self.gain_span * torch.tanh(raw[:, 1])
        b_right = self.max_bias_rpm * torch.tanh(raw[:, 2])
        b_left = self.max_bias_rpm * torch.tanh(raw[:, 3])
        return torch.stack([a_right, a_left, b_right, b_left], dim=-1)


class MotorController(Node):
    DEBUG_FIELDS = [
        "node_time_sec",
        "controller_state",
        "base_v",
        "base_omega",
        "applied_v",
        "applied_omega",
        "model_corrected_v",
        "model_corrected_omega",
        "delta_v",
        "delta_omega",
        "a_v",
        "a_omega",
        "b_v",
        "b_omega",
        "used_model_estimate",
        "used_model_applied",
        "history_len",
        "history_ready",
        "history_span_sec",
        "latest_meas_v",
        "latest_meas_omega",
        "right_rpm",
        "left_rpm",
        "can_right_rpm",
        "can_left_rpm",
        "rpm_weights_loaded",
        "rpm_correction_ready",
        "rpm_model_applied",
        "rpm_model_right_rpm",
        "rpm_model_left_rpm",
        "rpm_a_right",
        "rpm_a_left",
        "rpm_b_right",
        "rpm_b_left",
        "rpm_history_len",
        "rpm_history_ready",
    ]

    RPM_FEATURE_COLUMNS = ["cmd_v", "cmd_omega", "meas_v", "meas_omega", "ideal_right_rpm", "ideal_left_rpm"]
    RPM_COMMAND_COLUMNS = ["ideal_right_rpm", "ideal_left_rpm", "cmd_v", "cmd_omega"]

    STATE_IDEAL = 0
    STATE_BKUP = 1
    STATE_CORRECTED = 2
    VALID_STATES = {STATE_IDEAL, STATE_BKUP, STATE_CORRECTED}

    def __init__(self):
        super().__init__("motor_controller")

        self.declare_parameter("controller_state", self.STATE_BKUP)
        self.declare_parameter("publish_timer_loop_duration", 0.01)
        self.declare_parameter("wheel.tread", 0.60)
        self.declare_parameter("wheel.diameter", 0.254)
        self.declare_parameter("wheel.gear_ratio", 1.1)
        self.declare_parameter("model_path", "")
        self.declare_parameter("velocity_body_topic", "/aiformula_sensing/vectornav/velocity_body")
        self.declare_parameter("odom_topic", "/aiformula_sensing/gyro_odometry_publisher/odom")
        self.declare_parameter("joy_topic", "/aiformula_control/joy_node/joy")
        self.declare_parameter("state0_button_triangle", 2)
        self.declare_parameter("state1_button_circle", 1)
        self.declare_parameter("state2_button_cross", 0)
        self.declare_parameter("debug_topic", "/aiformula_control/motor_controller/correction_debug")
        self.declare_parameter("rpm_weight_topic", "/aiformula_control/correction_controller_trainer/rpm_weights")
        self.declare_parameter("rpm_correction_enabled", True)
        self.declare_parameter("rpm_fallback_to_body_correction", False)
        self.declare_parameter("history_sample_period_sec", 0.05)
        self.declare_parameter("history_span_tolerance_sec", 0.30)
        self.declare_parameter("rpm_min_history_samples", 4)
        self.declare_parameter("rpm_history_window_sec", 0.0)
        self.declare_parameter("rpm_history_max_sample_gap_sec", 0.50)
        self.declare_parameter("rpm_hold_last_valid_sec", 0.50)
        self.declare_parameter("rpm_hold_command_v_tolerance", 0.50)
        self.declare_parameter("rpm_hold_command_omega_tolerance", 0.20)
        self.declare_parameter("max_command_v", 4.0)
        self.declare_parameter("max_corrected_v", 3.0)
        self.declare_parameter("max_corrected_omega", 1.0)
        self.declare_parameter("stop_deadband", 1.0e-4)
        self.declare_parameter("debug_compare_enabled", True)
        self.declare_parameter("debug_csv_path", "correction_control_debug.csv")

        self.controller_state = self.coerce_controller_state(int(self.get_parameter("controller_state").value))
        publish_timer_loop_duration = self.get_parameter("publish_timer_loop_duration").get_parameter_value().double_value
        self.tread = float(self.get_parameter("wheel.tread").value)
        self.diameter = float(self.get_parameter("wheel.diameter").value)
        self.gear_ratio = float(self.get_parameter("wheel.gear_ratio").value)
        self.velocity_body_topic = str(self.get_parameter("velocity_body_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.joy_topic = str(self.get_parameter("joy_topic").value)
        self.state0_button_triangle = int(self.get_parameter("state0_button_triangle").value)
        self.state1_button_circle = int(self.get_parameter("state1_button_circle").value)
        self.state2_button_cross = int(self.get_parameter("state2_button_cross").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)
        self.rpm_weight_topic = str(self.get_parameter("rpm_weight_topic").value)
        self.rpm_correction_enabled = bool(self.get_parameter("rpm_correction_enabled").value)
        self.rpm_fallback_to_body_correction = bool(self.get_parameter("rpm_fallback_to_body_correction").value)
        self.history_sample_period_sec = float(self.get_parameter("history_sample_period_sec").value)
        self.history_span_tolerance_sec = float(self.get_parameter("history_span_tolerance_sec").value)
        self.rpm_min_history_samples = max(2, int(self.get_parameter("rpm_min_history_samples").value))
        self.rpm_history_window_sec = max(0.0, float(self.get_parameter("rpm_history_window_sec").value))
        self.rpm_history_max_sample_gap_sec = max(0.0, float(self.get_parameter("rpm_history_max_sample_gap_sec").value))
        self.rpm_hold_last_valid_sec = max(0.0, float(self.get_parameter("rpm_hold_last_valid_sec").value))
        self.rpm_hold_command_v_tolerance = max(0.0, float(self.get_parameter("rpm_hold_command_v_tolerance").value))
        self.rpm_hold_command_omega_tolerance = max(
            0.0,
            float(self.get_parameter("rpm_hold_command_omega_tolerance").value),
        )
        self.max_command_v = max(0.0, float(self.get_parameter("max_command_v").value))
        self.max_corrected_v = float(self.get_parameter("max_corrected_v").value)
        self.max_corrected_omega = float(self.get_parameter("max_corrected_omega").value)
        self.stop_deadband = max(0.0, float(self.get_parameter("stop_deadband").value))
        self.debug_compare_enabled = bool(self.get_parameter("debug_compare_enabled").value)
        self.debug_csv_path = self.resolve_debug_csv_path(str(self.get_parameter("debug_csv_path").value))
        self.debug_csv_file = None
        self.debug_csv_writer = None
        self.debug_sample_index = 0

        self.model = None
        (
            self.model,
            self.history_steps,
            self.feature_columns,
            self.command_columns,
            self.hist_mean,
            self.hist_std,
            self.cmd_mean,
            self.cmd_std,
        ) = self.load_model()

        expected_features = ["cmd_v", "cmd_omega", "meas_v", "meas_omega"]
        expected_commands = ["cmd_v", "cmd_omega"]
        if self.feature_columns != expected_features:
            raise RuntimeError(f"Expected affine history features {expected_features}, got {self.feature_columns}")
        if self.command_columns != expected_commands:
            raise RuntimeError(f"Expected affine command columns {expected_commands}, got {self.command_columns}")

        self.rpm_model = None
        self.rpm_config: dict = {}
        self.rpm_history_steps = 20
        self.rpm_max_abs_rpm = 650.0
        self.rpm_v_scale = 3.0
        self.rpm_omega_scale = 1.0
        self.rpm_rpm_scale = 350.0
        self.rpm_loaded_updates = -1
        self.rpm_loaded_samples = 0
        self.rpm_live_weights_loaded = False
        self.rpm_latest_params = np.asarray([1.0, 1.0, 0.0, 0.0], dtype=np.float32)
        self.rpm_latest_model_right = 0.0
        self.rpm_latest_model_left = 0.0
        self.rpm_history_raw_capacity = self.rpm_history_capacity()
        self.rpm_history: deque[list[float]] = deque(maxlen=self.rpm_history_raw_capacity)
        self.rpm_history_times: deque[float] = deque(maxlen=self.rpm_history_raw_capacity)
        self.last_rpm_history_sample_time: float | None = None
        self.last_valid_rpm_model_time: float | None = None
        self.last_valid_rpm_model_right: float | None = None
        self.last_valid_rpm_model_left: float | None = None
        self.last_valid_rpm_model_base_v: float | None = None
        self.last_valid_rpm_model_base_omega: float | None = None

        self.history: deque[list[float]] = deque(maxlen=self.history_steps)
        self.history_times: deque[float] = deque(maxlen=self.history_steps)
        self.last_history_sample_time: float | None = None
        self.latest_meas_v: float | None = None
        self.latest_meas_omega: float | None = None
        self.last_sent_v: float | None = None
        self.last_sent_omega: float | None = None
        self.previous_joy_buttons: list[int] = []

        buffer_size = 10
        self.twist_sub = self.create_subscription(Twist, "sub_speed_command", self.twist_callback, buffer_size)
        self.velocity_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            self.velocity_body_topic,
            self.velocity_body_callback,
            buffer_size,
        )
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, buffer_size)
        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_callback, buffer_size)
        self.rpm_weight_sub = self.create_subscription(
            ByteMultiArray,
            self.rpm_weight_topic,
            self.rpm_weight_callback,
            self.live_weight_qos_profile(),
        )
        self.can_pub = self.create_publisher(Frame, "pub_can", buffer_size)
        self.debug_pub = self.create_publisher(Float64MultiArray, self.debug_topic, buffer_size)
        self.publish_timer = self.create_timer(publish_timer_loop_duration, self.publish_canframe_callback)

        self.frame_msg = Frame()
        self.frame_msg.header.frame_id = "can0"
        self.frame_msg.id = 0x210
        self.frame_msg.dlc = 8
        self.frame_msg.data = self.toCanCmd(0) + self.toCanCmd(0)

        self.add_on_set_parameters_callback(self.on_parameter_update)
        self.open_debug_csv()
        self.log_info("Loaded selectable motor controller.")
        self.log_info("State 0: ideal diff-drive, no empirical tuning.")
        self.log_info("State 1: copied BKUP controller tuning, no correction applied.")
        self.log_info("State 2: live RPM correction when weights are received; ideal RPM while warming up.")
        self.log_info(f"Initial controller_state={self.controller_state}.")
        self.log_info(
            "PS4 state buttons: triangle->0, circle->1, cross/X->2 "
            f"on Joy topic {self.joy_topic}."
        )
        self.log_info(f"Correction debug topic: {self.debug_topic}")
        self.log_info(f"Live RPM weight topic: {self.rpm_weight_topic}")

    def destroy_node(self):
        if self.debug_csv_file is not None:
            self.debug_csv_file.flush()
            self.debug_csv_file.close()
            self.debug_csv_file = None
        return super().destroy_node()

    def on_parameter_update(self, params):
        for param in params:
            if param.name == "controller_state":
                try:
                    next_state = self.coerce_controller_state(int(param.value))
                except Exception as exc:
                    return SetParametersResult(successful=False, reason=str(exc))
                if next_state != self.controller_state:
                    self.controller_state = next_state
                    if next_state != self.STATE_CORRECTED:
                        self.unload_live_rpm_model()
                    self.get_logger().warning(f"controller_state changed to {self.controller_state}")
        return SetParametersResult(successful=True)

    def coerce_controller_state(self, value: int) -> int:
        state = int(value)
        if state not in self.VALID_STATES:
            raise ValueError(f"controller_state must be 0, 1, or 2; got {value}")
        return state

    def unload_live_rpm_model(self) -> None:
        # 关键：离开 mode 2 就清掉实时权重，mode 0/1 不继续用旧模型。
        if self.rpm_model is not None or self.rpm_live_weights_loaded:
            self.get_logger().warning("Unloading live RPM correction model because controller_state left state 2.")
        self.rpm_model = None
        self.rpm_live_weights_loaded = False
        self.rpm_loaded_updates = -1
        self.rpm_loaded_samples = 0
        self.rpm_latest_params = np.asarray([1.0, 1.0, 0.0, 0.0], dtype=np.float32)
        self.rpm_latest_model_right = 0.0
        self.rpm_latest_model_left = 0.0
        self.rpm_history.clear()
        self.rpm_history_times.clear()
        self.last_rpm_history_sample_time = None
        self.clear_last_valid_rpm_model()

    def clear_last_valid_rpm_model(self) -> None:
        self.last_valid_rpm_model_time = None
        self.last_valid_rpm_model_right = None
        self.last_valid_rpm_model_left = None
        self.last_valid_rpm_model_base_v = None
        self.last_valid_rpm_model_base_omega = None

    def resolve_debug_csv_path(self, configured: str) -> Path:
        path = Path(configured or "correction_control_debug.csv").expanduser()
        if not path.is_absolute():
            path = self.model_controller_dir() / path
        path.parent.mkdir(parents=True, exist_ok=True)
        return path

    def model_controller_dir(self) -> Path:
        return Path(get_package_share_directory("motor_controller")) / "model_controller"

    def log_info(self, message: str) -> None:
        self.get_logger().info(str(message))

    @staticmethod
    def live_weight_qos_profile() -> QoSProfile:
        return QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    def open_debug_csv(self) -> None:
        if not self.debug_compare_enabled:
            return
        self.debug_csv_file = self.debug_csv_path.open("w", newline="", encoding="utf-8")
        self.debug_csv_writer = csv.DictWriter(
            self.debug_csv_file,
            fieldnames=["sample", *self.DEBUG_FIELDS],
        )
        self.debug_csv_writer.writeheader()
        self.debug_csv_file.flush()
        self.log_info(f"Writing correction debug CSV: {self.debug_csv_path}")

    def rpm_weight_callback(self, msg: ByteMultiArray) -> None:
        # 关键：只有 mode 2 会加载实时权重，mode 0/1 保持卸载状态。
        if not self.rpm_correction_enabled:
            return
        if self.controller_state != self.STATE_CORRECTED:
            return
        try:
            raw_payload = bytes(
                value[0] if isinstance(value, (bytes, bytearray)) else int(value) & 0xFF
                for value in msg.data
            )
            buffer = io.BytesIO(raw_payload)
            try:
                payload = torch.load(buffer, map_location="cpu", weights_only=False)
            except TypeError:
                buffer.seek(0)
                payload = torch.load(buffer, map_location="cpu")
            self.load_rpm_checkpoint(payload)
        except Exception as exc:
            self.get_logger().error(f"Failed to load live RPM correction weights: {exc}")

    def load_rpm_checkpoint(self, checkpoint: dict) -> None:
        feature_columns = list(checkpoint.get("feature_cols", []))
        command_columns = list(checkpoint.get("command_cols", []))
        if feature_columns != self.RPM_FEATURE_COLUMNS:
            raise RuntimeError(f"Expected RPM features {self.RPM_FEATURE_COLUMNS}, got {feature_columns}")
        if command_columns != self.RPM_COMMAND_COLUMNS:
            raise RuntimeError(f"Expected RPM commands {self.RPM_COMMAND_COLUMNS}, got {command_columns}")
        config = dict(checkpoint.get("config", {}))
        model = RpmAffineModel(
            history_dim=len(feature_columns),
            command_dim=len(command_columns),
            hidden_size=int(config.get("hidden_size", 48)),
            gru_layers=int(config.get("gru_layers", 1)),
            dropout=float(config.get("dropout", 0.0)),
            gain_span=float(config.get("gain_span", 2.0)),
            max_bias_rpm=float(config.get("max_bias_rpm", 260.0)),
        )
        model.load_state_dict(checkpoint["model"])
        model.eval()

        history_steps = int(config.get("history_steps", 20))
        if history_steps != self.rpm_history_steps:
            self.rpm_history_steps = history_steps
            self.rpm_history_raw_capacity = self.rpm_history_capacity()
            self.rpm_history = deque(list(self.rpm_history)[-self.rpm_history_raw_capacity :], maxlen=self.rpm_history_raw_capacity)
            self.rpm_history_times = deque(
                list(self.rpm_history_times)[-self.rpm_history_raw_capacity :],
                maxlen=self.rpm_history_raw_capacity,
            )

        self.rpm_model = model
        self.rpm_config = config
        self.rpm_max_abs_rpm = float(config.get("max_abs_rpm", 650.0))
        self.rpm_v_scale = float(config.get("v_scale", 3.0))
        self.rpm_omega_scale = float(config.get("omega_scale", 1.0))
        self.rpm_rpm_scale = float(config.get("rpm_scale", 350.0))
        self.rpm_loaded_updates = int(checkpoint.get("updates", -1))
        self.rpm_loaded_samples = int(checkpoint.get("sample_rows", 0))
        self.rpm_live_weights_loaded = True
        self.clear_last_valid_rpm_model()
        self.get_logger().warning(
            "Loaded live RPM correction weights "
            f"(samples={self.rpm_loaded_samples}, updates={self.rpm_loaded_updates})."
        )

    def resolve_model_path(self) -> Path:
        configured = str(self.get_parameter("model_path").value or "").strip()
        if configured:
            path = Path(configured).expanduser()
            if path.exists():
                return path
        return self.model_controller_dir() / "correction_control.pt"

    def load_model(self):
        model_path = self.resolve_model_path()
        if not model_path.exists():
            raise FileNotFoundError(f"CorrectionControl weight not found: {model_path}")
        self.install_numpy_pickle_compat()
        try:
            checkpoint = torch.load(model_path, map_location="cpu", weights_only=False)
        except TypeError:
            checkpoint = torch.load(model_path, map_location="cpu")

        config = checkpoint.get("config", {})
        feature_columns = list(checkpoint["feature_cols"])
        command_columns = list(checkpoint["command_cols"])
        history_steps = int(config.get("history_steps", 20))
        model = CorrectionControlModel(
            history_dim=len(feature_columns),
            hidden_size=int(config.get("hidden_size", 32)),
            gru_layers=int(config.get("gru_layers", 1)),
            dropout=float(config.get("dropout", 0.0)),
            gain_span=float(config.get("gain_span", 0.75)),
            max_bias_v=float(config.get("max_bias_v", 0.75)),
            max_bias_omega=float(config.get("max_bias_omega", 0.35)),
        )
        model.load_state_dict(checkpoint["model"])
        model.eval()

        hist_mean = np.asarray(checkpoint["hist_mean"], dtype=np.float32)
        hist_std = np.asarray(checkpoint["hist_std"], dtype=np.float32)
        cmd_mean = np.asarray(checkpoint["cmd_mean"], dtype=np.float32)
        cmd_std = np.asarray(checkpoint["cmd_std"], dtype=np.float32)
        return model, history_steps, feature_columns, command_columns, hist_mean, hist_std, cmd_mean, cmd_std

    @staticmethod
    def install_numpy_pickle_compat() -> None:
        if "numpy._core" in sys.modules:
            return
        sys.modules.setdefault("numpy._core", np.core)
        for module_name in ("multiarray", "numeric", "numerictypes", "umath", "fromnumeric"):
            module = getattr(np.core, module_name, None)
            if module is not None:
                sys.modules.setdefault(f"numpy._core.{module_name}", module)

    def velocity_body_callback(self, msg: TwistWithCovarianceStamped) -> None:
        self.latest_meas_v = float(msg.twist.twist.linear.x)

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_meas_omega = float(msg.twist.twist.angular.z)

    def joy_callback(self, msg: Joy) -> None:
        buttons = [int(value) for value in msg.buttons]
        try:
            if self.button_rising_edge(buttons, self.state0_button_triangle):
                self.set_controller_state_from_joy(self.STATE_IDEAL, "triangle")
            elif self.button_rising_edge(buttons, self.state1_button_circle):
                self.set_controller_state_from_joy(self.STATE_BKUP, "circle")
            elif self.button_rising_edge(buttons, self.state2_button_cross):
                self.set_controller_state_from_joy(self.STATE_CORRECTED, "cross/X")
        finally:
            self.previous_joy_buttons = buttons

    def button_rising_edge(self, buttons: list[int], index: int) -> bool:
        if index < 0 or index >= len(buttons):
            return False
        previous = self.previous_joy_buttons[index] if index < len(self.previous_joy_buttons) else 0
        return bool(buttons[index]) and not bool(previous)

    def set_controller_state_from_joy(self, state: int, button_name: str) -> None:
        state = self.coerce_controller_state(state)
        if state == self.controller_state:
            self.get_logger().info(f"Joy {button_name} pressed; controller_state already {state}.")
            return
        results = self.set_parameters([RclpyParameter("controller_state", RclpyParameter.Type.INTEGER, state)])
        if results and results[0].successful:
            self.get_logger().warning(f"Joy {button_name} selected controller_state={state}.")
        else:
            reason = results[0].reason if results else "no result returned"
            self.get_logger().error(f"Failed to switch controller_state from Joy {button_name}: {reason}")

    @staticmethod
    def clamp_command(value: float, limit: float) -> float:
        if limit <= 0.0:
            return float(value)
        return float(np.clip(value, -limit, limit))

    def twist_callback(self, msg: Twist) -> None:
        raw_v = float(msg.linear.x)
        base_v = (
            raw_v
            if self.controller_state == self.STATE_BKUP
            else self.clamp_command(raw_v, self.max_command_v)
        )
        base_omega = float(msg.angular.z)
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        if self.controller_state == self.STATE_BKUP:
            is_stop_command = base_v == 0.0 and base_omega == 0.0
        else:
            is_stop_command = abs(base_v) <= self.stop_deadband and abs(base_omega) <= self.stop_deadband

        self.append_latest_response_to_history(now_sec)
        history_ready, history_span_sec = self.history_ready()
        rpm_history_ready = False
        if self.controller_state == self.STATE_CORRECTED and self.rpm_correction_enabled and self.rpm_model is not None:
            self.append_latest_response_to_rpm_history(base_v, base_omega, now_sec)
            rpm_history_ready, _ = self.rpm_history_ready()
        if self.controller_state == self.STATE_BKUP:
            model_v = base_v
            model_omega = base_omega
            params = np.asarray([1.0, 1.0, 0.0, 0.0], dtype=np.float32)
            used_model_estimate = False
        else:
            model_v, model_omega, params, used_model_estimate = self.correct_command(
                base_v,
                base_omega,
                history_ready,
            )

        if is_stop_command:
            applied_v = 0.0
            applied_omega = 0.0
            used_model_applied = False
        elif (
            self.controller_state == self.STATE_CORRECTED
            and (self.rpm_model is None or not rpm_history_ready)
            and self.rpm_fallback_to_body_correction
            and used_model_estimate
        ):
            applied_v = model_v
            applied_omega = model_omega
            used_model_applied = True
        else:
            applied_v = base_v
            applied_omega = base_omega
            used_model_applied = False

        rpm_model_applied = False
        rpm_params = np.asarray([1.0, 1.0, 0.0, 0.0], dtype=np.float32)
        rpm_model_right = math.nan
        rpm_model_left = math.nan
        if (
            not is_stop_command
            and self.controller_state == self.STATE_CORRECTED
            and self.rpm_correction_enabled
            and self.rpm_model is not None
            and rpm_history_ready
        ):
            # 关键：mode 2 权重和历史窗口都就绪时，直接输出左右轮 RPM。
            right_rpm, left_rpm, rpm_params = self.correct_rpm(base_v, base_omega)
            rpm_model_right = right_rpm
            rpm_model_left = left_rpm
            rpm_model_applied = True
            used_model_applied = True
            self.remember_valid_rpm_model_output(base_v, base_omega, right_rpm, left_rpm, now_sec)
        elif (
            not is_stop_command
            and self.controller_state == self.STATE_CORRECTED
            and self.rpm_correction_enabled
            and self.rpm_model is not None
            and self.can_hold_last_rpm_model_output(base_v, base_omega, now_sec)
        ):
            # 关键：历史窗口短暂抖动时不立刻掉回弱的 ideal RPM，避免“没反应->突然大转向”。
            right_rpm = float(self.last_valid_rpm_model_right)
            left_rpm = float(self.last_valid_rpm_model_left)
            rpm_model_right = right_rpm
            rpm_model_left = left_rpm
            rpm_model_applied = True
            used_model_applied = True
        else:
            right_rpm, left_rpm = self.cmd_to_can_rpms(applied_v, applied_omega, self.controller_state)
        if is_stop_command:
            right_rpm = 0.0
            left_rpm = 0.0
            rpm_model_right = 0.0
            rpm_model_left = 0.0
            self.clear_last_valid_rpm_model()
        cmd_right = self.toCanCmd(right_rpm)
        cmd_left = self.toCanCmd(left_rpm)
        self.frame_msg.data = cmd_right + cmd_left

        self.last_sent_v = applied_v
        self.last_sent_omega = applied_omega
        debug_row = self.make_debug_row(
            now_sec=now_sec,
            base_v=base_v,
            base_omega=base_omega,
            applied_v=applied_v,
            applied_omega=applied_omega,
            model_v=model_v,
            model_omega=model_omega,
            params=params,
            history_ready=history_ready,
            history_span_sec=history_span_sec,
            used_model_estimate=used_model_estimate or rpm_history_ready,
            used_model_applied=used_model_applied,
            right_rpm=right_rpm,
            left_rpm=left_rpm,
            rpm_params=rpm_params,
            rpm_history_ready=rpm_history_ready,
            rpm_model_applied=rpm_model_applied,
            rpm_model_right=rpm_model_right,
            rpm_model_left=rpm_model_left,
        )
        self.publish_debug(debug_row)
        self.log_debug_compare(debug_row)

    def append_latest_response_to_history(self, now_sec: float) -> bool:
        if self.latest_meas_v is None or self.latest_meas_omega is None:
            return False
        if self.last_sent_v is None or self.last_sent_omega is None:
            return False
        if (
            self.last_history_sample_time is not None
            and now_sec - self.last_history_sample_time < self.history_sample_period_sec
        ):
            return False
        sample = [
            float(self.last_sent_v),
            float(self.last_sent_omega),
            float(self.latest_meas_v),
            float(self.latest_meas_omega),
        ]
        self.history.append(sample)
        self.history_times.append(float(now_sec))
        self.last_history_sample_time = float(now_sec)
        return True

    def append_latest_response_to_rpm_history(self, base_v: float, base_omega: float, now_sec: float) -> bool:
        if self.latest_meas_v is None or self.latest_meas_omega is None:
            return False
        if (
            self.last_rpm_history_sample_time is not None
            and now_sec - self.last_rpm_history_sample_time < self.history_sample_period_sec
        ):
            return False
        ideal_right, ideal_left = self.ideal_cmd_to_can_rpms(base_v, base_omega)
        sample = self.normalized_rpm_feature(
            base_v,
            base_omega,
            self.latest_meas_v,
            self.latest_meas_omega,
            ideal_right,
            ideal_left,
        )
        self.rpm_history.append(sample)
        self.rpm_history_times.append(float(now_sec))
        self.last_rpm_history_sample_time = float(now_sec)
        return True

    def history_span_sec(self) -> float | None:
        if len(self.history_times) < 2:
            return None
        return float(self.history_times[-1] - self.history_times[0])

    def history_ready(self) -> tuple[bool, float | None]:
        if self.latest_meas_v is None or self.latest_meas_omega is None:
            return False, self.history_span_sec()
        if len(self.history) < self.history_steps or len(self.history_times) < self.history_steps:
            return False, self.history_span_sec()
        span = self.history_span_sec()
        expected_span = (self.history_steps - 1) * self.history_sample_period_sec
        if span is None or abs(span - expected_span) > self.history_span_tolerance_sec:
            return False, span
        return True, span

    def rpm_history_ready(self) -> tuple[bool, float | None]:
        if self.rpm_model is None:
            return False, self.rpm_history_span_sec()
        if self.latest_meas_v is None or self.latest_meas_omega is None:
            return False, self.rpm_history_span_sec()
        samples, times = self.recent_rpm_history_arrays()
        if len(times) < min(self.rpm_min_history_samples, self.rpm_history_steps):
            return False, self.rpm_history_span_sec()
        span = float(times[-1] - times[0]) if len(times) >= 2 else 0.0
        min_span = min(
            self.rpm_history_target_window_sec(),
            max(0.15, (min(self.rpm_min_history_samples, self.rpm_history_steps) - 1) * self.history_sample_period_sec),
        )
        return span >= min_span, span

    def rpm_history_span_sec(self) -> float | None:
        if len(self.rpm_history_times) < 2:
            return None
        return float(self.rpm_history_times[-1] - self.rpm_history_times[0])

    def history_sequence(self) -> np.ndarray:
        return np.asarray(list(self.history), dtype=np.float32)

    def rpm_history_sequence(self) -> np.ndarray:
        samples, times = self.recent_rpm_history_arrays()
        if len(samples) == 0:
            return np.zeros((self.rpm_history_steps, len(self.RPM_FEATURE_COLUMNS)), dtype=np.float32)
        if len(samples) == 1:
            return np.repeat(samples, self.rpm_history_steps, axis=0).astype(np.float32)

        target_end = float(times[-1])
        target_start = target_end - self.rpm_history_target_window_sec()
        target_times = np.linspace(target_start, target_end, self.rpm_history_steps, dtype=np.float64)
        unique_times, unique_indices = np.unique(times, return_index=True)
        unique_samples = samples[unique_indices]
        resampled = np.empty((self.rpm_history_steps, unique_samples.shape[1]), dtype=np.float32)
        for column in range(unique_samples.shape[1]):
            resampled[:, column] = np.interp(
                target_times,
                unique_times,
                unique_samples[:, column],
                left=unique_samples[0, column],
                right=unique_samples[-1, column],
            )
        return resampled

    def rpm_history_capacity(self) -> int:
        return max(64, self.rpm_history_steps * 4)

    def rpm_history_target_window_sec(self) -> float:
        if self.rpm_history_window_sec > 0.0:
            return self.rpm_history_window_sec
        return max(self.history_sample_period_sec, (self.rpm_history_steps - 1) * self.history_sample_period_sec)

    def recent_rpm_history_arrays(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.rpm_history or not self.rpm_history_times:
            return (
                np.empty((0, len(self.RPM_FEATURE_COLUMNS)), dtype=np.float32),
                np.empty((0,), dtype=np.float64),
            )
        samples = np.asarray(list(self.rpm_history), dtype=np.float32)
        times = np.asarray(list(self.rpm_history_times), dtype=np.float64)
        if len(times) >= 2 and self.rpm_history_max_sample_gap_sec > 0.0:
            gaps = np.diff(times)
            break_indices = np.flatnonzero(gaps > self.rpm_history_max_sample_gap_sec)
            if len(break_indices) > 0:
                start_index = int(break_indices[-1] + 1)
                samples = samples[start_index:]
                times = times[start_index:]
        if len(times) == 0:
            return samples, times
        window_start = float(times[-1]) - self.rpm_history_target_window_sec()
        keep = times >= window_start
        if np.any(keep):
            samples = samples[keep]
            times = times[keep]
        return samples, times

    def remember_valid_rpm_model_output(
        self,
        base_v: float,
        base_omega: float,
        right_rpm: float,
        left_rpm: float,
        now_sec: float,
    ) -> None:
        self.last_valid_rpm_model_time = float(now_sec)
        self.last_valid_rpm_model_right = float(right_rpm)
        self.last_valid_rpm_model_left = float(left_rpm)
        self.last_valid_rpm_model_base_v = float(base_v)
        self.last_valid_rpm_model_base_omega = float(base_omega)

    def can_hold_last_rpm_model_output(self, base_v: float, base_omega: float, now_sec: float) -> bool:
        if self.rpm_hold_last_valid_sec <= 0.0:
            return False
        if (
            self.last_valid_rpm_model_time is None
            or self.last_valid_rpm_model_right is None
            or self.last_valid_rpm_model_left is None
            or self.last_valid_rpm_model_base_v is None
            or self.last_valid_rpm_model_base_omega is None
        ):
            return False
        if now_sec - self.last_valid_rpm_model_time > self.rpm_hold_last_valid_sec:
            return False
        if abs(base_v - self.last_valid_rpm_model_base_v) > self.rpm_hold_command_v_tolerance:
            return False
        if abs(base_omega - self.last_valid_rpm_model_base_omega) > self.rpm_hold_command_omega_tolerance:
            return False
        return True

    def normalized_rpm_feature(
        self,
        cmd_v: float,
        cmd_omega: float,
        meas_v: float,
        meas_omega: float,
        ideal_right_rpm: float,
        ideal_left_rpm: float,
    ) -> list[float]:
        return [
            float(cmd_v) / self.rpm_v_scale,
            float(cmd_omega) / self.rpm_omega_scale,
            float(meas_v) / self.rpm_v_scale,
            float(meas_omega) / self.rpm_omega_scale,
            float(ideal_right_rpm) / self.rpm_rpm_scale,
            float(ideal_left_rpm) / self.rpm_rpm_scale,
        ]

    def normalized_rpm_command(
        self,
        ideal_right_rpm: float,
        ideal_left_rpm: float,
        cmd_v: float,
        cmd_omega: float,
    ) -> np.ndarray:
        return np.asarray(
            [
                float(ideal_right_rpm) / self.rpm_rpm_scale,
                float(ideal_left_rpm) / self.rpm_rpm_scale,
                float(cmd_v) / self.rpm_v_scale,
                float(cmd_omega) / self.rpm_omega_scale,
            ],
            dtype=np.float32,
        )

    def correct_command(
        self,
        base_v: float,
        base_omega: float,
        history_ready: bool,
    ) -> tuple[float, float, np.ndarray, bool]:
        if not history_ready:
            return base_v, base_omega, np.asarray([1.0, 1.0, 0.0, 0.0], dtype=np.float32), False

        history_raw = self.history_sequence()
        command_raw = np.asarray([base_v, base_omega], dtype=np.float32)
        history_norm = (history_raw - self.hist_mean) / np.clip(self.hist_std, 1.0e-8, None)
        command_norm = (command_raw - self.cmd_mean) / np.clip(self.cmd_std, 1.0e-8, None)

        with torch.no_grad():
            params = self.model(
                torch.as_tensor(history_norm[None, :, :], dtype=torch.float32),
                torch.as_tensor(command_norm[None, :], dtype=torch.float32),
            ).cpu().numpy()[0]

        a_v = max(float(params[0]), 1.0e-4)
        a_omega = max(float(params[1]), 1.0e-4)
        b_v = float(params[2])
        b_omega = float(params[3])
        corrected_v = (base_v - b_v) / a_v
        corrected_omega = (base_omega - b_omega) / a_omega
        corrected_v = float(np.clip(corrected_v, -self.max_corrected_v, self.max_corrected_v))
        corrected_omega = float(np.clip(corrected_omega, -self.max_corrected_omega, self.max_corrected_omega))
        return corrected_v, corrected_omega, params, True

    def correct_rpm(self, base_v: float, base_omega: float) -> tuple[float, float, np.ndarray]:
        if self.rpm_model is None:
            right, left = self.ideal_cmd_to_can_rpms(base_v, base_omega)
            return right, left, np.asarray([1.0, 1.0, 0.0, 0.0], dtype=np.float32)

        ideal_right, ideal_left = self.ideal_cmd_to_can_rpms(base_v, base_omega)
        history_norm = self.rpm_history_sequence()
        command_norm = self.normalized_rpm_command(ideal_right, ideal_left, base_v, base_omega)
        command_raw = np.asarray([ideal_right, ideal_left, base_v, base_omega], dtype=np.float32)

        with torch.no_grad():
            params = self.rpm_model(
                torch.as_tensor(history_norm[None, :, :], dtype=torch.float32),
                torch.as_tensor(command_norm[None, :], dtype=torch.float32),
            ).cpu().numpy()[0]

        a_right = max(float(params[0]), 1.0e-4)
        a_left = max(float(params[1]), 1.0e-4)
        b_right = float(params[2])
        b_left = float(params[3])
        right_rpm = a_right * float(command_raw[0]) + b_right
        left_rpm = a_left * float(command_raw[1]) + b_left
        right_rpm = float(np.clip(right_rpm, -self.rpm_max_abs_rpm, self.rpm_max_abs_rpm))
        left_rpm = float(np.clip(left_rpm, -self.rpm_max_abs_rpm, self.rpm_max_abs_rpm))

        self.rpm_latest_params = np.asarray([a_right, a_left, b_right, b_left], dtype=np.float32)
        self.rpm_latest_model_right = right_rpm
        self.rpm_latest_model_left = left_rpm
        return right_rpm, left_rpm, self.rpm_latest_params

    def cmd_to_can_rpms(self, linear_velocity: float, angular_velocity: float, state: int) -> tuple[float, float]:
        if state == self.STATE_BKUP:
            return self.bkup_cmd_to_can_rpms(linear_velocity, angular_velocity)
        if abs(linear_velocity) <= self.stop_deadband and abs(angular_velocity) <= self.stop_deadband:
            return 0.0, 0.0
        if state == self.STATE_IDEAL:
            return self.ideal_cmd_to_can_rpms(linear_velocity, angular_velocity)
        if state == self.STATE_CORRECTED:
            return self.corrected_cmd_to_can_rpms(linear_velocity, angular_velocity)
        raise ValueError(f"Unsupported controller state: {state}")

    def ideal_cmd_to_can_rpms(self, linear_velocity: float, angular_velocity: float) -> tuple[float, float]:
        right_wheel, left_wheel = self.cmd_to_wheel_rad_per_sec(linear_velocity, angular_velocity)
        scale = 60.0 / (2.0 * math.pi)
        return right_wheel * scale * self.gear_ratio, left_wheel * scale * self.gear_ratio

    def corrected_cmd_to_can_rpms(self, linear_velocity: float, angular_velocity: float) -> tuple[float, float]:
        return self.ideal_cmd_to_can_rpms(linear_velocity, angular_velocity)

    def bkup_cmd_to_can_rpms(self, linear_velocity: float, angular_velocity: float) -> tuple[float, float]:
        wheel_angular_velocities = np.array(
            [
                (linear_velocity / (self.diameter * 0.5))
                + (self.tread / self.diameter) * angular_velocity,
                (linear_velocity / (self.diameter * 0.5))
                - (self.tread / self.diameter) * angular_velocity,
            ]
        )
        a_left = 0.834
        w0_left = 2.76
        a_right = 0.844
        w0_right = 2.81
        wL_cmd = np.sign(wheel_angular_velocities[0]) * (
            abs(wheel_angular_velocities[0]) / a_left + w0_left
        )
        wR_cmd = np.sign(wheel_angular_velocities[1]) * (
            abs(wheel_angular_velocities[1]) / a_right + w0_right
        )
        minute_to_second = 60.0
        rpm_left = (wL_cmd * (minute_to_second / (2.0 * np.pi))) * self.gear_ratio
        rpm_right = (wR_cmd * (minute_to_second / (2.0 * np.pi))) * self.gear_ratio
        return float(rpm_left), float(rpm_right)

    def cmd_to_wheel_rad_per_sec(self, linear_velocity: float, angular_velocity: float) -> tuple[float, float]:
        right = (linear_velocity / (self.diameter * 0.5)) + (self.tread / self.diameter) * angular_velocity
        left = (linear_velocity / (self.diameter * 0.5)) - (self.tread / self.diameter) * angular_velocity
        return float(right), float(left)

    def make_debug_row(
        self,
        *,
        now_sec: float,
        base_v: float,
        base_omega: float,
        applied_v: float,
        applied_omega: float,
        model_v: float,
        model_omega: float,
        params: np.ndarray,
        history_ready: bool,
        history_span_sec: float | None,
        used_model_estimate: bool,
        used_model_applied: bool,
        right_rpm: float,
        left_rpm: float,
        rpm_params: np.ndarray,
        rpm_history_ready: bool,
        rpm_model_applied: bool,
        rpm_model_right: float,
        rpm_model_left: float,
    ) -> dict:
        can_right_rpm = int.from_bytes(bytes(self.frame_msg.data[0:4]), "little", signed=True)
        can_left_rpm = int.from_bytes(bytes(self.frame_msg.data[4:8]), "little", signed=True)
        return {
            "node_time_sec": now_sec,
            "controller_state": float(self.controller_state),
            "base_v": base_v,
            "base_omega": base_omega,
            "applied_v": applied_v,
            "applied_omega": applied_omega,
            "model_corrected_v": model_v,
            "model_corrected_omega": model_omega,
            "delta_v": model_v - base_v,
            "delta_omega": model_omega - base_omega,
            "a_v": float(params[0]),
            "a_omega": float(params[1]),
            "b_v": float(params[2]),
            "b_omega": float(params[3]),
            "used_model_estimate": float(bool(used_model_estimate)),
            "used_model_applied": float(bool(used_model_applied)),
            "history_len": float(len(self.history)),
            "history_ready": float(bool(history_ready)),
            "history_span_sec": float("nan") if history_span_sec is None else float(history_span_sec),
            "latest_meas_v": float("nan") if self.latest_meas_v is None else float(self.latest_meas_v),
            "latest_meas_omega": float("nan") if self.latest_meas_omega is None else float(self.latest_meas_omega),
            "right_rpm": float(right_rpm),
            "left_rpm": float(left_rpm),
            "can_right_rpm": float(can_right_rpm),
            "can_left_rpm": float(can_left_rpm),
            "rpm_weights_loaded": float(bool(self.rpm_live_weights_loaded)),
            "rpm_correction_ready": float(bool(rpm_history_ready and self.rpm_model is not None)),
            "rpm_model_applied": float(bool(rpm_model_applied)),
            "rpm_model_right_rpm": float(rpm_model_right),
            "rpm_model_left_rpm": float(rpm_model_left),
            "rpm_a_right": float(rpm_params[0]),
            "rpm_a_left": float(rpm_params[1]),
            "rpm_b_right": float(rpm_params[2]),
            "rpm_b_left": float(rpm_params[3]),
            "rpm_history_len": float(len(self.rpm_history)),
            "rpm_history_ready": float(bool(rpm_history_ready)),
        }

    def publish_debug(self, debug_row: dict) -> None:
        msg = Float64MultiArray()
        msg.data = [float(debug_row[field]) for field in self.DEBUG_FIELDS]
        self.debug_pub.publish(msg)

    def log_debug_compare(self, debug_row: dict) -> None:
        if self.debug_csv_writer is None or self.debug_csv_file is None:
            return
        row = {"sample": self.debug_sample_index}
        row.update(debug_row)
        self.debug_csv_writer.writerow(row)
        self.debug_sample_index += 1
        self.debug_csv_file.flush()

    def publish_canframe_callback(self):
        self.can_pub.publish(self.frame_msg)

    @staticmethod
    def toCanCmd(rpm: float) -> List[int]:
        rounded = int(round(rpm))
        return list(rounded.to_bytes(4, "little", signed=True))


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
