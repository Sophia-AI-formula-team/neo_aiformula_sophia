from __future__ import annotations

import csv
import io
import json
import math
import random
import shutil
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch import nn


FEATURE_COLS = ["cmd_v", "cmd_omega", "meas_v", "meas_omega", "ideal_right_rpm", "ideal_left_rpm"]
COMMAND_COLS = ["ideal_right_rpm", "ideal_left_rpm", "cmd_v", "cmd_omega"]
TARGET_COLS = ["target_right_rpm", "target_left_rpm"]


@dataclass(frozen=True)
class OnlineRpmConfig:
    history_steps: int = 20
    learning_rate: float = 1.0e-3
    weight_decay: float = 1.0e-5
    batch_size: int = 64
    replay_capacity: int = 5000
    min_replay: int = 32
    train_every_samples: int = 1
    hidden_size: int = 48
    gru_layers: int = 1
    dropout: float = 0.0
    gain_span: float = 2.0
    max_bias_rpm: float = 260.0
    split_loss_weight: float = 10.0
    omega_error_loss_weight: float = 5.0
    lambda_gain: float = 1.0e-4
    lambda_bias: float = 1.0e-5
    adaptation_gain: float = 0.35
    omega_adaptation_gain: float = 4.5
    max_delta_rpm: float = 360.0
    max_abs_rpm: float = 650.0
    max_abs_v_error: float = 3.0
    max_abs_omega_error: float = 1.5
    max_history_gap_sec: float = 0.20
    wheel_tread: float = 0.60
    wheel_diameter: float = 0.254
    wheel_gear_ratio: float = 1.1
    stop_deadband: float = 1.0e-4
    v_scale: float = 3.0
    omega_scale: float = 1.0
    rpm_scale: float = 350.0
    divergence_loss_threshold: float = 250000.0
    checkpoint_eval_window: int = 400
    checkpoint_min_samples: int = 120
    checkpoint_min_updates: int = 20
    checkpoint_min_improvement_rpm: float = 1.0
    checkpoint_min_relative_improvement: float = 0.03
    checkpoint_max_split_rmse: float = 40.0
    checkpoint_max_wheel_rmse: float = 90.0
    checkpoint_moving_only: bool = True
    checkpoint_min_motion_v: float = 0.05
    checkpoint_min_motion_omega: float = 0.03
    checkpoint_min_turn_samples: int = 60
    checkpoint_min_omega_error: float = 0.15
    checkpoint_min_turn_split_boost_rpm: float = 45.0
    checkpoint_min_current_score_gain: float = 15.0
    checkpoint_min_current_relative_gain: float = 0.20
    checkpoint_min_turn_split_boost_improvement_rpm: float = 20.0
    checkpoint_max_turn_actual_over_cmd_p95: float = 1.35
    seed: int = 23


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
        last = self.head[-1]
        if isinstance(last, nn.Linear):
            nn.init.zeros_(last.weight)
            nn.init.zeros_(last.bias)

    def forward(self, history_norm: torch.Tensor, command_norm: torch.Tensor) -> torch.Tensor:
        _, hidden = self.gru(history_norm)
        context = hidden[-1]
        raw = self.head(torch.cat([context, command_norm], dim=-1))
        a_right = 1.0 + self.gain_span * torch.tanh(raw[:, 0])
        a_left = 1.0 + self.gain_span * torch.tanh(raw[:, 1])
        b_right = self.max_bias_rpm * torch.tanh(raw[:, 2])
        b_left = self.max_bias_rpm * torch.tanh(raw[:, 3])
        return torch.stack([a_right, a_left, b_right, b_left], dim=-1)


def predict_rpm(params: torch.Tensor, command_raw: torch.Tensor) -> torch.Tensor:
    right = params[:, 0] * command_raw[:, 0] + params[:, 2]
    left = params[:, 1] * command_raw[:, 1] + params[:, 3]
    return torch.stack([right, left], dim=-1)


def rpm_loss_fn(
    params: torch.Tensor,
    pred: torch.Tensor,
    target: torch.Tensor,
    config: OnlineRpmConfig,
    sample_weight: torch.Tensor | None = None,
):
    if sample_weight is None:
        sample_weight = torch.ones(pred.shape[0], dtype=pred.dtype, device=pred.device)
    sample_weight = torch.clamp(sample_weight, min=1.0e-6)
    wheel_sq = (pred - target) ** 2
    wheel_loss = torch.sum(wheel_sq * sample_weight[:, None]) / torch.sum(sample_weight[:, None].expand_as(wheel_sq))
    split_sq = ((pred[:, 0] - pred[:, 1]) - (target[:, 0] - target[:, 1])) ** 2
    split_loss = torch.sum(split_sq * sample_weight) / torch.sum(sample_weight)
    gain_loss = torch.mean((params[:, 0] - 1.0) ** 2 + (params[:, 1] - 1.0) ** 2)
    bias_loss = torch.mean(params[:, 2] ** 2 + params[:, 3] ** 2)
    total = wheel_loss + config.split_loss_weight * split_loss + config.lambda_gain * gain_loss + config.lambda_bias * bias_loss
    return total, wheel_loss, split_loss, gain_loss, bias_loss


def finite(*values: float) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def clamp(value: float, low: float, high: float) -> float:
    return float(max(low, min(high, value)))


class OnlineRpmTrainer:
    def __init__(self, output_dir: Path, config: OnlineRpmConfig | None = None, logger: Any | None = None) -> None:
        self.output_dir = output_dir
        self.config = config or OnlineRpmConfig()
        self.logger = logger
        self.weights_dir = self.output_dir / "weights"
        self.checkpoints_dir = self.weights_dir / "checkpoints"
        self.plots_dir = self.output_dir / "plots"
        self.artifacts_dir = self.output_dir / "artifacts"
        for directory in [self.output_dir, self.weights_dir, self.checkpoints_dir, self.plots_dir, self.artifacts_dir]:
            directory.mkdir(parents=True, exist_ok=True)

        random.seed(self.config.seed)
        np.random.seed(self.config.seed)
        torch.manual_seed(self.config.seed)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = RpmAffineModel(
            history_dim=len(FEATURE_COLS),
            command_dim=len(COMMAND_COLS),
            hidden_size=self.config.hidden_size,
            gru_layers=self.config.gru_layers,
            dropout=self.config.dropout,
            gain_span=self.config.gain_span,
            max_bias_rpm=self.config.max_bias_rpm,
        ).to(self.device)
        self.optimizer = torch.optim.AdamW(
            self.model.parameters(),
            lr=self.config.learning_rate,
            weight_decay=self.config.weight_decay,
        )
        self.history: deque[tuple[float, np.ndarray]] = deque(maxlen=self.config.history_steps)
        self.replay: deque[dict[str, np.ndarray]] = deque(maxlen=self.config.replay_capacity)
        self.sample_rows: list[dict[str, Any]] = []
        self.train_rows: list[dict[str, Any]] = []
        self.divergence_rows: list[dict[str, Any]] = []
        self.checkpoint_rows: list[dict[str, Any]] = []
        self.update_index = 0
        self.last_sample_time: float | None = None
        self.training_halted = False
        self.best_checkpoint_score: float | None = None
        self.best_checkpoint_turn_split_boost_median: float | None = None
        self.best_checkpoint_update = -1
        self.accepted_checkpoint_payload: dict[str, Any] | None = None
        self.accepted_checkpoint_update = -1
        self.accepted_checkpoint_path: Path | None = None
        self.last_checkpoint_eval_update = -1

    def log_warning(self, message: str) -> None:
        if self.logger is not None:
            self.logger.warning(message)

    def ideal_can_rpm(self, linear_velocity: float, angular_velocity: float) -> tuple[float, float]:
        radius = self.config.wheel_diameter * 0.5
        right = (linear_velocity / radius) + (self.config.wheel_tread / self.config.wheel_diameter) * angular_velocity
        left = (linear_velocity / radius) - (self.config.wheel_tread / self.config.wheel_diameter) * angular_velocity
        scale = 60.0 / (2.0 * math.pi)
        return right * scale * self.config.wheel_gear_ratio, left * scale * self.config.wheel_gear_ratio

    def normalized_feature(
        self,
        cmd_v: float,
        cmd_omega: float,
        meas_v: float,
        meas_omega: float,
        ideal_right_rpm: float,
        ideal_left_rpm: float,
    ) -> np.ndarray:
        return np.asarray(
            [
                cmd_v / self.config.v_scale,
                cmd_omega / self.config.omega_scale,
                meas_v / self.config.v_scale,
                meas_omega / self.config.omega_scale,
                ideal_right_rpm / self.config.rpm_scale,
                ideal_left_rpm / self.config.rpm_scale,
            ],
            dtype=np.float32,
        )

    def normalized_command(
        self,
        ideal_right_rpm: float,
        ideal_left_rpm: float,
        cmd_v: float,
        cmd_omega: float,
    ) -> np.ndarray:
        return np.asarray(
            [
                ideal_right_rpm / self.config.rpm_scale,
                ideal_left_rpm / self.config.rpm_scale,
                cmd_v / self.config.v_scale,
                cmd_omega / self.config.omega_scale,
            ],
            dtype=np.float32,
        )

    def adaptive_target_rpm(
        self,
        cmd_v: float,
        cmd_omega: float,
        meas_v: float,
        meas_omega: float,
        current_right_rpm: float,
        current_left_rpm: float,
    ) -> tuple[float, float, float, float, float, float]:
        if abs(cmd_v) <= self.config.stop_deadband and abs(cmd_omega) <= self.config.stop_deadband:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        # 关键：把车身速度误差换成左右轮 RPM 目标，让模型学该补多少轮速。
        error_v = clamp(cmd_v - meas_v, -self.config.max_abs_v_error, self.config.max_abs_v_error)
        error_omega = clamp(cmd_omega - meas_omega, -self.config.max_abs_omega_error, self.config.max_abs_omega_error)
        delta_v_right, delta_v_left = self.ideal_can_rpm(error_v, 0.0)
        delta_omega_right, delta_omega_left = self.ideal_can_rpm(0.0, error_omega)
        delta_right = clamp(
            delta_v_right * self.config.adaptation_gain + delta_omega_right * self.config.omega_adaptation_gain,
            -self.config.max_delta_rpm,
            self.config.max_delta_rpm,
        )
        delta_left = clamp(
            delta_v_left * self.config.adaptation_gain + delta_omega_left * self.config.omega_adaptation_gain,
            -self.config.max_delta_rpm,
            self.config.max_delta_rpm,
        )
        target_right = clamp(current_right_rpm + delta_right, -self.config.max_abs_rpm, self.config.max_abs_rpm)
        target_left = clamp(current_left_rpm + delta_left, -self.config.max_abs_rpm, self.config.max_abs_rpm)
        return target_right, target_left, error_v, error_omega, delta_right, delta_left

    def add_sample(
        self,
        *,
        timestamp: float,
        segment: str,
        state: int | None,
        cmd_v: float,
        cmd_omega: float,
        meas_v: float,
        meas_omega: float,
        current_right_rpm: float,
        current_left_rpm: float,
    ) -> dict[str, Any] | None:
        if not finite(timestamp, cmd_v, cmd_omega, meas_v, meas_omega, current_right_rpm, current_left_rpm):
            return None

        if self.last_sample_time is not None and timestamp - self.last_sample_time > self.config.max_history_gap_sec:
            self.history.clear()
        self.last_sample_time = float(timestamp)

        ideal_right, ideal_left = self.ideal_can_rpm(cmd_v, cmd_omega)
        feature = self.normalized_feature(cmd_v, cmd_omega, meas_v, meas_omega, ideal_right, ideal_left)
        self.history.append((float(timestamp), feature))
        # 关键：历史窗口没满之前不训练，避免用短历史乱更新。
        if len(self.history) < self.config.history_steps:
            return None

        target_right, target_left, error_v, error_omega, delta_right, delta_left = self.adaptive_target_rpm(
            cmd_v,
            cmd_omega,
            meas_v,
            meas_omega,
            current_right_rpm,
            current_left_rpm,
        )
        history_norm = np.stack([row[1] for row in self.history], axis=0).astype(np.float32)
        command_norm = self.normalized_command(ideal_right, ideal_left, cmd_v, cmd_omega)
        command_raw = np.asarray([ideal_right, ideal_left, cmd_v, cmd_omega], dtype=np.float32)
        target = np.asarray([target_right, target_left], dtype=np.float32)
        omega_ratio = min(1.0, abs(error_omega) / max(self.config.max_abs_omega_error, 1.0e-6))
        cmd_omega_ratio = min(1.0, abs(cmd_omega) / max(self.config.max_abs_omega_error, 1.0e-6))
        turn_weight = 1.0 + self.config.omega_error_loss_weight * max(omega_ratio, cmd_omega_ratio)

        sample = {
            "history_norm": history_norm,
            "command_norm": command_norm,
            "command_raw": command_raw,
            "target": target,
            "current": np.asarray([current_right_rpm, current_left_rpm], dtype=np.float32),
            "cmd_omega": float(cmd_omega),
            "meas_omega": float(meas_omega),
            "error_omega": float(error_omega),
            "turn_weight": float(turn_weight),
        }
        self.replay.append(sample)
        loss_row = self.train_step() if self.should_train() else None
        pred_right, pred_left, params = self.predict_one(history_norm, command_norm, command_raw)

        row = {
            "timestamp": float(timestamp),
            "segment": segment,
            "state": "" if state is None else int(state),
            "cmd_v": float(cmd_v),
            "cmd_omega": float(cmd_omega),
            "meas_v": float(meas_v),
            "meas_omega": float(meas_omega),
            "error_v": float(error_v),
            "error_omega": float(error_omega),
            "ideal_right_rpm": float(ideal_right),
            "ideal_left_rpm": float(ideal_left),
            "current_right_rpm": float(current_right_rpm),
            "current_left_rpm": float(current_left_rpm),
            "target_right_rpm": float(target_right),
            "target_left_rpm": float(target_left),
            "target_delta_right_rpm": float(delta_right),
            "target_delta_left_rpm": float(delta_left),
            "target_delta_split_rpm": float(delta_right - delta_left),
            "turn_weight": float(turn_weight),
            "pred_right_rpm": float(pred_right),
            "pred_left_rpm": float(pred_left),
            "a_right": float(params[0]),
            "a_left": float(params[1]),
            "b_right": float(params[2]),
            "b_left": float(params[3]),
            "train_loss": "" if loss_row is None else float(loss_row["loss"]),
        }
        self.sample_rows.append(row)
        return row

    def should_train(self) -> bool:
        if self.training_halted:
            return False
        if len(self.replay) < self.config.min_replay:
            return False
        return len(self.replay) % max(1, self.config.train_every_samples) == 0

    def train_step(self) -> dict[str, Any]:
        batch_count = min(self.config.batch_size, len(self.replay))
        batch = random.sample(list(self.replay), batch_count)
        history = torch.as_tensor(np.stack([item["history_norm"] for item in batch]), dtype=torch.float32, device=self.device)
        command_norm = torch.as_tensor(
            np.stack([item["command_norm"] for item in batch]),
            dtype=torch.float32,
            device=self.device,
        )
        command_raw = torch.as_tensor(
            np.stack([item["command_raw"] for item in batch]),
            dtype=torch.float32,
            device=self.device,
        )
        target = torch.as_tensor(np.stack([item["target"] for item in batch]), dtype=torch.float32, device=self.device)
        sample_weight = torch.as_tensor(
            np.asarray([float(item.get("turn_weight", 1.0)) for item in batch], dtype=np.float32),
            dtype=torch.float32,
            device=self.device,
        )

        self.model.train()
        self.optimizer.zero_grad(set_to_none=True)
        params = self.model(history, command_norm)
        pred = predict_rpm(params, command_raw)
        total, wheel_loss, split_loss, gain_loss, bias_loss = rpm_loss_fn(
            params,
            pred,
            target,
            self.config,
            sample_weight,
        )
        if self.is_divergent_loss(total):
            return self.record_divergence(
                reason="loss_threshold_or_nonfinite",
                loss=float(total.detach().cpu()) if torch.isfinite(total).item() else math.nan,
                wheel_loss=float(wheel_loss.detach().cpu()) if torch.isfinite(wheel_loss).item() else math.nan,
                split_loss=float(split_loss.detach().cpu()) if torch.isfinite(split_loss).item() else math.nan,
                gain_loss=float(gain_loss.detach().cpu()) if torch.isfinite(gain_loss).item() else math.nan,
                bias_loss=float(bias_loss.detach().cpu()) if torch.isfinite(bias_loss).item() else math.nan,
            )
        total.backward()
        grad_norm = torch.nn.utils.clip_grad_norm_(self.model.parameters(), 5.0)
        if not torch.isfinite(grad_norm).item():
            self.optimizer.zero_grad(set_to_none=True)
            return self.record_divergence(
                reason="gradient_nonfinite",
                loss=float(total.detach().cpu()),
                wheel_loss=float(wheel_loss.detach().cpu()),
                split_loss=float(split_loss.detach().cpu()),
                gain_loss=float(gain_loss.detach().cpu()),
                bias_loss=float(bias_loss.detach().cpu()),
            )
        self.optimizer.step()
        self.update_index += 1

        row = {
            "update": self.update_index,
            "sample_count": len(self.sample_rows),
            "replay_size": len(self.replay),
            "loss": float(total.item()),
            "wheel_loss": float(wheel_loss.item()),
            "split_loss": float(split_loss.item()),
            "gain_loss": float(gain_loss.item()),
            "bias_loss": float(bias_loss.item()),
            "skipped_step": 0,
            "divergence_reason": "",
        }
        self.train_rows.append(row)
        return row

    def is_divergent_loss(self, loss: torch.Tensor) -> bool:
        if not torch.isfinite(loss).item():
            return True
        return float(loss.detach().cpu()) > self.config.divergence_loss_threshold

    def record_divergence(
        self,
        *,
        reason: str,
        loss: float,
        wheel_loss: float,
        split_loss: float,
        gain_loss: float,
        bias_loss: float,
    ) -> dict[str, Any]:
        self.training_halted = True
        row = {
            "update": self.update_index,
            "sample_count": len(self.sample_rows),
            "replay_size": len(self.replay),
            "loss": loss,
            "wheel_loss": wheel_loss,
            "split_loss": split_loss,
            "gain_loss": gain_loss,
            "bias_loss": bias_loss,
            "skipped_step": 1,
            "divergence_reason": reason,
        }
        self.train_rows.append(row)
        event = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "update": self.update_index,
            "sample_count": len(self.sample_rows),
            "replay_size": len(self.replay),
            "reason": reason,
            "loss": loss,
            "threshold": self.config.divergence_loss_threshold,
        }
        self.divergence_rows.append(event)
        self.log_warning(f"Online RPM training halted: {reason}, loss={loss}")
        return row

    @torch.no_grad()
    def predict_one(
        self,
        history_norm: np.ndarray,
        command_norm: np.ndarray,
        command_raw: np.ndarray,
    ) -> tuple[float, float, np.ndarray]:
        self.model.eval()
        history = torch.as_tensor(history_norm[None, :, :], dtype=torch.float32, device=self.device)
        command_norm_t = torch.as_tensor(command_norm[None, :], dtype=torch.float32, device=self.device)
        command_raw_t = torch.as_tensor(command_raw[None, :], dtype=torch.float32, device=self.device)
        params = self.model(history, command_norm_t)
        pred = predict_rpm(params, command_raw_t)
        return float(pred[0, 0].cpu()), float(pred[0, 1].cpu()), params[0].cpu().numpy()

    @staticmethod
    def rmse(values: np.ndarray) -> float:
        values = values[np.isfinite(values)]
        if values.size == 0:
            return math.nan
        return float(np.sqrt(np.mean(values**2)))

    def checkpoint_candidate_metrics(self) -> tuple[dict[str, Any], dict[str, np.ndarray] | None]:
        if self.update_index < self.config.checkpoint_min_updates:
            return {"reliable": False, "reason": "insufficient_updates"}, None

        samples = list(self.replay)[-max(1, self.config.checkpoint_eval_window) :]
        if self.config.checkpoint_moving_only:
            samples = [
                item
                for item in samples
                if abs(float(item["command_raw"][2])) > self.config.checkpoint_min_motion_v
                or abs(float(item["command_raw"][3])) > self.config.checkpoint_min_motion_omega
            ]
        if len(samples) < self.config.checkpoint_min_samples:
            return {
                "reliable": False,
                "reason": "insufficient_reliable_samples",
                "sample_count": len(samples),
            }, None

        history = torch.as_tensor(
            np.stack([item["history_norm"] for item in samples]),
            dtype=torch.float32,
            device=self.device,
        )
        command_norm = torch.as_tensor(
            np.stack([item["command_norm"] for item in samples]),
            dtype=torch.float32,
            device=self.device,
        )
        command_raw = torch.as_tensor(
            np.stack([item["command_raw"] for item in samples]),
            dtype=torch.float32,
            device=self.device,
        )
        target = np.stack([item["target"] for item in samples]).astype(np.float64)
        current = np.stack([item["current"] for item in samples]).astype(np.float64)
        cmd_omega = np.asarray([float(item.get("cmd_omega", item["command_raw"][3])) for item in samples], dtype=np.float64)
        meas_omega = np.asarray([float(item.get("meas_omega", math.nan)) for item in samples], dtype=np.float64)

        self.model.eval()
        with torch.no_grad():
            params = self.model(history, command_norm)
            pred = predict_rpm(params, command_raw).cpu().numpy().astype(np.float64)
        pred = np.clip(pred, -self.config.max_abs_rpm, self.config.max_abs_rpm)

        model_err = pred - target
        current_err = current - target
        model_split_err = (pred[:, 0] - pred[:, 1]) - (target[:, 0] - target[:, 1])
        current_split_err = (current[:, 0] - current[:, 1]) - (target[:, 0] - target[:, 1])
        correction_split = (pred[:, 0] - pred[:, 1]) - (current[:, 0] - current[:, 1])
        omega_error = cmd_omega - meas_omega
        turn_mask = (
            np.isfinite(cmd_omega)
            & np.isfinite(meas_omega)
            & (np.abs(cmd_omega) >= self.config.checkpoint_min_motion_omega)
            & (np.abs(omega_error) >= self.config.checkpoint_min_omega_error)
        )
        turn_sample_count = int(np.count_nonzero(turn_mask))
        turn_split_boost_mean = math.nan
        turn_split_boost_median = math.nan
        turn_split_boost_p10 = math.nan
        turn_abs_omega_error_mean = math.nan
        turn_actual_over_cmd_p95 = math.nan
        if turn_sample_count > 0:
            turn_direction = np.sign(omega_error[turn_mask])
            turn_direction[turn_direction == 0.0] = np.sign(cmd_omega[turn_mask][turn_direction == 0.0])
            aligned_boost = correction_split[turn_mask] * turn_direction
            turn_split_boost_mean = float(np.mean(aligned_boost))
            turn_split_boost_median = float(np.median(aligned_boost))
            turn_split_boost_p10 = float(np.percentile(aligned_boost, 10))
            turn_abs_omega_error_mean = float(np.mean(np.abs(omega_error[turn_mask])))
            turn_actual_over_cmd = np.abs(meas_omega[turn_mask]) / np.maximum(np.abs(cmd_omega[turn_mask]), 1.0e-6)
            turn_actual_over_cmd_p95 = float(np.percentile(turn_actual_over_cmd, 95))

        model_right_rmse = self.rmse(model_err[:, 0])
        model_left_rmse = self.rmse(model_err[:, 1])
        model_split_rmse = self.rmse(model_split_err)
        current_right_rmse = self.rmse(current_err[:, 0])
        current_left_rmse = self.rmse(current_err[:, 1])
        current_split_rmse = self.rmse(current_split_err)
        model_wheel_rmse = 0.5 * (model_right_rmse + model_left_rmse)
        current_wheel_rmse = 0.5 * (current_right_rmse + current_left_rmse)
        score = model_split_rmse + 0.10 * model_wheel_rmse
        current_score = current_split_rmse + 0.10 * current_wheel_rmse
        current_window_score_gain = current_score - score
        current_window_relative_gain = current_window_score_gain / max(current_score, 1.0e-6)
        if self.best_checkpoint_turn_split_boost_median is None or not math.isfinite(
            float(self.best_checkpoint_turn_split_boost_median)
        ):
            turn_split_boost_improvement = math.inf
        elif math.isfinite(float(turn_split_boost_median)):
            turn_split_boost_improvement = turn_split_boost_median - self.best_checkpoint_turn_split_boost_median
        else:
            turn_split_boost_improvement = math.nan

        metrics = {
            "sample_count": len(samples),
            "turn_sample_count": turn_sample_count,
            "model_score": score,
            "current_score": current_score,
            "current_window_score_gain": current_window_score_gain,
            "current_window_relative_gain": current_window_relative_gain,
            "model_right_rmse": model_right_rmse,
            "model_left_rmse": model_left_rmse,
            "model_wheel_rmse": model_wheel_rmse,
            "model_split_rmse": model_split_rmse,
            "current_right_rmse": current_right_rmse,
            "current_left_rmse": current_left_rmse,
            "current_wheel_rmse": current_wheel_rmse,
            "current_split_rmse": current_split_rmse,
            "correction_split_p95": float(np.percentile(np.abs(correction_split), 95)),
            "turn_split_boost_mean": turn_split_boost_mean,
            "turn_split_boost_median": turn_split_boost_median,
            "turn_split_boost_p10": turn_split_boost_p10,
            "turn_split_boost_improvement": turn_split_boost_improvement,
            "turn_abs_omega_error_mean": turn_abs_omega_error_mean,
            "turn_actual_over_cmd_p95": turn_actual_over_cmd_p95,
        }
        base_metric_keys = [
            "sample_count",
            "model_score",
            "current_score",
            "current_window_score_gain",
            "current_window_relative_gain",
            "model_right_rmse",
            "model_left_rmse",
            "model_wheel_rmse",
            "model_split_rmse",
            "current_right_rmse",
            "current_left_rmse",
            "current_wheel_rmse",
            "current_split_rmse",
            "correction_split_p95",
        ]
        finite_metrics = all(math.isfinite(float(metrics[key])) for key in base_metric_keys)
        turn_metrics_finite = turn_sample_count == 0 or all(
            math.isfinite(float(metrics[key]))
            for key in [
                "turn_split_boost_mean",
                "turn_split_boost_median",
                "turn_split_boost_p10",
                "turn_abs_omega_error_mean",
                "turn_actual_over_cmd_p95",
            ]
        )
        reliable = (
            finite_metrics
            and turn_metrics_finite
            and score < current_score
            and model_split_rmse <= self.config.checkpoint_max_split_rmse
            and model_wheel_rmse <= self.config.checkpoint_max_wheel_rmse
            and turn_sample_count >= self.config.checkpoint_min_turn_samples
            and turn_split_boost_median >= self.config.checkpoint_min_turn_split_boost_rpm
            and turn_actual_over_cmd_p95 <= self.config.checkpoint_max_turn_actual_over_cmd_p95
        )
        if not finite_metrics:
            reason = "nonfinite_metrics"
        elif score >= current_score:
            reason = "not_better_than_current_command"
        elif model_split_rmse > self.config.checkpoint_max_split_rmse:
            reason = "split_rmse_too_high"
        elif model_wheel_rmse > self.config.checkpoint_max_wheel_rmse:
            reason = "wheel_rmse_too_high"
        elif turn_sample_count < self.config.checkpoint_min_turn_samples:
            reason = "insufficient_turn_samples"
        elif not turn_metrics_finite:
            reason = "nonfinite_turn_metrics"
        elif turn_split_boost_median < self.config.checkpoint_min_turn_split_boost_rpm:
            reason = "turn_split_boost_too_low"
        elif turn_actual_over_cmd_p95 > self.config.checkpoint_max_turn_actual_over_cmd_p95:
            reason = "turn_overshoot_p95_too_high"
        else:
            reason = "reliable"

        metrics["reliable"] = bool(reliable)
        metrics["reason"] = reason
        return metrics, {"pred": pred, "target": target, "current": current}

    def checkpoint_is_meaningful(self, metrics: dict[str, Any]) -> bool:
        score = float(metrics["model_score"])
        if self.best_checkpoint_score is None:
            return True
        absolute_gain = self.best_checkpoint_score - score
        if absolute_gain >= self.config.checkpoint_min_improvement_rpm:
            return True
        if self.best_checkpoint_score > 1.0e-9:
            relative_gain = absolute_gain / self.best_checkpoint_score
            if relative_gain >= self.config.checkpoint_min_relative_improvement:
                return True

        current_gain = float(metrics.get("current_window_score_gain", math.nan))
        current_relative_gain = float(metrics.get("current_window_relative_gain", math.nan))
        split_boost_improvement = float(metrics.get("turn_split_boost_improvement", math.nan))
        # 关键：旧 best 来自早期窗口，不能永远压住后面更会转向的模型。
        # 当前窗口里明显优于原始 RPM，且转向补偿又变强时，允许替换 checkpoint。
        return (
            math.isfinite(current_gain)
            and math.isfinite(current_relative_gain)
            and math.isfinite(split_boost_improvement)
            and current_gain >= self.config.checkpoint_min_current_score_gain
            and current_relative_gain >= self.config.checkpoint_min_current_relative_gain
            and split_boost_improvement >= self.config.checkpoint_min_turn_split_boost_improvement_rpm
        )

    def record_checkpoint_event(
        self,
        *,
        accepted: bool,
        reliable: bool,
        reason: str,
        metrics: dict[str, Any],
        checkpoint_path: Path | None = None,
    ) -> dict[str, Any]:
        row = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "accepted": int(bool(accepted)),
            "reliable": int(bool(reliable)),
            "reason": reason,
            "update": self.update_index,
            "sample_rows": len(self.sample_rows),
            "replay_size": len(self.replay),
            "checkpoint_path": "" if checkpoint_path is None else str(checkpoint_path),
            "best_score": "" if self.best_checkpoint_score is None else float(self.best_checkpoint_score),
            "model_score": metrics.get("model_score", ""),
            "current_score": metrics.get("current_score", ""),
            "current_window_score_gain": metrics.get("current_window_score_gain", ""),
            "current_window_relative_gain": metrics.get("current_window_relative_gain", ""),
            "model_split_rmse": metrics.get("model_split_rmse", ""),
            "current_split_rmse": metrics.get("current_split_rmse", ""),
            "model_wheel_rmse": metrics.get("model_wheel_rmse", ""),
            "current_wheel_rmse": metrics.get("current_wheel_rmse", ""),
            "correction_split_p95": metrics.get("correction_split_p95", ""),
            "turn_sample_count": metrics.get("turn_sample_count", ""),
            "turn_split_boost_mean": metrics.get("turn_split_boost_mean", ""),
            "turn_split_boost_median": metrics.get("turn_split_boost_median", ""),
            "turn_split_boost_p10": metrics.get("turn_split_boost_p10", ""),
            "turn_split_boost_improvement": metrics.get("turn_split_boost_improvement", ""),
            "turn_abs_omega_error_mean": metrics.get("turn_abs_omega_error_mean", ""),
            "turn_actual_over_cmd_p95": metrics.get("turn_actual_over_cmd_p95", ""),
            "eval_sample_count": metrics.get("sample_count", ""),
        }
        self.checkpoint_rows.append(row)
        return row

    def maybe_accept_checkpoint(self, *, archive: bool = False, reason: str = "periodic") -> dict[str, Any]:
        if self.training_halted:
            return self.record_checkpoint_event(
                accepted=False,
                reliable=False,
                reason="training_halted",
                metrics={},
            )
        if self.update_index == self.last_checkpoint_eval_update and reason == "periodic":
            return self.record_checkpoint_event(
                accepted=False,
                reliable=False,
                reason="already_evaluated_update",
                metrics={},
            )
        self.last_checkpoint_eval_update = self.update_index
        metrics, _ = self.checkpoint_candidate_metrics()
        reliable = bool(metrics.get("reliable", False))
        if not reliable:
            return self.record_checkpoint_event(
                accepted=False,
                reliable=False,
                reason=str(metrics.get("reason", "not_reliable")),
                metrics=metrics,
            )
        score = float(metrics["model_score"])
        if not self.checkpoint_is_meaningful(metrics):
            return self.record_checkpoint_event(
                accepted=False,
                reliable=True,
                reason="reliable_but_not_meaningful_improvement",
                metrics=metrics,
            )

        path = self.save_checkpoint(final=False, archive=archive, checkpoint_metrics=metrics)
        self.best_checkpoint_score = score
        self.best_checkpoint_turn_split_boost_median = float(metrics.get("turn_split_boost_median", math.nan))
        self.best_checkpoint_update = self.update_index
        self.accepted_checkpoint_update = self.update_index
        self.accepted_checkpoint_path = path
        return self.record_checkpoint_event(
            accepted=True,
            reliable=True,
            reason=reason,
            metrics=metrics,
            checkpoint_path=path,
        )

    def accept_current_checkpoint(self, *, archive: bool = False, reason: str = "forced") -> dict[str, Any]:
        metrics = {
            "reliable": True,
            "reason": reason,
            "sample_count": len(self.sample_rows),
            "model_score": self.best_checkpoint_score if self.best_checkpoint_score is not None else math.inf,
        }
        path = self.save_checkpoint(final=False, archive=archive, checkpoint_metrics=metrics)
        if self.best_checkpoint_score is not None:
            self.best_checkpoint_turn_split_boost_median = self.best_checkpoint_turn_split_boost_median
        self.accepted_checkpoint_update = self.update_index
        self.accepted_checkpoint_path = path
        return self.record_checkpoint_event(
            accepted=True,
            reliable=True,
            reason=reason,
            metrics=metrics,
            checkpoint_path=path,
        )

    def save_artifacts(
        self,
        *,
        final: bool = False,
        archive: bool = False,
        evaluate_checkpoint: bool = True,
        max_archived_checkpoints: int | None = None,
    ) -> dict[str, Any] | None:
        self.write_csv(self.output_dir / "online_training_samples.csv", self.sample_fieldnames(), self.sample_rows)
        self.write_csv(self.output_dir / "online_training_history.csv", self.history_fieldnames(), self.train_rows)
        self.write_csv(self.output_dir / "online_training_divergence_events.csv", self.divergence_fieldnames(), self.divergence_rows)
        self.write_csv(self.output_dir / "online_checkpoint_events.csv", self.checkpoint_fieldnames(), self.checkpoint_rows)
        self.write_manifest(final=final)
        event = None
        if evaluate_checkpoint:
            event = self.maybe_accept_checkpoint(archive=archive, reason="final" if final else "periodic")
        if final:
            self.write_best_final_checkpoint()
        if max_archived_checkpoints is not None:
            self.prune_archived_checkpoints(max_archived_checkpoints)
        if final:
            self.generate_plots()
        self.write_csv(self.output_dir / "online_checkpoint_events.csv", self.checkpoint_fieldnames(), self.checkpoint_rows)
        return event

    def checkpoint_payload(self, checkpoint_metrics: dict[str, Any] | None = None) -> dict[str, Any]:
        return {
            "model": {key: value.detach().cpu().clone() for key, value in self.model.state_dict().items()},
            "config": asdict(self.config),
            "feature_cols": FEATURE_COLS,
            "command_cols": COMMAND_COLS,
            "target_cols": TARGET_COLS,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "sample_rows": len(self.sample_rows),
            "updates": self.update_index,
            "training_halted": self.training_halted,
            "divergence_events": len(self.divergence_rows),
            "checkpoint_metrics": checkpoint_metrics or {},
            "target_note": (
                "Online target RPM is current sent RPM plus an ideal-kinematic RPM nudge from "
                "body tracking error: target = current_rpm + adaptation_gain * rpm(cmd - measured)."
            ),
        }

    def checkpoint_bytes(self) -> bytes:
        if self.accepted_checkpoint_payload is None:
            raise RuntimeError("No accepted RPM checkpoint is available for publication.")
        payload = self.accepted_checkpoint_payload
        buffer = io.BytesIO()
        torch.save(payload, buffer)
        return buffer.getvalue()

    def has_publishable_checkpoint(self) -> bool:
        return self.accepted_checkpoint_payload is not None

    def load_checkpoint(self, checkpoint_path: Path) -> None:
        try:
            checkpoint = torch.load(checkpoint_path, map_location=self.device, weights_only=False)
        except TypeError:
            checkpoint = torch.load(checkpoint_path, map_location=self.device)
        feature_cols = list(checkpoint.get("feature_cols", []))
        command_cols = list(checkpoint.get("command_cols", []))
        if feature_cols != FEATURE_COLS:
            raise RuntimeError(f"Expected RPM features {FEATURE_COLS}, got {feature_cols}")
        if command_cols != COMMAND_COLS:
            raise RuntimeError(f"Expected RPM commands {COMMAND_COLS}, got {command_cols}")
        self.model.load_state_dict(checkpoint["model"])
        self.update_index = int(checkpoint.get("updates", self.update_index))
        self.training_halted = bool(checkpoint.get("training_halted", False))
        self.accepted_checkpoint_payload = self.checkpoint_payload(checkpoint.get("checkpoint_metrics", {}))
        self.accepted_checkpoint_update = self.update_index
        self.accepted_checkpoint_path = checkpoint_path

    def save_checkpoint(
        self,
        *,
        final: bool = False,
        archive: bool = False,
        checkpoint_metrics: dict[str, Any] | None = None,
    ) -> Path:
        payload = self.checkpoint_payload(checkpoint_metrics)
        latest = self.weights_dir / "corrected_controller_rpm_latest.pt"
        torch.save(payload, latest)
        self.accepted_checkpoint_payload = payload
        self.accepted_checkpoint_path = latest
        archived_path = None
        if archive:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            archived_path = self.checkpoints_dir / f"checkpoint_{stamp}_u{self.update_index:06d}.pt"
            torch.save(payload, archived_path)
        if final:
            torch.save(payload, self.weights_dir / "corrected_controller_rpm_final.pt")
            shutil.copyfile(self.weights_dir / "corrected_controller_rpm_final.pt", self.weights_dir / "corrected_controller_rpm_startpoint.pt")
        return archived_path or latest

    def write_best_final_checkpoint(self) -> None:
        if self.accepted_checkpoint_payload is None:
            self.log_warning("No reliable RPM checkpoint was accepted; final .pt was not updated.")
            return
        final_path = self.weights_dir / "corrected_controller_rpm_final.pt"
        torch.save(self.accepted_checkpoint_payload, final_path)
        shutil.copyfile(final_path, self.weights_dir / "corrected_controller_rpm_startpoint.pt")

    def prune_archived_checkpoints(self, max_archived_checkpoints: int) -> None:
        if max_archived_checkpoints <= 0:
            return
        checkpoints = sorted(self.checkpoints_dir.glob("checkpoint_*.pt"), key=lambda path: path.stat().st_mtime)
        for path in checkpoints[:-max_archived_checkpoints]:
            try:
                path.unlink()
            except OSError:
                self.log_warning(f"Could not prune old checkpoint: {path}")

    def write_manifest(self, *, final: bool) -> None:
        manifest = {
            "mode": "online_rpm_correction_training",
            "final": bool(final),
            "samples": len(self.sample_rows),
            "updates": self.update_index,
            "training_halted": self.training_halted,
            "divergence_events": len(self.divergence_rows),
            "accepted_checkpoints": sum(1 for row in self.checkpoint_rows if int(row.get("accepted", 0)) == 1),
            "best_checkpoint_score": self.best_checkpoint_score,
            "best_checkpoint_update": self.best_checkpoint_update,
            "config": asdict(self.config),
            "outputs": {
                "samples_csv": "online_training_samples.csv",
                "history_csv": "online_training_history.csv",
                "divergence_csv": "online_training_divergence_events.csv",
                "checkpoint_events_csv": "online_checkpoint_events.csv",
                "latest_weights": "weights/corrected_controller_rpm_latest.pt",
                "final_weights": "weights/corrected_controller_rpm_final.pt",
                "startpoint_weights": "weights/corrected_controller_rpm_startpoint.pt",
                "archived_checkpoints": "weights/checkpoints/",
                "plots": "plots/",
            },
            "target_note": (
                "This learns an RPM command correction target from live body tracking error. "
                "It does not require measured wheel RPM; measured wheel RPM would be better if later available."
            ),
        }
        (self.output_dir / "run_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    def summary_metrics(self) -> dict[str, float | int | None]:
        if not self.sample_rows:
            return {"samples": 0, "updates": self.update_index}
        rows = self.sample_rows
        pred = np.asarray([[row["pred_right_rpm"], row["pred_left_rpm"]] for row in rows], dtype=np.float64)
        target = np.asarray([[row["target_right_rpm"], row["target_left_rpm"]] for row in rows], dtype=np.float64)
        current = np.asarray([[row["current_right_rpm"], row["current_left_rpm"]] for row in rows], dtype=np.float64)
        cmd = np.asarray([[row["cmd_v"], row["cmd_omega"]] for row in rows], dtype=np.float64)
        meas = np.asarray([[row["meas_v"], row["meas_omega"]] for row in rows], dtype=np.float64)
        return {
            "samples": len(rows),
            "updates": self.update_index,
            "pred_target_rmse_right_rpm": float(np.sqrt(np.mean((pred[:, 0] - target[:, 0]) ** 2))),
            "pred_target_rmse_left_rpm": float(np.sqrt(np.mean((pred[:, 1] - target[:, 1]) ** 2))),
            "pred_target_split_rmse": float(
                np.sqrt(np.mean(((pred[:, 0] - pred[:, 1]) - (target[:, 0] - target[:, 1])) ** 2))
            ),
            "current_target_split_rmse": float(
                np.sqrt(np.mean(((current[:, 0] - current[:, 1]) - (target[:, 0] - target[:, 1])) ** 2))
            ),
            "tracking_rmse_v": float(np.sqrt(np.mean((cmd[:, 0] - meas[:, 0]) ** 2))),
            "tracking_rmse_omega": float(np.sqrt(np.mean((cmd[:, 1] - meas[:, 1]) ** 2))),
        }

    def generate_plots(self) -> None:
        if not self.sample_rows:
            return
        try:
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.log_warning(f"Could not generate online-training plots because matplotlib is unavailable: {exc}")
            (self.plots_dir / "plots_unavailable.txt").write_text(str(exc), encoding="utf-8")
            return

        time_values = np.asarray([row["timestamp"] for row in self.sample_rows], dtype=np.float64)
        rel_time = time_values - time_values[0]
        cmd_v = np.asarray([row["cmd_v"] for row in self.sample_rows], dtype=np.float64)
        cmd_omega = np.asarray([row["cmd_omega"] for row in self.sample_rows], dtype=np.float64)
        meas_v = np.asarray([row["meas_v"] for row in self.sample_rows], dtype=np.float64)
        meas_omega = np.asarray([row["meas_omega"] for row in self.sample_rows], dtype=np.float64)
        ideal_right = np.asarray([row["ideal_right_rpm"] for row in self.sample_rows], dtype=np.float64)
        ideal_left = np.asarray([row["ideal_left_rpm"] for row in self.sample_rows], dtype=np.float64)
        current_right = np.asarray([row["current_right_rpm"] for row in self.sample_rows], dtype=np.float64)
        current_left = np.asarray([row["current_left_rpm"] for row in self.sample_rows], dtype=np.float64)
        target_right = np.asarray([row["target_right_rpm"] for row in self.sample_rows], dtype=np.float64)
        target_left = np.asarray([row["target_left_rpm"] for row in self.sample_rows], dtype=np.float64)
        pred_right = np.asarray([row["pred_right_rpm"] for row in self.sample_rows], dtype=np.float64)
        pred_left = np.asarray([row["pred_left_rpm"] for row in self.sample_rows], dtype=np.float64)

        fig, axes = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
        axes[0].plot(rel_time, cmd_v, label="cmd v")
        axes[0].plot(rel_time, meas_v, label="measured v")
        axes[0].set_ylabel("v [m/s]")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc="best")
        axes[1].plot(rel_time, cmd_omega, label="cmd omega")
        axes[1].plot(rel_time, meas_omega, label="measured omega")
        axes[1].set_xlabel("run time [s]")
        axes[1].set_ylabel("omega [rad/s]")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc="best")
        fig.suptitle("Online RPM Trainer Tracking")
        fig.tight_layout()
        fig.savefig(self.plots_dir / "tracking_cmd_vs_measured.png", dpi=160)
        plt.close(fig)

        fig, axes = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
        axes[0].plot(rel_time, current_right, label="current right rpm", alpha=0.8)
        axes[0].plot(rel_time, target_right, label="target right rpm")
        axes[0].plot(rel_time, pred_right, label="model right rpm")
        axes[0].set_ylabel("right RPM")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc="best")
        axes[1].plot(rel_time, current_left, label="current left rpm", alpha=0.8)
        axes[1].plot(rel_time, target_left, label="target left rpm")
        axes[1].plot(rel_time, pred_left, label="model left rpm")
        axes[1].set_xlabel("run time [s]")
        axes[1].set_ylabel("left RPM")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc="best")
        fig.suptitle("Online RPM Target vs Model")
        fig.tight_layout()
        fig.savefig(self.plots_dir / "rpm_target_vs_model.png", dpi=160)
        plt.close(fig)

        fig, axis = plt.subplots(figsize=(11, 4.8))
        axis.plot(rel_time, ideal_right - ideal_left, label="ideal rpm split", alpha=0.8)
        axis.plot(rel_time, current_right - current_left, label="current rpm split", alpha=0.8)
        axis.plot(rel_time, target_right - target_left, label="target rpm split")
        axis.plot(rel_time, pred_right - pred_left, label="model rpm split")
        axis.set_xlabel("run time [s]")
        axis.set_ylabel("right-left RPM")
        axis.set_title("Online RPM Split Correction")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
        fig.tight_layout()
        fig.savefig(self.plots_dir / "rpm_split.png", dpi=160)
        plt.close(fig)

        if self.train_rows:
            fig, axis = plt.subplots(figsize=(8.5, 4.5))
            axis.plot([row["update"] for row in self.train_rows], [row["loss"] for row in self.train_rows], label="loss")
            axis.set_yscale("log")
            axis.set_xlabel("update")
            axis.set_ylabel("loss")
            axis.set_title("Online RPM Training Loss")
            axis.grid(True, alpha=0.3)
            axis.legend(loc="best")
            fig.tight_layout()
            fig.savefig(self.plots_dir / "online_loss.png", dpi=160)
            plt.close(fig)

        metrics = self.summary_metrics()
        (self.artifacts_dir / "online_summary_metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")

    @staticmethod
    def sample_fieldnames() -> list[str]:
        return [
            "timestamp",
            "segment",
            "state",
            "cmd_v",
            "cmd_omega",
            "meas_v",
            "meas_omega",
            "error_v",
            "error_omega",
            "ideal_right_rpm",
            "ideal_left_rpm",
            "current_right_rpm",
            "current_left_rpm",
            "target_right_rpm",
            "target_left_rpm",
            "target_delta_right_rpm",
            "target_delta_left_rpm",
            "target_delta_split_rpm",
            "turn_weight",
            "pred_right_rpm",
            "pred_left_rpm",
            "a_right",
            "a_left",
            "b_right",
            "b_left",
            "train_loss",
        ]

    @staticmethod
    def history_fieldnames() -> list[str]:
        return [
            "update",
            "sample_count",
            "replay_size",
            "loss",
            "wheel_loss",
            "split_loss",
            "gain_loss",
            "bias_loss",
            "skipped_step",
            "divergence_reason",
        ]

    @staticmethod
    def divergence_fieldnames() -> list[str]:
        return [
            "timestamp",
            "update",
            "sample_count",
            "replay_size",
            "reason",
            "loss",
            "threshold",
        ]

    @staticmethod
    def checkpoint_fieldnames() -> list[str]:
        return [
            "timestamp",
            "accepted",
            "reliable",
            "reason",
            "update",
            "sample_rows",
            "replay_size",
            "checkpoint_path",
            "best_score",
            "model_score",
            "current_score",
            "current_window_score_gain",
            "current_window_relative_gain",
            "model_split_rmse",
            "current_split_rmse",
            "model_wheel_rmse",
            "current_wheel_rmse",
            "correction_split_p95",
            "turn_sample_count",
            "turn_split_boost_mean",
            "turn_split_boost_median",
            "turn_split_boost_p10",
            "turn_split_boost_improvement",
            "turn_abs_omega_error_mean",
            "turn_actual_over_cmd_p95",
            "eval_sample_count",
        ]

    @staticmethod
    def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = path.with_name(path.name + ".tmp")
        try:
            with tmp_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
                writer.writeheader()
                for row in rows:
                    writer.writerow(row)
                handle.flush()
            tmp_path.replace(path)
        finally:
            if tmp_path.exists():
                try:
                    tmp_path.unlink()
                except OSError:
                    pass
