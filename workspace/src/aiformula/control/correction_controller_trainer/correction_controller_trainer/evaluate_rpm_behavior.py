from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch import nn


FEATURE_COLS = ["cmd_v", "cmd_omega", "meas_v", "meas_omega", "ideal_right_rpm", "ideal_left_rpm"]
COMMAND_COLS = ["ideal_right_rpm", "ideal_left_rpm", "cmd_v", "cmd_omega"]


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate a live RPM correction run/startpoint.")
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--checkpoint", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser.parse_args()


def as_float(raw: Any, default: float = math.nan) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def read_rows(path: Path) -> list[dict[str, Any]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows: list[dict[str, Any]] = []
        for raw in reader:
            row: dict[str, Any] = {}
            for key, value in raw.items():
                if key in {"segment", "state"}:
                    row[key] = value
                else:
                    row[key] = as_float(value)
            rows.append(row)
    if not rows:
        raise RuntimeError(f"No sample rows found in {path}")
    return rows


def array(rows: list[dict[str, Any]], column: str) -> np.ndarray:
    return np.asarray([as_float(row.get(column)) for row in rows], dtype=np.float64)


def normalized_feature(row: dict[str, Any], config: dict[str, Any]) -> np.ndarray:
    return np.asarray(
        [
            as_float(row["cmd_v"]) / float(config.get("v_scale", 3.0)),
            as_float(row["cmd_omega"]) / float(config.get("omega_scale", 1.0)),
            as_float(row["meas_v"]) / float(config.get("v_scale", 3.0)),
            as_float(row["meas_omega"]) / float(config.get("omega_scale", 1.0)),
            as_float(row["ideal_right_rpm"]) / float(config.get("rpm_scale", 350.0)),
            as_float(row["ideal_left_rpm"]) / float(config.get("rpm_scale", 350.0)),
        ],
        dtype=np.float32,
    )


def normalized_command(row: dict[str, Any], config: dict[str, Any]) -> np.ndarray:
    return np.asarray(
        [
            as_float(row["ideal_right_rpm"]) / float(config.get("rpm_scale", 350.0)),
            as_float(row["ideal_left_rpm"]) / float(config.get("rpm_scale", 350.0)),
            as_float(row["cmd_v"]) / float(config.get("v_scale", 3.0)),
            as_float(row["cmd_omega"]) / float(config.get("omega_scale", 1.0)),
        ],
        dtype=np.float32,
    )


def command_raw(row: dict[str, Any]) -> np.ndarray:
    return np.asarray(
        [
            as_float(row["ideal_right_rpm"]),
            as_float(row["ideal_left_rpm"]),
            as_float(row["cmd_v"]),
            as_float(row["cmd_omega"]),
        ],
        dtype=np.float32,
    )


def load_checkpoint(path: Path) -> dict[str, Any]:
    try:
        return torch.load(path, map_location="cpu", weights_only=False)
    except TypeError:
        return torch.load(path, map_location="cpu")


@torch.no_grad()
def evaluate_final_checkpoint(rows: list[dict[str, Any]], checkpoint: dict[str, Any]) -> dict[str, np.ndarray]:
    config = dict(checkpoint["config"])
    feature_cols = list(checkpoint.get("feature_cols", []))
    command_cols = list(checkpoint.get("command_cols", []))
    if feature_cols != FEATURE_COLS:
        raise RuntimeError(f"Expected features {FEATURE_COLS}, got {feature_cols}")
    if command_cols != COMMAND_COLS:
        raise RuntimeError(f"Expected commands {COMMAND_COLS}, got {command_cols}")

    model = RpmAffineModel(
        history_dim=len(FEATURE_COLS),
        command_dim=len(COMMAND_COLS),
        hidden_size=int(config.get("hidden_size", 48)),
        gru_layers=int(config.get("gru_layers", 1)),
        dropout=float(config.get("dropout", 0.0)),
        gain_span=float(config.get("gain_span", 2.0)),
        max_bias_rpm=float(config.get("max_bias_rpm", 260.0)),
    )
    model.load_state_dict(checkpoint["model"])
    model.eval()

    history_steps = int(config.get("history_steps", 20))
    max_abs_rpm = float(config.get("max_abs_rpm", 650.0))
    features = np.stack([normalized_feature(row, config) for row in rows], axis=0)
    pred = np.full((len(rows), 2), np.nan, dtype=np.float64)
    params = np.full((len(rows), 4), np.nan, dtype=np.float64)
    valid = np.zeros(len(rows), dtype=bool)

    for index in range(history_steps - 1, len(rows)):
        history = features[index - history_steps + 1 : index + 1]
        command_norm = normalized_command(rows[index], config)
        raw = command_raw(rows[index])
        output = model(
            torch.as_tensor(history[None, :, :], dtype=torch.float32),
            torch.as_tensor(command_norm[None, :], dtype=torch.float32),
        )
        p = output[0].cpu().numpy().astype(np.float64)
        right = p[0] * float(raw[0]) + p[2]
        left = p[1] * float(raw[1]) + p[3]
        pred[index] = np.clip([right, left], -max_abs_rpm, max_abs_rpm)
        params[index] = p
        valid[index] = True

    return {"pred": pred, "params": params, "valid": valid}


def rmse(values: np.ndarray) -> float:
    values = values[np.isfinite(values)]
    if values.size == 0:
        return math.nan
    return float(np.sqrt(np.mean(values**2)))


def percentile(values: np.ndarray, q: float) -> float:
    values = values[np.isfinite(values)]
    if values.size == 0:
        return math.nan
    return float(np.percentile(values, q))


def metric_block(target: np.ndarray, pred: np.ndarray, mask: np.ndarray) -> dict[str, float]:
    err = pred[mask] - target[mask]
    split_err = (pred[mask, 0] - pred[mask, 1]) - (target[mask, 0] - target[mask, 1])
    return {
        "rmse_right_rpm": rmse(err[:, 0]),
        "rmse_left_rpm": rmse(err[:, 1]),
        "rmse_split_rpm": rmse(split_err),
    }


def grouped_metrics(rows: list[dict[str, Any]], target: np.ndarray, current: np.ndarray, final: np.ndarray, valid: np.ndarray):
    states = sorted({str(row.get("state", "")) for row in rows})
    groups: dict[str, Any] = {}
    cmd_v = array(rows, "cmd_v")
    cmd_w = array(rows, "cmd_omega")
    moving = (np.abs(cmd_v) > 0.05) | (np.abs(cmd_w) > 0.03)
    for name, group_mask in [("all", np.ones(len(rows), dtype=bool)), ("moving", moving)]:
        mask = valid & group_mask
        groups[name] = {
            "rows": int(mask.sum()),
            "sent": metric_block(target, current, mask) if mask.any() else {},
            "final": metric_block(target, final, mask) if mask.any() else {},
        }
    for state in states:
        state_mask = np.asarray([str(row.get("state", "")) == state for row in rows], dtype=bool)
        mask = valid & state_mask
        groups[f"state_{state}"] = {
            "rows": int(mask.sum()),
            "sent": metric_block(target, current, mask) if mask.any() else {},
            "final": metric_block(target, final, mask) if mask.any() else {},
        }
        moving_mask = valid & state_mask & moving
        groups[f"state_{state}_moving"] = {
            "rows": int(moving_mask.sum()),
            "sent": metric_block(target, current, moving_mask) if moving_mask.any() else {},
            "final": metric_block(target, final, moving_mask) if moving_mask.any() else {},
        }
    return groups


def rolling_mean(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return values.copy()
    kernel = np.ones(window, dtype=np.float64) / float(window)
    return np.convolve(values, kernel, mode="same")


def state_mask(rows: list[dict[str, Any]], state: str) -> np.ndarray:
    return np.asarray([str(row.get("state", "")) == state for row in rows], dtype=bool)


def choose_state_window(mask: np.ndarray, activity: np.ndarray, window: int = 450) -> np.ndarray:
    indices = np.flatnonzero(mask)
    if indices.size == 0:
        return np.asarray([], dtype=np.int64)
    if indices.size <= window:
        return indices
    best_score = -1.0
    best = indices[:window]
    for start in range(0, indices.size - window + 1):
        chunk = indices[start : start + window]
        if chunk[-1] - chunk[0] > window * 2:
            continue
        score = float(np.nanstd(activity[chunk]))
        if score > best_score:
            best_score = score
            best = chunk
    return best


def write_summary(path: Path, summary: dict[str, Any]) -> None:
    path.write_text(json.dumps(summary, indent=2), encoding="utf-8")


def generate_plots(run_dir: Path, rows: list[dict[str, Any]], final_eval: dict[str, np.ndarray], summary: dict[str, Any]) -> list[Path]:
    import matplotlib.pyplot as plt

    output_dir = Path(summary["output_dir"])
    output_dir.mkdir(parents=True, exist_ok=True)
    x = np.arange(len(rows), dtype=np.float64)
    cmd_v = array(rows, "cmd_v")
    cmd_w = array(rows, "cmd_omega")
    meas_v = array(rows, "meas_v")
    meas_w = array(rows, "meas_omega")
    current = np.column_stack([array(rows, "current_right_rpm"), array(rows, "current_left_rpm")])
    target = np.column_stack([array(rows, "target_right_rpm"), array(rows, "target_left_rpm")])
    logged = np.column_stack([array(rows, "pred_right_rpm"), array(rows, "pred_left_rpm")])
    final = final_eval["pred"]
    params = final_eval["params"]
    mask = final_eval["valid"]
    moving = (np.abs(cmd_v) > 0.05) | (np.abs(cmd_w) > 0.03)
    state2 = state_mask(rows, "2")
    moving_state2 = mask & moving & state2

    paths: list[Path] = []

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axes[0].plot(x, cmd_v, label="cmd v", linewidth=1.0)
    axes[0].plot(x, meas_v, label="measured v", linewidth=0.9, alpha=0.85)
    axes[0].set_ylabel("m/s")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    axes[1].plot(x, cmd_w, label="cmd omega", linewidth=1.0)
    axes[1].plot(x, meas_w, label="measured omega", linewidth=0.9, alpha=0.85)
    axes[1].set_ylabel("rad/s")
    axes[1].set_xlabel("sample index")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")
    fig.suptitle("Command vs Measured Motion")
    fig.tight_layout()
    path = output_dir / "behavior_cmd_vs_measured.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    paths.append(path)

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    labels = [("right", 0), ("left", 1)]
    for axis, (label, idx) in zip(axes, labels):
        axis.plot(x, current[:, idx], label=f"sent {label}", linewidth=0.85, alpha=0.8)
        axis.plot(x, target[:, idx], label=f"adaptive target {label}", linewidth=1.0)
        axis.plot(x, final[:, idx], label=f"final model {label}", linewidth=0.95)
        axis.set_ylabel("RPM")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    axes[-1].set_xlabel("sample index")
    fig.suptitle("Sent RPM vs Target vs Current Final Model")
    fig.tight_layout()
    path = output_dir / "behavior_rpm_target_fit.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    paths.append(path)

    fig, axis = plt.subplots(figsize=(12, 5.5))
    current_split = current[:, 0] - current[:, 1]
    target_split = target[:, 0] - target[:, 1]
    logged_split = logged[:, 0] - logged[:, 1]
    final_split = final[:, 0] - final[:, 1]
    axis.plot(x, current_split, label="sent split", alpha=0.75)
    axis.plot(x, target_split, label="target split", linewidth=1.15)
    axis.plot(x, logged_split, label="logged online-model split", alpha=0.8)
    axis.plot(x, final_split, label="final checkpoint split", linewidth=1.0)
    axis.set_xlabel("sample index")
    axis.set_ylabel("right-left RPM")
    axis.set_title("Steering Signal: RPM Split")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    fig.tight_layout()
    path = output_dir / "behavior_rpm_split.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    paths.append(path)

    correction = final - current
    target_nudge = target - current
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for axis, label, idx in [(axes[0], "right", 0), (axes[1], "left", 1)]:
        axis.plot(x[mask], target_nudge[mask, idx], label=f"target-current {label}", linewidth=1.0)
        axis.plot(x[mask], correction[mask, idx], label=f"model-current {label}", linewidth=0.95)
        axis.axhline(0.0, color="black", linewidth=0.7)
        axis.set_ylabel("RPM delta")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    axes[-1].set_xlabel("sample index")
    fig.suptitle("Correction Size")
    fig.tight_layout()
    path = output_dir / "behavior_correction_delta.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    paths.append(path)

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axes[0].plot(x[mask], params[mask, 0], label="a_right")
    axes[0].plot(x[mask], params[mask, 1], label="a_left")
    axes[0].axhline(1.0, color="black", linestyle="--", linewidth=0.75)
    axes[0].set_ylabel("gain")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    axes[1].plot(x[mask], params[mask, 2], label="b_right")
    axes[1].plot(x[mask], params[mask, 3], label="b_left")
    axes[1].axhline(0.0, color="black", linestyle="--", linewidth=0.75)
    axes[1].set_ylabel("RPM bias")
    axes[1].set_xlabel("sample index")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")
    fig.suptitle("Final Checkpoint Affine Parameters")
    fig.tight_layout()
    path = output_dir / "behavior_affine_params.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    paths.append(path)

    history_path = run_dir / "online_training_history.csv"
    if history_path.exists():
        with history_path.open("r", newline="", encoding="utf-8") as handle:
            train_rows = list(csv.DictReader(handle))
        updates = np.asarray([as_float(row.get("update")) for row in train_rows], dtype=np.float64)
        loss = np.asarray([as_float(row.get("loss")) for row in train_rows], dtype=np.float64)
        fig, axis = plt.subplots(figsize=(10, 4.8))
        axis.plot(updates, loss, linewidth=0.85)
        axis.set_yscale("log")
        axis.set_xlabel("update")
        axis.set_ylabel("loss")
        axis.set_title("Online-Style Training Loss")
        axis.grid(True, alpha=0.3)
    fig.tight_layout()
    path = output_dir / "behavior_loss.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    paths.append(path)

    # Cleaner summary view: sent vs final split RMSE by subset.
    groups = summary["grouped_metrics"]
    labels = ["all", "moving", "state_2", "state_2_moving"]
    sent_rmse = [groups[name]["sent"]["rmse_split_rpm"] for name in labels]
    final_rmse = [groups[name]["final"]["rmse_split_rpm"] for name in labels]
    positions = np.arange(len(labels), dtype=np.float64)
    width = 0.36
    fig, axis = plt.subplots(figsize=(8.5, 4.8))
    axis.bar(positions - width / 2.0, sent_rmse, width=width, label="sent split error")
    axis.bar(positions + width / 2.0, final_rmse, width=width, label="final model split error")
    axis.set_xticks(positions, ["all", "moving", "state2", "state2 moving"])
    axis.set_ylabel("split RMSE [RPM]")
    axis.set_title("Steering Error Summary")
    axis.grid(True, axis="y", alpha=0.3)
    axis.legend(loc="best")
    fig.tight_layout()
    path = output_dir / "behavior_split_rmse_summary.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    paths.append(path)

    # Moving-only scatter is easier to read than the full line plot.
    current_split = current[:, 0] - current[:, 1]
    target_split = target[:, 0] - target[:, 1]
    final_split = final[:, 0] - final[:, 1]
    moving_mask = mask & moving
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.8), sharex=True, sharey=True)
    for axis, y_values, title in [
        (axes[0], current_split[moving_mask], "Sent vs Target"),
        (axes[1], final_split[moving_mask], "Final Model vs Target"),
    ]:
        axis.scatter(target_split[moving_mask], y_values, s=8, alpha=0.18)
        low = float(min(np.nanmin(target_split[moving_mask]), np.nanmin(y_values)))
        high = float(max(np.nanmax(target_split[moving_mask]), np.nanmax(y_values)))
        axis.plot([low, high], [low, high], color="black", linestyle="--", linewidth=0.8)
        axis.set_title(title)
        axis.set_xlabel("target split [RPM]")
        axis.grid(True, alpha=0.3)
    axes[0].set_ylabel("predicted split [RPM]")
    fig.suptitle("Moving Samples: Steering Fit")
    fig.tight_layout()
    path = output_dir / "behavior_split_scatter_moving.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    paths.append(path)

    # Error histogram shows whether the model narrows the steering error or spreads it.
    sent_split_error = current_split[moving_mask] - target_split[moving_mask]
    final_split_error = final_split[moving_mask] - target_split[moving_mask]
    hist_limit = float(np.percentile(np.abs(np.concatenate([sent_split_error, final_split_error])), 98))
    bins = np.linspace(-hist_limit, hist_limit, 50)
    fig, axis = plt.subplots(figsize=(9.5, 4.8))
    axis.hist(sent_split_error, bins=bins, alpha=0.45, label="sent split error")
    axis.hist(final_split_error, bins=bins, alpha=0.45, label="final model split error")
    axis.axvline(0.0, color="black", linewidth=0.8)
    axis.set_xlabel("split error [RPM]")
    axis.set_ylabel("count")
    axis.set_title("Moving Samples: Steering Error Distribution")
    axis.grid(True, axis="y", alpha=0.3)
    axis.legend(loc="best")
    fig.tight_layout()
    path = output_dir / "behavior_split_error_hist_moving.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    paths.append(path)

    # Show one active state-2 window instead of the whole run.
    window_idx = choose_state_window(moving_state2, np.abs(target_split))
    if window_idx.size > 0:
        window_x = np.arange(window_idx.size, dtype=np.float64)
        fig, axis = plt.subplots(figsize=(10.5, 4.8))
        axis.plot(window_x, current_split[window_idx], label="sent split", linewidth=1.0, alpha=0.85)
        axis.plot(window_x, target_split[window_idx], label="target split", linewidth=1.15)
        axis.plot(window_x, final_split[window_idx], label="final model split", linewidth=1.0)
        axis.set_xlabel("sample index inside active state2 window")
        axis.set_ylabel("right-left RPM")
        axis.set_title("State 2 Active Window")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
        fig.tight_layout()
        path = output_dir / "behavior_state2_window.png"
        fig.savefig(path, dpi=170)
        plt.close(fig)
        paths.append(path)

        # Smoothed version for the same window to show trend instead of spikes.
        smooth_window = min(25, max(5, window_idx.size // 20))
        fig, axis = plt.subplots(figsize=(10.5, 4.8))
        axis.plot(window_x, rolling_mean(current_split[window_idx], smooth_window), label="sent split mean", linewidth=1.3)
        axis.plot(window_x, rolling_mean(target_split[window_idx], smooth_window), label="target split mean", linewidth=1.3)
        axis.plot(window_x, rolling_mean(final_split[window_idx], smooth_window), label="final model split mean", linewidth=1.3)
        axis.set_xlabel("sample index inside active state2 window")
        axis.set_ylabel("right-left RPM")
        axis.set_title("State 2 Active Window (Smoothed)")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
        fig.tight_layout()
        path = output_dir / "behavior_state2_window_smoothed.png"
        fig.savefig(path, dpi=170)
        plt.close(fig)
        paths.append(path)

    return paths


def main() -> None:
    args = parse_args()
    run_dir = args.run_dir.resolve()
    sample_path = run_dir / "online_training_samples.csv"
    checkpoint_path = args.checkpoint or run_dir / "weights" / "corrected_controller_rpm_startpoint.pt"
    output_dir = (args.output_dir or run_dir / "behavior_analysis").resolve()

    rows = read_rows(sample_path)
    checkpoint = load_checkpoint(checkpoint_path)
    final_eval = evaluate_final_checkpoint(rows, checkpoint)
    valid = final_eval["valid"]

    current = np.column_stack([array(rows, "current_right_rpm"), array(rows, "current_left_rpm")])
    target = np.column_stack([array(rows, "target_right_rpm"), array(rows, "target_left_rpm")])
    logged = np.column_stack([array(rows, "pred_right_rpm"), array(rows, "pred_left_rpm")])
    final = final_eval["pred"]
    params = final_eval["params"]
    correction = final - current
    target_nudge = target - current
    split_correction = (final[:, 0] - final[:, 1]) - (current[:, 0] - current[:, 1])
    split_target_nudge = (target[:, 0] - target[:, 1]) - (current[:, 0] - current[:, 1])

    summary: dict[str, Any] = {
        "run_dir": str(run_dir),
        "checkpoint": str(checkpoint_path.resolve()),
        "output_dir": str(output_dir),
        "rows": len(rows),
        "valid_final_eval_rows": int(valid.sum()),
        "checkpoint_updates": int(checkpoint.get("updates", -1)),
        "checkpoint_samples": int(checkpoint.get("sample_rows", -1)),
        "training_halted": bool(checkpoint.get("training_halted", False)),
        "divergence_events": int(checkpoint.get("divergence_events", 0)),
        "fit_to_adaptive_target": {
            "sent_current": metric_block(target, current, valid),
            "logged_online_model": metric_block(target, logged, valid),
            "final_checkpoint": metric_block(target, final, valid),
        },
        "correction_delta_rpm": {
            "right_median_abs": percentile(np.abs(correction[valid, 0]), 50),
            "right_p95_abs": percentile(np.abs(correction[valid, 0]), 95),
            "left_median_abs": percentile(np.abs(correction[valid, 1]), 50),
            "left_p95_abs": percentile(np.abs(correction[valid, 1]), 95),
            "split_median_abs": percentile(np.abs(split_correction[valid]), 50),
            "split_p95_abs": percentile(np.abs(split_correction[valid]), 95),
            "target_split_p95_abs": percentile(np.abs(split_target_nudge[valid]), 95),
        },
        "final_affine_params": {
            "a_right_median": percentile(params[valid, 0], 50),
            "a_left_median": percentile(params[valid, 1], 50),
            "b_right_median": percentile(params[valid, 2], 50),
            "b_left_median": percentile(params[valid, 3], 50),
            "a_right_p05": percentile(params[valid, 0], 5),
            "a_right_p95": percentile(params[valid, 0], 95),
            "a_left_p05": percentile(params[valid, 1], 5),
            "a_left_p95": percentile(params[valid, 1], 95),
            "b_right_p05": percentile(params[valid, 2], 5),
            "b_right_p95": percentile(params[valid, 2], 95),
            "b_left_p05": percentile(params[valid, 3], 5),
            "b_left_p95": percentile(params[valid, 3], 95),
        },
        "rpm_range": {
            "final_max_abs_rpm": float(np.nanmax(np.abs(final))),
            "sent_max_abs_rpm": float(np.nanmax(np.abs(current))),
            "target_max_abs_rpm": float(np.nanmax(np.abs(target))),
        },
        "grouped_metrics": grouped_metrics(rows, target, current, final, valid),
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    paths = generate_plots(run_dir, rows, final_eval, summary)
    summary["plots"] = [str(path) for path in paths]
    write_summary(output_dir / "behavior_summary.json", summary)
    (output_dir / "behavior_summary.md").write_text(
        "# RPM Behavior Summary\n\n"
        f"- Rows: {summary['rows']}\n"
        f"- Valid final checkpoint rows: {summary['valid_final_eval_rows']}\n"
        f"- All rows split RMSE: sent {summary['grouped_metrics']['all']['sent']['rmse_split_rpm']:.3f} RPM, "
        f"final {summary['grouped_metrics']['all']['final']['rmse_split_rpm']:.3f} RPM\n"
        f"- Moving rows split RMSE: sent {summary['grouped_metrics']['moving']['sent']['rmse_split_rpm']:.3f} RPM, "
        f"final {summary['grouped_metrics']['moving']['final']['rmse_split_rpm']:.3f} RPM\n"
        f"- State2 split RMSE: sent {summary['grouped_metrics']['state_2']['sent']['rmse_split_rpm']:.3f} RPM, "
        f"final {summary['grouped_metrics']['state_2']['final']['rmse_split_rpm']:.3f} RPM\n"
        f"- State2 moving split RMSE: sent {summary['grouped_metrics']['state_2_moving']['sent']['rmse_split_rpm']:.3f} RPM, "
        f"final {summary['grouped_metrics']['state_2_moving']['final']['rmse_split_rpm']:.3f} RPM\n"
        f"- Final right/left RMSE on all rows: {summary['fit_to_adaptive_target']['final_checkpoint']['rmse_right_rpm']:.3f} / "
        f"{summary['fit_to_adaptive_target']['final_checkpoint']['rmse_left_rpm']:.3f} RPM\n"
        f"- Correction split p95: {summary['correction_delta_rpm']['split_p95_abs']:.3f} RPM\n"
        f"- Training halted: {summary['training_halted']}\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
