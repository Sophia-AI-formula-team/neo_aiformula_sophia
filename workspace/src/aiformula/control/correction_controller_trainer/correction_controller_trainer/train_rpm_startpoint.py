from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

try:
    from .online_rpm_trainer import OnlineRpmConfig, OnlineRpmTrainer
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from correction_controller_trainer.online_rpm_trainer import OnlineRpmConfig, OnlineRpmTrainer


REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_LOG_ROOT = REPO_ROOT / "CorrectionControl" / "Training" / "data"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "pid_ws" / "src" / "correction_controller_trainer" / "startpoint_weights"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train a runtime-compatible live RPM startpoint from local log CSVs.")
    parser.add_argument("--log-root", type=Path, default=DEFAULT_LOG_ROOT)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--states", default="s0,s1,s2")
    parser.add_argument("--passes", type=int, default=4)
    parser.add_argument("--learning-rate", type=float, default=5.0e-4)
    parser.add_argument("--adaptation-gain", type=float, default=0.35)
    parser.add_argument("--omega-adaptation-gain", type=float, default=4.5)
    parser.add_argument("--max-delta-rpm", type=float, default=360.0)
    parser.add_argument("--split-loss-weight", type=float, default=10.0)
    parser.add_argument("--omega-error-loss-weight", type=float, default=5.0)
    parser.add_argument("--divergence-loss-threshold", type=float, default=250000.0)
    parser.add_argument("--max-archived-checkpoints", type=int, default=20)
    return parser.parse_args()


def parse_states(raw: str) -> list[str]:
    states = [part.strip().lower() for part in raw.split(",") if part.strip()]
    bad = [state for state in states if state not in {"s0", "s1", "s2"}]
    if bad:
        raise ValueError(f"states must be s0, s1, or s2; got {bad}")
    return states or ["s0", "s1", "s2"]


def finite_float(raw: Any, default: float = math.nan) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def flag_true(raw: Any) -> bool:
    return str(raw).strip().lower() in {"1", "1.0", "true", "yes"}


def command_columns(row: dict[str, str]) -> tuple[str, str]:
    v_col = "cmd_ideal_v" if "cmd_ideal_v" in row else "cmd_v"
    w_col = "cmd_ideal_omega" if "cmd_ideal_omega" in row else "cmd_omega"
    return v_col, w_col


def current_rpm(row: dict[str, str], cmd_v: float, cmd_omega: float) -> tuple[float, float] | None:
    if flag_true(row.get("valid_debug")):
        base_v = finite_float(row.get("controller_base_v"))
        base_omega = finite_float(row.get("controller_base_omega"))
        right = finite_float(row.get("controller_can_right_rpm"))
        left = finite_float(row.get("controller_can_left_rpm"))
        if all(math.isfinite(value) for value in [base_v, base_omega, right, left]):
            if abs(base_v - cmd_v) <= 1.0e-4 and abs(base_omega - cmd_omega) <= 1.0e-4:
                return right, left

    if flag_true(row.get("valid_can")):
        right = finite_float(row.get("can_right_rpm"))
        left = finite_float(row.get("can_left_rpm"))
        if math.isfinite(right) and math.isfinite(left):
            return right, left
    return None


def iter_run_dirs(log_root: Path):
    if log_root.is_dir() and log_root.name.startswith("run_"):
        yield log_root
    for run_dir in sorted(path for path in log_root.glob("run_*") if path.is_dir()):
        yield run_dir


def iter_log_rows(log_root: Path, states: list[str]):
    for run_dir in iter_run_dirs(log_root):
        for state in states:
            state_dir = run_dir / state
            if not state_dir.exists():
                continue
            for path in sorted(state_dir.glob("log_*.csv")):
                with path.open("r", newline="", encoding="utf-8") as handle:
                    reader = csv.DictReader(handle)
                    for row in reader:
                        yield run_dir.name, state, path, row
        joy_dir = run_dir / "joy"
        if "s2" in states and joy_dir.exists():
            for path in sorted(joy_dir.glob("log_*.csv")):
                with path.open("r", newline="", encoding="utf-8") as handle:
                    reader = csv.DictReader(handle)
                    for row in reader:
                        yield run_dir.name, "s2", path, row


def train_startpoint(args: argparse.Namespace) -> dict[str, Any]:
    states = parse_states(args.states)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_root / f"run_{stamp}"
    config = OnlineRpmConfig(
        learning_rate=max(1.0e-6, float(args.learning_rate)),
        adaptation_gain=max(0.0, float(args.adaptation_gain)),
        omega_adaptation_gain=max(0.0, float(args.omega_adaptation_gain)),
        max_delta_rpm=max(1.0, float(args.max_delta_rpm)),
        split_loss_weight=max(0.0, float(args.split_loss_weight)),
        omega_error_loss_weight=max(0.0, float(args.omega_error_loss_weight)),
        divergence_loss_threshold=max(1.0, float(args.divergence_loss_threshold)),
        seed=31,
    )
    trainer = OnlineRpmTrainer(output_dir, config)

    total_rows = 0
    used_rows = 0
    accepted_samples = 0
    source_files: set[str] = set()

    for pass_index in range(max(1, int(args.passes))):
        last_path: Path | None = None
        last_global_time = pass_index * 100000.0
        file_base_time = last_global_time
        for run_name, state, path, row in iter_log_rows(args.log_root, states):
            if path != last_path:
                file_base_time = last_global_time + (10.0 if last_path is not None else 0.0)
                last_path = path
            total_rows += 1
            v_col, w_col = command_columns(row)
            timestamp = finite_float(row.get("timestamp"))
            cmd_v = finite_float(row.get(v_col))
            cmd_omega = finite_float(row.get(w_col))
            meas_v = finite_float(row.get("vn_body_vx"))
            meas_omega = finite_float(row.get("odom_omega_z"))
            rpm = current_rpm(row, cmd_v, cmd_omega)
            if rpm is None or not all(math.isfinite(value) for value in [timestamp, cmd_v, cmd_omega, meas_v, meas_omega]):
                continue
            used_rows += 1
            source_files.add(str(path))
            sample_time = file_base_time + timestamp
            last_global_time = max(last_global_time, sample_time)
            result = trainer.add_sample(
                timestamp=sample_time,
                segment=f"{run_name}/{state}",
                state=int(state[1]),
                cmd_v=cmd_v,
                cmd_omega=cmd_omega,
                meas_v=meas_v,
                meas_omega=meas_omega,
                current_right_rpm=rpm[0],
                current_left_rpm=rpm[1],
            )
            accepted_samples += int(result is not None)
        if trainer.training_halted:
            break

    if not trainer.training_halted:
        trainer.accept_current_checkpoint(archive=True, reason="offline_startpoint_final")
    trainer.save_artifacts(
        final=True,
        archive=False,
        evaluate_checkpoint=False,
        max_archived_checkpoints=max(1, int(args.max_archived_checkpoints)),
    )
    manifest = {
        "mode": "offline_runtime_rpm_startpoint",
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "log_root": str(args.log_root),
        "states": states,
        "passes": max(1, int(args.passes)),
        "total_rows_seen": total_rows,
        "used_rows": used_rows,
        "accepted_samples": accepted_samples,
        "updates": trainer.update_index,
        "training_halted": trainer.training_halted,
        "divergence_events": len(trainer.divergence_rows),
        "source_files": sorted(source_files),
        "outputs": {
            "run_dir": str(output_dir),
            "startpoint": str(output_dir / "weights" / "corrected_controller_rpm_startpoint.pt"),
            "final": str(output_dir / "weights" / "corrected_controller_rpm_final.pt"),
            "latest": str(output_dir / "weights" / "corrected_controller_rpm_latest.pt"),
            "checkpoints": str(output_dir / "weights" / "checkpoints"),
        },
    }
    (output_dir / "offline_startpoint_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


def main() -> None:
    manifest = train_startpoint(parse_args())
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
