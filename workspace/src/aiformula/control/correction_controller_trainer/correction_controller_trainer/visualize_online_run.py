from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Regenerate online RPM trainer plots from a run directory.")
    parser.add_argument("run_dir", nargs="?", type=Path, default=None)
    parser.add_argument("--root", type=Path, default=Path("~/Desktop/correctioncontrol_temp").expanduser())
    return parser.parse_args()


def latest_run(root: Path) -> Path:
    candidates = sorted(path for path in root.glob("run_*") if path.is_dir())
    if not candidates:
        raise FileNotFoundError(f"No run_* directories found under {root}")
    return candidates[-1]


def read_rows(path: Path) -> list[dict[str, float | str]]:
    if not path.exists():
        raise FileNotFoundError(f"Missing online training sample CSV: {path}")
    rows: list[dict[str, float | str]] = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            row: dict[str, float | str] = {}
            for key, value in raw.items():
                if key in ("segment", "state"):
                    row[key] = value
                    continue
                try:
                    row[key] = float(value)
                except (TypeError, ValueError):
                    row[key] = math.nan
            rows.append(row)
    if not rows:
        raise RuntimeError(f"No rows found in {path}")
    return rows


def column(rows: list[dict[str, float | str]], name: str) -> np.ndarray:
    return np.asarray([float(row.get(name, math.nan)) for row in rows], dtype=np.float64)


def plot_run(run_dir: Path) -> list[Path]:
    import matplotlib.pyplot as plt

    rows = read_rows(run_dir / "online_training_samples.csv")
    plots_dir = run_dir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)

    time_values = column(rows, "timestamp")
    rel_time = time_values - time_values[0]
    cmd_v = column(rows, "cmd_v")
    cmd_omega = column(rows, "cmd_omega")
    meas_v = column(rows, "meas_v")
    meas_omega = column(rows, "meas_omega")
    current_right = column(rows, "current_right_rpm")
    current_left = column(rows, "current_left_rpm")
    target_right = column(rows, "target_right_rpm")
    target_left = column(rows, "target_left_rpm")
    pred_right = column(rows, "pred_right_rpm")
    pred_left = column(rows, "pred_left_rpm")
    ideal_right = column(rows, "ideal_right_rpm")
    ideal_left = column(rows, "ideal_left_rpm")

    paths: list[Path] = []

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
    fig.tight_layout()
    path = plots_dir / "tracking_cmd_vs_measured.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(path)

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
    fig.tight_layout()
    path = plots_dir / "rpm_target_vs_model.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(path)

    fig, axis = plt.subplots(figsize=(11, 4.8))
    axis.plot(rel_time, ideal_right - ideal_left, label="ideal rpm split", alpha=0.8)
    axis.plot(rel_time, current_right - current_left, label="current rpm split", alpha=0.8)
    axis.plot(rel_time, target_right - target_left, label="target rpm split")
    axis.plot(rel_time, pred_right - pred_left, label="model rpm split")
    axis.set_xlabel("run time [s]")
    axis.set_ylabel("right-left RPM")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    fig.tight_layout()
    path = plots_dir / "rpm_split.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths.append(path)

    return paths


def main() -> None:
    args = parse_args()
    run_dir = args.run_dir.expanduser() if args.run_dir else latest_run(args.root.expanduser())
    paths = plot_run(run_dir)
    for path in paths:
        print(path)


if __name__ == "__main__":
    main()
