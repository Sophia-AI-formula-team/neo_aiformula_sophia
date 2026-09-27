#!/usr/bin/env python3
"""Synthetic, deterministic closed-loop evaluation; never publishes ROS commands.

The plant is an ideal unicycle (no actuator latency/slip/GNSS error). These are
algorithm/unit-scale demonstrations, not vehicle-performance or safety proof.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from lane_mapping_lya_reference.controller import (ClosedRouteController, Command,
                                                   ControlFault, Pose, SafetyArbiter)
from lane_mapping_lya_reference.route import build_route


def stadium():
    """New 77.7 m stadium track with two straights and two 6 m-radius bends."""
    bottom = np.column_stack((np.linspace(-10, 10, 401, endpoint=False), np.full(401, -6.0)))
    angle = np.linspace(-math.pi / 2, math.pi / 2, 378, endpoint=False)
    right = np.column_stack((10 + 6 * np.cos(angle), 6 * np.sin(angle)))
    top = np.column_stack((np.linspace(10, -10, 401, endpoint=False), np.full(401, 6.0)))
    angle = np.linspace(math.pi / 2, 3 * math.pi / 2, 378, endpoint=False)
    left = np.column_stack((-10 + 6 * np.cos(angle), 6 * np.sin(angle)))
    center = np.vstack((bottom, right, top, left))
    tangent = np.roll(center, -1, axis=0) - np.roll(center, 1, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    yaw = np.arctan2(tangent[:, 1], tangent[:, 0])
    phase = np.linspace(0, 2 * math.pi, len(center), endpoint=False)
    # First-lap driver has a small smooth weave; map boundaries stay fixed.
    trace = center + (0.07 * np.sin(phase * 3))[:, None] * normal
    trajectory = np.column_stack((trace, yaw))
    trajectory = np.vstack((trajectory, trajectory[:1]))
    lanes = np.vstack((center + 1.3 * normal, center - 1.3 * normal))
    return trajectory, lanes


def simulate(route, speed, lateral_offset, heading_offset, laps=2):
    controller = ClosedRouteController(route, maximum_speed=speed)
    start = route["route_samples"][0]
    yaw = start["yaw_rad"] + heading_offset
    x = start["x_m"] - math.sin(start["yaw_rad"]) * lateral_offset
    y = start["y_m"] + math.cos(start["yaw_rad"]) * lateral_offset
    dt = 0.05
    arbiter = SafetyArbiter(maximum_speed=speed, safety_mode="fixed_only", stopped_settle_s=0.1)
    arbiter.prepare_repeat(controller)
    for ns in (1_000_000_000, 1_110_000_000):
        arbiter.update_pose(Pose(x, y, yaw, 0.0, ns, ns))
    assert arbiter.start_repeat(ns, ns)[0]
    previous = Command()
    rows, status, error = [], "timeout", None
    for index in range(int((controller.length * laps / speed + 30) / dt)):
        stamp = 1_110_000_000 + int(index * dt * 1e9)
        p = Pose(x, y, yaw, previous.speed, stamp, stamp)
        arbiter.update_pose(p)
        command = arbiter.command(stamp, stamp, dt)
        if arbiter.state != "REPEAT":
            status, error = "rejected", arbiter.reason
            break
        metrics = controller.last_metrics
        rows.append(dict(time_s=index * dt, x_m=x, y_m=y, yaw_rad=yaw,
                         speed_mps=command.speed, yaw_rate_rps=command.yaw_rate,
                         acceleration_mps2=(command.speed - previous.speed) / dt,
                         yaw_acceleration_rps2=(command.yaw_rate - previous.yaw_rate) / dt,
                         **metrics))
        # Exact constant-twist integration avoids Euler drift in this ideal plant.
        if abs(command.yaw_rate) > 1e-8:
            angle = yaw + command.yaw_rate * dt
            x += command.speed / command.yaw_rate * (math.sin(angle) - math.sin(yaw))
            y += command.speed / command.yaw_rate * (math.cos(yaw) - math.cos(angle))
            yaw = angle
        else:
            x += command.speed * math.cos(yaw) * dt
            y += command.speed * math.sin(yaw) * dt
        previous = command
        if controller.progress >= controller.length * laps:
            status = "completed"
            break
    distances = np.asarray([row["cross_track_m"] for row in rows])
    progress = np.asarray([row["progress_m"] for row in rows])
    advances = np.diff(progress)
    steady = distances[np.asarray([row["time_s"] for row in rows]) >= 10.0]
    report = dict(status=status, error=error, speed_cap_mps=speed,
                  initial_lateral_offset_m=lateral_offset,
                  initial_heading_offset_rad=heading_offset,
                  requested_laps=laps, completed_laps=controller.progress / controller.length,
                  duration_s=rows[-1]["time_s"] if rows else 0,
                  cross_track_max_m=float(np.max(distances)) if len(rows) else None,
                  cross_track_p95_m=float(np.percentile(distances, 95)) if len(rows) else None,
                  cross_track_rmse_m=float(np.sqrt(np.mean(distances ** 2))) if len(rows) else None,
                  after_10s_cross_track_p95_m=float(np.percentile(steady, 95)) if len(steady) else None,
                  progress_is_monotonic=bool(np.all(advances >= -1e-9)),
                  maximum_progress_advance_m=float(np.max(advances)) if len(advances) else 0,
                  branch_jump_gate_pass=bool(np.all(advances <= speed * dt + 0.15 + 1e-9)),
                  peak_speed_mps=max([abs(row["speed_mps"]) for row in rows], default=0),
                  peak_yaw_rate_rps=max([abs(row["yaw_rate_rps"]) for row in rows], default=0),
                  peak_acceleration_mps2=max([abs(row["acceleration_mps2"]) for row in rows], default=0),
                  peak_yaw_acceleration_rps2=max([abs(row["yaw_acceleration_rps2"]) for row in rows], default=0))
    report["command_limits_pass"] = (report["peak_speed_mps"] <= speed + 1e-9
        and report["peak_yaw_rate_rps"] <= 0.4 + 1e-9
        and report["peak_acceleration_mps2"] <= 0.5 + 1e-9
        and report["peak_yaw_acceleration_rps2"] <= 0.8 + 1e-9)
    return report, rows


def safety_cases(route):
    start = route["route_samples"][0]

    def p(ns, x=None, yaw=None):
        return Pose(start["x_m"] if x is None else x, start["y_m"],
                    start["yaw_rad"] if yaw is None else yaw, 0.0, ns, ns)

    results = {}
    for label, bad_pose in (("start_off_route_rejected", p(1, x=start["x_m"] + 5)),
                            ("start_wrong_heading_rejected", p(1, yaw=start["yaw_rad"] + math.pi))):
        try:
            ClosedRouteController(route).attach(bad_pose)
            results[label] = False
        except ControlFault:
            results[label] = True
    for mode in ("fixed_only", "lya_reference"):
        a = SafetyArbiter(safety_mode=mode, stopped_settle_s=0.1)
        a.prepare_repeat(ClosedRouteController(route))
        for ns in (1_000_000_000, 1_110_000_000):
            a.update_pose(p(ns))
            a.set_teacher((0.8, 0, 0, 0, 0, -0.4), ns, ns)
        assert a.start_repeat(ns, ns)[0]
        command = a.command(ns, ns, 0.05)
        if mode == "lya_reference":
            results["reference_divergence_latches_fallback"] = (a.state == "FALLBACK"
                and 0 < command.speed <= 0.025 + 1e-9 and -0.04 - 1e-9 <= command.yaw_rate < 0)
            results["reference_fallback_stops_on_pose_loss"] = (a.command(ns + 500_000_000, ns + 500_000_000, 0.05) == Command() and a.state == "HOLD")
        else:
            results["fixed_does_not_forward_teacher"] = command != a.teacher
            a.set_teacher((float("nan"),) * 6, ns, ns)
            results["fixed_ignores_invalid_teacher_after_handover"] = a.state == "REPEAT"
            a.estop("demo")
            results["manual_estop_zero"] = a.command(ns, ns, 0.05) == Command()
            a.reset_estop()
            results["reset_does_not_rearm"] = a.state == "DISARMED" and a.command(ns, ns, 0.05) == Command()
    for label, desired_speed, teacher_omega in (("slower_fixed_bend_does_not_false_fallback", 0.2, 0.0),
                                               ("yaw_fallback_preserves_candidate_slowdown", 0.4, -0.4)):
        candidate_route = json.loads(json.dumps(route))
        for row in candidate_route["route_samples"]:
            row["speed_mps"] = desired_speed
        a = SafetyArbiter(safety_mode="lya_reference", stopped_settle_s=0.1)
        a.prepare_repeat(ClosedRouteController(candidate_route))
        for ns in (1_000_000_000, 1_110_000_000):
            a.update_pose(p(ns))
            a.set_teacher((0.8, 0, 0, 0, 0, teacher_omega), ns, ns)
        assert a.start_repeat(ns, ns)[0]
        outputs = []
        for index in range(40):
            ns = 1_110_000_000 + index * 50_000_000
            a.update_pose(p(ns))
            a.set_teacher((0.8, 0, 0, 0, 0, teacher_omega), ns, ns)
            outputs.append(a.command(ns, ns, 0.05))
        results[label] = (a.state == ("REPEAT" if teacher_omega == 0 else "FALLBACK")
                          and max(command.speed for command in outputs) <= desired_speed + 1e-9)
    return results


def arbitration_output_cases(route):
    """Measure the final arbiter, not only the inner controller's limits."""
    start = route["route_samples"][0]
    reports, rows = {}, []
    for mode in ("TEACH", "REPEAT", "FALLBACK"):
        a = SafetyArbiter(safety_mode="lya_reference", stopped_settle_s=0.1)
        if mode != "TEACH":
            a.prepare_repeat(ClosedRouteController(route))
        for ns in (1_000_000_000, 1_110_000_000):
            a.update_pose(Pose(start["x_m"], start["y_m"], start["yaw_rad"], 0.0, ns, ns))
            a.set_teacher((0.8, 0, 0, 0, 0, 0), ns, ns)
        assert (a.arm(ns, ns) if mode == "TEACH" else a.start_repeat(ns, ns))[0]
        previous, peak_accel, peak_yaw_accel = Command(), 0.0, 0.0
        first_state = None
        for index in range(40):
            ns = 1_110_000_000 + index * 50_000_000
            a.update_pose(Pose(start["x_m"], start["y_m"], start["yaw_rad"], 0.0, ns, ns))
            a.set_teacher((0.8, 0, 0, 0, 0, -0.4 if mode == "FALLBACK" else 0.0), ns, ns)
            output = a.command(ns, ns, 0.05)
            peak_accel = max(peak_accel, abs(output.speed - previous.speed) / 0.05)
            peak_yaw_accel = max(peak_yaw_accel, abs(output.yaw_rate - previous.yaw_rate) / 0.05)
            first_state = first_state or a.state
            rows.append(dict(mode=mode, step=index, state=a.state, speed_mps=output.speed,
                             yaw_rate_rps=output.yaw_rate, teacher_speed_mps=a.teacher.speed,
                             teacher_yaw_rate_rps=a.teacher.yaw_rate))
            previous = output
        a.set_teacher((0, 0, 0, 0, 0, 0), ns, ns)
        immediate_zero = a.command(ns, ns, 0.05) == Command()
        reports[mode] = dict(first_state=first_state, expected_first_state=mode,
                            peak_normal_acceleration_mps2=peak_accel,
                            peak_normal_yaw_acceleration_rps2=peak_yaw_accel,
                            teacher_stop_is_immediate_zero=immediate_zero,
                            passed=first_state == mode and peak_accel <= 0.5 + 1e-9
                            and peak_yaw_accel <= 0.8 + 1e-9 and immediate_zero)
    return reports, rows


def run(output):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    trace, lanes = stadium()
    route = build_route(trace, lanes)
    (output / "route.json").write_text(json.dumps(route, indent=2, allow_nan=False), encoding="utf-8")
    reports, trajectories = {}, {}
    for speed in (0.4, 0.8):
        for offset, heading in ((0.0, 0.0), (0.25, 0.12)):
            name = "speed_{}_offset_{}".format(speed, offset)
            report, rows = simulate(route, speed, offset, heading)
            reports[name], trajectories[name] = report, rows
            with (output / (name + ".csv")).open("x", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
    safety = safety_cases(route)
    arbitration, command_rows = arbitration_output_cases(route)
    with (output / "final_arbiter_commands.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(command_rows[0]))
        writer.writeheader()
        writer.writerows(command_rows)
    result = dict(evidence="synthetic ideal-unicycle closed-loop, not vehicle validation",
                  route_source="new stadium synthetic first-lap trace + ideal observed lane boundaries",
                  source_sha256={name: hashlib.sha256((PACKAGE / "lane_mapping_lya_reference" / name).read_bytes()).hexdigest()
                                 for name in ("controller.py", "route.py")},
                  route_diagnostics=route["diagnostics"], scenarios=reports,
                  safety=safety, final_arbiter=arbitration,
                  all_pass=all(safety.values()) and all(r["passed"] for r in arbitration.values())
                          and all(r["status"] == "completed" and r["command_limits_pass"]
                          and r["progress_is_monotonic"] and r["branch_jump_gate_pass"] for r in reports.values()))
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
        route_xy = np.asarray([[row["x_m"], row["y_m"]] for row in route["route_samples"]])
        for ax, (name, rows) in zip(axes.flat, trajectories.items()):
            ax.scatter(lanes[:, 0], lanes[:, 1], s=0.8, color="silver", label="observed lane boundary")
            ax.plot(route_xy[:, 0], route_xy[:, 1], color="black", linewidth=1.3, label="built fixed route")
            ax.plot([row["x_m"] for row in rows], [row["y_m"] for row in rows], color="#087fa6", linewidth=1.0, label="2-lap closed-loop plant")
            report = reports[name]
            ax.set_title("{} | {} | p95 {:.3f}m".format(name, report["status"], report["cross_track_p95_m"]))
            ax.set_aspect("equal")
            ax.set(xlabel="ENU x [m]", ylabel="ENU y [m]", xlim=(-19, 19), ylim=(-10, 10))
            ax.grid(alpha=0.2)
            ax.legend(fontsize=7, loc="upper right")
        fig.suptitle("Synthetic control-only evaluation: ideal plant, no real vehicle / ROS / GNSS error")
        fig.savefig(output / "closed_loop.png", dpi=150)
        plt.close(fig)
    except ImportError:
        result["plot"] = "matplotlib unavailable; CSV/JSON remain authoritative"
    (output / "metrics.json").write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result["all_pass"] else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=str(PACKAGE / ".artifacts" / "control_demo"))
    raise SystemExit(run(parser.parse_args().output))
