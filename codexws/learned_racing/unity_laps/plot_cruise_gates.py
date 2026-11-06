"""Plot every preregistered empty-course trial from the frozen Unity holdout."""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager, pyplot as plt
import numpy as np


def plot(report_path, runs, destination):
    report = json.loads(report_path.read_bytes())
    cases = [c for c in report["pairs"] if c["family"] in ("clear", "full-clear")]
    segments = [c for c in cases if c["family"] == "clear"]
    laps = [c for c in cases if c["family"] == "full-clear"]
    if (len(segments), len(laps)) != (4, 2):
        raise ValueError("Expected the four clear sections and both clear laps")
    font = Path("C:/Windows/Fonts/msyh.ttc")
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(font)).get_name()
    bg, green, ink = "#f7f8f3", "#00775d", "#183e3d"
    fig = plt.figure(figsize=(11.7, 8.8), facecolor=bg)
    grid = fig.add_gridspec(4, 2, left=.066, right=.975, top=.845, bottom=.07,
                            hspace=1.15, wspace=.2)
    slots = [grid[i//2, i%2] for i in range(4)] + [grid[2, :], grid[3, :]]
    fig.text(.066, .952, "新起点实测：车在直道能跑多快？", fontsize=24, weight="bold", color=ink)
    fig.text(.066, .901, "绿线是预先划定直道内的实测车速；灰线保留起步和弯道。虚线标出 4 m/s。",
             fontsize=12, color="#4a625d")
    for case, slot in zip(segments + laps, slots):
        raw = (runs / (case["id"] + "_learned.json")).read_bytes()
        if hashlib.sha256(raw).hexdigest() != case["runSha256"][1]:
            raise ValueError("Source trace differs from the scored trace for " + case["id"])
        run = json.loads(raw)
        rows = np.asarray(run["rows"])
        distance, speed = rows[:, 7], rows[:, 4]
        arm = case["learned"]
        gates = arm["straightGates"]
        data = gates["totals"]
        ax = fig.add_subplot(slot, facecolor=bg)
        ax.axhline(4., ls="--", lw=1, color="#a6512d")
        ax.plot(distance, speed, color="#adb7b4", lw=1.4)
        for gate in gates["gates"]:
            mask = (distance >= gate["fromM"]) & (distance <= gate["toM"])
            ax.plot(distance[mask], speed[mask], color=green, lw=2.2)
        label = "空场整圈" if case["scope"] == "lap" else "空直道"
        state = "完成" if arm["physicalSuccess"] else "提前终止"
        mean = data["measuredBodyMeanMps"]
        fraction = data["fractionPlannedAtLeast4"]
        measured = "没有驶入测速区" if mean is None else f"直道实速均值 {mean:.2f} m/s，达速里程 {fraction:.0%}"
        ax.set_title(f"{case['id']}  {label} · {state}    {measured}", loc="left",
                     fontsize=10, color=ink, pad=7)
        ax.set_xlim(0, max(arm["targetM"], distance.max()))
        ax.set_ylim(0, max(5.1, speed.max()+.1))
        ax.set_yticks([0, 2, 4])
        ax.tick_params(colors="#506863", labelsize=9, length=2)
        ax.spines[["right", "top"]].set_visible(False)
        ax.spines[["bottom", "left"]].set_color("#acb9b3")
    fig.text(.5, .019, "从该起点沿赛道前进的距离（米）", ha="center", color=ink, fontsize=10)
    fig.savefig(destination, dpi=140, facecolor=bg)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("runs", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    plot(args.report, args.runs, args.output)
