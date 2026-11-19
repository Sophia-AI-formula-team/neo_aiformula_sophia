from pathlib import Path
import json
import statistics
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

root = Path(sys.argv[1])
evidence = root / "evidence"
names = ("camera-only", "a3-v14", "a4", "a4light")
labels = ("Camera only", "A3 V14", "A4", "A4Light")
colors = ("#6B7280", "#096A70", "#BA542B", "#445AAA")
runs = [json.loads((evidence / f"{name}.json").read_text()) for name in names]
samples = [[sample["gpu_percent"] for sample in run["samples"]] for run in runs]
averages = [statistics.mean(series) for series in samples]

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "text.color": "#172033",
                     "axes.labelcolor": "#334155", "axes.edgecolor": "#CBD5E1",
                     "xtick.color": "#334155", "ytick.color": "#334155"})
fig, (line, bars) = plt.subplots(1, 2, figsize=(10, 4.6), gridspec_kw={"width_ratios": [1.8, 1]})
fig.subplots_adjust(left=.09, right=.96, top=.76, bottom=.23, wspace=.42)
fig.text(.09, .92, "Jetson GPU utilization", fontsize=21, fontweight="bold")
fig.text(.09, .85, "2026-11-19 JST  ·  Live ZED camera  ·  20 samples per run at 500 ms intervals",
         fontsize=9.5, color="#475569")
for label, color, run, series in zip(labels, colors, runs, samples):
    x = [sample["elapsed_s"] for sample in run["samples"]]
    line.plot(x, series, label=label, color=color, lw=1.8, marker="o", ms=2.7, alpha=.93)
line.set(xlabel="Elapsed time (s)", ylabel="GPU utilization (%)", ylim=(-3, 105),
         xlim=(0, 10.55), xticks=np.arange(0, 11, 2))
line.legend(loc="upper left", bbox_to_anchor=(-.04, -.25), ncol=2, frameon=False, fontsize=9)
positions = np.arange(len(labels))
bars.barh(positions, averages, color=colors, height=.58)
bars.set(yticks=positions, yticklabels=labels, xlabel="Mean GPU utilization (%)", xlim=(0, 113))
bars.invert_yaxis()
for i, value in enumerate(averages):
    bars.text(value + 2, i, f"{value:.1f}%", va="center", fontweight="bold", fontsize=9)
for ax, axis in ((line, "y"), (bars, "x")):
    ax.spines[["right", "top"]].set_visible(False)
    ax.grid(axis=axis, color="#E2E8F0", linewidth=.75)
    ax.set_axisbelow(True)
fig.savefig(root / "gpu-load.png", dpi=150)
plt.close(fig)

def hhmmss(iso):
    return iso.split("T", 1)[1].split("+", 1)[0]


def count(run, key):
    return run["message_counts"].get(key, 0)


power = (evidence / "power-mode.log").read_text().strip().splitlines()
power = next((x.split(":", 1)[1].strip() for x in power if x.startswith("NV Power Mode:")), "未读到")
report = [
    "# 2026-11-19 YOLOP 车端实验报告",
    "",
    "## GPU 负载与物体检测",
    "",
    f"同一台 Jetson、同一台 ZED 相机、固定 640×360 输入，按 A3 V14 → A4 → A4Light 顺序各测一轮。电源模式为 `{power}`。关闭整车 ROS 应用后，仅启动相机和当轮识别节点，使用 localhost ROS 域 193。每轮收到相机、车道和物体输出后开始计时。",
    "",
    "![Jetson GPU 负载曲线与各轮平均值](gpu-load.png)",
    "",
    "| 模式 | 开始时间 JST | 采样秒数 | GPU 均值 | 中位数 | 最低–最高 | 相机帧 | 车道掩膜 | 物体掩膜 | 物体标注图 | 检测 JSON | 有检测的帧 |",
    "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
]
for label, run, series, mean in zip(labels, runs, samples, averages):
    objects = run["detection_lists"].get("frames_with_detections", 0)
    report.append(
        f"| {label} | {hhmmss(run['started_jst'])} | {run['duration_s']:.3f} | "
        f"{mean:.1f}% | {statistics.median(series):.1f}% | {min(series)}–{max(series)}% | "
        f"{count(run, 'camera')} | {count(run, 'lane_mask')} | {count(run, 'object_mask')} | "
        f"{count(run, 'object_overlay')} | {count(run, 'detections')} | {objects} |"
    )

faster = min(range(1, 4), key=lambda idx: averages[idx])
delta = averages[2] - averages[3]
if delta >= 0:
    comparison = f"A4Light 比 A4 低 {delta:.1f} 个百分点。"
else:
    comparison = f"A4Light 比 A4 高 {-delta:.1f} 个百分点。"
report += [
    "",
    f"本轮 GPU 均值最低的识别模式为 **{labels[faster]}（{averages[faster]:.1f}%）**。{comparison} 每轮采集 20 个 `tegrastats` `GR3D_FREQ` 样本，间隔 500 ms。该指标是整机 GPU 活跃率，包含相机和系统背景开销。相机单独运行一轮作为参照。",
    "",
    "三档均确认了车道掩膜、车道标注图、物体掩膜、物体标注图、检测 JSON 和两路信号灯话题，采样结束时每个话题均只有一个发布者。物体栏记录检测器上报的非空列表。现场物体没有人工标注，因此本次测试范围是 GPU 负载和实时接线。",
    "",
    "## 今日时间线（JST）",
    "",
    "- **09:34–09:47** 准备并核验 A3 V14、A4、A4Light 三档，完成 A4 车辆话题的隔离 DDS 检查。",
    "- **09:55:54** 安装并核验终端命令 `yolop`，可打开三档选择菜单。",
    "- **09:59:06** 将 A4Light 接到与 A4 相同的 7 个车辆感知话题；09:59:58 完成替代推理的本地隔离 DDS 检查。",
    "- **10:03 起** 停止原先运行的 allnodes 和 A4。使用真实 ZED 图像顺序测量三档。测试后关闭相机和测试节点。",
    "",
    "## 随包材料",
    "",
    "- `evidence/` 含按 JST 时间戳记录的 GPU 原始样本、各 ROS 话题计数和运行时间线。每个样本保留 `tegrastats` 原行。",
    "- `scripts/` 含相机专用启动文件、测量脚本和制图脚本。归档仅含报告、PNG、文本证据和脚本。",
    "",
]
(root / "README.md").write_text("\n".join(report))
