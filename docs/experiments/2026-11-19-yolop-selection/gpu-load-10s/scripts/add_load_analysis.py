from pathlib import Path
import json
import re
import statistics
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

root = Path(sys.argv[1])
names = ("camera-only", "a3-v14", "a4", "a4light")
labels = ("相机基线", "A3 V14", "A4", "A4Light")
runs = [json.loads((root / "evidence/extended" / f"{name}.json").read_text()) for name in names]


def extract(pattern, rows):
    return [float(re.search(pattern, row)[1]) for row in rows]


def fmt(value):
    return f"{value:.1f}" if value is not None else "—"


def frame_metrics(run, key):
    frames = run["frame_arrivals"].get(key, [])
    gaps = np.diff([frame[0] for frame in frames])
    delays = [frame[1] for frame in frames if frame[1] is not None and frame[1] >= 0]
    stamps = {frame[2] for frame in frames if frame[2]}
    return {
        "count": len(frames), "p95_gap": np.percentile(gaps, 95) if len(gaps) else None,
        "max_gap": max(gaps) if len(gaps) else None,
        "long_gaps": sum(gaps > 133.3), "median_delay": statistics.median(delays) if delays else None,
        "p95_delay": np.percentile(delays, 95) if delays else None,
        "max_delay": max(delays) if delays else None, "unique_stamps": len(stamps),
    }


resource_lines = []
resource_heading = (
    "| 模式 | GPU 均值 | GPU 实测频率范围 MHz | 整机 CPU 均值 / 峰值 | 最忙单核峰值 | "
    "识别进程 CPU | 内存峰值 / 识别进程 RSS 峰值 | Swap 峰值 | GPU / CPU 最高温 | 识别磁盘读 / 写 |"
)
for label, run in zip(labels, runs):
    samples = run["samples"]
    rows = [sample["raw"] for sample in samples]
    cpu_per_sample = [
        [float(x) for x in re.findall(r"(\d+)%@", re.search(r"CPU \[([^\]]+)\]", raw)[1])]
        for raw in rows
    ]
    cpu_means = [statistics.mean(values) for values in cpu_per_sample]
    cpu_peak = max(max(values) for values in cpu_per_sample)
    ram = max(extract(r"\bRAM (\d+)/", rows))
    swap = max(extract(r"\bSWAP (\d+)/", rows))
    gpu_temp = max(extract(r"\bGPU@(-?[\d.]+)C", rows))
    cpu_temp = max(extract(r"\bCPU@(-?[\d.]+)C", rows))
    gpu_mean = statistics.mean(sample["gpu_percent"] for sample in samples)
    clock = [sample["gpu_clock_hz"] / 1e6 for sample in samples if sample["gpu_clock_hz"] is not None]
    clock_range = f"{min(clock):.0f}–{max(clock):.0f}" if clock else "—"
    stats = [sample["process"] for sample in samples if sample["process"]]
    if stats and any(s["rss_bytes_sum"] for s in stats):
        elapsed = samples[-1]["elapsed_s"] - samples[0]["elapsed_s"]
        process_cpu = (stats[-1]["cpu_ticks"] - stats[0]["cpu_ticks"]) / run["clock_ticks_per_s"] / elapsed * 100
        rss = max(s["rss_bytes_sum"] for s in stats) / 2**20
        io_read = (stats[-1]["read_bytes"] - stats[0]["read_bytes"]) / 2**10
        io_write = (stats[-1]["write_bytes"] - stats[0]["write_bytes"]) / 2**10
        process_label, rss_label, io_label = f"{process_cpu:.1f}%", f"{rss:.0f} MiB", f"{io_read:.0f} / {io_write:.0f} KiB"
    else:
        process_label = rss_label = io_label = "—"
    resource_lines.append(f"| {label} | {gpu_mean:.1f}% | {clock_range} | "
                          f"{statistics.mean(cpu_means):.1f}% / {max(cpu_means):.1f}% | {cpu_peak:.0f}% | "
                          f"{process_label} | {ram:.0f} MB / {rss_label} | {swap:.0f} MB | "
                          f"{gpu_temp:.1f}°C / {cpu_temp:.1f}°C | {io_label} |")

gap_keys = ("camera", "lane_mask", "object_mask", "object_overlay", "detections")
gap_labels = ("相机", "车道掩膜", "物体掩膜", "物体标注图", "检测 JSON")
all_frames = [{key: frame_metrics(run, key) for key in gap_keys} for run in runs]
frame_lines = []
for label, run, metrics in zip(labels, runs, all_frames):
    for key, topic_label in zip(gap_keys, gap_labels):
        values = metrics[key]
        if values["count"] == 0:
            continue
        fps = values["count"] / run["duration_s"]
        frame_lines.append(f"| {label} | {topic_label} | {fps:.1f} | {fmt(values['p95_gap'])} | "
                           f"{fmt(values['max_gap'])} | {values['long_gaps']} | "
                           f"{fmt(values['p95_delay'])} | {fmt(values['max_delay'])} |")

fig, (left, right) = plt.subplots(1, 2, figsize=(10, 4.35))
fig.subplots_adjust(left=.09, right=.97, top=.76, bottom=.22, wspace=.32)
fig.text(.09, .92, "Frame timing on Jetson", fontsize=21, fontweight="bold", color="#172033")
fig.text(.09, .85, "Second pass  ·  Same live ZED feed  ·  One 10-second window per profile",
         fontsize=9.5, color="#475569")
x = np.arange(3)
for offset, (key, label, color) in enumerate((
    ("camera", "Camera", "#7C879A"), ("object_mask", "Object mask", "#096A70"),
    ("object_overlay", "Object overlay", "#BA542B"), ("detections", "Detections JSON", "#445AAA"),
)):
    midpoint = x + (offset - 1.5) * .18
    for axis, metric in ((left, "max_gap"), (right, "p95_delay")):
        points = [run[key][metric] if run[key][metric] is not None else np.nan for run in all_frames[1:]]
        axis.bar(midpoint, points, width=.16, color=color, label=label)
left.axhline(133.3, color="#475569", linewidth=.85, linestyle="--")
for axis, label in ((left, "Maximum message gap (ms)"), (right, "P95 source stamp to receipt (ms)")):
    axis.set(xticks=x, xticklabels=labels[1:], ylabel=label)
    axis.spines[["right", "top"]].set_visible(False)
    axis.grid(axis="y", color="#E2E8F0", linewidth=.75)
    axis.set_axisbelow(True)
handles, legend_labels = left.get_legend_handles_labels()
fig.legend(handles, legend_labels, loc="lower center", ncol=4, frameon=False, bbox_to_anchor=(.5, .045))
fig.savefig(root / "frame-timing.png", dpi=150)
plt.close(fig)

emc = [extract(r"\bEMC_FREQ (\d+)%", [s["raw"] for s in run["samples"]]) for run in runs]
if any(any(row) for row in emc):
    emc_summary = " / ".join(f"{label} {statistics.mean(row):.1f}%（峰值 {max(row):.0f}%）" for label, row in zip(labels, emc))
    emc_text = f"EMC 活跃率均值和峰值为 {emc_summary}。"
else:
    emc_text = "四组 EMC 遥测的读数均为 0%，本轮无法据此判断实际内存带宽。"
largest_detection_gap = max((metrics["detections"]["max_gap"], label) for metrics, label in zip(all_frames[1:], labels[1:]))
largest_overlay_gap = max((metrics["object_overlay"]["max_gap"], label) for metrics, label in zip(all_frames[1:], labels[1:]))
camera_max = max(metrics["camera"]["max_gap"] for metrics in all_frames)
timing_summary = (
    f"补测中，跨三档的最长检测 JSON 间隔为 **{largest_detection_gap[0]:.1f} ms（{largest_detection_gap[1]}）**，"
    f"最长物体标注图间隔为 **{largest_overlay_gap[0]:.1f} ms（{largest_overlay_gap[1]}）**，"
    f"相机最长间隔为 {camera_max:.1f} ms。"
)
section = [
    "## 负载与卡顿补测",
    "",
    "追加一轮相同顺序、相同相机设置的 10 秒采样。整机 CPU 为所有在线核心的平均活跃率，识别进程 CPU 以一个逻辑核为 100%。RSS 是进程组中各进程 RSS 的合计，可能重复计入共享页。磁盘读写是采样期内识别进程组的实际 I/O 差值，启动和模型加载已在计时前完成。",
    "",
    resource_heading,
    "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    *resource_lines,
    "",
    f"{emc_text} 可用的采样输出没有功率读数，因此未报告瓦数。",
    "",
    timing_summary,
    "",
    "![物体输出帧间隔及源时间戳至接收时间](frame-timing.png)",
    "",
    "| 模式 | 话题 | 接收 FPS | 间隔 P95 ms | 最大间隔 ms | >133.3 ms 次数 | 接收延迟 P95 ms | 最大接收延迟 ms |",
    "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    *frame_lines,
    "",
    "相机设为 15 FPS，两帧周期约 133.3 ms。间隔由同机 ROS 订阅端测量，延迟是消息源时间戳与本机收到消息的差值，包含相机/DDS/回调调度开销。图像采用 best-effort 订阅，订阅端的间隔也会受接收队列影响。各模式分时测量，因此物体数量会随场景变化。本报告的短时采样不覆盖整车节点同时运行时的长期卡顿。",
    "",
]
report = root / "README.md"
text = report.read_text().replace("## 今日时间线（JST）", "\n".join(section) + "\n## 今日时间线（JST）")
start = runs[0]["started_jst"].split("T")[1].split(".")[0]
end = runs[-1]["finished_jst"].split("T")[1].split(".")[0]
text = text.replace("## 随包材料", f"- **{start}–{end}** 完成负载与帧时序补测。\n\n## 随包材料")
report.write_text(text)
