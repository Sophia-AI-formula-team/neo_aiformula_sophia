"""Time sequential, camera-fed Jetson detector runs with small text evidence."""
from collections import Counter
from datetime import datetime
import json
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String

ROOT = Path(__file__).resolve().parent
TOPICS = {
    "camera": ("/aiformula_sensing/zed_node/left_image/undistorted", Image),
    "lane_mask": ("/aiformula_perception/road_detector/mask_image", Image),
    "lane_overlay": ("/aiformula_visualization/road_detector/annotated_mask_image", Image),
    "object_mask": ("/cone_mask", Image),
    "object_overlay": ("/cone_bbox_image", Image),
    "detections": ("/aiformula_perception/yolop_a3/detections", String),
    "traffic_light": ("/aiformula_perception/yolop_a3/traffic_light/state", String),
    "red_pixels": ("/aiformula_perception/yolop_a3/red_pixels_count", Int32),
}
FRAME_TOPICS = ("camera", "lane_mask", "lane_overlay", "object_mask", "object_overlay", "detections")
GPU_CLOCK = Path("/sys/class/devfreq/17000000.ga10b/cur_freq")
GPU = re.compile(r"\bGR3D_FREQ\s+(\d+)%")


def stamp():
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def note(message):
    line = f"{stamp()} {message}"
    print(line, flush=True)
    with (ROOT / "timeline.log").open("a") as log:
        print(line, file=log)


def start(command, name):
    with (ROOT / f"{name}.log").open("w") as log:
        return subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def stop(process, name):
    if process is None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            continue
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break
    process.wait(timeout=4)
    note(f"Stopped {name}; exit {process.returncode}.")


def read_counter(path):
    try:
        return int(path.read_text())
    except (OSError, ValueError):
        return None


def process_stats(group):
    if group is None:
        return {}
    ticks = rss = read_bytes = write_bytes = 0
    pids = subprocess.check_output(["ps", "-g", str(group), "-o", "pid="], text=True).split()
    for pid in pids:
        proc = Path("/proc") / pid
        try:
            stat = (proc / "stat").read_text().rsplit(")", 1)[1].split()
            if int(stat[2]) != group:
                continue
            ticks += int(stat[11]) + int(stat[12])
            rss += int(stat[21]) * os.sysconf("SC_PAGE_SIZE")
            io = dict(line.split(": ", 1) for line in (proc / "io").read_text().splitlines())
            read_bytes += int(io["read_bytes"])
            write_bytes += int(io["write_bytes"])
        except (OSError, ValueError, KeyError):
            continue
    return {"cpu_ticks": ticks, "rss_bytes_sum": rss, "read_bytes": read_bytes, "write_bytes": write_bytes}


def collect_gpu(acquired, samples, errors, group):
    try:
        with subprocess.Popen(["tegrastats", "--interval", "500"], stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True) as monitor:
            started = time.monotonic()
            acquired.set()
            try:
                for _ in range(20):
                    line = monitor.stdout.readline().strip()
                    match = GPU.search(line)
                    if not match:
                        raise RuntimeError(f"No GR3D_FREQ reading: {line}")
                    samples.put({"time_jst": stamp(), "elapsed_s": round(time.monotonic() - started, 3),
                                 "gpu_percent": int(match[1]), "gpu_clock_hz": read_counter(GPU_CLOCK),
                                 "process": process_stats(group), "raw": line})
            finally:
                monitor.terminate()
    except Exception as exc:
        errors.append(exc)
        acquired.set()


def wait_output(node, readings, expected, processes, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for process in processes:
            if process.poll() is not None:
                raise RuntimeError(f"Process {process.pid} exited with {process.returncode} during warmup")
        if all(readings[name] > 0 for name in expected):
            return
        rclpy.spin_once(node, timeout_sec=0.15)
    raise RuntimeError(f"Timeout waiting for these topics: {[name for name in expected if not readings[name]]}")


def measure(node, name, readings, image_shapes, detections, arrivals, detector=None):
    readings.clear()
    image_shapes.clear()
    detections.clear()
    arrivals.clear()
    acquired, samples, errors = threading.Event(), queue.Queue(), []
    worker = threading.Thread(target=collect_gpu, args=(acquired, samples, errors, detector.pid if detector else None), daemon=True)
    started = stamp()
    worker.start()
    if not acquired.wait(5):
        raise RuntimeError("tegrastats did not start")
    deadline = time.monotonic() + 13
    while worker.is_alive() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if worker.is_alive():
        raise RuntimeError("tegrastats timeout")
    if errors:
        raise errors[0]
    values = [samples.get_nowait() for _ in range(20)]
    if not 9.5 <= values[-1]["elapsed_s"] <= 11:
        raise RuntimeError(f"Invalid sampling window {values[-1]['elapsed_s']} s")
    result = {
        "name": name, "started_jst": started, "finished_jst": stamp(),
        "duration_s": values[-1]["elapsed_s"], "samples": values,
        "message_counts": dict(readings), "image_shapes": dict(image_shapes),
        "detection_lists": dict(detections), "frame_arrivals": dict(arrivals),
        "clock_ticks_per_s": os.sysconf("SC_CLK_TCK"),
        "publisher_counts": {key: node.count_publishers(topic) for key, (topic, _) in TOPICS.items()},
    }
    (ROOT / f"{name}.json").write_text(json.dumps(result, indent=2) + "\n")
    note(f"{name}: recorded 20 GPU samples across {result['duration_s']} s; messages: {dict(readings)}.")
    expected = ("camera",) if name == "camera-only" else TOPICS
    if any(readings[key] == 0 or result["publisher_counts"][key] != 1 for key in expected):
        raise RuntimeError(f"{name}: missing messages or duplicate publishers")
    if name != "camera-only" and detections["invalid_json"]:
        raise RuntimeError(f"{name}: invalid detection JSON")


def main():
    os.chdir(ROOT)
    rclpy.init()
    node = rclpy.create_node("gpu_load_benchmark")
    qos = QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT)
    readings, image_shapes, detections, arrivals = Counter(), {}, Counter(), {}

    def observe(key):
        def callback(message):
            readings[key] += 1
            source_ns = None
            if isinstance(message, Image):
                source_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
                image_shapes[key] = [message.width, message.height, message.encoding]
            if key == "detections":
                try:
                    payload = json.loads(message.data)
                except ValueError:
                    detections["invalid_json"] += 1
                    return
                if isinstance(payload, dict):
                    source = payload.get("header", {}).get("stamp", {})
                    if "sec" in source and "nanosec" in source:
                        source_ns = source["sec"] * 10**9 + source["nanosec"]
                objects = payload.get("detections") if isinstance(payload, dict) else payload
                if isinstance(objects, list):
                    detections["valid_lists"] += 1
                    detections["frames_with_detections"] += bool(objects)
                    detections["reported_objects"] += len(objects)
                else:
                    detections["other_json"] += 1
            if key in FRAME_TOPICS:
                now = time.time_ns()
                delay_ms = (now - source_ns) / 1e6 if source_ns else None
                arrivals.setdefault(key, []).append([time.monotonic_ns() / 1e6, delay_ms, source_ns])
        return callback

    for key, (topic, kind) in TOPICS.items():
        node.create_subscription(kind, topic, observe(key), qos)

    camera = detector = None
    try:
        camera = start(["ros2", "launch", str(ROOT / "camera_only.launch.py")], "camera")
        note("Started only the ZED camera in localhost ROS domain 193.")
        wait_output(node, readings, ("camera",), (camera,), 30)
        note(f"Camera frames ready: {image_shapes['camera']}. Starting 10 s camera-only baseline.")
        measure(node, "camera-only", readings, image_shapes, detections, arrivals)
        for profile, choice in (("a3-v14", "a3"), ("a4", "a4"), ("a4light", "a4l")):
            readings.clear()
            detector = start([str(ROOT.parent / "yolop.launch"), choice], profile)
            note(f"Started {profile}; waiting for camera, lane and object outputs.")
            wait_output(node, readings, TOPICS, (camera, detector), 45)
            note(f"{profile}: all eight input/output streams ready. Starting 10 s sampling.")
            measure(node, profile, readings, image_shapes, detections, arrivals, detector)
            if detector.poll() is not None:
                raise RuntimeError(f"{profile} exited during sampling")
            stop(detector, profile)
            detector = None
            rest = time.monotonic() + 1.5
            while time.monotonic() < rest:
                rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        stop(detector, "detector cleanup")
        stop(camera, "ZED camera")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
