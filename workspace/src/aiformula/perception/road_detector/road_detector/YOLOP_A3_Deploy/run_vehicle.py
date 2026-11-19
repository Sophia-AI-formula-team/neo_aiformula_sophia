"""Run a packaged A4 profile with the neo camera and perception topics."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parent
INPUT = "/aiformula_sensing/zed_node/left_image/undistorted"
NAMESPACE = "/aiformula_perception"
WEIGHT_SHA256 = "b4d3556ded29ada232682b607a11296d723ffe74814a85cc4f2bcbf573e5b1e3"
ROUTES = {
    "pub_mask_image": ("/aiformula_perception/road_detector/mask_image", "lane_mask"),
    "pub_annotated_mask_image": ("/aiformula_visualization/road_detector/annotated_mask_image", "lane_overlay"),
    "/cone_mask": ("/cone_mask", "obstacle_mask"),
    "/cone_bbox_image": ("/cone_bbox_image", "object_overlay"),
    "/aiformula_perception/yolop_a3/detections": ("/aiformula_perception/yolop_a3/detections", "detections"),
    "/traffic_light/state": ("/aiformula_perception/yolop_a3/traffic_light/state", "traffic_light_state"),
    "/red_pixels_count": ("/aiformula_perception/yolop_a3/red_pixels_count", "red_pixels_count"),
}


def routes(live):
    return {source: canonical if live else "/a3_shadow/" + shadow
            for source, (canonical, shadow) in ROUTES.items()}


def profile(path, expected):
    config = json.loads(path.read_text(encoding="utf-8"))
    if config != {"name": expected, "lean_runtime": expected == "A4_lite"}:
        raise ValueError("Installed profile.json does not match " + expected)
    return config


def node_args(args, config, weights):
    result = [str(__file__), "--ros-args", "-r", "__ns:=" + NAMESPACE,
              "-p", "weights:=" + str(weights), "-p", "device:=" + args.device,
              "-p", "score_threshold:=" + str(args.score_threshold),
              "-p", "input_image_topic:=" + args.input_image_topic,
              "-p", "use_sim_time:=" + args.use_sim_time,
              "-p", "lean_runtime:=" + str(config["lean_runtime"]).lower()]
    for source, target in routes(args.live).items():
        if source != target:
            result.extend(("-r", source + ":=" + target))
    return result


def reject_duplicate_publishers(targets):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor

    context = Context()
    rclpy.init(args=[], context=context)
    observer = rclpy.create_node("a4_startup_guard", context=context,
                                 enable_rosout=False, start_parameter_services=False)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(observer)
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        duplicates = [topic for topic in targets if observer.count_publishers(topic)]
        if duplicates:
            raise RuntimeError("Existing publishers: " + ", ".join(duplicates))
    finally:
        executor.shutdown()
        observer.destroy_node()
        rclpy.shutdown(context=context)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True, choices=["A4", "A4_lite"])
    parser.add_argument("--identity", action="store_true")
    parser.add_argument("--live", action="store_true", help="Use vehicle topics instead of shadow topics.")
    parser.add_argument("--input-image-topic", default=INPUT)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--score-threshold", default=.5, type=float)
    parser.add_argument("--use-sim-time", default="false", choices=["true", "false"])
    args = parser.parse_args()
    config = profile(ROOT / "profile.json", args.profile)
    weights = ROOT / "weights/shared_best.pth"
    digest = hashlib.sha256(weights.read_bytes()).hexdigest()
    if digest != WEIGHT_SHA256:
        raise ValueError("A4 checkpoint SHA256 mismatch")
    print(json.dumps(dict(config, weights=str(weights), sha256=digest, python=sys.executable,
                          input=args.input_image_topic, outputs=routes(args.live), live=args.live)), flush=True)
    if args.identity:
        return
    if sys.version_info < (3, 10):
        raise RuntimeError("A4 requires a compatible Python >=3.10, rclpy and cv_bridge; not stock Foxy Python 3.8")
    sys.path.insert(0, str(ROOT / "ros2/yolop_a3_perception"))
    from yolop_a3_perception.node import main as ros_main

    reject_duplicate_publishers(routes(args.live).values())
    ros_main(node_args(args, config, weights))


if __name__ == "__main__":
    main()
