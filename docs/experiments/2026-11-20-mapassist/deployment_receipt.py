"""Record the 1120 vehicle installation. This reads files without starting ROS nodes."""
import hashlib
import importlib.util
import json
import os
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree

from ament_index_python.packages import PackageNotFoundError, get_package_prefix

PACKAGES = (
    "semantic_vehicle_wiring", "semantic_planner_core", "semantic_local_planner_ros",
    "launchers", "obsticle_avoidence", "kalman_filter", "lane_points", "lane_line_publisher",
    "trajectory_follower", "motor_controller", "vehicle", "zed_wrapper", "odometry_publisher",
)


def describe(path):
    path = Path(path)
    if not path.is_file():
        return {"path": str(path), "available": False}
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"path": str(path.resolve()), "sha256": digest.hexdigest()}


def main():
    repo, runtime, oa_module = map(Path, sys.argv[1:])
    here = Path(__file__).resolve().parent
    lock = json.loads((here / "av314-version.json").read_text())
    def git(*args):
        return subprocess.check_output(["git", "-C", str(repo), *args], text=True).strip()

    packages = {}
    for name in PACKAGES:
        try:
            prefix = Path(get_package_prefix(name))
        except PackageNotFoundError:
            packages[name] = {"available": False}
            continue
        manifest = prefix / "share" / name / "package.xml"
        packages[name] = {"prefix": str(prefix), "version": ElementTree.parse(manifest).findtext("version")}

    vehicle = Path("/home/workspace/install")
    scripts = runtime / "install/launchers/share/launchers/shellscript/mapassist_045"
    files = {
        "oa2": oa_module, "selector": scripts / "fresh_oa_switch.py",
        "pixels": scripts / "pixel_stop_flag.py", "stop_gate": scripts / "guarded_stop_wait.py",
        "allnodes": vehicle / "launchers/share/launchers/launch/allnodes.launch.py",
        "v14_launch": vehicle / "launchers/share/launchers/launch/yolop_a3.launch.py",
        "v14_weight": Path.home() / "aiformula_runtime/yolop_a3_v14_20261012/upstream/YOLOP_A3_Deploy/weights/shared_best.pth",
        "yolop_menu": Path.home() / "aiformula_runtime/2026-11-19-yolop-selection/yolop.launch",
        "lya_entry": vehicle / "trajectory_follower/lib/trajectory_follower/lya_follower_connected_omegat_global",
        "lane_entry": vehicle / "lane_points/lib/lane_points/lane_0927oa",
        "motor_entry": vehicle / "motor_controller/lib/motor_controller/motor_controller",
        "command_source": here / "vehicle-command",
    }
    actual = {key: describe(path) for key, path in files.items()}
    for key, module in {"lya_module": "trajectory_follower.lya_follower_connected_omegat_global",
                        "kalman_module": "kalman_filter.withoutkalman_0312"}.items():
        try:
            spec = importlib.util.find_spec(module)
        except ModuleNotFoundError:
            spec = None
        actual[key] = describe(spec.origin) if spec and spec.origin else {"available": False}
    matches = {key: actual[key].get("sha256") == digest for key, digest in lock["sha256"].items()}
    now = datetime.now(timezone.utc)
    receipt = {
        "version": lock["version"], "recorded_utc": now.isoformat(), "commit": git("rev-parse", "HEAD"),
        "branch": git("rev-parse", "--abbrev-ref", "HEAD"), "worktree_status": git("status", "--porcelain"),
        "machine": platform.machine(), "ros": os.getenv("ROS_DISTRO"), "python": platform.python_version(),
        "packages": packages, "files": actual, "matches_tested": matches,
    }
    target = runtime / ("deployment-" + now.strftime("%Y%m%dT%H%M%S%fZ") + ".json")
    with target.open("x") as stream:
        json.dump(receipt, stream, indent=2)
        stream.write("\n")
    print("Deployment receipt:", target, flush=True)
    for key, matched in matches.items():
        if not matched:
            print("AV314 reference differs or file unavailable:", key, file=sys.stderr)
    if not (matches["oa2"] and matches["selector"] and matches["pixels"] and matches["stop_gate"]):
        raise SystemExit("Installed wiring differs from the tested version. Review the receipt.")


if __name__ == "__main__":
    main()
