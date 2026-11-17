"""Run one owned MCP GUI Unity joint-package case; never launches or installs Unity."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import time


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name")
    parser.add_argument("--lane", required=True, choices=("inner", "outer"))
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--worker-root", type=Path, required=True)
    parser.add_argument("--simulator", type=Path, required=True)
    parser.add_argument("--bridge-script", type=Path, required=True)
    parser.add_argument("--expected-harness-sha256", required=True)
    parser.add_argument("--expected-transport-sha256", required=True)
    parser.add_argument("--wall-timeout", type=float, default=900)
    parser.add_argument("--lane-recovery-mode", choices=("baseline", "component-only"))
    parser.add_argument("--port", type=int, default=6400)
    parser.add_argument("--leave-play", action="store_true")
    args = parser.parse_args()
    if Path(args.name).name != args.name or args.name in (".", ".."):
        raise ValueError("name must be a single output directory name")
    if not 60 <= args.wall_timeout <= 3600:
        raise ValueError("wall timeout must be 60-3600 seconds")
    output = args.output_root.resolve() / args.name
    if output.exists():
        raise ValueError("Preserve prior evidence; choose a new name")
    for key in ("source_root", "worker_root", "simulator"):
        setattr(args, key, getattr(args, key).resolve(strict=True))
    harness = args.simulator / "Assets/AIFormula/Editor/Validation/AIFormulaJoint1118Trial.cs"
    transport = args.simulator / "Assets/AIFormula/Runtime/Control/AIFormulaSemanticPlanning.cs"
    for path, expected in ((harness, args.expected_harness_sha256), (transport, args.expected_transport_sha256)):
        if sha(path).lower() != expected.lower():
            raise ValueError("Live file does not match reviewed SHA: " + str(path))
    spec = importlib.util.spec_from_file_location("owned_unity_bridge", args.bridge_script.resolve(strict=True))
    bridge = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bridge)

    def call(tool, params):
        value = bridge.invoke(args.port, tool, params)
        result = value.get("result", {})
        if value.get("status") != "success" or result.get("success") is not True:
            raise RuntimeError(json.dumps(value))
        return result.get("data")

    if output == args.source_root or args.source_root in output.parents:
        raise ValueError("Trial output must be outside the immutable ROS source tree")
    # Hash executable/configuration inputs, never mutable reports beside the worker.
    sources = [harness, transport, args.worker_root / "semantic_joint_worker.py"]
    sources.extend(p for p in args.source_root.rglob("*") if p.is_file()
                   and p.suffix in (".py", ".yaml", ".osm", ".json") and "__pycache__" not in p.parts)
    for rel in ("Assets/AIFormula/Runtime/Control/AIFormulaVisionLaneFollower.cs",
                "Assets/AIFormula/Runtime/Perception/AIFormulaRoadDetector.cs",
                "Assets/AIFormula/Runtime/Perception/AIFormulaRgbCameraTopic.cs",
                "Assets/StreamingAssets/RoadDetector/worker.py",
                "Assets/StreamingAssets/RoadDetector/signal_panel_rgb.py",
                "Assets/StreamingAssets/RoadDetector/signal_panel_range.py",
                "Assets/StreamingAssets/RoadDetector/a4/yolop_a3_runtime/shared_fcos.py",
                "Assets/StreamingAssets/RoadDetector/a4/yolop_a3_runtime/shared_runtime.py",
                "Assets/StreamingAssets/RoadDetector/a4/yolop_a3_runtime/color_semantics.py",
                "Assets/StreamingAssets/VisionLaneFollower/upstream_adapter.py",
                "Assets/StreamingAssets/VisionLaneFollower/lane_publisher.py",
                "Assets/StreamingAssets/VisionLaneFollower/current_corridor.py"):
        sources.append(args.simulator / rel)
    upstream_root = args.simulator / "Assets/StreamingAssets/VisionLaneFollower/upstream"
    provenance_path = upstream_root / "provenance.json"
    provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
    sources.append(provenance_path)
    for entry in provenance["files"]:
        dependency = (upstream_root / entry["path"]).resolve(strict=True)
        if upstream_root.resolve() not in dependency.parents:
            raise ValueError("Upstream provenance path escapes source root")
        sources.append(dependency)
    before = {str(p): sha(p) for p in sorted(set(sources))}
    state = call("get_editor_state", {})
    if state["editor"]["play_mode"]["is_playing"]:
        raise RuntimeError("The owner must finish preceding Play session first")
    x = 68.76 if args.lane == "inner" else 72.53
    lane_recovery = args.lane_recovery_mode or ("component-only" if args.lane == "inner" else "baseline")

    def cs(value):
        return '@"' + str(value).replace('"', '""') + '"'

    code = ("return AIFormula.Editor.AIFormulaJoint1118Trial.Start(" + ", ".join((
        cs(output), cs(args.name), f"{x}f", "-10f", cs(args.source_root), cs(args.worker_root),
        str(args.wall_timeout), "true", cs(lane_recovery))) + ");")
    began = time.monotonic()
    completed = started = False
    last_log = -10.0
    try:
        started = True
        print(json.dumps(call("execute_code", {"action": "execute", "code": code})), flush=True)
        (output / "integration_hashes.json").write_text(json.dumps({"captured": "before_start", "sources": before}, indent=2), encoding="utf-8")
        while time.monotonic() - began < args.wall_timeout + 35:
            if (output / "result.json").exists():
                result = json.loads((output / "result.json").read_text(encoding="utf-8-sig"))
                completed = True
                after = {str(p): sha(p) for p in sorted(set(sources))}
                (output / "integration_hashes_after.json").write_text(json.dumps({"sources": after, "unchanged": before == after}, indent=2), encoding="utf-8")
                print(json.dumps(result), flush=True)
                return 0 if result["accepted"] and before == after else 2
            elapsed = time.monotonic() - began
            if elapsed - last_log >= 10 and (output / "status.json").exists():
                try:
                    print(json.dumps(json.loads((output / "status.json").read_text(encoding="utf-8-sig"))), flush=True)
                except json.JSONDecodeError:
                    pass
                last_log = elapsed
            time.sleep(.5)
        raise TimeoutError("Bounded joint trial expired; preserve the failed attempt")
    finally:
        if started and not completed:
            try:
                call("execute_code", {"action": "execute", "code": 'AIFormula.Editor.AIFormulaJoint1118Trial.Stop("Runner watchdog or failure"); return "Stop requested";'})
            finally:
                call("manage_editor", {"action": "stop"})
        elif completed and not args.leave_play:
            call("manage_editor", {"action": "stop"})


if __name__ == "__main__":
    raise SystemExit(main())
