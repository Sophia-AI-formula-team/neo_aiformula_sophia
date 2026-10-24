"""Run one unique bounded neo stop_wait case through the installed Unity MCP bridge.

Root must own Unity/CPU before invoking. No Editor launch, automatic retry, source
installation, parameter tuning or bbox installation occurs here.
"""
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
    parser.add_argument("--kind", required=True, choices=("traffic", "closure", "recovery"))
    parser.add_argument("--lane", required=True, choices=("inner", "outer"))
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--motor-gate-source", type=Path, required=True)
    parser.add_argument("--worker-root", type=Path, required=True)
    parser.add_argument("--simulator", type=Path, required=True)
    parser.add_argument("--bridge-script", type=Path, required=True)
    parser.add_argument("--expected-harness-sha256", required=True)
    parser.add_argument("--partial-restart-distance", type=float, default=120)
    parser.add_argument("--lane-recovery-mode", choices=("baseline", "component-only"), default="baseline")
    parser.add_argument("--no-panel-rgb", action="store_true")
    parser.add_argument("--leave-play", action="store_true")
    args = parser.parse_args()
    if Path(args.name).name != args.name or args.name in (".", ".."):
        raise ValueError("name must be a single output directory name")
    if not 20 <= args.partial_restart_distance <= 200:
        raise ValueError("restart distance must be 20-200 m")
    output = args.output_root.resolve() / args.name
    if output.exists():
        raise ValueError("Preserve prior evidence; choose a new name")
    for key in ("source_root", "worker_root", "simulator"):
        setattr(args, key, getattr(args, key).resolve(strict=True))
    args.motor_gate_source = args.motor_gate_source.resolve(strict=True)
    harness = args.simulator / "Assets/AIFormula/Editor/Validation/AIFormulaNeoTopoTrial.cs"
    if sha(harness).lower() != args.expected_harness_sha256.lower():
        raise ValueError("Live harness does not match reviewed SHA")
    spec = importlib.util.spec_from_file_location("owned_unity_bridge", args.bridge_script.resolve(strict=True))
    bridge = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bridge)

    def call(tool, params):
        value = bridge.invoke(6400, tool, params)
        result = value.get("result", {})
        if value.get("status") != "success" or result.get("success") is not True:
            raise RuntimeError(json.dumps(value))
        return result.get("data")

    source_files = [harness, args.motor_gate_source]
    for root in (args.source_root, args.worker_root):
        source_files.extend(p for p in root.rglob("*") if p.is_file() and p.suffix in (".py", ".yaml", ".osm", ".json"))
    for rel in (
        "Assets/AIFormula/Runtime/Control/AIFormulaSemanticPlanning.cs",
        "Assets/AIFormula/Runtime/Control/AIFormulaVisionLaneFollower.cs",
        "Assets/AIFormula/Runtime/Perception/AIFormulaRoadDetector.cs",
        "Assets/StreamingAssets/RoadDetector/worker.py",
        "Assets/StreamingAssets/RoadDetector/signal_panel_rgb.py",
        "Assets/StreamingAssets/VisionLaneFollower/upstream_adapter.py",
        "Assets/StreamingAssets/VisionLaneFollower/lane_publisher.py",
        "Assets/StreamingAssets/VisionLaneFollower/current_corridor.py",
    ):
        source_files.append(args.simulator / rel)
    before = {str(path): sha(path) for path in sorted(set(source_files))}
    state = call("get_editor_state", {})
    if state["editor"]["play_mode"]["is_playing"]:
        raise RuntimeError("Root must finish preceding Play session first")
    x = 68.76 if args.lane == "inner" else 72.53
    def cs(value):
        return '@"' + str(value).replace('"', '""') + '"'
    code = ("return AIFormula.Editor.AIFormulaNeoTopoTrial.Start("
            + ", ".join((cs(output), cs(args.name), cs(args.kind), f"{x}f", "-10f",
                         cs(args.source_root), cs(args.motor_gate_source), cs(args.worker_root),
                         str(not args.no_panel_rgb).lower(), f"{args.partial_restart_distance}f", cs(args.lane_recovery_mode))) + ");")
    began = time.monotonic()
    last_log = -10.0
    completed = False
    start_attempted = False
    try:
        start_attempted = True
        print(json.dumps(call("execute_code", {"action": "execute", "code": code})), flush=True)
        (output / "integration_hashes.json").write_text(json.dumps({"captured": "before_start", "sources": before}, indent=2), encoding="utf-8")
        limit = 200 if args.kind == "traffic" else 450
        while time.monotonic() - began < limit:
            if (output / "result.json").exists():
                result = json.loads((output / "result.json").read_text(encoding="utf-8-sig"))
                completed = True
                print(json.dumps(result), flush=True)
                after = {str(path): sha(path) for path in sorted(set(source_files))}
                (output / "integration_hashes_after.json").write_text(json.dumps({"sources": after, "unchanged": before == after}, indent=2), encoding="utf-8")
                return 0 if result["accepted"] and before == after else 2
            elapsed = time.monotonic() - began
            if elapsed - last_log >= 10 and (output / "status.json").exists():
                try:
                    status = json.loads((output / "status.json").read_text(encoding="utf-8-sig"))
                    print(json.dumps({key: status.get(key) for key in ("phase", "stage", "reason", "wall", "travelled", "semantic", "follower")}), flush=True)
                except json.JSONDecodeError:
                    pass
                last_log = elapsed
            time.sleep(.5)
        raise TimeoutError("Bounded trial expired; preserve failure and do not retry automatically")
    finally:
        if start_attempted and not completed:
            try:
                call("execute_code", {"action": "execute", "code": 'AIFormula.Editor.AIFormulaNeoTopoTrial.Stop("Runner watchdog or failure"); return "Stop requested";'})
            finally:
                call("manage_editor", {"action": "stop"})
        elif not args.leave_play:
            call("manage_editor", {"action": "stop"})


if __name__ == "__main__":
    raise SystemExit(main())
