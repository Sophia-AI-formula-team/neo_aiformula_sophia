"""Train a local response table in Unity and test one initial choice between two cones."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
STATIONS = np.linspace(0, 11, 221)
PRACTICE = sorted([*np.linspace(-1.8, 1.8, 9), -1.6, 1.6])
VALIDATION = {-1.35: (.69, 1.48), 0.: (-1.48, -.72, .69, 1.52), 1.35: (-1.48, -.69)}
LAYOUTS = {"second_left": [[5.5, 0], [8.0, -1.05]], "second_right": [[5.5, 0], [8.0, 1.05]]}
COLUMNS = ["t_s", "forward_m", "right_m", "yaw_right_rad", "speed_mps", "yaw_right_rad_s",
           "right_can_rpm", "left_can_rpm"]


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, ensure_ascii=False, separators=(",", ":"), allow_nan=False), encoding="utf8")
    tmp.replace(path)


def invoke_editor(bridge, code):
    response = bridge(6400, "execute_code", {"action": "execute", "code": code})
    payload = response.get("result", {})
    if not payload.get("success"):
        raise RuntimeError(json.dumps(response, ensure_ascii=False)[:4200])
    return payload["data"]["result"]


def float_literal(value):
    return f"{float(value):.8f}f"


def trial(bridge, offset, cones, *, trend=0., observe=False, frames=None):
    points = ",".join("new float[]{" + ",".join(map(float_literal, p)) + "}" for p in cones)
    if frames:
        frames.mkdir(parents=True, exist_ok=True)
    params = (f"float offset={float_literal(offset)}, trend={float_literal(trend)}; "
              f"var coneLayout=new float[][]{{{points}}}; "
              f"bool observeOnly={str(observe).lower()}; string frameDirectory="
              + json.dumps(str(frames.resolve()) if frames else "") + ";\n")
    result = invoke_editor(bridge, params + (HERE / "trial.cs.txt").read_text(encoding="utf8"))
    if not observe and not result.get("rows"):
        raise RuntimeError(f"Missing Unity trace: {result}")
    return result


def to_stations(result):
    rows = np.asarray(result["rows"])
    if result["status"] != "finished" or np.min(np.diff(rows[:, 1])) < -.002:
        raise ValueError(f"Practice must reach the endpoint moving forward: {result['status']}")
    advance = np.r_[True, np.diff(rows[:, 1]) > 0]
    rows = rows[advance]
    return np.column_stack([np.interp(STATIONS, rows[:, 1], rows[:, j]) for j in (2, 3, 0)])


def fit_responses(practice):
    # A failed practice maneuver limits support. Never interpolate across it.
    center = next(i for i, item in enumerate(practice) if abs(item["target"]) < 1e-6)
    if practice[center]["run"]["status"] != "finished":
        raise ValueError("No completed straight maneuver")
    left = right = center
    while left > 0 and practice[left-1]["run"]["status"] == "finished":
        left -= 1
    while right+1 < len(practice) and practice[right+1]["run"]["status"] == "finished":
        right += 1
    supported = practice[left:right+1]
    if len(supported) < 3:
        raise ValueError("Insufficient contiguous maneuver support")
    return np.array([v["target"] for v in supported]), np.stack([to_stations(v["run"]) for v in supported])


def predict(model, target):
    knots, table = model
    if not knots[0] <= target <= knots[-1]:
        raise ValueError(f"Outside measured practice support: {target}")
    hi = int(np.clip(np.searchsorted(knots, target), 1, len(knots) - 1))
    mix = (target - knots[hi-1]) / (knots[hi] - knots[hi-1])
    return table[hi-1] * (1-mix) + table[hi] * mix


def gap(s, x, yaw, cone):
    """Separating-axis margin between an oriented car envelope and the square cone base."""
    cs, cx, radius = cone
    co, si = np.cos(yaw), np.sin(yaw)
    ds, dx = s-.30*co-cs, x-.30*si-cx
    projection = radius*(abs(co)+abs(si))
    return np.maximum.reduce([abs(ds)-radius-.58*abs(co)-.38*abs(si),
                              abs(dx)-radius-.58*abs(si)-.38*abs(co),
                              abs(ds*co+dx*si)-.58-projection, abs(-ds*si+dx*co)-.38-projection])


def road_gap(road, x, yaw):
    co, si = np.cos(yaw), np.sin(yaw)
    gaps = []
    for lon in (-.88, .28):
        for lat in (-.38, .38):
            station = STATIONS + lon*co - lat*si
            corner = x + lon*si + lat*co
            gaps.extend((corner-np.interp(station, road[:, 0], road[:, 1]),
                         np.interp(station, road[:, 0], road[:, 2])-corner))
    return float(np.min(gaps))


def select(models, observations, road):
    options = []
    for trend, model in models.items():
        knots, _ = model
        for target in np.arange(-60, 61)*.03:
            if target < knots[0] or target > knots[-1]:
                continue
            x, yaw, duration = predict(model, target).T
            margin = min(float(gap(STATIONS, x, yaw, cone).min()) for cone in observations)
            road_margin = road_gap(road, x, yaw)
            options.append({"target": round(float(target), 3), "trend": trend, "margin": margin,
                            "road": road_margin, "headroom": min(margin-.22, road_margin-.10),
                            "cost": float(duration[-1] + .35*np.mean(abs(x)) + .05*abs(trend)),
                            "first_side": int(np.sign(np.interp(observations[0][0], STATIONS, x)))})
    feasible = [o for o in options if o["headroom"] >= 0]
    if not feasible:
        raise RuntimeError("No admissible initial maneuver")
    rank = lambda o: (o["cost"], -o["margin"])
    chosen = min(feasible, key=rank)
    # For comparison, preserve the best candidate on each side even if infeasible.
    sides = []
    for sign in (-1, 1):
        side = [o for o in options if o["first_side"] == sign]
        admissible = [o for o in side if o["headroom"] >= 0]
        item = min(admissible, key=rank) if admissible else max(side, key=lambda o: o["headroom"])
        sides.append({**item, "prediction": predict(models[item["trend"]], item["target"]).round(6).tolist()})
    return {"chosen": chosen, "sides": sides, "candidate_count": len(options)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sim", type=Path, required=True, help="Demo_LaneLineTrainer root containing Simulator")
    p.add_argument("--output", type=Path, default=HERE/"results")
    p.add_argument("--frames", type=Path, help="optional folder for Unity camera PNGs")
    p.add_argument("--probe", type=float, help="run only one empty-track maneuver")
    p.add_argument("--record-repeat", action="store_true", help="re-run the four saved decisions and check their real trajectories")
    args = p.parse_args()
    sim = args.sim.resolve()
    sys.path.insert(0, str(sim/"workspace/tools"))
    from invoke_unity_bridge import invoke
    from oct27_closedloop.unity import STATE
    output = args.output.resolve()
    # Guard against stale training/evaluation mixes. Every full invocation starts from scratch.
    before = invoke_editor(invoke, STATE)
    if before["dirty"] or before["play"] or before["compiling"]:
        raise RuntimeError("Expected clean Edit Mode")
    project_path = invoke_editor(invoke, "return UnityEngine.Application.dataPath;")
    if Path(project_path).resolve() != sim/"Simulator/Assets":
        raise RuntimeError(f"Bridge 6400 belongs to a different Unity project: {project_path}")
    if args.probe is not None:
        run = trial(invoke, args.probe, [], frames=args.frames)
        print(run["profile"], run["status"], np.asarray(run["rows"])[::20, :6])
        return
    assets = sim/"Simulator/Assets"
    protect = [assets/"Scenes/MotorControllerValidation.unity", assets/"Scenes/LaneTeachRepeatValidation.unity",
               assets/"TrainingEnvironment/Prefabs/TrafficCone_75cm.prefab",
               assets/"AIFormula/Runtime/Physics/AIFormulaDifferentialDrive.cs",
               assets/"AIFormula/Runtime/Physics/AIFormulaTireContact.cs",
               assets/"AIFormula/Runtime/Physics/AIFormulaArticulatedContact.cs",
               assets/"AIFormula/AIFormulaVehicle.prefab"]
    def source_state():
        return {f.relative_to(assets).as_posix(): hashlib.sha256(f.read_bytes()).hexdigest() for f in protect}
    first_hashes = source_state()
    if args.record_repeat:
        if args.frames is None:
            p.error("--record-repeat requires --frames")
        original = json.loads((output/"experiment.json").read_text(encoding="utf8"))
        if first_hashes != original["source_sha256"]:
            raise ValueError("Unity source differs from the measured baseline")
        comparison = {}
        for label, modes in original["evaluations"].items():
            comparison[label] = {}
            for mode, case in modes.items():
                pick = case["decision"]["chosen"]
                folder = args.frames/label/mode
                result = trial(invoke, pick["target"], LAYOUTS[label], trend=pick["trend"], frames=folder)
                old, new = np.asarray(case["run"]["rows"]), np.asarray(result["rows"])
                if result["status"] != case["run"]["status"] or old.shape != new.shape:
                    raise ValueError(f"Repeat differs: {label} {mode}")
                position_error = float(np.linalg.norm(old[:,1:3]-new[:,1:3], axis=1).max())
                if position_error > .002 or not np.allclose(old[:,0], new[:,0], atol=.001):
                    raise ValueError(f"Unmatched camera run: {position_error:.4f} m")
                comparison[label][mode] = {"max_position_difference_m":position_error,
                                            "status":result["status"], "frames":result["frames"]}
                print(label, mode, result["status"], position_error, flush=True)
        if before != invoke_editor(invoke, STATE) or source_state() != first_hashes:
            raise RuntimeError("Editor or source changed during replay")
        write(output/"repeat.json", comparison)
        return
    data = {"columns": COLUMNS, "station_m": STATIONS.tolist(), "practice": [], "validation": [],
            "evaluations": {}, "source_sha256": first_hashes, "editor_before": before}
    try:
        models = {}
        for trend, holdout in VALIDATION.items():
            for key, offsets in [("practice", PRACTICE), ("validation", holdout)]:
                for offset in offsets:
                    run = trial(invoke, offset, [], trend=trend)
                    print(key, trend, round(offset, 2), run["status"], flush=True)
                    data[key].append({"target": float(offset), "trend":trend, "run": run})
            models[trend] = fit_responses([r for r in data["practice"] if r["trend"] == trend])
        data["supported_offsets"] = {str(t): v[0].tolist() for t,v in models.items()}
        for item in data["validation"]:
            error = predict(models[item["trend"]], item["target"]) - to_stations(item["run"])
            item["lateral_mae_m"] = float(np.abs(error[:, 0]).mean())
            item["lateral_max_m"] = float(np.abs(error[:, 0]).max())
        for label, layout in LAYOUTS.items():
            semantic = trial(invoke, 0, layout, observe=True)
            observed, road = semantic["observed"], np.array(semantic["road"])
            if len(observed) != 2:
                raise ValueError(f"Expected both cones visible at the start: {observed}")
            observed.sort(key=lambda c: c[0])
            data["evaluations"][label] = {}
            for mode, inputs in [("both", observed), ("nearest_only", observed[:1])]:
                decision = select(models, inputs, road)
                target = decision["chosen"]["target"]
                folder = args.frames/label/mode if args.frames else None
                run = trial(invoke, target, layout, trend=decision["chosen"]["trend"], frames=folder)
                if not np.allclose(sorted(run["observed"]), observed, atol=1e-4):
                    raise ValueError("Observations changed when scenario was recreated")
                data["evaluations"][label][mode] = {"planner_input": inputs, "road":road.tolist(),
                                                     "decision": decision, "run": run}
                print(label, mode, target, run["status"], run["min_gap"], flush=True)
    except Exception:
        write(output/"incomplete.json", data)
        raise
    finally:
        data["editor_after"] = invoke_editor(invoke, STATE)
        data["scene_restored"] = before == data["editor_after"]
        data["sources_preserved"] = source_state() == first_hashes
    if not data["scene_restored"] or not data["sources_preserved"]:
        raise RuntimeError("Editor or source changed during the experiment")
    write(output/"experiment.json", data)


if __name__ == "__main__":
    main()
