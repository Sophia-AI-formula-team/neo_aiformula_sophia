"""Score the independently preregistered paired encounter segments from Unity logs."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from statistics import median

import numpy as np
from risk_metrics import CONE_MARGIN_M, ROAD_MARGIN_M, ZONE_HALF_M, _free_pace, _groups

HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_bytes())


def check_layout(case, reference, learned, length):
    a, b = reference["request"], learned["request"]
    for key in (a.keys() | b.keys()) - {"encounter"}:
        if a.get(key) != b.get(key):
            raise ValueError(f"{case['id']}: unmatched request field {key}")
    for key in ("props", "length", "profile", "coupled", "physicsStep", "unity"):
        if reference[key] != learned[key]:
            raise ValueError(f"{case['id']}: unmatched simulation field {key}")
    expected = [[m / length, *values] for m, *values in case["props"]]
    seed_rng = np.random.default_rng(case["seed"])
    spawn = (seed_rng.random(), seed_rng.uniform(-.30, .30), seed_rng.uniform(-3.2, 3.2))
    if (not a["vision"] or a["seed"] != case["seed"] or
        not np.allclose([a["start"], a["offset"], a["yaw"]], spawn, rtol=0, atol=1e-8) or
        a["timing"] != {"mode": "both", "approach": case["approach"]} or
        not math.isclose(a["limit"], case["limitM"] / length, rel_tol=0, abs_tol=1e-8) or
        len(a["obstacles"]) != len(expected) or len(reference["props"]) != len(expected) or
        not np.allclose(a["obstacles"], expected, rtol=0, atol=1e-8)):
        raise ValueError(f"{case['id']}: run differs from registered layout")


def status(run):
    complete = run["status"] == "segment"
    gap = min(run["propMin"], default=None)
    safe = (complete and all(t >= 0 for t in run["zoneEnd"]) and run["minRoad"] >= ROAD_MARGIN_M
            and (gap is None or gap >= CONE_MARGIN_M))
    return dict(status=run["status"], complete=complete, safe=safe, minRoadM=run["minRoad"],
                minConeM=gap, elapsedS=run["seconds"], progressM=run["progress"])


def matched_pace(run, end):
    rows = np.array(run["rows"])
    groups = _groups(run)
    free = _free_pace(rows, groups, end)
    # Interpolate the first forward crossing. Stopping or backing up between the two gates
    # still consumes time; interpolation over sorting by station would understate this.
    def crossing(station):
        idx = np.flatnonzero((rows[:-1, 7] < station) & (rows[1:, 7] >= station))[0]
        return float(np.interp(station, rows[idx:idx+2, 7], rows[idx:idx+2, 0]))
    duration = sum(crossing(g["end"] + ZONE_HALF_M) - crossing(g["begin"] - ZONE_HALF_M) for g in groups)
    return dict(freeRoad=free, zoneSeconds=duration if groups else None)


def first_control_change(reference, learned):
    for a, b in zip(reference["decisions"], learned["decisions"]):
        if abs(a["t"] - b["t"]) > .0001:
            raise ValueError("Decision clocks differ before the first control change")
        if (a["leftCan"], a["rightCan"]) != (b["leftCan"], b["rightCan"]) or any(
                abs(a[k]-b[k]) > .000001 for k in ("speedCmd", "yawCmd")):
            same = all(a[k] == b[k] for k in ("speed", "yaw", "observed", "tracks"))
            return dict(t=b["t"], sameObservation=same, baseReason=a["reason"], learnedReason=b["reason"],
                        baseTargetMps=a["speedCmd"], learnedTargetMps=b["speedCmd"])
    return None


def audit_loaded_selector(run, artifact):
    cfg = run["request"].get("encounter", {})
    for key in ("mode", "modelId", "actions", "contexts", "returns", "scales", "width"):
        if key not in cfg or cfg[key] != artifact[key]:
            raise ValueError(f"{run['id']}: embedded selector differs from frozen artifact at {key}")
    if cfg["mode"] == "probe":
        raise ValueError("Forced actions cannot count as learned evaluations")
    contexts = np.array(cfg["contexts"])
    scales, values = np.array(cfg["scales"]), np.array(cfg["returns"])
    if contexts.ndim != 2 or values.shape != (len(contexts), len(cfg["actions"])) or np.any(scales<=0):
        raise ValueError("Ill-formed return model")
    selected = []
    max_score_error = 0.
    for decision in run["decisions"]:
        probe = decision.get("encounter") or {}
        if probe.get("modelId") != cfg["modelId"]:
            raise ValueError("Reported model ID differs from embedded selector")
        if probe.get("state") != "selected":
            continue
        distances = np.sum(((np.array(probe["features"])-contexts)/scales)**2, axis=1)
        w = np.exp(-(distances-distances.min())/(2*cfg["width"]**2))
        scores = w @ values / sum(w)
        error = float(np.max(np.abs(scores-np.array(probe["actionScores"]))))
        max_score_error = max(max_score_error, error)
        if not np.allclose(scores, probe["actionScores"], atol=.0003, rtol=.00003) or scores.argmax()!=probe["chosen"]:
            raise ValueError(f"Independent selector disagrees with Unity at {decision['t']}")
        selected.append(dict(t=decision["t"], choice=probe["chosen"], reason=decision["reason"],
                             targetMps=decision["speedCmd"], bodyMps=decision["speed"]))
    return dict(selected=selected, maxReplayScoreError=max_score_error,
                activeCommandUpdates=sum(d["reason"] == "learned_encounter" for d in run["decisions"]))


def pair_metrics(reference, learned):
    a, b = status(reference), status(learned)
    if a["safe"] and b["safe"]:
        end = min(a["progressM"], b["progressM"])
        a.update(matched_pace(reference, end))
        b.update(matched_pace(learned, end))
        if abs(a["freeRoad"]["meters"]-b["freeRoad"]["meters"]) > .001:
            raise ValueError("Free pace was not measured over the same distance")
    return dict(base=a, learned=b, firstCommandChange=first_control_change(reference, learned))


def lap_metrics(reference, learned):
    data = {}
    for arm, run in (("base",reference),("learned",learned)):
        state = status(run)
        complete = run["status"] == "lap" and run["gate"] == 32
        passed = [t >= 0 for t in run["zoneEnd"]]
        safe = complete and state["minRoadM"] >= ROAD_MARGIN_M and state["minConeM"] >= CONE_MARGIN_M and all(passed)
        data[arm] = dict(state, complete=complete, safe=safe, courseM=run["length"], gate=run["gate"],
                         progressFraction=run["progress"]/run["length"], perProp=[
                             dict(id=i, entered=run["zoneStart"][i]>=0, exited=exited,
                                  observedGapM=run["propMin"][i] if run["zoneStart"][i]>=0 else None)
                             for i,exited in enumerate(passed)])
    both_safe = all(v["safe"] for v in data.values())
    data["sameSafeLapTimeChangeS"] = learned["seconds"] - reference["seconds"] if both_safe else None
    data["firstCommandChange"] = first_control_change(reference,learned)
    return data


def summarize(pairs):
    count = lambda arm, flag: sum(p[arm][flag] for p in pairs)
    both = [p for p in pairs if p["base"]["safe"] and p["learned"]["safe"]]
    obstacles = [p for p in both if p["base"]["zoneSeconds"] is not None]
    def pooled(arm):
        meters = sum(p[arm]["freeRoad"]["meters"] for p in both)
        seconds = sum(p[arm]["freeRoad"]["seconds"] for p in both)
        return meters/seconds if seconds else None
    base, learned = pooled("base"), pooled("learned")
    free_gain = learned/base - 1 if base else None
    change = median(p["learned"]["zoneSeconds"]/p["base"]["zoneSeconds"] - 1 for p in obstacles) if obstacles else None
    completed = [count(a, "complete") for a in ("base", "learned")]
    safe = [count(a, "safe") for a in ("base", "learned")]
    regressions = [p["id"] for p in pairs if p["base"]["safe"] and not p["learned"]["safe"]]
    improved = (not regressions and completed[1] >= completed[0] and safe[1] >= safe[0]
                and free_gain is not None and free_gain >= -.02 and change is not None and change <= .02
                and (completed[1] > completed[0] or safe[1] > safe[0] or free_gain > .02 or change < -.02))
    return dict(order=["base", "learned"], completed=completed, safe=safe, newlyUnsafe=regressions,
                bothSafe=[p["id"] for p in both], timedObstacles=[p["id"] for p in obstacles],
                pooledFreeRoadMps=[base, learned], freeGainFraction=free_gain,
                medianPairedZoneTimeChangeFraction=change, passesRegisteredImprovement=improved)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("runs", type=Path)
    p.add_argument("--layouts", type=Path, help="Registered matrix; defaults to the chosen experiment")
    p.add_argument("--full-lap", action="store_true", help="Score the supplementary full lap separately")
    p.add_argument("--model", type=Path, required=True, help="Frozen selector configuration JSON")
    p.add_argument("--out", type=Path, required=True)
    args = p.parse_args()
    if args.layouts is None:
        args.layouts = HERE/"learned_holdout"/("full_lap_layout.json" if args.full_lap else "layouts.json")
    plan = read(args.layouts)
    pairs, versions = [], set()
    artifact = read(args.model)
    for c in plan["cases"]:
        paths = [args.runs/f"{c['id']}_{arm}.json" for arm in ("base", "learned")]
        base, learned = map(read, paths)
        check_layout(c, base, learned, plan["courseLengthForFractionsM"])
        versions.add((base["request"]["model"], base["request"]["solver"]))
        if len(versions) != 1:
            raise ValueError("The compiled solver or camera model changed across the test")
        if base["request"].get("encounter") is not None:
            raise ValueError("Baseline unexpectedly enables an encounter selector")
        measurements = lap_metrics(base,learned) if args.full_lap else pair_metrics(base,learned)
        pairs.append(dict(id=c["id"], context=c["context"], approachCapMps=c["approach"], runSha256=[
            hashlib.sha256(path.read_bytes()).hexdigest() for path in paths], **measurements,
            selectorAudit=audit_loaded_selector(learned,artifact)))
    report = dict(protocolCommit="2b647e0" if args.full_lap else "4bfe2d7", layoutGitBlobNormalizedSha256=hashlib.sha256(
        args.layouts.read_bytes().replace(b"\r\n", b"\n")).hexdigest(), solver=versions.pop()[1],
        modelFileSha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
        pairs=pairs)
    if not args.full_lap:
        report.update(summary=summarize(pairs), approachSubgroups={
            "natural": summarize([p for p in pairs if p["approachCapMps"] == 0]),
            "configured": summarize([p for p in pairs if p["approachCapMps"] > 0])})
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf8")


if __name__ == "__main__":
    main()
