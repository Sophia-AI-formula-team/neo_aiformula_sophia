"""Evaluate Unity driving by safety and free-road pace. Referee geometry stays offline."""
import argparse
import json
from pathlib import Path

import numpy as np


# Score free pace away from the approach, pass and immediate recovery.
APPROACH_M = 12.0
RECOVERY_M = 10.0
ZONE_HALF_M = 6.0  # UnityRace's first-to-last timing gates
ROAD_MARGIN_M = 0.08
CONE_MARGIN_M = 0.15


def _groups(run):
    """Merge overlapping ±6 m gates so a staggered pair counts as one encounter."""
    groups = []
    for i in sorted(range(len(run["props"])), key=lambda k: run["props"][k][7]):
        station = run["props"][i][7]
        if groups and station - ZONE_HALF_M <= groups[-1]["end"] + ZONE_HALF_M:
            group = groups[-1]
            group["end"] = station
            group["ids"].append(i)
        else:
            groups.append({"ids": [i], "begin": station, "end": station})
    return groups


def _free_pace(rows, groups, end_progress=None):
    """Distance/time outside windows, with fractional samples at the section boundaries."""
    distances = rows[:, 7]
    times = rows[:, 0]
    lo, hi = distances.min(), distances.max()
    if end_progress is not None:
        hi = min(hi, end_progress)
    cuts = [lo, hi]
    for g in groups:
        cuts.extend([np.clip(g["begin"] - APPROACH_M, lo, hi),
                     np.clip(g["end"] + RECOVERY_M, lo, hi)])
    edges = sorted(set(cuts))
    ds, dt = np.diff(distances), np.diff(times)
    start, stop = np.minimum(distances[1:], distances[:-1]), np.maximum(distances[1:], distances[:-1])
    meters = seconds = 0.0
    for left, right in zip(edges, edges[1:]):
        mid = (left + right) / 2
        if any(g["begin"] - APPROACH_M <= mid <= g["end"] + RECOVERY_M for g in groups):
            continue
        overlap = np.maximum(0., np.minimum(stop, right) - np.maximum(start, left))
        moving = np.abs(ds) > 1e-10
        stationary_inside = (start >= left) & ((start < right) | ((right == hi) & (start == hi)))
        portion = np.divide(overlap, np.abs(ds), out=stationary_inside.astype(float), where=moving)
        meters += float(np.dot(ds, portion))
        seconds += float(np.dot(dt, portion))
    return {"meters": meters, "seconds": seconds, "mps": meters / seconds if seconds > 0 else None}


def _moving_risk(rows, prop):
    """Counterfactual short-horizon conflict under constant ego heading and cone velocity."""
    onset = prop[7] - APPROACH_M
    available = np.where((rows[:, 7] >= onset) & (rows[:, 7] <= prop[7]))[0]
    if not len(available):
        return None
    r = rows[available]
    dt = np.linspace(0., 3., 61)[None, :]
    yaw = r[:, 3]
    forward = np.column_stack((np.sin(yaw), np.cos(yaw)))
    right = np.column_stack((np.cos(yaw), -np.sin(yaw)))
    phase = prop[5] + prop[6] * r[:, 0]
    cross = np.array(prop[2:4])
    cone = np.array(prop[:2]) + (prop[4] * np.sin(phase))[:, None] * cross
    cone_v = (prop[4] * prop[6] * np.cos(phase))[:, None] * cross
    rel = cone - r[:, 1:3]
    distance = np.einsum("ij,ij->i", rel, forward)
    rightward = np.einsum("ij,ij->i", rel, right)
    approach_v = np.einsum("ij,ij->i", cone_v, forward) - r[:, 4]
    lateral_v = np.einsum("ij,ij->i", cone_v, right)
    z = distance[:, None] + approach_v[:, None] * dt
    x = rightward[:, None] + lateral_v[:, None] * dt
    dz = np.maximum(0., np.maximum(z - .28, -.88 - z))
    dx = np.maximum(0., np.abs(x) - .38)
    clearance = np.hypot(dz, dx) - .34
    risk = (clearance.min(axis=1) < .25) & (distance > 0.)
    if not risk.any():
        return {"conflict": False, "samples": int(len(r))}
    i = int(np.flatnonzero(risk)[0])
    raw_i = available[i]
    t = float(r[i, 0])
    # Response is observational, never a claim that a particular detection caused an action.
    forward_window = rows[(rows[:, 0] >= t) & (rows[:, 0] <= t + 1.5)]
    prior = rows[max(0, raw_i - 1)]
    return {
        "conflict": True, "refereeOnsetS": t, "distanceAheadM": float(distance[i]),
        "predictedGapM": float(clearance[i].min()),
        "ttcMarginS": float(dt[0, np.flatnonzero(clearance[i] < .25)[0]]),
        "priorTargetMps": float(prior[5]), "lowestTargetNext1p5s": float(forward_window[:, 5].min()),
        "observedTargetReductionMps": max(0., float(prior[5] - forward_window[:, 5].min())),
        "actualOnsetMps": float(r[i, 4]), "minActualNext1p5s": float(forward_window[:, 4].min()),
        "maxTargetOffsetChangeM": float(np.max(np.abs(forward_window[:, 11] - prior[11]))),
        "responseWindowS": float(forward_window[-1, 0] - t),
    }


def measure(run):
    """One run. Clear-road pace is diagnostic for unfinished laps, which stay failures."""
    rows = np.asarray(run["rows"], dtype=float)
    groups = _groups(run)
    zones = []
    for g in groups:
        ids = g["ids"]
        moving = [i for i in ids if run["props"][i][4] != 0 and run["props"][i][6] != 0]
        enter = run["zoneStart"][ids[0]]
        leave = run["zoneEnd"][ids[-1]]
        z = {"kind": "moving" if moving else "pair" if len(ids) > 1 else "static",
             "stationM": g["begin"], "endStationM": g["end"],
             "entered": enter >= 0, "passed": leave >= 0,
             "seconds": float(leave - enter) if leave >= 0 else None,
             "coneGapM": min(run["propMin"][i] for i in ids) if enter >= 0 else None}
        if moving:
            z["motionRisk"] = [_moving_risk(rows, run["props"][i]) for i in moving]
        zones.append(z)
    min_cone = min((z["coneGapM"] for z in zones if z["entered"]), default=None)
    return {"id": run.get("id"), "seed": run["request"]["seed"], "status": run["status"],
            "finished": run["status"] == "lap" and run["gate"] == 32,
            "lapSeconds": run["seconds"] if run["status"] == "lap" else None,
            "progressM": run["progress"], "courseM": run["length"],
            "minRoadM": run["minRoad"], "minConeM": min_cone,
            "freeRoad": _free_pace(rows, groups), "zones": zones}


def rank(runs):
    """Lexicographic batch fitness; only finished, adequately cleared laps earn a pace bonus.

    Use the same scenario set for every candidate. Margins are conservative simulation
    scoring thresholds, separate from Unity's terminal contact rule.
    """
    metrics = [measure(r) for r in runs]
    if not metrics:
        raise ValueError("At least one trial is required")
    fractions = [min(1., m["progressM"] / m["courseM"]) for m in metrics]
    accepted = [m for m in metrics if m["finished"] and m["minRoadM"] >= ROAD_MARGIN_M
                and (m["minConeM"] is None or m["minConeM"] >= CONE_MARGIN_M)]
    sec = sum(m["freeRoad"]["seconds"] for m in accepted)
    speed = sum(m["freeRoad"]["meters"] for m in accepted) / sec if sec > 0 else 0.
    return (sum(m["finished"] for m in metrics), len(accepted),
            sum(z["passed"] for m in metrics for z in m["zones"]), min(fractions),
            sum(fractions) / len(fractions), speed)


def compare(before, after):
    """Compare controller versions using identical spawn, course, vehicle and sensing setup."""
    for key in ("seed", "start", "offset", "yaw", "obstacles", "limit", "vision", "model"):
        if before["request"][key] != after["request"][key]:
            raise ValueError(f"Unmatched run setting: {key}")
    for key in ("length", "props", "profile", "coupled", "physicsStep", "unity"):
        if before[key] != after[key]:
            raise ValueError(f"Unmatched simulator setting: {key}")
    metrics = [measure(r) for r in (before, after)]
    end = min(m["progressM"] for m in metrics)
    free = [_free_pace(np.asarray(r["rows"]), _groups(r), end) for r in (before, after)]
    delta = free[1]["mps"] - free[0]["mps"] if all(v["mps"] is not None for v in free) else None
    return {"before": metrics[0], "after": metrics[1], "matchedUntilM": end,
            "matchedFreeRoad": free, "matchedFreeSpeedDeltaMps": delta}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path, nargs="?")
    args = parser.parse_args()
    a = json.loads(args.before.read_bytes())
    result = compare(a, json.loads(args.after.read_bytes())) if args.after else measure(a)
    print(json.dumps(result, indent=2, allow_nan=False))
