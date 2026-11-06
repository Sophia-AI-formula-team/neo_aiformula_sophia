"""Print the observed speeds before passing in the seven frozen-policy films.

Run: python audit_film_decisions.py PATH/TO/unity_laps
Reads only obstacle_films/cases.json and the named Unity run logs. No dependencies.
"""
import json
import sys
from bisect import bisect_left
from collections import Counter
from pathlib import Path

# Zero-based indices into each run's injected props; other cones along the lap are excluded.
CASE_PROPS = {"static": (0,), "pair": (1, 2), "moving": (3,), "mixed": (0, 1),
              "apex": (0,), "apex-mixed": (0, 1), "contact": (3,)}


def crossing_time(station, rows):
    """Interpolate the time at which the car centre reaches a cone's longitudinal station."""
    progress = [row[7] for row in rows]
    assert all(a <= b for a, b in zip(progress, progress[1:]))
    i = bisect_left(progress, station)
    if i == len(rows):
        return None
    if i == 0 or progress[i] == station:
        return rows[i][0]
    fraction = (station - progress[i-1]) / (progress[i] - progress[i-1])
    return rows[i-1][0] + fraction * (rows[i][0] - rows[i-1][0])


def main(root):
    cases = json.loads((root / "obstacle_films/cases.json").read_bytes())
    assert set(c["id"] for c in cases) == set(CASE_PROPS)
    pooled = []
    print("clip | ids0 | trial | first yield (s) | car-centre crossing (s) | cutoff (exclusive, s) | decisions | yield | recover | ratio=1 | yield cmd>actual")
    for case in cases:
        run = json.loads((root / case["captureRoot"] / "runs" / (case["trial"] + ".json")).read_bytes())
        ids = CASE_PROPS[case["id"]]
        crossings = [crossing_time(run["props"][i][7], run["rows"]) for i in ids]
        # Contact stops before the centre reaches the moving cone's station.
        if crossings[-1] is None:
            assert case["id"] == "contact" and run["status"] == "obstacle"
        end = crossings[-1] if crossings[-1] is not None else run["seconds"]
        film = [d for d in run["decisions"] if case["start"] <= d["t"] <= case["end"]]
        start = next(d["t"] for d in film if d["reason"] == "yield")
        window = [d for d in film if start <= d["t"] < end]
        yielding = [d for d in window if d["reason"] == "yield"]
        pooled.extend(window)
        times = ", ".join(f"{t:.4f}" if t is not None else "never (contact)" for t in crossings)
        print(case["id"], list(ids), run["id"], f"{start:.2f}", times, f"{end:.4f}", len(window),
              len(yielding), sum(d["reason"] == "recover" for d in window),
              sum(d["ratio"] == 1 for d in window), sum(d["speedCmd"] > d["speed"] for d in yielding), sep=" | ")
    yielding = [d for d in pooled if d["reason"] == "yield"]
    print("Decision counts by (reason, speed ratio):", dict(Counter((d["reason"], d["ratio"]) for d in pooled)))
    print("Total / yield / yield with target strictly below measured:", len(pooled), len(yielding),
          sum(d["speedCmd"] < d["speed"] for d in yielding))
    print("Largest (target - measured) during yield [m/s]:", max(d["speedCmd"] - d["speed"] for d in yielding))
    print("Limiting term during yield:", dict(Counter("curvature" if d["cruise"] < d["guardCap"] else "risk cap" for d in yielding)))


if __name__ == "__main__":
    main(Path(sys.argv[1]))
