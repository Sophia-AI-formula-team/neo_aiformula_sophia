"""Secondary steering check on the common free-road portions of jointly qualified trials."""
import argparse
import json
from math import sqrt
from pathlib import Path

from learned_holdout_score import read
from risk_metrics import free_steering


def audit(runs, result):
    pairs = []
    for pair in result["pairs"]:
        if not (pair["base"]["safe"] and pair["learned"]["safe"]):
            continue
        records = [read(runs / f"{pair['id']}_{arm}.json") for arm in ("base", "learned")]
        limit = min(r["progress"] for r in records)
        pairs.append({"id": pair["id"], "endProgressM": limit,
                      "arms": [free_steering(r, limit) for r in records]})
    pooled = []
    for arm in (0, 1):
        rows = [pair["arms"][arm] for pair in pairs]
        distance = sum(row["meters"] for row in rows)
        seconds = sum(row["seconds"] for row in rows)
        reversals = sum(row["rapidReturns"] for row in rows)
        pooled.append({
            "meters": distance, "seconds": seconds, "rapidReturns": reversals,
            "rapidReturnsPer100m": 100*reversals/distance if distance else None,
            "rmsYawAccelerationRadPerS2": sqrt(sum(row["squaredRateTime"] for row in rows)/seconds)
                                         if seconds else None,
        })
    return {"arms": ["base", "learned"], "scope": "Descriptive matched free road on pairs safe in both arms. "
            "Uses the pre-existing free_steering definition; does not alter the registered selection score.",
            "pairs": pairs, "pooled": pooled}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", type=Path)
    parser.add_argument("results", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(audit(args.runs, read(args.results)), indent=2, allow_nan=False)+"\n",
                           encoding="utf8")
