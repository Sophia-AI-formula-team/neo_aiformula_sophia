"""Audit the common physical approach and distinct simulator receipts for paired training probes."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def read(path):
    return json.loads(path.read_bytes())


def approach(run):
    decisions = run["decisions"]
    first = next((d for d in decisions if (d.get("smooth") or {}).get("state") == "selected"), None)
    cutoff = first["t"] if first is not None else float("inf")
    frames = [[d[k] for k in ("t","observed","tracks","speedCmd","yawCmd","rightCan","leftCan")]
              for d in decisions if d["t"] < cutoff]
    poses = [row for row in run["rows"] if row[0] < cutoff]
    selection = None if first is None else [first["t"],first["smooth"]["features"]]
    return [selection, poses, frames]


def audit(root, groups, arms):
    ids = [f"tr{i:03}_a{j}" for i in range(1,groups+1) for j in range(arms)]
    events = [json.loads(line) for line in (root/"events.jsonl").read_bytes().splitlines() if line]
    for kind in ("start", "finish"):
        seen = Counter(e.get("trial") for e in events if e.get("kind")==kind)
        if any(seen[key]!=1 for key in ids):
            raise ValueError(f"Missing or repeated trial {kind} events")
    sha = hashlib.sha256()
    finishes, cases, settings, case_hashes = Counter(), [], set(), set()
    frames, sim_seconds = 0, 0.
    for layout in range(1,groups+1):
        reference = None
        passed = []
        measured_gaps = []
        for arm in range(arms):
            key = f"tr{layout:03}_a{arm}"
            blob = (root/"runs"/(key+".json")).read_bytes()
            sha.update(key.encode()+b"\0"+hashlib.sha256(blob).digest())
            run = json.loads(blob)
            params = run["request"].copy()
            actions = params.pop("smooth")
            if (run["id"] != key or actions["mode"]!="probe" or actions["forced"]!=arm
                or not params["vision"] or params["timing"]["approach"]!=0):
                raise ValueError(f"{key}: mismatched probe request")
            # Ephemeral perception server ports can differ after a resumed process.
            params.pop("vision")
            if len(run["decisions"])!=run["seenFrames"] or run["seenFrames"]==0:
                raise ValueError(f"{key}: no complete A4 decision sequence")
            identity = [params, actions["actions"], run["props"], approach(run)]
            if arm==0:
                reference=identity
                seed_and_props = json.dumps([params["seed"],run["props"]],separators=(",",":"))
                if seed_and_props in case_hashes:
                    raise ValueError(f"Duplicated layout {layout}")
                case_hashes.add(seed_and_props)
            elif reference!=identity:
                raise ValueError(f"{key}: unmatched spawn, field, or pre-selection control history")
            settings.add((run["unity"],run["profile"],run["coupled"],run["physicsStep"],params["model"],params["solver"]))
            finishes[run["status"]]+=1
            frames+=run["seenFrames"]
            sim_seconds+=run["seconds"]
            passed.append(run["status"] in ("segment","lap"))
            if passed[-1]:
                measured_gaps.append(run["minGap"])
        cases.append({"layout":layout,"successfulArms":sum(passed),"v2Successful":passed[0],
                      "successfulWithinHalfMeter":sum(0<gap<.5 for gap in measured_gaps)})
    if len(settings)!=1:
        raise ValueError("Training simulator settings or A4 detector changed")
    return {"groups":groups,"arms":arms,"uniquePhysicsRuns":len(ids),
            "distinctSpawnAndProps":len(case_hashes),"matchedPreSelectionHistories":groups,
            "trainingTimeInSimulationSeconds":sim_seconds,"A4Decisions":frames,"status":dict(finishes),
            "atLeastOneSuccess":sum(c["successfulArms"]>0 for c in cases),
            "baselineSuccess":sum(c["v2Successful"] for c in cases),
            "allArmsFailed":[c["layout"] for c in cases if not c["successfulArms"]],
            "successfulWithinHalfMeter":sum(c["successfulWithinHalfMeter"] for c in cases),
            "orderedSourceSHA256":sha.hexdigest(),"simulatorAndDetector":list(settings.pop())}


if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("root",type=Path)
    p.add_argument("output",type=Path)
    p.add_argument("--groups",type=int,required=True)
    p.add_argument("--arms",type=int,required=True)
    options=p.parse_args()
    report=audit(options.root,options.groups,options.arms)
    options.output.write_bytes((json.dumps(report,indent=2,allow_nan=False)+"\n").encode())
