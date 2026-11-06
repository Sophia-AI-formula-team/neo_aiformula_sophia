"""Reconstruct the frozen context/action table from independent Unity run evidence."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def load(path):
    return json.loads(path.read_bytes())


def train_audit(folder, layouts):
    book = load(folder/"actions.json")
    train = load(folder/"training.json")
    model = load(folder/"model.json")
    content = {k:v for k,v in model.items() if k!="modelId"}
    encoded = json.dumps(content, sort_keys=True, separators=(",", ":")).encode()
    assert hashlib.sha256(encoded).hexdigest() == model["modelId"] == train["modelId"]
    old_seeds = {spec[0] for spec in train["layouts"].values()}
    new_seeds = {spec["seed"] for spec in load(layouts)["cases"]}
    assert not (old_seeds & new_seeds)
    assert book["actions"] == model["actions"]
    groups = {}
    seen, never = [], []
    files, probes, trigger_counts = 0, 0, []
    for case in train["layouts"]:
        outputs, features = [], []
        for index,action in enumerate(book["actions"]):
            path = folder/"runs"/f"{case}_a{index}.json"
            data = path.read_bytes()
            assert train["rawRunSHA256"][path.name] == hashlib.sha256(data).hexdigest()
            run = json.loads(data)
            config = run["request"]["encounter"]
            assert config["mode"] == "probe" and config["forced"] == index and config["actions"][index] == action
            chosen = [d for d in run["decisions"] if d["encounter"]["state"] == "selected"]
            assert all(d["encounter"]["chosen"]==index for d in chosen)
            trigger_counts.append(len(chosen))
            features.append((chosen[0]["t"], chosen[0]["encounter"]["features"]) if chosen else None)
            success = run["status"] == "segment" and all(t >= 0 for t in run["zoneEnd"])
            zone = max(run["zoneEnd"])-min(run["zoneStart"]) if success else None
            outputs.append((success, min(run["minGap"],run["minRoad"]), zone))
            files += 1
            probes += int(bool(chosen))
        assert all(f==features[0] for f in features)
        if features[0] is None:
            never.append(case)
            continue
        assert np.array_equal(model["contexts"][len(seen)], features[0][1])
        seen.append(case)
        groups[case] = outputs
    assert seen==train["caseOrder"] and never==train["neverTriggered"]
    quartile = float(np.quantile([m for g in groups.values() for good,m,_ in g if good], .25))
    assert abs(quartile-train["clearanceQuartileM"])<1e-8
    reward=[]
    for group in groups.values():
        times = [t for good,_,t in group if good]
        row = []
        for good,m,t in group:
            if good:
                row.append(float(m>=quartile)+.65*min(times)/t+.10*np.clip(m/quartile,0,1))
            else:
                row.append(-4.)
        reward.append(row)
    assert np.allclose(reward, model["returns"], rtol=0, atol=1e-8)
    return dict(modelId=model["modelId"], modelFileSha256=hashlib.sha256((folder/"model.json").read_bytes()).hexdigest(),
                recordedTrainingFiles=files, firstSelectionContexts=len(seen), probeRunsWithSelection=probes,
                runsWithMultipleSelections=sum(v>1 for v in trigger_counts), neverSelectedCases=never, trainingSeeds=sorted(old_seeds),
                disjointHoldoutSeeds=True, embeddedReturnMaxError=float(np.max(np.abs(np.array(reward)-model["returns"]))),
                developmentQuartileM=quartile, selectedFeatureSet=train["selected"]["featureSet"],
                activeFeatures=[name for name,scale in zip(book["featureNames"],model["scales"]) if scale<1e8],
                width=model["width"], learnedByAction=book["actionNames"], codebook=book["actions"],
                codebookSourceFiles=book["quantileSource"], sourceObservations=book["observations"])


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("evidence", type=Path)
    parser.add_argument("--layouts", type=Path, default=Path(__file__).parent/"learned_holdout/layouts.json")
    parser.add_argument("--out", type=Path, required=True)
    args=parser.parse_args()
    args.out.write_text(json.dumps(train_audit(args.evidence,args.layouts),ensure_ascii=False,indent=2)+"\n",encoding="utf8")
