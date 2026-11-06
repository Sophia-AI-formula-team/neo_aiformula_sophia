"""Cross-check deployed forest/controls, synchronized cameras and genuine trial counts."""
import argparse
import hashlib
import json
from collections import Counter
from pathlib import Path

import numpy as np
from learned_audit import images
from smooth_metrics import metrics
from smooth_trials import ROOT,NAMES

def predictions(config,features):
    vals=[]
    for tree in config["forest"]:
        n=tree[0]
        while n[0]>=0:
            n=tree[int(n[2] if np.float32(features[int(n[0])])<=np.float32(n[1]) else n[3])]
        vals.append(n[1:])
    return np.asarray(vals,dtype=np.float32).sum(axis=0,dtype=np.float32)/len(vals)

def audits(root,only=None):
    raw=root/"runs"; pairs={}
    for p in sorted(raw.glob("*_learned.json")):
        name=p.stem.removesuffix("_learned")
        if only and not any(name.startswith(k)for k in only):continue
        baseline=raw/(name+"_base.json")
        if not baseline.exists():continue
        a=json.loads(baseline.read_bytes());b=json.loads(p.read_bytes())
        req=b["request"].copy();cfg=req.pop("smooth");assert req==a["request"]
        choices=[];max_error=0.;first=None
        for d in b["decisions"]:
            e=d.get("smooth") or {}
            if e.get("features") is None or d["reason"]=="insufficient_lane":continue
            if first is None:first=d["t"]
            scores=predictions(cfg,e["features"]);actual=e["actionScores"]
            max_error=max(max_error,float(np.max(abs(scores-actual))))
            assert max_error<3e-5 and int(np.argmax(scores))==e["chosen"],(name,d["t"],actual,scores)
            choices.append(dict(t=d["t"],actualSpeedMps=d["speed"],chosen=NAMES[e["chosen"]],targetOffsetM=e["targetOffset"],
                                 targetSpeedMps=e["targetSpeed"] if e["chosen"] else None,decisionScores=actual))
        cutoff=float("inf")if first is None else first
        def prefix(r):return [[d[k]for k in ("t","observed","tracks","speedCmd","yawCmd","rightCan","leftCan")]for d in r["decisions"]if d["t"]<cutoff]
        assert prefix(a)==prefix(b),(name,"visual/command prefix changed")
        pairs[name]=dict(matchedRequest=True,prefixExact=True,physicsTraceExact=a["rows"]==b["rows"],
                         modelId=cfg["modelId"],maxCSharpScoreDifference=max_error,decisions=choices,
                         baseline=dict(metrics=metrics(a),gates=a["gate"],images=images(root,a)),
                         learned=dict(metrics=metrics(b),gates=b["gate"],images=images(root,b)))
    return pairs

def main():
    p=argparse.ArgumentParser();p.add_argument("--output",type=Path,default=ROOT);p.add_argument("--prefix",nargs="*")
    a=p.parse_args();root=a.output
    pairs=audits(root,a.prefix);events=[json.loads(l)for l in (root/"events.jsonl").read_text(encoding="utf8").splitlines()]
    complete=Counter(e["trial"] for e in events if e["kind"]=="finish")
    assert max(complete.values(),default=0)==1
    stored={p.stem for p in (root/"runs").glob("*.json")}
    assert stored==set(complete),"unlogged job output"
    report=dict(completedUniqueActualRuns=len(stored),training=sum(n.startswith("tr") for n in stored),pairs=pairs,
                recoveredOutputs=[e["trial"] for e in events if e["kind"]=="resume_finished_output"])
    dest=root/("paired_audit.json" if not a.prefix else "paired_audit_subset.json")
    dest.write_text(json.dumps(report,separators=(",",":")),encoding="utf8")
if __name__=="__main__":main()
