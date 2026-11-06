"""Development acceptance, fixed-action reference, and training-only abstention calibration."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from smooth_fit import COSTS, matrix, returns
from smooth_metrics import metrics
from smooth_trials import ROOT,NAMES,development_cases,write


def calibration(root,size):
    ck=root/"checkpoints"/f"ck{size:02}";f=json.loads((ck/"training.json").read_bytes())
    cfg=json.loads((ck/"model.json").read_bytes());pred=np.array(f["selected"]["oofScores"])
    groups=[q["outcomes"] for q in f["outcomes"] if q["trigger"] is not None]
    y,_=returns(groups,f["selected"]["spec"][2],list(f["scales"].values()))
    raw=np.argmax(pred,axis=1);advantage=pred-pred[:,[0]]
    # A single calibrated hurdle against the real reference. Derive it entirely
    # from out-of-fold predictions, including the no-intervention choice.
    positive=advantage[np.arange(len(raw)),raw];candidates=np.unique(np.r_[0,positive[positive>0]+1e-6])
    trials=[]
    for cutoff in candidates:
        out=np.where(positive>cutoff,raw,0)
        joined=[(b[0],b[a])for b,a in zip(groups,out)]
        trials.append(dict(cutoff=float(cutoff),completed=sum(b["completed"]for _,b in joined),
                           lost=sum(p["completed"] and not q["completed"]for p,q in joined),
                           benefit=float(np.mean(y[np.arange(len(y)),out]-y[:,0])),changed=int(np.sum(out!=0))))
    admissible=[z for z in trials if z["lost"]==0]
    best=max(admissible,key=lambda z:(z["completed"],z["benefit"],z["changed"]))
    safe=copy.deepcopy(cfg)
    # Entry 1 of a leaf is the score of option 0 (keep V2). Applying this
    # additive hurdle is equivalent to comparing gains with the fitted cutoff.
    for tree in safe["forest"]:
        for node in tree:
            if node[0]<0:node[1]+=best["cutoff"]
    safe.pop("modelId")
    safe["modelId"]=hashlib.sha256(json.dumps(safe,sort_keys=True,separators=(",",":")).encode()).hexdigest()
    folder=root/"checkpoints"/f"ck{size:02}s";write(folder/"model.json",safe)
    write(folder/"calibration.json",dict(source=cfg["modelId"],selected=best,all=trials,modelId=safe["modelId"]))
    return best

def fixed_reference(root,size):
    meta=json.loads((root/"checkpoints"/f"ck{size:02}"/"training.json").read_bytes())
    sc=np.array([meta["scales"][k]for k in COSTS])
    table=[]
    for j,name in enumerate(NAMES):
        q=[z["outcomes"][j]for z in meta["outcomes"] if z["outcomes"][j]["completed"]]
        turn=[1-float(np.dot(1/(1+np.array([r[k]for k in COSTS])/sc),[.35,.30,.20,.15])) for r in q]
        table.append(dict(action=j,name=name,completed=len(q),steeringCost=float(np.mean(turn)),zoneTimeS=float(np.mean([r["zoneTimeS"]for r in q]))))
    best=min(table,key=lambda q:(-q["completed"],q["steeringCost"],q["zoneTimeS"],q["action"]))
    write(root/"fixed_reference.json",dict(trainingLayouts=size,rankedBy=["completion","steeringCost","zoneTimeS","action"],actions=table,chosen=best))
    return best

def straight_gate(run):
    rows=np.array(run["rows"]);s=np.maximum.accumulate(rows[:,7]);t=rows[:,0]
    if s[-1]<34:return None
    start,end=np.interp([12,34],s,t)
    return float(22/(end-start))

def development(root,tags):
    out={};matrix=development_cases()["cases"]
    previous={}
    for tag in ["dev",*tags]:
        values={};gates={};selections={}
        for case in matrix:
            name=case["id"];suffix="base" if tag=="dev"else"learned"
            r=json.loads((root/"runs"/f"{tag}_{name}_{suffix}.json").read_bytes())
            values[name]=metrics(r)
            if name.startswith("dc"):gates[name]=straight_gate(r)
            selections[name]=[d["smooth"]["chosen"] for d in r["decisions"] if (d.get("smooth")or{}).get("state")=="selected"]
        if tag=="dev":previous=values
        matched=[k for k in values if values[k]["completed"] and previous[k]["completed"] and k.startswith("d0")]
        def ratio(key):
            new=np.array([values[k][key]for k in matched]);ref=np.array([previous[k][key]for k in matched])
            return float(new.sum()/ref.sum()) if ref.sum()>0 else None
        turns=[ratio(k)for k in COSTS]
        lost=[k for k in values if previous[k]["completed"] and not values[k]["completed"]]
        closed=all(v is not None and v>=4 for v in gates.values()) and all(values[k]["completed"] for k in values if k.startswith("dl"))
        reduced=any(t is not None and t<.99 for t in turns[:2]) and np.mean(turns[:2])<1.
        out[tag]=dict(completed=sum(v["completed"] for v in values.values()),lost=lost,
                      rescued=[k for k in values if not previous[k]["completed"] and values[k]["completed"]],
                      gates=gates,fullLaps=[k for k in values if k.startswith("dl") and values[k]["completed"]],
                      matchedObstacleSegments=matched,turnRatios=dict(zip(COSTS,turns)),zoneTimeRatio=ratio("zoneTimeS"),
                      satisfiesDevelopment=bool(closed and not lost and reduced),choices=selections,metrics=values)
    write(root/"development.json",out)
    if tags:
        def ranking(k):
            q=out[k];r=q["turnRatios"]
            return (q["satisfiesDevelopment"],len(q["fullLaps"]),q["completed"],-len(q["lost"]),
                    -(.35*r[COSTS[0]]+.30*r[COSTS[1]]+.20*r[COSTS[2]]+.15*min(2.,r[COSTS[3]])),-q["zoneTimeRatio"])
        chosen=max(tags,key=ranking)
        source=root/"checkpoints"/chosen/"model.json"
        write(root/"frozen.json",dict(checkpoint=chosen,sourceCodeSolver=json.loads((root/"baseline.json").read_bytes())["solverHash"],
              modelFileSHA256=hashlib.sha256(source.read_bytes()).hexdigest(),selectedModel=json.loads(source.read_bytes())["modelId"],
              meetsDevelopment=out[chosen]["satisfiesDevelopment"],tested=tags,coursePreviouslySeen=True))
        return chosen

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("cmd",choices=["calibrate","fixed","evaluate"]);p.add_argument("--size",type=int,default=80)
    p.add_argument("--output",type=Path,default=ROOT);p.add_argument("--tags",nargs="+");a=p.parse_args()
    if a.cmd=="calibrate":print(calibration(a.output,a.size))
    if a.cmd=="fixed":print(fixed_reference(a.output,a.size))
    if a.cmd=="evaluate":print(development(a.output,a.tags or []))
