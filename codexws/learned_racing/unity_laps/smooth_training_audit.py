"""Reconstruct action-probe training targets from raw simulator rows without using the fitter."""
import argparse
import json
from pathlib import Path
import re

import numpy as np
from scenario_shots import geometry
from smooth_fixed_effect import COSTS, constant_action
from smooth_holdout_score import PROTOCOL, load, sha


def values(run,track):
    center,station,angle,k,L=track
    rows=np.asarray(run["rows"]); t=rows[:,0]; s=rows[:,7]
    idx=np.argmin(np.linalg.norm(center-rows[0,1:3],axis=1))
    tangent=np.array([np.cos(angle[idx]),np.sin(angle[idx])])
    start=station[idx]+(rows[0,1:3]-center[idx])@tangent
    x=(start+s)%L
    road=np.interp(x,station,angle)
    residual=np.unwrap(np.pi/2-rows[:,3]-road)
    clear=(s>=10)&(np.abs(np.interp(x,station,k))<=.015)
    zone=np.zeros(len(s),bool)
    for prop in run["props"]:
        delta=s-prop[7]
        zone |= (-6<=delta)&(delta<=8)
        clear &= np.abs(delta)>=12.
    clear=clear[:-1]&clear[1:];zone=zone[:-1]&zone[1:]
    ds=np.maximum(0.,np.diff(s)); zdist=ds[zone].sum();dt=np.diff(t)
    free_t=dt[clear].sum();free_s=ds[clear].sum();free_v=free_s/free_t if free_t>.01 else None
    success=run["status"] in ("segment","lap") and all(t>=0 for t in run["zoneEnd"])
    costs=None
    if zdist>.01:
        curves=np.column_stack([np.abs(np.diff(residual)),np.abs(np.diff(rows[:,6])),np.abs(np.diff(rows[:,8])),
                                 np.linalg.norm(np.diff(rows[:,1:3],axis=0),axis=1)])
        costs=curves[zone].sum(axis=0)/zdist
        costs[3]=max(0.,costs[3]-1.)
    return dict(success=success,zoneTime=float(dt[zone].sum()),costs=costs,cap=min(.5,max(0.,run["minGap"])),freeS=free_s,freeV=free_v)


def audit(root,ckname,plan,dev=None):
    ck=root/"checkpoints"/ckname
    training=load(ck/"training.json");scale=np.array([training["scales"][k] for k in COSTS])
    track=geometry();all_seeds=set();nreads=multi=0;max_cost_error=0.;table=[]
    for case in training["outcomes"]:
        obs=[]
        for i,meta in enumerate(case["outcomes"]):
            name=case["id"]+f"_a{i}.json";path=root/"runs"/name;raw=load(path)
            if sha(path)!=training["rawRunSHA256"][name]:raise ValueError("Probe digest mismatch "+name)
            all_seeds.add(raw["request"]["seed"]);cfg=raw["request"]["smooth"]
            if cfg["mode"]!="probe" or cfg["forced"]!=i:raise ValueError("Unmatched forced action "+name)
            selections=[d for d in raw["decisions"] if (d.get("smooth") or {}).get("features") is not None]
            first=[selections[0]["t"],selections[0]["smooth"]["features"]] if selections else None
            if first!=case["trigger"]:raise ValueError("Initial observation mismatch "+name)
            if any(d["smooth"]["chosen"]!=i for d in selections):raise ValueError("Forced actions not held "+name)
            nreads+=1;multi+=len(selections)>1
            v=values(raw,track);obs.append(v)
            if v["success"]!=meta["completed"] or abs(v["zoneTime"]-meta["zoneTimeS"])>1e-6:
                raise ValueError("Probe completion or elapsed time mismatch "+name)
            if v["costs"] is not None:
                err=np.max(np.abs(v["costs"]-[meta[k] for k in COSTS]));max_cost_error=max(max_cost_error,float(err))
        if case["trigger"] is None:continue
        times=[v["zoneTime"] for v in obs if v["success"]];out=[]
        for v in obs:
            if not v["success"]:
                out.append(-float(training["selected"]["spec"][2]));continue
            turn=np.dot([.35,.30,.20,.15],1/(1+v["costs"]/scale))
            slow=max(0.,4.-v["freeV"]) if v["freeS"]>=8 else 0.
            out.append(1.+1.35*turn+1.05*min(times)/v["zoneTime"]+.16*v["cap"]/.5-.75*min(1.,slow))
        table.append(out)
    err=float(np.abs(np.array(table)-training["scoreTable"]).max())
    if err>1e-6 or max_cost_error>1e-6:raise ValueError("Training targets disagree with raw physics")
    test_seeds={c["seed"] for c in plan["cases"]};dev_seeds={c["seed"] for c in dev["cases"]} if dev else set()
    if all_seeds & test_seeds or dev_seeds & (test_seeds|all_seeds):raise ValueError("Train/dev/holdout seed overlap")
    action,ranking=constant_action(training)
    return dict(trainedCheckpoint=ckname,rawProbesChecked=nreads,contexts=len(table),layouts=training["cases"],
                multiSelectionProbeRuns=multi,independentTrainingSeeds=len(all_seeds),developmentSeedsChecked=len(dev_seeds),
                heldOutSeedsChecked=len(test_seeds),seedSetsDisjoint=True,maxCostReconstructionError=max_cost_error,
                maxReturnReconstructionError=err,trainingOnlyFixedChoice=action,fixedRanking=ranking,
                rawModelFileSha256=sha(ck/"model.json"),reconstructedReturnRows=len(table))


if __name__=="__main__":
    ap=argparse.ArgumentParser();ap.add_argument("root",type=Path);ap.add_argument("checkpoint",help="Raw checkpoint, e.g. ck80")
    ap.add_argument("--out",type=Path,required=True);args=ap.parse_args()
    dpath=args.root/"development_layouts.json"
    result=audit(args.root,args.checkpoint,load(PROTOCOL/"layouts.json"),load(dpath) if dpath.exists() else None)
    args.out.write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+"\n",encoding="utf8")
