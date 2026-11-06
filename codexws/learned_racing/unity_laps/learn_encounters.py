"""Paired local action rollouts and frozen contextual return model for A4 Unity."""
import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path

import numpy as np

from risk_trials import HERE, Lab, SIM, detector, write
from scenario_shots import geometry

ROOT=HERE/"learned_revision"
FROZEN=json.loads((HERE/"risk_revision/frozen.json").read_bytes())
LENGTH=geometry()[-1]
# Layout input to the Unity referee only. Relative stations are measured from
# the seeded spawn; no layout parameter enters Encounter.cs.
CASES={
    "t01":(25,2.4,[[24,-1.65,2.05,.42,-4.05]]),
    "t02":(32,2.3,[[22,-.35,.55,.42,-3.65]]),
    "t03":(36,2.6,[[20,-1.35,1.90,.54,-2.20]]),
    "t04":(25,2.0,[[23,.65,1.30,.65,1.1]]),
    "t05":(17,2.8,[[17,.25,0,0,0]]),
    "t06":(58,2.6,[[26,-.15,0,0,0]]),
    "t07":(8,2.2,[[21,.55,0,0,0],[24,.2,.85,.62,.8]]),
    "t08":(36,3.3,[[22,.55,0,0,0],[25,-.35,1.25,.72,-2.1]]),
    "t09":(32,3.9,[[27,-1.8,2.15,.52,-2.85]]),
    "t10":(17,2.2,[[18,-1.9,1.70,.67,-3.5]]),
    "t11":(58,2.7,[[22,-.75,1.55,.45,1.6]]),
    "t12":(36,2.4,[[24,-1.65,2.05,.42,-1.45]]),
}
NAMES=["baseline","same_line_fast","inner_slow","inner_fast","outward_slow","same_line_slow"]
FEATURES=["lateral_m","lateral_velocity_mps","extrapolated_lateral_m","range_m","track_count","observed_min_lateral_m","observed_max_lateral_m","observed_curvature_per_m","actual_speed_mps","yaw_rate_rps","current_offset_m"]

def codebook():
    paths=sorted((HERE/"risk_revision/runs").glob("frozen_*_traffic.json")); close=[]
    for p in paths:
        close.extend(q for q in json.loads(p.read_bytes())["decisions"] if any(.3<o[0]<8 for o in q["tracks"]))
    lateral=np.array([q["preferred"] for q in close]);speeds=[q["speedCmd"] for q in close]
    slow,fast=np.quantile(speeds,[.45,.90]);inner=np.quantile(lateral[lateral<0],.20);out=np.median(lateral[lateral>0])
    return dict(actions=[[0.,0.],[0.,fast],[inner,slow],[inner,fast],[out,slow],[0.,slow]],
                actionNames=NAMES,featureNames=FEATURES,quantileSource=[p.name for p in paths],observations=len(close))

def digest(data):return hashlib.sha256(json.dumps(data,sort_keys=True,separators=(",",":")).encode()).hexdigest()

def drive(lab,name,case,arm,config,port,model,film):
    seed,cap,layout=case[:3];specs=[[p[0]/LENGTH,*p[1:]] for p in layout]
    limit=(case[3] if len(case)>3 else max(p[0] for p in layout)+13)/LENGTH
    return lab.run(name,FROZEN["weights"],seed,vision=port,media=film,limit=limit,capture=.05,model=model,obstacle_specs=specs,
                   timing=dict(mode="both",approach=cap),encounter=config)

def train_rollouts(args):
    lab=Lab(SIM,args.output);book=codebook();write(args.output/"actions.json",book)
    with detector(lab,"outer-edge","training",film=False) as (port,release):
        if release!=FROZEN["model"]:raise ValueError("A4 release mismatch")
        for case in args.ids or CASES:
            for idx in range(len(NAMES)):
                cfg=dict(mode="probe",modelId="action_probes",actions=book["actions"],forced=idx)
                drive(lab,case+"_a"+str(idx),CASES[case],idx,cfg,port,release,False)

def selected(run):
    return next((d for d in run["decisions"] if (d.get("encounter") or {}).get("state")=="selected"),None)

def outcome(run):
    start=min(run["zoneStart"]);end=max(run["zoneEnd"])
    successful=run["status"]=="segment" and min(run["zoneEnd"])>=0
    return dict(id=run["id"],success=successful,minMargin=min(run["minGap"],run["minRoad"]),minCone=run["minGap"],
                zoneTime=end-start if successful else None,status=run["status"],seconds=run["seconds"])

def predictive_scores(features,returns,scales,width,query):
    d2=np.sum(((features-query)/scales)**2,axis=1)
    weights=np.exp(-(d2-d2.min())/(2*width**2))
    return weights@returns/weights.sum()

def fit(args):
    from itertools import product
    xs=[];rows=[];keys=[];hashes={};unobserved=[]
    book=codebook()
    for case in CASES:
        batch=[]
        for i in range(len(NAMES)):
            path=args.output/"runs"/f"{case}_a{i}.json"
            hashes[path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
            batch.append(json.loads(path.read_bytes()))
        trigger=selected(batch[0])
        if trigger is None:
            if any(selected(r) for r in batch):raise ValueError("Unbalanced input context")
            unobserved.append(case);continue
        common=trigger["encounter"]["features"]
        assert all(selected(r)["encounter"]["features"]==common and selected(r)["t"]==trigger["t"] for r in batch)
        xs.append(common);rows.append([outcome(r) for r in batch]);keys.append(case)
    xs=np.array(xs);successful=[z for batch in rows for z in batch if z["success"]]
    clearance=float(np.quantile([z["minMargin"] for z in successful],.25))
    # Utility is dimensionless. A contact or departure occupies a worse tier
    # than a completed tight pass, and a completed pass above the empirical
    # lower-quartile clearance is preferred. Within a tier score zone efficiency.
    def utility(z,batch,threshold):
        if not z["success"]:return -4.
        times=[v["zoneTime"] for v in batch if v["success"]]
        efficiency=min(times)/z["zoneTime"]
        tier=1. if z["minMargin"]>=threshold else 0.
        return tier+.65*efficiency+.10*min(1.,max(0.,z["minMargin"])/threshold)
    y=np.array([[utility(z,b,clearance) for z in b] for b in rows])
    scales=np.std(xs,axis=0)
    scales=np.where(scales<1e-6,1.,scales)
    feature_sets={
        "full":range(len(FEATURES)),
        "geometry_motion_speed":[0,1,3,4,5,6,7,8],
        "forecast_shape":[0,2,4,5,6,7,8],
        "motion_geometry":[0,1,2,4,7],
        "crossing_state":[0,1,2,7,8,10],
    }
    # Refit even the risk tier and feature scaling without the withheld
    # layout. The final deployment table is then re-fit on all training groups.
    folds=[]
    for i in range(len(y)):
        mask=np.arange(len(y))!=i
        train_rows=[rows[j] for j in np.where(mask)[0]]
        threshold=float(np.quantile([z["minMargin"] for batch in train_rows for z in batch if z["success"]],.25))
        spread=np.std(xs[mask],axis=0);spread=np.where(spread<1e-6,1.,spread)
        folded=np.array([[utility(z,b,threshold) for z in b] for b in train_rows])
        folds.append((mask,threshold,spread,folded))
    attempts=[]
    for (name,cols),width in product(feature_sets.items(),[.35,.65,1.,1.50,2.5,4.]):
        s=np.array([v if i in cols else 1e9 for i,v in enumerate(scales)])
        chosen=[];validation=[]
        for i,(mask,threshold,spread,folded) in enumerate(folds):
            subset=np.array([v if j in cols else 1e9 for j,v in enumerate(spread)])
            pick=int(np.argmax(predictive_scores(xs[mask],folded,subset,width,xs[i])))
            chosen.append(pick);validation.append(utility(rows[i][pick],rows[i],threshold))
        outs=[b[a] for a,b in zip(chosen,rows)]
        attempts.append(dict(featureSet=name,width=width,scales=s.tolist(),choices=chosen,
                         complete=sum(z["success"] for z in outs),adequate=sum(z["minMargin"]>=folds[i][1] for i,z in enumerate(outs)),
                         meanUtility=float(np.mean(validation))))
    # Leave one entire physical layout out, including all its paired actions.
    best=max(attempts,key=lambda v:(v["complete"],v["adequate"],v["meanUtility"]))
    model=dict(mode="learned",actions=book["actions"],contexts=xs.tolist(),returns=y.tolist(),
               scales=best["scales"],width=best["width"])
    model["modelId"]=digest(model)
    write(args.output/"model.json",model)
    write(args.output/"training.json",dict(layouts=CASES,caseOrder=keys,outcomes=rows,utilities=y.tolist(),
          featureNames=FEATURES,clearanceQuartileM=clearance,leaveOneLayoutOut=attempts,selected=best,
          neverTriggered=unobserved,rawRunSHA256=hashes,modelId=model["modelId"]))
    print("frozen",model["modelId"],best["featureSet"],best["width"],"LOO",best["complete"],"out of",len(keys),"choices",best["choices"],flush=True)

def parity(args):
    lab=Lab(SIM,args.output);req=json.loads((HERE/"timing_revision/runs/explanatory_go.json").read_bytes())
    with detector(lab,"outer-edge","default_parity") as (port,release):
        r=drive(lab,"default_parity",CASES["t01"],0,None,port,release,False)
    keys=lambda r:[{k:v for k,v in d.items() if k!="encounter"} for d in r["decisions"]]
    same_physics=r["rows"]==req["rows"];same_decisions=keys(r)==keys(req)
    write(args.output/"parity.json",dict(source=req["id"],reference=req["request"]["solver"],solver=r["request"]["solver"],
          physicsTraceExact=same_physics,decisionTraceExact=same_decisions))
    assert same_physics and same_decisions

def evaluate_cases(args,matrix):
    lab=Lab(SIM,args.output);file=args.output/"model.json";cfg=json.loads(file.read_bytes())
    model_id=cfg.pop("modelId");assert digest(cfg)==model_id;cfg["modelId"]=model_id
    def work(label,case,who,film,port,release):
        if not os.environ.get("AIFORMULA_LAP_COPY") or release!=FROZEN["model"]:raise ValueError("Unexpected test environment")
        data=cfg if who=="learned" else None
        return drive(lab,label+"_"+who,case,who,data,port,release,film)
    with ExitStack() as stack:
        shared=None
        for label,case,order,film in matrix:
            for who in order:
                if film:
                    with detector(lab,"outer-edge",label+"_"+who,film=True) as (port,release):
                        work(label,case,who,film,port,release)
                else:
                    if not shared:shared=stack.enter_context(detector(lab,"outer-edge","evaluation"))
                    work(label,case,who,film,*shared)

def development(args):
    evaluate_cases(args,[("dev_"+k,CASES[k],["base","learned"],args.film) for k in args.ids])

def external(args):
    matrix=json.loads(args.matrix.read_bytes())
    assert matrix["schema"]==1 and abs(matrix["courseLengthForFractionsM"]-LENGTH)<.05
    groups=[]
    for i,c in enumerate(matrix["cases"]):
        if args.ids and c["id"] not in args.ids:continue
        group=(c["seed"],c["approach"],c["props"],c["limitM"])
        order=["base","learned"] if i%2==0 else ["learned","base"]
        groups.append((c["id"],group,order,args.film))
    evaluate_cases(args,groups)


def main():
    p=argparse.ArgumentParser();p.add_argument("cmd",choices=["train","fit","eval","external","parity"])
    p.add_argument("ids",nargs="*");p.add_argument("--film",action="store_true")
    p.add_argument("--output",type=Path,default=ROOT);p.add_argument("--matrix",type=Path)
    a=p.parse_args()
    if a.cmd=="train":
        if not os.environ.get("AIFORMULA_LAP_COPY"):raise RuntimeError("Private Unity required")
        return train_rollouts(a)
    {"fit":fit,"eval":development,"external":external,"parity":parity}[a.cmd](a)
if __name__=="__main__":main()
