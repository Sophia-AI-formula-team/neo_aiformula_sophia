"""Fit quantitative action returns and a small policy forest to paired A4 Unity encounters."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from sklearn.ensemble import ExtraTreesRegressor

from smooth_metrics import metrics
from smooth_trials import ROOT,NAMES,FEATURES,bank,write

COSTS=("headingResidualTVperM","yawCommandTVperM","lateralTravelPerM","extraPathPerM")
def matrix(root,size):
    all_cases=[];contexts=[];outcomes=[];hashes={}
    for layout in range(1,size+1):
        label=f"tr{layout:03}";batch=[];triggers=[]
        for a in range(len(NAMES)):
            p=root/"runs"/f"{label}_a{a}.json";hashes[p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
            run=json.loads(p.read_bytes());batch.append(metrics(run))
            d=next((d for d in run["decisions"] if (d.get("smooth") or {}).get("features") is not None),None)
            triggers.append(None if d is None else (d["t"],d["smooth"]["features"]))
        if not all(t==triggers[0]for t in triggers):raise ValueError("Nonidentical approach "+label)
        all_cases.append(dict(id=label,outcomes=batch,trigger=triggers[0]))
        if triggers[0] is not None:contexts.append(triggers[0][1]);outcomes.append(batch)
    return all_cases,np.array(contexts),outcomes,hashes

def returns(samples,failure_penalty,scales=None):
    if scales is None:
        good=[q for b in samples for q in b if q["completed"] and q["headingResidualTVperM"] is not None]
        scales=[max(1e-5,float(np.quantile([q[k] for q in good],.65))) for k in COSTS]
    table=[]
    for b in samples:
        times=[q["zoneTimeS"] for q in b if q["completed"]];row=[]
        for q in b:
            if not q["completed"]:row.append(-float(failure_penalty));continue
            scaled=np.array([q[k] for k in COSTS])/scales
            smooth=float(np.dot(1/(1+scaled),[.35,.30,.20,.15]))
            certified_free=q["freeDistanceM"]>=8
            cruise=max(0.,4.-q["freeAverageMps"]) if certified_free else 0.
            clearance=q["positiveCappedConeM"]/.5
            row.append(1.+1.35*smooth+1.05*min(times)/q["zoneTimeS"]+.16*clearance-.75*min(1.,cruise))
        table.append(row)
    return np.array(table),scales

def estimator(spec):
    return ExtraTreesRegressor(n_estimators=40,max_depth=spec[0],min_samples_leaf=spec[1],random_state=911260,n_jobs=-1)

def export_forest(est):
    forest=[]
    for tree in est.estimators_:
        t=tree.tree_;nodes=[]
        for j in range(t.node_count):
            if t.children_left[j]<0:nodes.append([-1,*t.value[j,:,0].astype(float).tolist()])
            else:nodes.append([int(t.feature[j]),float(t.threshold[j]),int(t.children_left[j]),int(t.children_right[j])])
        forest.append(nodes)
    return forest

def fit(root,size):
    cases,x,obs,hashes=matrix(root,size)
    rng=np.random.default_rng(199711);order=rng.permutation(len(x));folds=np.array_split(order,5)
    specs=[(d,leaf,p) for d in (3,5,8) for leaf in (1,3) for p in (5.,10.)]
    attempts=[]
    for spec in specs:
        choices=np.zeros(len(x),int);scores=[];predicted=np.zeros((len(x),len(NAMES)))
        for test in folds:
            train=np.ones(len(x),bool);train[test]=False
            y,sc=returns([b for i,b in enumerate(obs)if train[i]],spec[2]);model=estimator(spec).fit(x[train],y)
            predicted[test]=model.predict(x[test]);choices[test]=np.argmax(predicted[test],axis=1)
            test_y,_=returns([obs[i]for i in test],spec[2],sc);scores.extend(test_y[np.arange(len(test)),choices[test]].tolist())
        pairs=[(b[0],b[j])for b,j in zip(obs,choices)]
        attempts.append(dict(spec=spec,choices=choices.tolist(),oofScores=predicted.tolist(),completed=sum(r["completed"]for _,r in pairs),
                   newFailure=sum(a["completed"] and not b["completed"]for a,b in pairs),meanUtility=float(np.mean(scores)),
                   changed=int(np.sum(choices!=0))))
    best=max(attempts,key=lambda a:(a["completed"],-a["newFailure"],a["meanUtility"]))
    y,sc=returns(obs,best["spec"][2]);fit_model=estimator(best["spec"]).fit(x,y)
    cfg=dict(mode="smooth",actions=bank(),forest=export_forest(fit_model))
    cfg["modelId"]=hashlib.sha256(json.dumps(cfg,sort_keys=True,separators=(",",":")).encode()).hexdigest()
    ck=root/"checkpoints"/f"ck{size:02}";write(ck/"model.json",cfg)
    persisted=[q["outcomes"]for q in cases]
    write(ck/"training.json",dict(trainingIds=list(hashes),rawRunSHA256=hashes,featureNames=FEATURES,scales=dict(zip(COSTS,sc)),
          folds=[f.tolist() for f in folds],validation=attempts,selected=best,outcomes=cases,scoreTable=y.tolist(),modelId=cfg["modelId"],
          cases=size,actionCount=len(NAMES),triggered=len(x),baselineCompleted=sum(b[0]["completed"]for b in persisted),
          anyCandidateCompleted=sum(any(r["completed"]for r in b)for b in persisted),allFailed=[c["id"]for c in cases if not any(r["completed"]for r in c["outcomes"])]))
    print(ck,best["spec"],best["completed"],"/",len(x),cfg["modelId"],flush=True)
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("size",type=int);p.add_argument("--output",type=Path,default=ROOT)
    a=p.parse_args();fit(a.output,a.size)
