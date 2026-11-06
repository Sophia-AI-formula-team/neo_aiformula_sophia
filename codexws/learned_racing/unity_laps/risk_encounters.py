"""Second policy, fit only to V1 training groups plus sampled practice variants."""
import argparse
import hashlib
import json
import os
from pathlib import Path

import numpy as np
from learn_encounters import (CASES, FEATURES, FROZEN, HERE, LENGTH, NAMES, ROOT as V1, Lab, SIM, codebook, detector,
                             digest, drive, evaluate_cases, external, outcome, predictive_scores, selected, write)

ROOT=HERE/"learned_risk_revision"

def augmentations():
    rng=np.random.default_rng(116071)
    variants={}
    parents=("t11","t07","t03","t11","t06","t07","t01","t11")
    for i,parent in enumerate(parents):
        seed,cap,original=CASES[parent]
        offset=rng.uniform(-2.5,2.5)
        layout=[]
        for station,lateral,amp,omega,phase in original:
            layout.append([station+offset,lateral+rng.uniform(-.3,.3),amp*rng.uniform(.88,1.12),
                           omega*rng.uniform(.88,1.12),phase+rng.uniform(-1.5,1.5) if amp else 0.])
        variants[f"r{i+1:02}"]=dict(parent=parent,case=(seed,cap+rng.uniform(-.35,.35),layout))
    return variants

def rollouts(args):
    lab=Lab(SIM,args.output);book=codebook()
    variants=augmentations();write(args.output/"practice_layouts.json",variants)
    with detector(lab,"outer-edge","practice") as (port,release):
        if release!=FROZEN["model"]:raise RuntimeError("A4 artifact changed")
        for name,v in variants.items():
            for idx in range(len(NAMES)):
                cfg=dict(mode="probe",modelId="action_probes",actions=book["actions"],forced=idx)
                drive(lab,name+"_a"+str(idx),v["case"],idx,cfg,port,release,False)

def groups(root):
    names=list(CASES)+list(augmentations());parents=[];x=[];out=[];files={}
    for name in names:
        variants=augmentations()
        base=V1 if name in CASES else root
        batch=[]
        for i in range(len(NAMES)):
            p=base/"runs"/f"{name}_a{i}.json"
            files[p.name]=hashlib.sha256(p.read_bytes()).hexdigest();batch.append(json.loads(p.read_bytes()))
        chosen=[selected(r) for r in batch]
        if all(c is None for c in chosen):
            continue
        assert all(c and c["encounter"]["features"]==chosen[0]["encounter"]["features"] for c in chosen)
        parents.append(name if name in CASES else variants[name]["parent"])
        x.append(chosen[0]["encounter"]["features"]);out.append([outcome(r) for r in batch])
    return np.array(x),out,np.array(parents),files

def training_values(out):
    threshold=float(np.quantile([z["minMargin"] for b in out for z in b if z["success"]],.25))
    ordinary=[];harms=[]
    for batch in out:
        good=[z["success"] and z["minMargin"]>=threshold for z in batch]
        successful_times=[z["zoneTime"] for z in batch if z["success"]]
        ordinary.append([(float(g)+.65*min(successful_times)/z["zoneTime"]+.10*min(1.,z["minMargin"]/threshold))
                         if z["success"] else -4. for g,z in zip(good,batch)])
        harms.append([bool(batch[0]["success"] and not z["success"] or good[0] and not g) for g,z in zip(good,batch)])
    return np.array(ordinary),np.array(harms),threshold

def tables(x,out,parents,risk_width):
    selected_features=(0,1,2,7,8,10)  # feature set selected in the first grouped training experiment
    scale=np.std(x,axis=0);scale=np.where(scale<1e-6,1.,scale)
    scale=np.array([v if j in selected_features else 1e9 for j,v in enumerate(scale)])
    ordinary,harm,threshold=training_values(out);families=np.unique(parents)
    # Replicate practice variations of one layout must not provide the same
    # confidence as observing several independent layout families.
    inverse_family=np.array([1/np.sum(parents==p) for p in parents])
    squared=np.sum(((x[:,None,:]-x[None,:,:])/scale)**2,axis=2)
    w=np.exp(-squared/(2*risk_width**2))*inverse_family[None,:];w/=w.sum(axis=1)[:,None]
    rates=w@harm
    totals=np.stack([w[:,parents==p].sum(axis=1) for p in families],axis=1)
    effective=1/(totals**2).sum(axis=1)
    # A Wilson-style uncertainty score at one standard error. Smoothness and
    # dependence of scene families preclude interpreting this as a safety bound.
    n=effective[:,None]
    upper=(rates+1/(2*n)+np.sqrt(rates*(1-rates)/n+1/(4*n*n)))/(1+1/n)
    upper[:,0]=0.
    return ordinary,upper,rates,effective,scale,threshold

def fit_risk(args):
    from itertools import product
    x,out,parents,files=groups(args.output)
    tuning=[];families=np.unique(parents)
    for radius in (.85,1.6,2.8):
        fold_data=[]
        for family in families:
            train=parents!=family
            values,upper,_,_,scale,threshold=tables(x[train],[b for b,f in zip(out,train) if f],parents[train],radius)
            fold_data.append((train,values,upper,scale,threshold))
        for width,penalty in product((.35,.7,1.4),(0.,.75,1.75,3.5,6.0)):
            choices=np.zeros(len(x),int);res=[]
            for train,values,upper,scale,threshold in fold_data:
                family_stats=[]
                for i in np.where(~train)[0]:
                    a=int(np.argmax(predictive_scores(x[train],values-penalty*upper,scale,width,x[i])))
                    choices[i]=a;base,z=out[i][0],out[i][a]
                    ok=z["success"] and z["minMargin"]>=threshold;ref_ok=base["success"] and base["minMargin"]>=threshold
                    efficiency=(min(q["zoneTime"] for q in out[i] if q["success"])/z["zoneTime"]) if z["success"] else 0.
                    family_stats.append([base["success"] and not z["success"],ref_ok and not ok,z["success"],ok,efficiency])
                res.append(np.mean(family_stats,axis=0))
            m=np.mean(res,axis=0)
            tuning.append(dict(radius=radius,width=width,penalty=penalty,choices=choices.tolist(),
                  newFailure=m[0],newInadequate=m[1],complete=m[2],adequate=m[3],efficiency=m[4]))
    objective=lambda r:(-r["newFailure"],-r["newInadequate"],r["complete"],r["adequate"],r["efficiency"])
    best=max((r for r in tuning if r["penalty"]>0),key=objective)
    ablation=max((r for r in tuning if r["penalty"]==0),key=objective)
    raw,upper,rates,n,scale,threshold=tables(x,out,parents,best["radius"])
    model=dict(mode="learned",actions=codebook()["actions"],contexts=x.tolist(),scales=scale.tolist(),
               returns=(raw-best["penalty"]*upper).tolist(),width=best["width"],rawReturns=raw.tolist(),
               riskUpper=upper.tolist(),riskRates=rates.tolist(),riskWeight=best["penalty"],riskRadius=best["radius"],effectiveFamilies=n.tolist())
    model["modelId"]=digest(model)
    write(args.output/"model.json",model)
    write(args.output/"training.json",dict(layoutFamilies=parents.tolist(),rawRunSHA256=files,observedOutcomes=out,
          trainingMarginQuartileM=threshold,featureNames=FEATURES,leaveOneFamilyOut=tuning,selected=best,
          strongestNoPenaltyAblation=ablation,modelId=model["modelId"]))
    print("frozen",model["modelId"],"radius / width / penalty",best["radius"],best["width"],best["penalty"],flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument("cmd",choices=["train","fit","external","replay"]);p.add_argument("ids",nargs="*")
    p.add_argument("--film",action="store_true");p.add_argument("--output",type=Path,default=ROOT);p.add_argument("--matrix",type=Path)
    args=p.parse_args()
    if args.cmd!="fit" and not os.environ.get("AIFORMULA_LAP_COPY"):raise ValueError("Private Unity required")
    if args.cmd=="train":return rollouts(args)
    if args.cmd=="fit":return fit_risk(args)
    if args.cmd=="replay":
        cases=json.loads(args.matrix.read_bytes())["cases"]
        return evaluate_cases(args,[("diag_"+c["id"],(c["seed"],c["approach"],c["props"],c["limitM"]),["learned"],args.film)
                                    for c in cases if c["id"] in args.ids])
    return external(args)

if __name__=="__main__":main()
