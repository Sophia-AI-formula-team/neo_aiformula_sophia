"""Train and freeze local speed-and-opening planner settings in the calibrated Unity."""
import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess

import numpy as np

from smooth_trials import HERE, LENGTH, Lab, SIM, detector, geometry, run_one, write

ROOT = HERE / "gap_revision"


def load(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def practice_cases():
    rng = np.random.default_rng(12807221)
    _, station, _, curvature, length = geometry()
    cases = []
    for i in range(12):
        distance = rng.uniform(21., 27.)
        curved = i % 4 == 3
        while True:
            seed = int(rng.integers(1000000, 90000000))
            start = np.random.default_rng(seed).random() * length
            idx = np.argmin(abs(station - (start+distance) % length))
            if (abs(curvature[idx]) > .025) == curved:
                break
        props = []
        for row in range(1+i%3):
            mid = rng.uniform(-1.9, -.4) if row % 2 == 0 else rng.uniform(-.55, .05)
            half = rng.uniform(1.15, 1.47)
            for side in (-1, 1):
                amplitude = rng.uniform(.16, .32) if (i+row)%3 and side == 1 else 0.
                props.append([distance, mid+side*half, amplitude,
                              rng.uniform(.36,.68) if amplitude else 0.,
                              rng.uniform(-np.pi,np.pi) if amplitude else 0.])
            distance += rng.uniform(6.5, 9.5)
        cases.append(dict(id=f"gp{i+1:02}", seed=seed, approach=0., props=props,
                          limitM=props[-1][0]+12, curved=curved))
    return dict(cases=cases, courseLengthForFractionsM=LENGTH)


def identify(training):
    rates, lags, clear, all_speeds, safe_gaps = [], [], [], [], []
    inputs = []
    for p in sorted((training/"runs").glob("tr???_a?.json")):
        record = load(p)
        if record["status"] != "segment":
            continue
        inputs.append([p.name,sha(p)])
        r = np.asarray(record["rows"])
        t,x,command = r[:,0],r[:,8],r[:,11]
        v = r[:,4]
        rate = (x[4:]-x[:-4]) / (t[4:]-t[:-4])
        all_speeds.extend(v[v>1.])
        clear.extend(v[(v>1.) & (abs(r[:,14])<.1)])
        safe_gaps.append(record["minGap"])
        if np.ptp(x)<.6 or np.ptp(command)<.6:
            continue
        selection=next((d for d in record["decisions"] if (d.get("smooth") or {}).get("state")=="selected"),None)
        if selection is None:continue
        start=int(np.searchsorted(t,selection["t"]))
        # Detect the onset of a commanded quarter-half-width lateral shift and
        # time the first corresponding physical displacement of that size.
        threshold=.38/4
        steps=np.flatnonzero(abs(command[start:]-command[start])>=threshold)
        if not len(steps):continue
        change=start+steps[0];direction=np.sign(command[change]-command[start])
        finish=np.searchsorted(t,t[change]+3.)
        moved=np.flatnonzero(direction*(x[change:finish]-x[change])>=threshold)
        if not len(moved):continue
        lags.append(float(t[change+moved[0]]-t[change]))
        # Characterize attainable motion during a maneuver, distinct from the
        # many logged straight samples with little demand for lateral movement.
        segment=direction*rate[change:min(len(rate),finish)]
        rates.append(float(np.quantile(segment,.90)))
    if len(inputs)<250 or not lags:
        raise ValueError("Need the recorded long training set")
    fast=np.quantile(clear,.9)
    quantiles=np.quantile(all_speeds,[.12,.40,.72])
    fractions=sorted(set([1.,*(min(.96,float(v/fast)) for v in quantiles)]),reverse=True)
    estimate=dict(lateralRate=float(np.quantile(rates,.75)), responseTime=float(np.median(lags)),
                  speedFractions=fractions, measuredFastMps=float(fast), gaps=np.quantile(safe_gaps,[.10,.30]).tolist(),
                  matchedLateralResponses=len(lags),successfulSources=inputs)
    return estimate


def prepare(training):
    if len(list((training/"runs").glob("tr???_a?.json")))!=560:
        raise ValueError("Prepare only from the complete 80-layout training archive")
    envelope=identify(training)
    write(ROOT/"identified.json",envelope)
    p,q=[float(np.clip(x,.035,.5)) for x in envelope["gaps"]]
    menu={}
    for i,(scale,pad) in enumerate(((1.,p),(1.,q),(1.13,p),(1.13,q),(1.26,p),(1.26,q))):
        menu[f"g{i+1}"]=dict(openings=True,paceScale=scale,padding=pad,
            lateralRate=envelope["lateralRate"],responseTime=envelope["responseTime"],
            speedFractions=envelope["speedFractions"])
    write(ROOT/"menu.json",menu)
    write(ROOT/"practice.json",practice_cases())


def run_cases(cases, options, film):
    if not os.environ.get("AIFORMULA_LAP_COPY"):
        raise RuntimeError("This experiment requires the dedicated simulator")
    lab=Lab(SIM,ROOT)
    with ExitStack() as stack:
        shared=None
        for i,case in enumerate(cases):
            order=list(options)
            order=order[i%len(order):]+order[:i%len(order)]
            for name in order:
                rid=case["id"]+"_"+name
                if film:
                    with detector(lab,"outer-edge",rid,film=True) as (port,release):
                        run_one(lab,rid,case,options[name],port,release,True)
                else:
                    if shared is None:
                        shared=stack.enter_context(detector(lab,"outer-edge","practice"))
                    run_one(lab,rid,case,options[name],*shared,False)


def evaluate_practice(model):
    frozen=ROOT/"frozen.json"
    if frozen.exists():raise RuntimeError("Frozen protocol already exists")
    menu=load(ROOT/"menu.json");cases=load(ROOT/"practice.json")["cases"]
    scores={}
    for arm in ["base",*menu]:
        scores[arm]=[]
        for c in cases:
            path=ROOT/"runs"/(c["id"]+"_"+arm+".json")
            r=load(path)
            expected=model if arm=="base" else dict(model,gap=menu[arm])
            if r["request"].get("smooth")!=expected or r["request"]["seed"]!=c["seed"]:
                raise ValueError("Practice trace does not belong to the model and seed: "+path.name)
            success=(r["status"]=="segment" and r["minGap"]>0 and r["minRoad"]>=0 and all(t>=0 for t in r["zoneEnd"]))
            z=min(r["zoneStart"]);end=max(r["zoneEnd"])
            scores[arm].append(dict(id=c["id"],result=r["status"],success=success,zoneS=end-z if success else None,
                 wholeS=r["seconds"],minGap=r["minGap"],sourceSHA=sha(path)))
    def ranking(arm):
        paired=list(zip(scores["base"],scores[arm]))
        common=[(a,b) for a,b in paired if a["success"] and b["success"]]
        ratios=[b["zoneS"]/a["zoneS"] for a,b in common]
        return (sum(b["success"] for _,b in paired),-sum(a["success"] and not b["success"] for a,b in paired),
                -float(np.mean(ratios)) if ratios else -100.,-float(np.mean([b["wholeS"] for _,b in common])) if common else -100.)
    chosen=max(menu,key=ranking)
    challenger=dict(model,gap=menu[chosen])
    pace_only=dict(model,gap=dict(menu[chosen],openings=False))
    write(ROOT/"challenger.json",challenger);write(ROOT/"pace_only.json",pace_only);write(ROOT/"base_model.json",model)
    write(ROOT/"practice_scores.json",dict(arms=scores,ranking={k:ranking(k) for k in scores}))
    write(frozen,dict(utc=datetime.now(timezone.utc).isoformat(),trainingCases=len(cases),chosen=chosen,
                      sourceCommit=subprocess.check_output(["git","rev-parse","HEAD"],cwd=HERE,text=True).strip(),
                      reference=sha(ROOT/"base_model.json"),challenger=sha(ROOT/"challenger.json"),
                      pace=sha(ROOT/"pace_only.json"),report=sha(ROOT/"practice_scores.json")))


if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("stage",choices=["prepare","practice","freeze","holdout"])
    p.add_argument("--training",type=Path);p.add_argument("--base",type=Path);p.add_argument("--cases",type=Path)
    p.add_argument("--ids",nargs="+")
    args=p.parse_args()
    if args.stage=="prepare":prepare(args.training)
    elif args.stage=="practice":
        base=load(args.base)
        run_cases(load(ROOT/"practice.json")["cases"],dict(base=base,**{k:dict(base,gap=v) for k,v in load(ROOT/"menu.json").items()}),False)
    elif args.stage=="freeze":evaluate_practice(load(args.base))
    else:
        frozen=load(ROOT/"frozen.json")
        arms={}
        for name,file,key in (("base","base_model","reference"),("learned","challenger","challenger"),("pace","pace_only","pace")):
            path=ROOT/(file+".json")
            if sha(path)!=frozen[key]:raise ValueError("Model changed after freeze")
            arms[name]=load(path)
        cases=load(args.cases)["cases"]
        if args.ids:cases=[c for c in cases if c["id"] in args.ids]
        run_cases(cases,arms,True)
