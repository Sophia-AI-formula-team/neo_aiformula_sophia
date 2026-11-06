"""Describe changes of recorded legacy steering commands; controller replay is verified before attribution."""
import argparse
import json
import math
from pathlib import Path

import numpy as np


def read(path):
    return json.loads(path.read_bytes())


FACTORS=("trackedRoadMinusEgo","issuedLateralOffset","lookahead","curvatureTimesSpeed","yawFeedback")


def shapley(d,gain):
    a=np.array([[z["target"]-z["offset"],z["offset"],z["look"],.9*z["speed"]*z["curvature"],-.38*z["yaw"]] for z in d])
    def command(v):
        return np.clip(gain*np.arctan2(v[:,0]+v[:,1],v[:,2])+v[:,3]+v[:,4],-1.60119,1.69546)
    predicted=command(a)
    if np.max(np.abs(predicted-np.array([z["yawCmd"] for z in d])))>2e-5:
        raise ValueError("Recorded commands do not match this controller; inspect the actual version")
    n=a.shape[1];delta=np.zeros((len(a)-1,n));values={}
    for mask in range(1<<n):
        selector=np.array([bool(mask&(1<<j)) for j in range(n)])
        values[mask]=command(np.where(selector,a[1:],a[:-1]))
    for mask,value in values.items():
        count=mask.bit_count()
        if count==n:continue
        weight=1/(n*math.comb(n-1,count))
        for j in range(n):
            if not mask&(1<<j):delta[:,j]+=weight*(values[mask|(1<<j)]-value)
    if np.max(np.abs(delta.sum(axis=1)-np.diff(predicted)))>1e-8:raise ValueError("Mismatch on additive reconstruction")
    return delta


def fast_returns(t,yaw):
    sign=0;last=None;events=[]
    for when,v in zip(t,yaw):
        side=1 if v>=.08 else -1 if v<=-.08 else 0
        if not side:continue
        if sign and side!=sign:
            if last is not None and when-last<=1.00001:events.append(float(when))
            last=when
        sign=side
    return events


def measurements(run,stop_progress=None):
    rows=np.asarray(run["rows"])
    if stop_progress is not None: rows=rows[rows[:,7]<=stop_progress+.0001]
    d=[v for v in run["decisions"] if v["t"]<=rows[-1,0]+.00001]
    subs=shapley(d,run["request"]["weights"][2]);t=np.array([v["t"] for v in d])
    command=np.array([v["yawCmd"] for v in d]);desired=np.array([v["preferred"] for v in d])
    offset=np.array([v["offset"] for v in d]);counts=np.array([len(v["tracks"]) for v in d])
    poses=rows[:,14];dt=np.diff(rows[:,0]); meters=float(rows[-1,7]-rows[0,7])
    def transition(k):
        old,new=d[k],d[k+1]
        points=[(x,z) for x,z in zip(old["observed"],(2.,3.5,5.5,8.)) if x is not None]
        pose_rows=[min(rows,key=lambda row:abs(row[0]-v["t"])) for v in (old,new)]
        carried=None
        if len(points)>=2 and all(abs(row[0]-v["t"])<.011 for row,v in zip(pose_rows,(old,new))):
            def rotation(y):
                return np.array([[math.cos(y),math.sin(y)],[-math.sin(y),math.cos(y)]])
            before,after=pose_rows
            world=before[1:3]+np.asarray(points)@rotation(before[3]).T
            current=(world-after[1:3])@rotation(after[3])
            if np.all(np.diff(current[:,1])>0) and current[0,1]<=new["look"]<=current[-1,1]:
                carried=float(np.interp(new["look"],current[:,1],current[:,0]))
        road=new["target"]-new["offset"]
        return dict(tS=new["t"],oldReason=old["reason"],reason=new["reason"],
                    encounter=(new.get("encounter") or {}).get("state"),tracks=[int(counts[k]),int(counts[k+1])],
                    desiredShiftM=[old["preferred"],new["preferred"]],actualShiftM=[old["offset"],new["offset"]],
                    prevRoadAtNewLookAfterEgoMotionM=carried,currentRoadM=road,
                    residualRoadUpdateM=None if carried is None else road-carried,
                    receivedRoadSamples=[old["observed"],new["observed"]],
                    yawCommand=[old["yawCmd"],new["yawCmd"]],contributions=dict(zip(FACTORS,subs[k].tolist())))
    ranking=np.argsort(-abs(np.diff(command)))
    per=np.abs(subs).sum(axis=0);state=[(v.get("encounter") or {}).get("state") for v in d]
    dx_t=t[1:];jump=np.abs(np.diff(command))
    ref_change=np.abs(np.diff(desired))>1
    change=np.diff(counts)!=0
    events=fast_returns(rows[:,0],poses)
    return dict(id=run["id"],throughM=meters,seconds=float(rows[-1,0]),statusOfWholeRun=run["status"],
                decisionIntervals=len(d)-1,totalCommandVariation=float(jump.sum()),cmdVariationPerM=float(jump.sum()/meters),
                physicalYawAccelerationRMS=float(np.sqrt(np.sum(np.diff(poses)**2/dt)/sum(dt))),
                physicalYawTVperM=float(np.abs(np.diff(poses)).sum()/meters),rapidReturnCount=len(events),rapidReturnTimes=events,
                desiredMovesOver1M=int(ref_change.sum()),maxDesiredStepM=float(np.abs(np.diff(desired)).max()),
                maxIssuedOffsetStepM=float(np.abs(np.diff(offset)).max()),wheelDifferenceTVperM=float(np.abs(np.diff(rows[:,12]-rows[:,13])).sum()/meters),
                maxCommandStepRadS=float(jump.max()),saturatedDecisionCount=int(sum((command<=-1.60118)|(command>=1.69545))),
                factorMagnitudeShare=dict(zip(FACTORS,(per/per.sum()).tolist())),
                commandTVAtTrackCountChange=float(jump[change].sum()/jump.sum()),commandTVAtDesiredStepOver1M=float(jump[ref_change].sum()/jump.sum()),
                selectionTimes=[float(t[i]) for i in range(len(t)) if state[i]=="selected"],
                newObjectTimes=[float(t[i]) for i in range(len(t)) if state[i]=="new_object"],
                topTransitions=[transition(int(i)) for i in ranking[:7]])


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--root",type=Path,default=Path(__file__).parent)
    p.add_argument("--out",type=Path,required=True);args=p.parse_args()
    cases={}
    for lab in ("h05","h07","h08","full01"):
        sources={arm:args.root/"learned_revision"/"runs"/f"{lab}_{arm}.json" for arm in ("base","learned")}
        v2=args.root/"learned_risk_revision"/"runs"/f"diag_{lab}_learned.json"
        if v2.exists():sources["riskLearned"]=v2
        runs={arm:read(path) for arm,path in sources.items()}
        limit=min(r["progress"] for r in runs.values())
        cases[lab]={"matchedThroughM":limit,"matched":{arm:measurements(r,limit) for arm,r in runs.items()},
                    "whole":{arm:measurements(r) for arm,r in runs.items()}}
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps({"factorNote":"Sum of absolute Shapley contributions to the replayed yaw-command change. Road component includes ego motion and camera geometry. Associations with track-count changes are descriptive and not identification of reacquisition.",
                                    "cases":cases},indent=2)+"\n",encoding="utf8")


if __name__=="__main__":
    main()
