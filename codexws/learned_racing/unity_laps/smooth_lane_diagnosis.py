"""Offline, causal lane outlier replay. Only the 80 *training* layouts are read."""
from pathlib import Path
import json

import numpy as np
from smooth_metrics import CENTER,TREE
from smooth_trials import ROOT,write

AHEAD=np.array([2.,3.5,5.5,8.])


def project(line,angle,forward,right):
    c=np.cos(angle)[:,None];s=np.sin(angle)[:,None]
    x=line-right[:,None];z=AHEAD[None,:]-forward[:,None]
    u=c*z+s*x;v=c*x-s*z
    out=np.full(line.shape,np.nan);supported=np.zeros(line.shape,bool)
    finite=np.isfinite(line).all(axis=1)&np.all(np.diff(u,axis=1)>.02,axis=1)
    for n,d in enumerate(AHEAD):
        i=np.clip(np.sum(u<=d,axis=1)-1,0,2);r=np.arange(len(line))
        slope=(v[r,i+1]-v[r,i])/(u[r,i+1]-u[r,i])
        out[finite,n]=(v[r,i]+(d-u[r,i])*slope)[finite]
        supported[:,n]=finite&(u[:,0]<=d)&(d<=u[:,-1])
    return out,supported


def reference(pos,heading):
    fw=np.column_stack((np.sin(heading),np.cos(heading)))
    right=np.column_stack((np.cos(heading),-np.sin(heading)))
    near=TREE.query(pos)[1]
    delta=CENTER[(near[:,None]+np.arange(-12,58))%len(CENTER)]-pos[:,None,:]
    z=np.sum(delta*fw[:,None,:],axis=2);x=np.sum(delta*right[:,None,:],axis=2)
    truth=np.full((len(pos),4),np.nan)
    for j,d in enumerate(AHEAD):
        crossing=(z[:,:-1]<=d)&(z[:,1:]>d)
        valid=np.any(crossing,axis=1);r=np.flatnonzero(valid);i=np.argmax(crossing,axis=1)[r]
        truth[r,j]=x[r,i]+(x[r,i+1]-x[r,i])*(d-z[r,i])/(z[r,i+1]-z[r,i])
    return truth


def load_trial(path):
    row=json.loads(path.read_bytes());ds=row["decisions"];t=np.array([d["t"] for d in ds])
    log=np.array(row["rows"]);r=log[np.abs(t[:,None]-log[None,:,0]).argmin(axis=1)]
    if max(abs(r[:,0]-t))>1e-4:raise ValueError("Unsynchronized decisions")
    live=np.array([d["observed"]for d in ds],float);truth=reference(r[:,1:3],r[:,3])
    dt=np.diff(t);theta=.5*(r[:-1,14]+r[1:,14])*dt
    dist=.5*(r[:-1,4]+r[1:,4])*dt
    forward=dist*np.sinc(theta/np.pi);side=dist*.5*theta*np.sinc(theta/(2*np.pi))**2
    pred,support=project(live[:-1],theta,forward,side)
    change=np.diff(r[:,1:3],axis=0)
    fw=np.column_stack((np.sin(r[:-1,3]),np.cos(r[:-1,3])))
    right=np.column_stack((np.cos(r[:-1,3]),-np.sin(r[:-1,3])))
    exact,_=project(live[:-1],np.diff(np.unwrap(r[:,3])),np.sum(change*fw,axis=1),np.sum(change*right,axis=1))
    return dict(name=path.stem,raw=live,truth=truth,pred=np.vstack([live[:1]*np.nan,pred]),
                support=np.vstack([np.zeros((1,4),bool),support]),exact=np.vstack([live[:1]*np.nan,exact]),time=t)


def clean(run,threshold):
    live=run["raw"];pred=run["pred"]
    # A four-point line does not observe new pavement beyond last frame's far point.
    # Replay only overlapping, finite beam samples; never replace an occluded beam.
    suspect=run["support"]&(abs(live-pred)>threshold)
    filtered=live.copy();hold=np.zeros_like(suspect)
    for i in range(1,len(live)):
        hold[i]=suspect[i]&~hold[i-1]
    filtered[hold]=pred[hold]
    return filtered,hold


def curvature(line):
    return (np.arctan((line[:,3]-line[:,2])/2.5)-np.arctan((line[:,1]-line[:,0])/1.5))/4


def stats(a):
    a=a[np.isfinite(a)]
    return {"N":len(a),"mae":float(np.mean(abs(a))),"p95":float(np.percentile(abs(a),95)),
            "p99":float(np.percentile(abs(a),99))}if len(a)else None


def evaluate(runs,threshold):
    allraw=[];alltruth=[];allclean=[];allmask=[];allerr=[];delays=[];baseline_delays=[]
    for run in runs:
        cleanline,h=clean(run,threshold);allraw.append(run["raw"]);allclean.append(cleanline)
        alltruth.append(run["truth"]);allmask.append(h);allerr.append((run["exact"]-run["pred"])[run["support"]])
        rt=curvature(run["truth"]);rc=curvature(cleanline);rr=curvature(run["raw"])
        # Newly visible real curvature, at least 0.6s after a clear approach; three
        # preceding low samples and two subsequent same-sign strong samples.
        for i in range(3,len(rt)-2):
            if np.max(abs(rt[i-3:i]))>.045 or min(abs(rt[i:i+2]))<.045 or rt[i]*rt[i+1]<=0:continue
            window=slice(i,min(i+5,len(rt)));sign=np.sign(rt[i])
            for series,target in [(rr,baseline_delays),(rc,delays)]:
                hits=np.flatnonzero(series[window]*sign>=.035)
                target.append(float(run["time"][i+hits[0]]-run["time"][i]) if len(hits) else 1.)
    raw=np.vstack(allraw);truth=np.vstack(alltruth);f=np.vstack(allclean);h=np.vstack(allmask)
    def subgroup(a):
        k=curvature(a);kt=curvature(truth);valid=np.isfinite(k)&np.isfinite(kt)
        straight=valid&(abs(kt)<=.015);bend=valid&(abs(kt)>=.045)
        return dict(laneErrorM=[stats(row) for row in (a-truth).T],curvatureError=stats(k[valid]-kt[valid]),
                    straightN=int(straight.sum()),straightLookaheadFalseAbove0035=int(np.sum(straight&(abs(k)>.035))),
                    trueBendN=int(bend.sum()),trueBendWrongSignOrBelow0035=int(np.sum(bend&(k*np.sign(kt)<.035))))
    good=abs(raw-truth)<=.20;assess=h&np.isfinite(truth)
    better=abs(f-truth)<abs(raw-truth);bad=abs(raw-truth)>=.45
    return dict(runs=len(runs),frames=len(raw),distinctLayouts=len(set(r["name"].split("_")[0]for r in runs)),
                raw=subgroup(raw),filtered=subgroup(f),replacedBeams=h.sum(axis=0).tolist(),
                replacedWithTruth=int(assess.sum()),replacementsImproving=int((assess&better).sum()),
                goodRawReplaced=int((good&h).sum()),goodRawTotal=int(good.sum()),largeRawErrorN=int(bad.sum()),
                largeRawErrorReplaced=int((bad&h).sum()),changedFrames=int(np.any(h,axis=1).sum()),
                twistProjectionDifferenceFromExactRigidPoseM=stats(np.concatenate(allerr)),
                bendEntries=len(delays),bendEntriesDelayedComparedToRaw=int(np.sum(np.array(delays)>np.array(baseline_delays)+.05)),
                bendEntriesEarlierThanRaw=int(np.sum(np.array(delays)<np.array(baseline_delays)-.05)),
                bendMeanLatencyRawS=float(np.mean(baseline_delays))if delays else None,
                bendMeanLatencyFilteredS=float(np.mean(delays))if delays else None)


def main():
    runs=sorted((ROOT/"runs").glob("tr???_a?.json"));assert len(runs)==560
    train=[load_trial(p)for p in runs if int(p.stem[2:5])<=64]
    held=[load_trial(p)for p in runs if int(p.stem[2:5])>64]
    # Calibrate a per-beam innovation cutoff exclusively on accurate training
    # measurements. Quantile fixed before reporting the last 16 training layouts.
    learn=[]
    for r in train:
        acc=(abs(r["raw"]-r["truth"])<=.20)&r["support"]
        learn.append(np.where(acc,abs(r["raw"]-r["pred"]),np.nan))
    learn=np.vstack(learn)
    thresholds=np.array([np.nanpercentile(q,99)if np.isfinite(q).sum()>=100 else np.inf for q in learn.T])
    # Far beam stays raw even during low-speed support produced by stationary turns.
    thresholds[-1]=np.inf
    result=dict(offlineOnly=True,source="training tr001..080, all seven actions; matched approach prefixes repeated",
                cutoff="99th percentile of odometry-compensated innovation when measured beam is within 0.20m of independent true lane",
                thresholdsM=[float(v)if np.isfinite(v)else None for v in thresholds],
                goodCalibrationSamples=np.isfinite(learn).sum(axis=0).tolist(),
                calibration=evaluate(train,thresholds),withheldTrainingLayouts=evaluate(held,thresholds))
    write(ROOT/"lane_diagnosis.json",result)
if __name__=="__main__":main()
