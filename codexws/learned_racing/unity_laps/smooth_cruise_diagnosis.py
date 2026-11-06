"""Explain the four preregistered 12–34 m speed gates from unchanged control-period evidence."""
import argparse
import json
from pathlib import Path
import numpy as np
from scenario_shots import geometry
from smooth_holdout_score import PROTOCOL,load,sha
from smooth_window_metrics import extract,intervals_report


def inspect(run,case,stations,curvature,length):
    rows=np.asarray(run["rows"],float);gate=extract(rows,12.,34.)
    lo,hi=gate[[0,-1],0];decisions=run["decisions"];series=[];reasons={};budgets={}
    for i,d in enumerate(decisions):
        a=max(lo,d["t"]);b=min(hi,decisions[i+1]["t"] if i+1<len(decisions) else run["seconds"])
        if b<=a:continue
        s=np.interp(d["t"],rows[:,0],rows[:,7]);nom=(case["nominalStartM"]+s)%length
        ahead=(nom+np.arange(0.,8.51,.25))%length
        pieces=dict(curvatureCruiseBelow4=d["cruise"]<4,guardBelowCurve=d["guardCap"]<d["cruise"]-.001,
                    rateLimitedCommand=d["speedCmd"]+.002 < min(d["guardCap"],d["cruise"])*d["ratio"],
                    actualBelow4AtUpdate=d["speed"]<4,hasTrackedObject=bool(d["tracks"]))
        for k,v in pieces.items():budgets[k]=budgets.get(k,0.)+v*(b-a)
        reasons[d["reason"]]=reasons.get(d["reason"],0.)+b-a
        series.append(dict(t=d["t"],validGateSeconds=b-a,progressM=s,truthCurvatureAtCar=float(np.interp(nom,stations,curvature)),
                    truthMaxAbsCurvatureAhead8_5m=float(np.abs(np.interp(ahead,stations,curvature)).max()),a4Curvature=d["curvature"],
                    observedLane=d["observed"],trackCount=len(d["tracks"]),reason=d["reason"],actualMps=d["speed"],curveCruiseMps=d["cruise"],
                    guardCapMps=d["guardCap"],plannerRatio=d["ratio"],targetMps=d["speedCmd"],actualYaw=d["yaw"],targetYaw=d["yawCmd"],
                    rightRpm=d["rightCan"],leftRpm=d["leftCan"],**pieces))
    times=np.asarray([d["validGateSeconds"]for d in series]);base=dict(seconds=float(hi-lo),entryMps=float(gate[0,4]),exitMps=float(gate[-1,4]),
            gate=intervals_report(rows,[[12.,34.]])["totals"],reasonSeconds=reasons,conditionSeconds=budgets)
    for key in ("curveCruiseMps","targetMps","a4Curvature","rightRpm","leftRpm","truthCurvatureAtCar","truthMaxAbsCurvatureAhead8_5m"):
        x=np.array([d[key]for d in series]);base[key]=dict(min=float(x.min()),max=float(x.max()),timeWeightedMean=float(np.average(x,weights=times)),
                                                          timeWeightedAbsMean=float(np.average(abs(x),weights=times)))
    base["series"]=series
    return base


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("runs",type=Path);parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args();_,stations,_,curvature,L=geometry();out=[]
    for case in load(PROTOCOL/"layouts.json")["cases"][:4]:
        row=dict(id=case["id"])
        for arm in ("base","learned"):
            path=args.runs/(case["id"]+"_"+arm+".json")
            row[arm]=inspect(load(path),case,stations,curvature,L);row[arm]["sha256"]=sha(path)
        out.append(row)
    args.out.write_text(json.dumps(out,ensure_ascii=False,indent=2,allow_nan=False)+"\n",encoding="utf8")
