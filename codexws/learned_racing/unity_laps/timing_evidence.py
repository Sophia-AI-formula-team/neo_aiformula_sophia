"""Independent geometric checks on the locally recorded paired runs."""
from collections import Counter
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from timing_trials import ROOT
from risk_trials import write

PAIRS=("explanatory","blocked","static","full","h1","h2","h3","h4","after")

def trajectory(run):
    rows=np.asarray(run["rows"])
    prop=np.asarray(run["props"][0])
    right=prop[2:4]
    along=np.array([-right[1], right[0]])
    heading=rows[:,3]
    forward=np.stack([np.sin(heading),np.cos(heading)],axis=-1)
    side=np.stack([np.cos(heading),-np.sin(heading)],axis=-1)
    body=rows[:,None,1:3]+np.array([[-.88],[-.88],[.28],[.28]])*forward[:,None,:]+np.array([[-.38],[.38],[-.38],[.38]])*side[:,None,:]
    along_corners=(body-prop[:2])@along
    along_center=(rows[:,1:3]-prop[:2])@along
    # Crossing between adjacent genuine 10Hz rows, linear interpolation only.
    def first_positive(v):
        high=np.where(v>=0)[0]
        if not len(high):return None
        i=high[0]
        return float(np.interp(0.,v[max(0,i-1):i+1],rows[max(0,i-1):i+1,0]))
    enter=first_positive(along_corners.max(1)+.34)
    rear=first_positive(along_corners.min(1)-.34)
    center=first_positive(along_center)
    chosen=next((d for d in run["decisions"] if d["timing"] and d["timing"]["state"]=="accepted"),None)
    out=dict(frontEntersStationS=enter,egoCenterAtStationS=center,rearClearsStationS=rear,
             actualSpeedAtStationMps=float(np.interp(center,rows[:,0],rows[:,4])) if center is not None else None)
    if chosen is None or center is None or rear is None:return out
    before=(rows[:,0]>=chosen["t"])&(rows[:,0]<=center)
    out.update(decisionTimeS=chosen["t"],actualSpeedAtDecisionMps=chosen["speed"],
               maxActualSpeedAfterDecisionBeforeStationMps=float(rows[before,4].max()),
               actualSpeedIncreaseByStationMps=out["actualSpeedAtStationMps"]-chosen["speed"])
    # Choose the lateral footprint of the actual car when its centre crosses the
    # cone's station, then ask when the real cone would occupy that footprint.
    body_x=(body-prop[:2])@right
    xlow=np.interp(center,rows[:,0],body_x.min(1))
    xhigh=np.interp(center,rows[:,0],body_x.max(1))
    times=np.arange(rear,rows[-1,0]+.0001,.002)
    cone=prop[4]*np.sin(prop[5]+times*prop[6])
    overlap=np.where((cone+.34>=xlow)&(cone-.34<=xhigh))[0]
    if len(overlap):
        out.update(nextConeEntersTraversedFootprintS=float(times[overlap[0]]),clearBeforeNextOccupationS=float(times[overlap[0]]-rear))
    return out

def audit(run,root):
    decisions=run["decisions"]
    result=dict(id=run["id"],status=run["status"],durationS=run["seconds"],progressM=run["progress"],minObstacleM=run["minGap"],
                minRoadM=run["minRoad"],offers=sum(d["timing"]["offered"] for d in decisions),states=dict(Counter(d["timing"]["state"] for d in decisions)))
    result.update(trajectory(run))
    if run["request"]["media"]:
        ids=run["id"]
        image_sets={}
        for name,folder,pattern,expected in [("A4",root/".work/camera"/ids/"a4_samples","*.jpg",run["seenFrames"]),
                                              ("chase",root/".work/frames"/ids,"*c.jpg",len(run["frames"])),
                                              ("RGB",root/".work/frames"/ids,"*f.jpg",len(run["frames"]))]:
            files=sorted(folder.glob(pattern))
            if len(files)!=expected:raise RuntimeError(f"{ids}: {name} incomplete")
            digest=hashlib.sha256()
            for path in files:digest.update(path.name.encode()+b"\0"+hashlib.sha256(path.read_bytes()).digest())
            image_sets[name]=dict(count=len(files),orderedDigest=digest.hexdigest())
        result["streams"]=image_sets
    return result

def paired(a,b,root):
    a_request=dict(a["request"]);b_request=dict(b["request"])
    # Only the availability of go-first may differ. Same approach and frozen hash.
    opt_a=a_request.pop("timing");opt_b=b_request.pop("timing")
    assert opt_a["mode"]=="both" and opt_b["mode"]=="yield_only" and opt_a["approach"]==opt_b["approach"]
    assert a_request==b_request
    first=next((d for d in a["decisions"] if d["timing"]["state"]=="accepted"),None)
    prefix=first["t"] if first else math.inf
    key=lambda d: (d["t"],d["speedCmd"],d["yawCmd"],d["rightCan"],d["leftCan"],d["observed"],d["tracks"])
    same=([key(d) for d in a["decisions"] if d["t"]<prefix]==[key(d) for d in b["decisions"] if d["t"]<prefix])
    return dict(requestMatchedExceptOption=True,approachCapMps=opt_a["approach"],prefixPerceptionAndCommandsExact=same,
                entirePhysicsTraceExact=a["rows"]==b["rows"],go=audit(a,root),yieldOnly=audit(b,root),
                segmentTimeSavedS=(b["seconds"]-a["seconds"]) if a["status"]==b["status"]=="segment" else None)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",type=Path,default=ROOT)
    root=parser.parse_args().output
    pairs={}
    for name in PAIRS:
        a=json.loads((root/"runs"/(name+"_go.json")).read_bytes()); b=json.loads((root/"runs"/(name+"_yield.json")).read_bytes())
        pairs[name]=paired(a,b,root)
    write(root/"pairs.json",pairs)
if __name__=="__main__":main()
