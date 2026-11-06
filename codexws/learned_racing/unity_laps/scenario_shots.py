"""Record illustrative mixed-traffic and mid-corner passages with unchanged frozen A4 policy."""
import argparse
import hashlib
from itertools import permutations
import json
import os

import numpy as np
from render import track
from risk_trials import HERE, Lab, SIM, detector, write

ROOT = HERE / "scenario_shots"
FROZEN = HERE / "risk_revision/frozen.json"

# Absolute metre stations of extra props are referee configuration only.
# Each tuple is [station, right offset, lateral amplitude, angular frequency, phase].
LAYOUTS = {
    "mixed_01": (25, [[69., .45, 0., 0., 0.], [71.8, -.10, 1.25, .8, .35]]),
    "apex_01": (58, [[114.1, -.10, 0., 0., 0.]]),
    "mixed_02": (36, [[76., .35, 0., 0., 0.], [78.9, .12, 1., .72, 1.05]]),
    "apex_02": (8, [[114.1, .32, 0., 0., 0.]]),
    "mixed_03": (32, [[67., -.45, 1.05, .69, 1.3], [70.1, .05, 0., 0., 0.]]),
    "apex_pair_01": (58, [[112.4, .50, 0., 0., 0.], [115.3, .25, .80, .7, 1.25]]),
}

def geometry():
    boundaries = track()
    center = (boundaries[0]+boundaries[1])/2
    delta = np.roll(center, -1, axis=0)-center
    distances = np.linalg.norm(delta, axis=1)
    stations = np.r_[0., np.cumsum(distances[:-1])]
    tangent = np.roll(center,-5,axis=0)-np.roll(center,5,axis=0)
    heading = np.unwrap(np.arctan2(tangent[:,1], tangent[:,0]))
    curvature = np.gradient(heading,stations)
    return center, stations, heading, curvature, float(distances.sum())

def matched_pair(run):
    """Audit the stored tracker outputs against both props without feeding truth into control."""
    if len(run["props"]) != 2:
        return None
    rows=np.asarray(run["rows"]);samples=[]
    for d in run["decisions"]:
        if len(d["tracks"])<2:
            continue
        t=d["t"];row=rows[np.argmin(abs(rows[:,0]-t))]
        forward=np.array([np.sin(row[3]), np.cos(row[3])])
        right=np.array([forward[1], -forward[0]])
        true=[]
        for prop in run["props"]:
            xz=np.array(prop[:2])+np.array(prop[2:4])*prop[4]*np.sin(prop[5]+prop[6]*t)
            delta=xz-row[1:3]
            true.append([delta@forward, delta@right])
        if min(obj[0] for obj in true)<0:
            continue
        observed=np.asarray(d["tracks"])[:,:2]
        errors=np.linalg.norm(observed[:,None,:]-np.array(true)[None,:,:],axis=2)
        best=min(max(errors[a,0],errors[b,1]) for a,b in permutations(range(len(observed)),2))
        if best<1.25:
            samples.append((t,float(best)))
    return dict(toleranceM=1.25,distinctMatchedFrames=len(samples),sampleTimesS=[x[0] for x in samples],
                maxMatchedPositionErrorM=max((x[1] for x in samples),default=None))

def metadata(run):
    center, station, heading, curvature, length = geometry()
    summaries=[]
    for j, prop in enumerate(run["props"]):
        i=int(np.argmin(np.linalg.norm(center-np.asarray(prop[:2]),axis=1)))
        at=station[i]
        lo,hi=np.searchsorted(station,[at-7.5,at+7.5])
        lo=max(0,lo);hi=min(len(station)-1,hi)
        summary=dict(index=j,worldXZ=prop[:2],courseStationM=float(at),distanceFromSpawnM=prop[-1],
                     curvaturePerM=float(curvature[i]),headingChangeOver15mDegrees=float(np.degrees(heading[hi]-heading[lo])),
                     amplitudeM=prop[4],phaseRad=prop[5],omegaRadS=prop[6],minClearanceM=run["propMin"][j],
                     zoneEntryS=run["zoneStart"][j],zoneExitS=run["zoneEnd"][j])
        summaries.append(summary)
    entry=max(run["zoneStart"]);exit_=min(run["zoneEnd"])
    return dict(trial=run["id"],seed=run["request"]["seed"],status=run["status"],seconds=run["seconds"],meters=run["progress"],
                minRoadM=run["minRoad"],minObstacleM=run["minGap"],props=summaries,
                commonZoneSeconds=max(0,exit_-entry) if entry>=0 and exit_>=0 else 0,
                distinctTrackedProps=matched_pair(run),decisions=len(run["decisions"]),
                renderedFramePairs=len(run["frames"]),a4Frames=run["seenFrames"])

def failure_reproduction(run):
    reference=json.loads((FROZEN.parent/"runs/frozen_20318_traffic.json").read_bytes())
    config_keys=("seed","weights","solver","model","start","offset","yaw","limit","obstacles","vision")
    same_request=all(reference["request"][key]==run["request"][key] for key in config_keys)
    commands=lambda trial: [(d["t"],d["leftCan"],d["rightCan"],d["speedCmd"],d["yawCmd"]) for d in trial["decisions"]]
    return dict(source=reference["id"],sameDrivingRequest=same_request,identicalPhysicsTrace=reference["rows"]==run["rows"],
                identicalCommands=commands(reference)==commands(run),sameTerminal=all(reference[k]==run[k] for k in ("status","seconds","progress","minGap","minRoad")))

def save_metadata(run):
    report=metadata(run)
    if run["id"]=="failure_20318":
        report["reproduction"]=failure_reproduction(run)
    streams={}
    optical=ROOT/".work/frames"/run["id"]
    detection=ROOT/".work/camera"/run["id"]/"a4_samples"
    for label, files, expected in [
        ("chase", optical.glob("*c.jpg"),len(run["frames"])),
        ("rgb", optical.glob("*f.jpg"),len(run["frames"])),
        ("a4",detection.glob("*.jpg"),run["seenFrames"])]:
        digest=hashlib.sha256();files=sorted(files)
        if len(files)!=expected:raise ValueError("Incomplete "+label+" frames for "+run["id"])
        for file in files:
            digest.update(file.name.encode()+b"\0"+hashlib.sha256(file.read_bytes()).digest())
        streams[label]=dict(frames=len(files),orderedFrameDigest=digest.hexdigest())
    report["imageStreams"]=streams
    write(ROOT/"encounters"/(run["id"]+".json"),report)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("ids",nargs="+",choices=(*LAYOUTS,"failure_20318"))
    args=parser.parse_args()
    if not os.environ.get("AIFORMULA_LAP_COPY"):raise RuntimeError("Only use a private Unity batch project")
    frozen=json.loads(FROZEN.read_bytes())
    if hashlib.sha256((HERE/"vision.py").read_bytes()).hexdigest()!=frozen["observerSha"]:
        raise RuntimeError("Frozen A4 observer hash mismatch")
    length=geometry()[-1];lab=Lab(SIM,ROOT,frozen["solver"])
    for name in args.ids:
        if name=="failure_20318":
            with detector(lab,"outer-edge",name,film=True) as (port,model):
                if model!=frozen["model"]:raise ValueError("Unexpected camera release")
                r=lab.run(name,frozen["weights"],20318,True,port,True,capture=.05,model=model)
            save_metadata(r)
            continue
        seed,layout=LAYOUTS[name];start=np.random.default_rng(seed).random()
        specs=[[(p[0]/length-start)%1,*p[1:]] for p in layout]
        limit=max(p[0] for p in specs)+13/length
        if limit>=1:raise ValueError("Invalid short-segment layout")
        with detector(lab,"outer-edge",name,film=True) as (port,model):
            if model!=frozen["model"]:raise ValueError("Unexpected camera release")
            r=lab.run(name,frozen["weights"],seed,vision=port,media=True,limit=limit,capture=.05,
                      model=model,obstacle_specs=specs)
        save_metadata(r)

if __name__=="__main__":main()
