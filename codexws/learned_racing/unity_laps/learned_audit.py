"""Verify every recorded learned action against its frozen weights and paired prefix."""
import argparse
import hashlib
import json

import numpy as np

from learn_encounters import ROOT, FEATURES, NAMES, predictive_scores, write


def images(root,r):
    if not r["request"]["media"]:return {}
    roll=r["id"]
    streams={}
    for tag,folder,pattern,n in [("a4",root/".work/camera"/roll/"a4_samples","*.jpg",r["seenFrames"]),
                                  ("chase",root/".work/frames"/roll,"*c.jpg",len(r["frames"])),
                                  ("rgb",root/".work/frames"/roll,"*f.jpg",len(r["frames"]))]:
        files=sorted(folder.glob(pattern))
        assert len(files)==n,(roll,tag,n,len(files))
        sha=hashlib.sha256()
        for p in files:sha.update(p.name.encode()+b"\0"+hashlib.sha256(p.read_bytes()).digest())
        streams[tag]=dict(frames=n,orderedSHA256=sha.hexdigest())
    return streams


def audit(root,model,stem):
    a,b=[json.loads((root/"runs"/(stem+"_"+tag+".json")).read_bytes()) for tag in ("base","learned")]
    params=b["request"].copy();config=params.pop("encounter")
    assert config==model and params==a["request"]
    selected=[d for d in b["decisions"] if d["encounter"]["state"]=="selected"]
    prefix=min((d["t"] for d in selected),default=1e6)
    def commands(run):
        return [[d[k] for k in ("t","observed","tracks","speedCmd","yawCmd","leftCan","rightCan")] for d in run["decisions"] if d["t"]<prefix]
    assert commands(a)==commands(b),(stem,"different prefix")
    selections=[]
    for d in selected:
        e=d["encounter"];scores=predictive_scores(np.asarray(model["contexts"]),np.asarray(model["returns"]),
                 np.asarray(model["scales"]),model["width"],np.array(e["features"]))
        assert int(np.argmax(scores))==e["chosen"] and np.allclose(scores,e["actionScores"],atol=3e-5)
        selections.append(dict(t=d["t"],actualSpeedMps=d["speed"],action=NAMES[e["chosen"]],target= model["actions"][e["chosen"]],
                               features=dict(zip(FEATURES,e["features"])),scores=e["actionScores"]))
    result={}
    for r in [a,b]:
        reached=bool(r["zoneEnd"]) and min(r["zoneEnd"])>=0
        result[r["id"]]=dict(status=r["status"],seconds=r["seconds"],minGapM=r["minGap"] if r["props"] else None,
                  minRoadM=r["minRoad"],zoneClearS=(max(r["zoneEnd"])-min(r["zoneStart"])) if reached else None,streams=images(root,r))
    return dict(matchedRequest=True,prefixPerceptionAndCommandsExact=True,entirePhysicsTraceExact=a["rows"]==b["rows"],
                checkedActionValues=True,choices=selections,results=result)

def main():
    p=argparse.ArgumentParser();p.add_argument("--output",default=ROOT,type=type(ROOT));a=p.parse_args()
    model=json.loads((a.output/"model.json").read_bytes())
    out={}
    for path in sorted((a.output/"runs").glob("*_learned.json")):
        name=path.stem.removesuffix("_learned")
        out[name]=audit(a.output,model,name)
    write(a.output/"action_audit.json",out)
if __name__=="__main__":main()
