"""Conservative policy tree fitted directly from matched safe driving alternatives."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from smooth_fit import COSTS,returns
from smooth_trials import ROOT,NAMES,bank,write

def make_tree(x,y,success,depth,minimum):
    nodes=[];audit=[]
    def leaf(indices):
        yr=y[indices];passed=success[indices]
        newly_failed=(passed[:,[0]] & ~passed).sum(0)
        # Use paired gains, not absolute pace from different sections. A leaf
        # cannot choose a non-reference action that caused an extra failure on
        # its examples. Other parts of the tree can still select this action.
        gains=(yr-yr[:,[0]]).mean(0);gains[newly_failed>0]=-20.
        action=int(np.argmax(gains))
        return gains,action,float(len(indices)*gains[action])
    def grow(ids,levels):
        idx=len(nodes);gains,action,original=leaf(ids)
        nodes.append([-1,*gains.tolist()]);audit.append(dict(examples=len(ids),action=action))
        if not levels or len(ids)<2*minimum:return idx
        best=original;branch=None
        for j in range(x.shape[1]):
            values=x[ids,j]
            cuts=np.unique(np.quantile(values,[.15,.3,.45,.6,.75,.85]))
            for cut in cuts:
                left=ids[values<=cut];right=ids[values>cut]
                if min(len(left),len(right))<minimum:continue
                value=leaf(left)[2]+leaf(right)[2]
                if value>best+1e-7:
                    best=value;branch=(j,float(cut),left,right)
        if branch:
            j,cut,left,right=branch
            # C# observations are float32. Keep the exact same membership
            # as Python at thresholds that landed on a recorded observation.
            stored=np.float32(cut)
            if stored>cut:stored=np.nextafter(stored,np.float32(-np.inf))
            lo=grow(left,levels-1);hi=grow(right,levels-1)
            nodes[idx]=[j,float(stored),lo,hi]
        return idx
    grow(np.arange(len(x)),depth)
    return nodes,audit

def infer(nodes,features):
    n=nodes[0]
    while n[0]>=0:n=nodes[n[2] if np.float32(features[n[0]])<=n[1] else n[3]]
    return int(np.argmax(n[1:]))

def fit(root,size):
    ck=root/"checkpoints"/f"ck{size:02}";rec=json.loads((ck/"training.json").read_bytes())
    cases=[r for r in rec["outcomes"]if r["trigger"] is not None]
    x=np.array([q["trigger"][1]for q in cases],dtype=np.float32)
    rows=[c["outcomes"]for c in cases];passed=np.array([[q["completed"]for q in b]for b in rows])
    specs=[(d,m) for d in (1,2,3)for m in (4,7,11)]
    folds=[np.array(g)for g in rec["folds"]];attempts=[]
    for depth,m in specs:
        chosen=np.zeros(len(x),int);advantages=[]
        for test in folds:
            train=np.ones(len(x),bool);train[test]=False
            y,sc=returns([b for i,b in enumerate(rows)if train[i]],6.)
            model,_=make_tree(x[train],y,passed[train],depth,m)
            chosen[test]=[infer(model,x[i])for i in test]
            vy,_=returns([rows[i]for i in test],6.,sc)
            advantages.extend((vy[np.arange(len(test)),chosen[test]]-vy[:,0]).tolist())
        realised=passed[np.arange(len(x)),chosen]
        attempts.append(dict(depth=depth,minLeaf=m,completed=int(realised.sum()),lost=int((passed[:,0]&~realised).sum()),
                        benefit=float(np.mean(advantages)),changed=int((chosen!=0).sum()),choices=chosen.tolist()))
    best=max(attempts,key=lambda v:(v["completed"],-v["lost"],v["benefit"]))
    y,_=returns(rows,6.);tree,leaf_report=make_tree(x,y,passed,best["depth"],best["minLeaf"])
    cfg=dict(mode="smooth",actions=bank(),forest=[tree])
    cfg["modelId"]=hashlib.sha256(json.dumps(cfg,sort_keys=True,separators=(",",":")).encode()).hexdigest()
    dest=root/"checkpoints"/f"ck{size:02}p";write(dest/"model.json",cfg)
    write(dest/"calibration.json",dict(modelId=cfg["modelId"],selected=best,groupedValidation=attempts,nodes=leaf_report))
    print(best,flush=True)

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("size",type=int);p.add_argument("--output",type=Path,default=ROOT)
    a=p.parse_args();fit(a.output,a.size)
