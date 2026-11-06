"""Recreate the held-out jury matrix only from track geometry, a fixed RNG stream and relative placements."""
import json
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scenario_shots import geometry


def create():
    _, stations, _, curvature, length = geometry()
    rng=np.random.default_rng(20711131)
    used=set()
    def bend(s):
        return np.abs(np.interp(np.asarray(s)%length,stations,curvature))
    def start_for(kind,dist):
        for _ in range(10000):
            seed=int(rng.integers(12000001,15999999))
            start=float(np.random.default_rng(seed).random()*length)
            if seed in used:continue
            shape=bend(start+np.arange(dist-2,dist+2.01,.25))
            if kind=="bend" and min(shape)<.032:continue
            if kind=="straight" and max(shape)>.010:continue
            if kind=="clear-straight" and max(bend(start+np.arange(10,37.1,.25)))>.010:continue
            used.add(seed)
            return seed,start
        raise ValueError("Geometry filter did not yield an unused seed")
    def u(a,b):
        return round(float(rng.uniform(a,b)),4)
    def clear_gates(start,props,end,cap):
        if cap:return []
        query=np.arange(12.,end+.000001,.25)
        valid=(bend(start+query)<=.012)
        for m,*_ in props:valid &= (query<m-12)|(query>m+8)
        gates=[]
        for block in np.split(query[valid],np.flatnonzero(np.diff(query[valid])>.251)+1):
            if len(block) and block[-1]-block[0]>=12:
                gates.append([float(block[0]+2),float(block[-1]-2)])
        return gates
    def prop(kind,d,j):
        if kind=='static':return [[d,u(-.55,.68),0,0,0]]
        if kind=='staggered':
            return [[d,u(-.65,-.15),0,0,0],[round(d+u(3.,5.2),3),u(.36,.83),0,0,0]]
        if kind=='crossing':
            sign=-1 if j%2==0 else 1
            return [[d,sign*u(1.15,1.65),u(1.8,2.30),u(.40,.60),u(-4.7,3.1)]]
        if kind=='central-moving':
            return [[d,u(-.45,.45),u(.65,1.48),u(.43,.85),u(-2.9,2.9)]]
        return [[d,u(.1,.62),0,0,0],[round(d+u(2.7,4.4),3),u(-.64,.29),u(.73,1.39),u(.46,.79),u(-3,3)]]
    cases=[]
    def add(cid,kind,road,cap,end,props,seed,start,full=False):
        cases.append(dict(id=cid,context=kind+" / "+road,seed=seed,approach=cap,limitM=end,props=props,
                          scope="lap" if full else "segment",family=kind,road=road,nominalStartM=start,
                          straightCruiseGatesM=clear_gates(start,props,end,cap)))
    for j in range(4):
        seed,start=start_for("clear-straight",23)
        add(f"b{j+1:02d}","clear","straight",0,38.,[],seed,start)
        # Exact same 22-m test gate in each of the four natural-cruise local runs.
        cases[-1]["straightCruiseGatesM"]=[[12.,34.]]
    for i,kind in enumerate(("static","staggered","crossing","central-moving","mixed")):
        for j in range(4):
            road="bend" if j in (1,3) else "straight"
            dist=u(20.,27.5)
            seed,start=start_for(road,dist)
            props=prop(kind,dist,j)
            cap=u(2.2,3.5) if j==(i%2+2) else 0
            add(f"b{5+4*i+j:02d}",kind,road,cap,round(props[-1][0]+u(14,19),3),props,seed,start)
    for i in range(8):
        seed,start=start_for("any",0)
        props=[]
        if i<6:
            for n,kind in enumerate(("static",("mixed","staggered")[i%2],("crossing","central-moving")[i%2])):
                props.extend(prop(kind,u([36.,111.,190.][n],[57.,137.,232.][n]),i))
        add(f"l{i+1:02d}", "full-traffic" if props else "full-clear","whole track",0,length,props,seed,start,True)
    assert len(used)==32 and all(c['straightCruiseGatesM'] for c in cases if c['family'] in ('clear','full-clear'))
    return dict(schema=1,courseLengthForFractionsM=length, cases=cases)

if __name__=="__main__":
    path=Path(__file__).with_name("layouts.json")
    matrix=create()
    path.write_text(json.dumps(matrix,ensure_ascii=False,indent=2)+"\n",encoding="utf8")
    _,stations,_,curvature,length=geometry()
    controls={}
    for c in matrix["cases"]:
        q=np.arange(12.,c["limitM"]+.000001,.25)
        free=np.ones_like(q,dtype=bool)
        for m,*_ in c["props"]:free &= (q<m-12)|(q>m+8)
        bend=np.abs(np.interp((c["nominalStartM"]+q)%length,stations,curvature))>=.032
        valid=q[free & bend]
        sections=np.split(valid,np.flatnonzero(np.diff(valid)>.251)+1)
        controls[c["id"]]=[[float(v[0]+2),float(v[-1]-2)] for v in sections if len(v) and v[-1]-v[0]>=10]
    path.with_name("bend_controls.json").write_text(json.dumps(controls,indent=2)+"\n",encoding="utf8")
