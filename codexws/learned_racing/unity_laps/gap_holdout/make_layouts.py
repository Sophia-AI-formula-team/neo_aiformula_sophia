"""Create hidden paired gate tests and certify a road-constrained geometric witness."""
import json
from pathlib import Path
import sys
import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.spatial import cKDTree

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from render import track
from scenario_shots import geometry

LANE=3.43051
CONE=.34
BODY=np.array([[x,z] for x in (-.38,.38) for z in (-.88,.28)])


def road_geometry():
    e=track()
    lane=(e[0]+e[1])*.5
    normal=e[0]-e[1]
    normal/=np.linalg.norm(normal,axis=1)[:,None]
    dist=np.linalg.norm(np.roll(lane,-1,axis=0)-lane,axis=1)
    return lane,normal,np.r_[0.,np.cumsum(dist)],cKDTree(lane)


def phase_minimum(position,heading,home,direction,amplitude):
    """Lower bound for the entire sinusoidal sweep: enclose it in a car-local AABB."""
    car_right=np.c_[heading[:,1],-heading[:,0]]
    rel=home-position
    x=np.sum(rel*car_right,axis=1);z=np.sum(rel*heading,axis=1)
    dx=amplitude*np.abs(car_right@direction)
    dz=amplitude*np.abs(heading@direction)
    lateral=np.maximum(np.abs(x)-dx-.38,0.)
    longitudinal=np.maximum.reduce([-.88-z-dz,z-dz-.28,np.zeros_like(z)])
    return float(np.hypot(lateral,longitudinal).min()-CONE)


def certify(case,road):
    lane,normal,stations,tree=road;length=stations[-1]
    rng=np.random.default_rng(case["seed"])
    start_idx=np.searchsorted(stations[:-1],rng.random()*length)%len(lane)
    initial_offset=rng.uniform(-.30,.30)
    indexes=[np.searchsorted(stations[:-1],(case["nominalStartM"]+p[0])%length)%len(lane) for p in case["props"]]
    placed=[(stations[i]-stations[start_idx])%length for i in indexes]
    gates=case["gates"]
    knots=[0.];offsets=[initial_offset]
    for gate in gates:
        a,b=gate["propIndices"]
        center=(case["props"][a][1]+case["props"][b][1])*.5
        for ds in (-1.3,1.3):
            knots.append(placed[a]+ds);offsets.append(center)
    knots.append(case["limitM"]);offsets.append(offsets[-1])
    rel_s=np.linspace(0.,case["limitM"],1+int(np.ceil(case["limitM"]/.015)))
    s=(stations[start_idx]+rel_s)%length
    off=PchipInterpolator(knots,offsets)(rel_s)
    center=np.column_stack([np.interp(s,stations,np.r_[lane[:,k],lane[0,k]]) for k in (0,1)])
    right=np.column_stack([np.interp(s,stations,np.r_[normal[:,k],normal[0,k]]) for k in (0,1)])
    right/=np.linalg.norm(right,axis=1)[:,None]
    pos=center+off[:,None]*right
    forward=np.gradient(pos,axis=0);forward/=np.linalg.norm(forward,axis=1)[:,None]
    body_right=np.c_[forward[:,1],-forward[:,0]]
    corners=pos[:,None,:]+BODY[None,:,0,None]*body_right[:,None,:]+BODY[None,:,1,None]*forward[:,None,:]
    ix=tree.query(corners)[1]
    x=np.sum((corners-lane[ix])*normal[ix],axis=2)
    road_min=float(min((.5*LANE-x).min(),(1.5*LANE+x).min()))
    cone_mins=[phase_minimum(pos,forward,lane[i]+p[1]*normal[i],normal[i],p[2]) for i,p in zip(indexes,case["props"])]
    # Conservative interpolation allowance exceeds the maximum displacement of any car corner between samples.
    displacement=float(np.linalg.norm(np.diff(corners,axis=0),axis=2).max())
    allowance=max(.035,2*displacement)
    cert=dict(nominalControlStationsM=knots,nominalControlOffsetsM=offsets,
              maxStepM=float(np.diff(rel_s).max()),maxAdjacentCornerStepM=displacement,
              allowanceM=allowance,worstSweptConeClearanceM=min(cone_mins)-allowance,
              minimumRoadMarginM=road_min-allowance,allPhaseConeLimitsM=[g-allowance for g in cone_mins],
              note="Geometric swept-footprint witness. It uses judging geometry, not an executed motor command.")
    if min(cert["worstSweptConeClearanceM"],cert["minimumRoadMarginM"])<.12:
        raise ValueError("Insufficient witness margin")
    return cert


def create():
    road=road_geometry();_,stations,_,curvature,length=geometry();assert abs(length-road[2][-1])<.001
    rng=np.random.default_rng(91106733)
    recipes=[
        ("static","straight",["outer-line"]),("static","bend",["shift"]),
        ("moving","straight",["outer-line"]),("moving","bend",["shift"]),
        ("double","straight",["outer-line","shift"]),("double","bend",["shift","outer-line"]),
        ("mixed-triple","bend",["outer-line","shift","outer-line"]),("mixed-triple","bend",["shift","outer-line","shift"]),
    ]
    def u(lo,hi):return round(float(rng.uniform(lo,hi)),4)
    cases=[];witnesses={}
    for j,(family,kind,pattern) in enumerate(recipes):
        dist=[u(22,26)]
        for _ in pattern[1:]:dist.append(round(dist[-1]+u(6.6,8.1),4))
        for _ in range(10000):
            seed=int(rng.integers(6300000,8200000));start=np.random.default_rng(seed).random()*length
            k=np.abs(np.interp((start+np.array(dist))%length,stations,curvature))
            if (kind=="straight" and max(k)<.010) or (kind=="bend" and min(k)>.026 and max(k)<.14):break
        else:raise ValueError("Missing road class")
        props=[];gates=[]
        for i,role in enumerate(pattern):
            left,right=(u(-2.45,-2.18),u(1.09,1.15)) if role=="outer-line" else (u(-3.25,-2.92),u(-.24,.06))
            moving=(family=="moving") or (family=="mixed-triple" and (i+j)%3!=0)
            ids=[]
            for lateral in (left,right):
                amp_limit=.20 if lateral>0 else .30
                params=[u(.14,amp_limit),u(.38,.8),u(-3.14,3.14)] if moving else [0,0,0]
                ids.append(len(props));props.append([dist[i],lateral,*params])
            a,b=map(props.__getitem__,ids)
            assert a[1]+a[2]+CONE<-.5*LANE and a[1]-a[2]-CONE>-1.5*LANE
            assert b[1]-b[2]-CONE>-.5*LANE and b[1]+b[2]+CONE<.5*LANE
            aperture=[a[1]+a[2]+CONE+.38,b[1]-b[2]-CONE-.38]
            assert aperture[1]-aperture[0]>.60
            gates.append(dict(index=i,propIndices=ids,role=role,stationM=dist[i],
                              worstPhaseAlignedAxleOffsetIntervalM=aperture))
        name=f"g{j+1:02}"
        case=dict(id=name,scope="segment",family=family,road=kind,context=family+" / "+kind,nominalStartM=start,
                  seed=seed,approach=0,limitM=round(dist[-1]+12.5,4),props=props,gates=gates,order=["base","learned"] if j%2==0 else ["learned","base"])
        witnesses[name]=certify(case,road);cases.append(case)
    return dict(schema=1,courseLengthForFractionsM=length,cases=cases),witnesses


if __name__=="__main__":
    matrix,certificates=create()
    for name,content in (("layouts",matrix),("witness",certificates)):
        Path(__file__).with_name(name+".json").write_text(json.dumps(content,indent=2)+"\n",encoding="utf8")
