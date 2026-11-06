"""Referee-only trajectory metrics. The controller never reads station or course geometry."""
import numpy as np
from scipy.spatial import cKDTree
from scenario_shots import geometry

CENTER,STATION,ROAD_ANGLE,CURVATURE,LENGTH=geometry()
TREE=cKDTree(CENTER)
def metrics(r):
    rows=np.asarray(r["rows"],float);t=rows[:,0];s=rows[:,7]
    if len(rows)<2:raise ValueError("Incomplete trace")
    idx=TREE.query(rows[0,1:3])[1]
    tangent=np.array([np.cos(ROAD_ANGLE[idx]),np.sin(ROAD_ANGLE[idx])])
    origin=STATION[idx]+(rows[0,1:3]-CENTER[idx])@tangent
    global_s=(origin+s)%LENGTH
    kappa=np.interp(global_s,STATION,CURVATURE)
    roadangle=np.interp(global_s,STATION,ROAD_ANGLE)
    residual=np.unwrap(np.pi/2-rows[:,3]-roadangle)
    props=[p[-1] for p in r["props"]]
    free=(s>=10)&(abs(kappa)<=.015)
    for pos in props:free &= abs(s-pos)>=12.
    free=free[1:]&free[:-1]
    dt=np.diff(t);ds=np.maximum(0,np.diff(s));move=np.linalg.norm(np.diff(rows[:,1:3],axis=0),axis=1)
    speed_mid=(rows[1:,4]+rows[:-1,4])*.5
    fd=ds[free].sum();ft=dt[free].sum()
    # Use the common referee progress window including recovery after the final
    # prop. Failures remain visible but their truncated kinematics earn no reward.
    domain=np.zeros(len(s),bool)
    for pos in props:domain|=(s>=pos-6)&(s<=pos+8)
    zone=domain[1:]&domain[:-1]
    zd=ds[zone].sum();zt=dt[zone].sum()
    def per_m(v):return float(v[zone].sum()/zd) if zd>.01 else None
    good=r["status"] in ("segment","lap") and all(x>=0 for x in r["zoneEnd"])
    return dict(status=r["status"],completed=bool(good),contact=r["status"]=="obstacle",outside=r["status"]=="boundary",
       seconds=r["seconds"],progressM=r["progress"],minConeM=r["minGap"] if props else None,minRoadM=r["minRoad"],
       positiveCappedConeM=float(np.clip(r["minGap"],0,.5)) if props else None,
       zoneReferenceDistanceM=float(zd),zoneTimeS=float(zt),
       headingResidualTVperM=per_m(abs(np.diff(residual))),yawCommandTVperM=per_m(abs(np.diff(rows[:,6]))),
       lateralTravelPerM=per_m(abs(np.diff(rows[:,8]))),extraPathPerM=max(0.,float(move[zone].sum()/zd-1.)) if zd>.01 else None,
       yawAccelerationRMS=float(np.sqrt(np.sum((np.diff(rows[:,14])[zone]/dt[zone])**2*dt[zone])/zt)) if zt>.01 else None,
       freeDistanceM=float(fd),freeTimeS=float(ft),freeAverageMps=float(fd/ft) if ft>.01 else None,
       freeDistanceAtLeast4Share=float(ds[free&(speed_mid>=4)].sum()/fd) if fd>.01 else None)
