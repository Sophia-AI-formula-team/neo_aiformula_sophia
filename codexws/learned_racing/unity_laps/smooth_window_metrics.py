"""Measured motion and cruise over referee-defined progress gates. No controller input."""
import numpy as np

def merged(windows):
    result=[]
    for a,b in sorted(windows):
        if b<=a:
            continue
        if result and a<=result[-1][1]:
            result[-1][1]=max(result[-1][1],b)
        else:
            result.append([a,b])
    return result


def obstacle_windows(run):
    return merged((p[7]-12.,p[7]+8.) for p in run["props"])


def first_station_time(rows,station):
    hits=np.flatnonzero(rows[:,7]>=station)
    if not len(hits):
        return None
    idx=int(hits[0])
    if not idx:
        return float(rows[0,0])
    s0,s1=rows[idx-1:idx+1,7]
    t0,t1=rows[idx-1:idx+1,0]
    return float(t0+(station-s0)/(s1-s0)*(t1-t0))


def extract(rows,left,right,stop_at=None):
    """Include lost time and turns inside a gate; limit observations to common first-pass progress when paired."""
    start=first_station_time(rows,left)
    if start is None:
        return None
    end=rows[-1,0]
    target=right if stop_at is None else min(right,stop_at)
    if target<=left:
        return None
    at=first_station_time(rows,target)
    if at is not None:
        end=at
    if end<=start:
        return None
    def sample(t):
        return [np.interp(t,rows[:,0],rows[:,j]) for j in range(rows.shape[1])]
    inside=rows[(rows[:,0]>start)&(rows[:,0]<end)]
    return np.vstack((sample(start),inside,sample(end)))


def reversals(values,threshold=.08):
    signs=np.sign(values[np.abs(values)>=threshold])
    return int(np.count_nonzero(np.diff(signs)))


def high_speed_distance(data):
    """First-traversed progress only; interpolate the threshold crossing and ignore re-traversed distance."""
    progress=data[:,7]
    best=np.maximum.accumulate(progress)
    high=longest=streak=0.
    for i,(a,b) in enumerate(zip(data[:-1,4],data[1:,4])):
        ds=best[i+1]-best[i]
        if b<4. and a<4.:
            streak=0.
            continue
        if ds<=0:
            if min(a,b)<4.:streak=0.
            continue
        underlying=progress[i+1]-progress[i]
        start_fraction=1.-ds/underlying
        speed_at_start=a+(b-a)*start_fraction
        if speed_at_start>=4. and b>=4.:
            part=ds
            if a<4.:streak=0.
        elif speed_at_start<4. and b<4.:
            streak=0.
            continue
        else:
            crossing=(4.-speed_at_start)/(b-speed_at_start)
            part=ds*((1.-crossing) if b>=4. else crossing)
            if speed_at_start<4.:streak=0.
        high+=part
        streak+=part
        longest=max(longest,streak)
        if b<4.:streak=0.
    return float(high),float(longest)


def abs_integral(values,times):
    """Exact integral of |piecewise linear signal|, including zero crossings inside a sample."""
    a,b=values[:-1],values[1:]
    weights=(np.abs(a)+np.abs(b))*.5
    crossing=a*b<0
    weights[crossing]=.5*(a[crossing]**2+b[crossing]**2)/(np.abs(a[crossing])+np.abs(b[crossing]))
    return float(np.dot(weights,np.diff(times)))


def measured(data):
    t=data[:,0]
    seconds=float(t[-1]-t[0])
    distance=float(max(data[:,7])-data[0,7])
    path=float(np.linalg.norm(np.diff(data[:,1:3],axis=0),axis=1).sum())
    yaw=data[:,14]
    signed=float(np.dot(.5*(yaw[1:]+yaw[:-1]),np.diff(t)))
    absolute=abs_integral(yaw,t)
    wheel=data[:,12]-data[:,13]
    elevated,longest=high_speed_distance(data)
    return dict(seconds=seconds,progressM=distance,bodyPathM=path,excessPathM=path-distance,
                yawTravelRad=absolute,signedYawRad=signed,excessYawRad=max(0.,absolute-abs(signed)),
                yawRateVariationRadS=float(np.abs(np.diff(yaw)).sum()),yawReversals=reversals(yaw),
                lateralTravelM=float(np.abs(np.diff(data[:,8])).sum()),commandYawReversals=reversals(data[:,6]),
                wheelDifferentialAbsRpmS=abs_integral(wheel,t),wheelDifferentialVariationRpm=float(np.abs(np.diff(wheel)).sum()),
                atLeast4M=elevated,longestAtLeast4M=longest)

def body_speed(parts):
    """Time-weighted mean and median of the measured longitudinal body speed with piecewise-linear interpolation."""
    if not parts:return dict(measuredBodyMeanMps=None,measuredBodyMedianMps=None)
    w=np.concatenate([np.diff(p[:,0]) for p in parts])
    v0=np.concatenate([p[:-1,4] for p in parts])
    v1=np.concatenate([p[1:,4] for p in parts])
    lo=np.minimum(v0,v1);hi=np.maximum(v0,v1)
    left,right=float(lo.min()),float(hi.max())
    for _ in range(45):
        mid=(left+right)*.5
        frac=np.divide(mid-lo,hi-lo,out=(mid>=lo).astype(float),where=hi>lo)
        if np.dot(np.clip(frac,0,1),w)>=.5*w.sum():right=mid
        else:left=mid
    return dict(measuredBodyMeanMps=float(np.dot(.5*(v0+v1),w)/w.sum()),measuredBodyMedianMps=right)


def intervals_report(rows,windows,stop_at=None):
    readings=[];parts=[]
    for lo,hi in windows:
        part=extract(rows,lo,hi,stop_at)
        value=None
        if part is not None:
            parts.append(part); value=measured(part)
        readings.append(dict(fromM=lo,toM=hi,measurement=value))
    stats=[g["measurement"] for g in readings if g["measurement"] is not None]
    totals={k:sum(s[k] for s in stats) for k in (stats[0] if stats else ()) if k!="longestAtLeast4M"}
    planned=sum(hi-lo for lo,hi in windows)
    metres,seconds=totals.get("progressM",0.),totals.get("seconds",0.)
    totals.update(plannedM=planned,progressM=metres,seconds=seconds,coverage=metres/planned if planned else None,
                  progressSpeedMps=metres/seconds if seconds else None,
                  fractionPlannedAtLeast4=totals.get("atLeast4M",0.)/planned if planned else None,
                  longestAtLeast4M=max((s["longestAtLeast4M"] for s in stats),default=0.))
    for key in ("yawTravelRad","excessYawRad","yawRateVariationRadS","yawReversals","excessPathM","wheelDifferentialVariationRpm","lateralTravelM","commandYawReversals"):
        totals[key+"Per10m"]=10.*totals.get(key,0.)/metres if metres else None
    totals["meanAbsWheelDifferentialRpm"]=totals.get("wheelDifferentialAbsRpmS",0.)/seconds if seconds else None
    totals.update(body_speed(parts))
    for gate,part in zip((g for g in readings if g["measurement"] is not None),parts):
        gate["measurement"].update(body_speed([part]))
    return dict(gates=readings,totals=totals)
