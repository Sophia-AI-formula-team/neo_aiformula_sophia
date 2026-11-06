"""Independent referee for two Unity runs with the same lateral crossing and initial state."""
import argparse
import json
import math
from pathlib import Path

import numpy as np


def read(path):
    return json.loads(Path(path).read_bytes())


def first_crossing(times, values):
    hits = np.flatnonzero((values[:-1] < 0) & (values[1:] >= 0))
    if len(hits):
        i = hits[0]
        return float(times[i] - values[i] * (times[i+1]-times[i]) / (values[i+1]-values[i]))
    return None


def section(run, prop):
    """Times at a fixed road section, with both the cone radius and vehicle orientation."""
    rows = np.asarray(run["rows"])
    rows = rows[np.abs(rows[:, 7]-prop[7]) < 18]  # exclude other straights in the same lap
    if len(rows) < 2:
        return {}
    t, car = rows[:, 0], rows[:, 1:3]
    road_right = np.array(prop[2:4])
    along = np.array([-road_right[1], road_right[0]])
    forward = np.column_stack((np.sin(rows[:, 3]), np.cos(rows[:, 3])))
    right = np.column_stack((forward[:, 1], -forward[:, 0]))
    corners = np.stack([car + z*forward + x*right for z in (-.88, .28) for x in (-.38, .38)], axis=1)
    offsets = corners - prop[:2]
    longitudinal = offsets @ along
    front = first_crossing(t, longitudinal.max(axis=1) + .34)
    rear = first_crossing(t, longitudinal.min(axis=1) - .34)
    axle = first_crossing(t, (car - prop[:2]) @ along)
    result = dict(frontEnterS=front, axleCrossS=axle, rearClearS=rear,
                  nextConeEntersCrossedStripS=None, rearLeadS=None, approachingStripAtRear=None)
    if axle is None or rear is None:
        return result
    # Fix a lateral band at the actually traversed location. Another spatial line
    # that never intersects the cone's motion is not evidence of crossing first.
    lateral = offsets @ road_right
    lo = float(np.interp(axle, t, lateral.min(axis=1)) - .34)
    hi = float(np.interp(axle, t, lateral.max(axis=1)) + .34)
    result["inflatedStripM"] = [lo, hi]
    if abs(prop[4]*prop[6]) < 1e-8:
        return result
    query = rear + np.arange(0, math.ceil(3/.005) + 1) * .005
    x = prop[4] * np.sin(prop[5]+prop[6]*query)
    separation = np.maximum(lo-x, x-hi)
    result["approachingStripAtRear"] = bool(separation[10] < separation[0] - .001)
    if separation[0] <= 0:
        result.update(nextConeEntersCrossedStripS=rear, rearLeadS=0.)
    else:
        arrival = first_crossing(query, -separation)
        if arrival is not None:
            result.update(nextConeEntersCrossedStripS=arrival, rearLeadS=float(arrival-rear))
    return result


def retimed_clearance(run, prop, delay):
    """Referee-only sensitivity. Replay the same spatial path later; other actions could change it."""
    rows = np.asarray(run["rows"])
    rows = rows[np.abs(rows[:, 7]-prop[7]) < 12]
    if len(rows) < 2:
        return None
    t = np.linspace(rows[0,0], rows[-1,0], 1+math.ceil((rows[-1,0]-rows[0,0])/.01))
    pos = np.stack([np.interp(t, rows[:,0], rows[:,k]) for k in (1,2)], axis=1)
    yaw = np.interp(t, rows[:,0], np.unwrap(rows[:,3]))
    obstacle = prop[:2] + np.outer(prop[4]*np.sin(prop[5]+(t+delay)*prop[6]), prop[2:4])
    diff = obstacle - pos
    z = diff[:,0]*np.sin(yaw) + diff[:,1]*np.cos(yaw)
    x = diff[:,0]*np.cos(yaw) - diff[:,1]*np.sin(yaw)
    return float((np.hypot(np.maximum.reduce([-0.88-z, z-.28, np.zeros_like(z)]),
                           np.maximum(np.abs(x)-.38, 0.)) - .34).min())


def evaluate(run, index, trigger):
    prop = run["props"][index]
    sec = section(run, prop)
    rows = np.asarray(run["rows"])
    early = None
    arrival = sec.get("axleCrossS")
    if trigger is not None and arrival is not None and trigger < arrival:
        window = rows[(rows[:,0]>=trigger-.00002) & (rows[:,0]<arrival)]
        onset_speed = float(np.interp(trigger, rows[:,0], rows[:,4]))
        peak_speed = max(onset_speed, float(np.interp(arrival, rows[:,0], rows[:,4])),
                         max(window[:,4],default=onset_speed))
        early = dict(atS=trigger, actualMps=onset_speed, peakBeforeAxleCrossMps=peak_speed,
                     actualAtAxleCrossMps=float(np.interp(arrival,rows[:,0],rows[:,4])),
                     increaseMps=float(peak_speed-onset_speed))
    zone_time = run["zoneEnd"][index]-run["zoneStart"][index] if run["zoneEnd"][index]>=0 else None
    safe = (run["status"] in ("lap","segment") and zone_time is not None
            and min(run["propMin"])>=.15 and run["minRoad"]>=.08)
    return dict(id=run["id"], status=run["status"], qualifiesForTimeComparison=safe,
                obstacleZoneS=zone_time, minConeClearanceM=run["propMin"][index],
                minRoadClearanceM=run["minRoad"], section=sec, earlyAcceleration=early,
                samePathLater=[dict(delayS=d, estimatedGapM=retimed_clearance(run,prop,d))
                               for d in (0., .25, .5, .75, 1., 1.5)])


def audit(on, off, trigger_name, index):
    for key in ("props","length","unity","physicsStep","profile","coupled"):
        if on[key] != off[key]:
            raise ValueError(f"Unmatched simulator field {key}")
    for key in ("start","seed","offset","yaw","obstacles","model","vision","limit","solver","weights"):
        if on["request"][key] != off["request"][key]:
            raise ValueError(f"Unmatched request field {key}")
    config_on = on["request"]["timing"]
    config_off = off["request"]["timing"]
    if (dict(config_on, mode="yield_only") != config_off or config_on["mode"] != "both"):
        raise ValueError("The two timing modes must be the only change")
    start = next((d for d in on["decisions"] if d["reason"]==trigger_name),None)
    trigger = start["t"] if start else None
    a, b = np.asarray(on["rows"]), np.asarray(off["rows"])
    ends = trigger if trigger is not None else min(a[-1,0], b[-1,0])+.0001
    # Trigger tick is excluded: it already includes the new actuator command.
    a, b = a[a[:,0]<ends-.00002], b[b[:,0]<ends-.00002]
    shared = bool(np.array_equal(a,b))
    measured = evaluate(on,index,trigger)
    reference = evaluate(off,index,trigger)
    saved = None
    if measured["qualifiesForTimeComparison"] and reference["qualifiesForTimeComparison"]:
        saved = reference["obstacleZoneS"] - measured["obstacleZoneS"]
    return dict(on=measured, off=reference, secondsSavedInMatchedSafeZone=saved,
                firstDecision=start, identicalPreTriggerRows=shared, commonPrefixRows=len(a) if shared else 0)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("on",type=Path)
    parser.add_argument("off",type=Path)
    parser.add_argument("--action",default="go_first")
    parser.add_argument("--prop",type=int,default=0)
    parser.add_argument("--out",type=Path,required=True)
    opts=parser.parse_args()
    result=audit(read(opts.on),read(opts.off),opts.action,opts.prop)
    opts.out.write_text(json.dumps(result,indent=2,allow_nan=False),encoding="utf8")
