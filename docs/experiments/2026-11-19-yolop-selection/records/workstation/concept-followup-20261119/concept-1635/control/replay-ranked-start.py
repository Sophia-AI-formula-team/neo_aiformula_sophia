"""Recheck the recorded local corridor under the physical action limits."""
import json
from pathlib import Path
import numpy as np
from concept_road.v3_model import Observation
from concept_drive.core import Drive
from concept_candidates.rear_reference617 import RecoveryReference
from concept_feasibility.response61515 import can_command, motor_profile

base=Path(__file__).parent
scene=json.loads((base/'scene.json').read_text())
row=next(d for d in scene if d['speed_estimate_mps']==0 and not d['objects'])
road=np.asarray(row['road_rows']); headings=np.asarray(row['heading'])
driver=Drive(4.)
planner=driver.planner
builder=driver.builder
cmd=np.zeros(2);cases=[]
for measured,applied in [(0.,(0.,0.)),(0.,None),(.12,None),(.3,None),(0.,(0.,0.))]:
    if applied is None: applied=cmd.copy()
    obs=RecoveryReference(road,np.asarray(row['objects']).reshape(-1,77),measured,0.,.2,headings,None,None,None)
    with motor_profile(True,applied):
        features,allowed,pick=builder.prepare(obs)
        if builder.network_allowed:
            pick,_=driver.ranker.rank(features,allowed,pick,builder.planner.last['audit'],obs)
        cmd=builder.command(pick)
    audit=builder.planner.last['audit']
    cases.append(dict(measured=measured,applied=list(applied),selected=cmd.tolist(),candidates=len(audit['table']),
                      feasible=sum(c[6]>0 for c in audit['table']),highest_candidate=max(c[0] for c in audit['table']),
                      wheels=[int(x) for x in can_command(*cmd,physical=True)]))

# Remove the observed corridor. A pending reserve must not cause motion without current support.
missing=RecoveryReference(road[:0],np.empty((0,77)),0.,0.,.2,headings[:0],None,None,None)
with motor_profile(True,cmd):
    features,allowed,pick=builder.prepare(missing)
    stop=builder.command(pick)
assert stop[0]==0.
print(json.dumps(dict(bag="16:35:33",observed_road_rows=len(road),original_command=row['command'],rechecked=cases,
                             no_observed_road_longitudinal=stop[0]),indent=2))
