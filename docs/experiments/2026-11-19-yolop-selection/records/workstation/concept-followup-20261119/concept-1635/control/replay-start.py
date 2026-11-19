"""Recheck the recorded local corridor under the physical action limits."""
import json
from pathlib import Path
import numpy as np
from concept_road.v3_model import Observation
from concept_candidates.planner625 import MarkedPlanner
from concept_feasibility.response61515 import can_command, motor_profile

base=Path(__file__).parent
scene=json.loads((base/'scene.json').read_text())
row=next(d for d in scene if d['speed_estimate_mps']==0 and not d['objects'])
road=np.asarray(row['road_rows']); headings=np.asarray(row['heading'])
planner=MarkedPlanner(dict(pace=4.,uncertainty=1.,yaw_limit=1.55))
cmd=np.zeros(2);cases=[]
for measured,applied in [(0.,(0.,0.)),(0.,None),(.12,None),(.3,None),(0.,(0.,0.))]:
    if applied is None: applied=cmd.copy()
    obs=Observation(road,np.asarray(row['objects']),measured,0.,.2)
    # Observation in the base model has no headings; field replay uses the recorded rows.
    from concept_road.compare61513 import Guided
    obs=Guided(road,obs.objects,measured,0.,.2,headings,None)
    with motor_profile(True,applied): cmd=planner.step(obs)
    audit=planner.last['audit']
    cases.append(dict(measured=measured,applied=list(applied),selected=cmd.tolist(),candidates=len(audit['table']),
                      feasible=sum(c[6]>0 for c in audit['table']),highest_candidate=max(c[0] for c in audit['table']),
                      wheels=[int(x) for x in can_command(*cmd,physical=True)]))
    planner.commit(audit['selected'])
# Remove the observed corridor. A pending reserve must not cause motion without current support.
missing=Guided(road[:0],np.asarray([]),0.,0.,.2,headings[:0],None)
with motor_profile(True,cmd): stop=planner.step(missing)
assert stop[0]==0
print(json.dumps(dict(bag="16:35:33",observed_road_rows=len(road),original_command=row['command'],rechecked=cases,
                             no_observed_road_longitudinal=stop[0]),indent=2))
