"""Runtime source extracted from the accepted V6.26 freeze: experiments/v615_recovery/road61524.py. """

import numpy as np

from concept_road.road6157 import RememberedRoad6157

def current_lane_heading(road,heading,evidence):
    result=np.asarray(heading).copy()
    audit=dict(source='existing_heading',applied_rows=0,rule='Only direction; no road or candidate-width changes')
    lane=evidence.get('current_lane');tracks=evidence.get('tracks',[])
    if not lane or lane['score']<=.45 or len(road)==0:
        return result,audit
    # Pair left/right are indices in the current tracks list, not track IDs.
    left,right=[tracks[lane[key]] for key in ['left','right']]
    lo=max(left['zmin'],right['zmin']);hi=min(left['zmax'],right['zmax'])
    z=np.asarray(road)[:,0];supported=(z>=lo)&(z<=hi)
    angles=[np.arctan(c['coef'][1]+2*c['coef'][2]*z[supported]) for c in [left,right]]
    result[supported]=np.mean(angles,axis=0)
    audit.update(source='B_current_lane_pair_within_observed_support',applied_rows=int(supported.sum()),
        support_z=[float(lo),float(hi)],track_ids=[left.get('id'),right.get('id')],
        pair_score=lane['score'],indices=[lane['left'],lane['right']])
    return result,audit

class RememberedRoad61524(RememberedRoad6157):
    def read(self,image,raw,lane,boxes=(),speed=0.,yaw=0.,dt=.2):
        # Run the original complete geometry pipeline before adjusting direction;
        # changing it earlier would also change startup near-field inference.
        road,pixels,heading=super().read(image,raw,lane,boxes,speed,yaw,dt)
        heading,audit=current_lane_heading(road,heading,self.last)
        self.last['heading_guidance61524']=audit
        # Keep the blackout transport consistent with the actually issued view.
        # Cached road geometry and cached source evidence remain unchanged.
        if self.cached is not None:
            self.cached=(self.cached[0], heading.copy())
        return road,pixels,heading
