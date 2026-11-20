"""Runtime source extracted from the accepted V6.26 freeze: experiments/v617_regression/trajectory624.py. """

import json

import numpy as np

from concept_candidates.ranker617 import choose

from concept_ranker.whole_lap621 import WholeLapSelector

INPUTS=('speed_command','turn_command','nominal_progress','road_margin','obstacle_margin',
        'lateral_at_quarter','lateral_at_half','lateral_at_end','forward_at_end',
        'ego_speed','ego_yaw','road_width','road_direction','nearest_z','nearest_x',
        'nearest_vx','nearest_vz','obstacle_count','previous_speed','previous_turn')

CAP=1.4

def encode(audit,road,heading,objects,speed,yaw):
    tab=np.asarray(audit['table'],float);paths=np.asarray(audit['paths'])
    road=np.asarray(road);heading=np.asarray(heading)
    near=[o for o in objects if 0<o[0]<=18.]
    closest=min(near,key=lambda o:o[0]) if near else (18.,0.,0.,0.)
    left,right=[np.interp(5.,road[:,0],road[:,i]) for i in (1,2)]
    prev=audit['previous_command']
    common=[speed/4,yaw/1.55,(right-left)/8,np.interp(5.,road[:,0],heading)/np.pi,
            closest[0]/18,closest[1]/6,closest[3]/3,closest[2]/3,len(near)/5,prev[0]/4,prev[1]/1.55]
    n=paths.shape[1]-1
    x=np.column_stack([tab[:,0]/4,tab[:,1]/1.55,tab[:,8]/4,
                       np.clip(audit['road_lower'],-6,6)/6,np.clip(audit['obstacle_lower'],-6,6)/6,
                       paths[:,n//4,1]/6,paths[:,n//2,1]/6,paths[:,n,1]/6,paths[:,n,0]/12,
                       np.broadcast_to(common,(len(tab),len(common)))])
    if x.shape!=(len(tab),len(INPUTS)) or not np.isfinite(x).all():raise ValueError('Trajectory input mismatch')
    return np.clip(x,-4,4).astype('float32')

def corrections(net,x):
    h=np.tanh(x @ np.asarray(net['w1']) + np.asarray(net['b1'])[:,None,:])
    y=(h @ np.asarray(net['w2']))[:,:,0]
    return CAP*np.tanh(y).mean(0)

class TrajectorySelector(WholeLapSelector):
    def __init__(self,path,pace):
        super().__init__(path,pace)
        cfg=json.loads(path.read_bytes())
        self.net=cfg.get('network')
        self.mode=cfg['mode']

    def rank(self,features,allowed,baseline,audit,obs,*,launch_evidence=None):
        if self.mode=='baseline':
            return super().rank(features,allowed,baseline,audit)
        table=np.asarray(audit['table'])
        from concept_ranker.sensor626 import encode as active_encode
        x=active_encode(audit,obs.road,obs.heading,obs.objects,obs.speed,obs.yaw)
        raw=corrections(self.net,x)
        base=table[:,7]
        final=base+raw
        pick=choose(final,allowed)
        if allowed[baseline] and final[pick]<=final[baseline]+1e-8:pick=baseline
        # A selected stop is a command. Never replace it with a launch or
        # change the ranking scores after H12 has made the choice.
        launch = None
        if launch_evidence is not None:
            launch = dict(h12_index=int(pick), applied=False,
                          reason='h12_selected_stop' if table[pick,0] == 0 else 'h12_selected_motion',
                          selectable_forward_count=int(np.count_nonzero(allowed & (table[:,0] > 0))),
                          obstacle_clear=bool(launch_evidence.get('obstacle_clear')))
        assert allowed[pick] and table[pick,6]>0
        return pick,dict(baseline=int(baseline),selected=int(pick),eligible=allowed.tolist(),
                         final_scores=final.tolist(),base_scores=base.tolist(),score_adjustments=raw.tolist(),
                         first_control_changed=not np.allclose(table[pick,:2],table[baseline,:2],atol=1e-5,rtol=0),
                         directTrajectory=True, launch_decision=launch)
