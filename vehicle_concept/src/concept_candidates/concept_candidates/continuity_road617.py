"""Runtime source extracted from the accepted V6.26 freeze: experiments/v617_regression/continuity_road617.py. """

import numpy as np

from concept_candidates.curve_road617 import RoadAwareCurvePlanner, preview_reason

class RoadAwareContinuityPlanner(RoadAwareCurvePlanner):
    def step(self,obs):
        output=super().step(obs);a=self.last['audit'];selected=a.get('selected')
        a['preview_reasons']=[preview_reason(o,obs.road) for o in obs.objects] if len(obs.road) else []
        if selected is None or self.pending is None or not a.get('schedule'):
            return output
        table=np.asarray(a['table']);nominal=np.asarray(a['nominal_feasible'],bool)
        choices=nominal & (table[:,0]>0) & (table[:,6]>0)
        before=int(selected);after=before
        if a['schedule'][before]=='drive_2s_then_stop_and_wait' and choices.any():
            after=int(np.argmax(np.where(choices,table[:,7],-np.inf)))
            pending=self.pending
            controls=pending['brake_controls' if pending['brake_ok'][after] else 'escape_controls'][after]
            self.command=controls[0].copy()
            self.last['path']=a['paths'][after];self.last['target'][1]=float(table[after,2])
            self.last['margin']=float(table[after,5]);self.last['recovery']='not_needed'
            a['selected']=after
        a['continuity_rank']=dict(original_selected=before,selected=after,nominal_forward=int(choices.sum()),
            rule='When a continuing forward trajectory is feasible, rank those before a drive-then-stop profile; no hard gate changes',
            safety_mask_changed=False,trained_network=False)
        return self.command.copy()
