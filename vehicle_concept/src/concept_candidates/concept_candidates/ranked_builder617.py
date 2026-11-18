"""Runtime source extracted from the accepted V6.26 freeze: experiments/v617_regression/ranked_builder617.py. """

import numpy as np

from concept_candidates.ranker617 import observation_features

class RankedBuilder:
    def __init__(self,planner):
        self.planner=planner;self.table=None;self.baseline=None;self.fallback=None
        self.network_allowed=False;self.features=None

    def prepare(self,obs):
        self.planner.base.reset_candidate_evaluation()
        self.fallback=self.planner.step(obs).copy();last=self.planner.last;audit=last['audit']
        # Recovery can replace the forward audit after trying one or more forward
        # batches. Report the work performed in this decision in either case.
        audit['candidate_evaluation'] = self.planner.base.candidate_evaluation.copy()
        table=np.asarray(audit.get('table',[]),float);self.baseline=audit.get('selected')
        self.table=table if len(table) else None;self.features=None
        if self.table is None:
            if self.baseline is not None:raise ValueError('Empty table has a selected candidate')
            self.network_allowed=False
            return None,np.zeros(0,bool),None
        if self.table.shape not in ((1,12),(75,12)):raise ValueError('Unexpected candidate table')
        allowed=self.table[:,6]>0
        if self.baseline is None:
            if allowed.any():
                raise ValueError('Missing selection with checked candidates')
            self.network_allowed=False
            return None,allowed,None
        if not 0<=self.baseline<len(table) or not allowed[self.baseline]:
            raise ValueError('Baseline must be in the checked set')
        self.network_allowed=(self.table.shape==(75,12) and self.planner.phase=='forward'
            and self.planner.entry is None and self.planner.pending is None
            and self.planner.base.pending is not None and not last.get('contingency_execution')
            and not audit.get('recovery_exit') and not audit.get('resume_after_rear_loss'))
        if self.network_allowed:
            candidate,objects,road,feature_mask=observation_features(obs,audit)
            if not np.array_equal(feature_mask,allowed):raise AssertionError('Feature mask differs from checked set')
            self.features=(candidate,objects,road)
            return self.features,allowed,self.baseline
        # Recovery, release and active escape stay under their own checked owner.
        owned=np.zeros(len(table),bool);owned[self.baseline]=True
        return None,owned,self.baseline

    def command(self,index):
        if self.baseline is None:
            if index is not None:raise ValueError('Cannot select from an empty set')
            self.planner.commit(None);return self.fallback.copy()
        if index is None or not 0<=int(index)<len(self.table) or not self.table[int(index),6]>0:
            raise ValueError('Selection is not a checked candidate')
        index=int(index)
        if not self.network_allowed and index!=self.baseline:
            raise ValueError('Recovery or contingency owns this action')
        control=self.fallback.copy()
        if index!=self.baseline:
            pending=self.planner.base.pending
            if pending['retimed'][index]:sequence=pending['profile_controls'][index]
            elif pending['brake_ok'][index]:sequence=pending['brake_controls'][index]
            else:sequence=pending['escape_controls'][index]
            control=np.asarray(sequence[0],float).copy()
            if not np.allclose(control,self.table[index,:2],atol=1e-5,rtol=0):
                raise ValueError('Candidate command differs from its checked control sequence')
            self.planner.command=control.copy();self.planner.base.command=control.copy()
            last=self.planner.last;audit=last['audit'];audit['selected']=index
            last['path']=audit['paths'][index];last['target']=last['path'][-1]
            last['margin']=float(self.table[index,5])
        self.planner.commit(index)
        return control
