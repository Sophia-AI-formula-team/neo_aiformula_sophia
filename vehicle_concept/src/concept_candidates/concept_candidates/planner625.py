"""Runtime source extracted from the accepted V6.26 freeze: experiments/v625_boundary/planner625.py. """

from dataclasses import replace

from concept_road.onboard625 import HALF_TRACK, HALF_TREAD

from concept_candidates.continuity_road617 import RoadAwareContinuityPlanner

from concept_road.v3_model import BODY_HALF_WIDTH

class MarkedPlanner(RoadAwareContinuityPlanner):
    def target_points(self,obs,look):
        # The parent insets targets for the body; use the inside edge of the
        # outside tyre here. Its full body rectangle still checks every cone.
        adjustment=BODY_HALF_WIDTH-(HALF_TRACK-HALF_TREAD)
        virtual=obs.road.copy()
        virtual[:,1]-=adjustment
        virtual[:,2]+=adjustment
        guidance=getattr(obs,'guidance',None)
        if guidance is not None:
            guidance=guidance.copy()
            guidance[:,1]-=adjustment
            guidance[:,2]+=adjustment
        return super().target_points(replace(obs,road=virtual,guidance=guidance),look)
