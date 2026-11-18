"""Runtime source extracted from the accepted V6.26 freeze: experiments/v616_decision/rear_road.py. """

import numpy as np

from dataclasses import dataclass

from concept_road.compare61513 import Guided

from concept_road.road61527 import RememberedRoad61527

from concept_road.rear_memory import RearSurfaceMemory

@dataclass(frozen=True)
class RecoveryObservation(Guided):
    rear_observed_limit: 'float | None' = None

class RearRoad(RememberedRoad61527):
    def __init__(self):
        super().__init__()
        self.surface_memory=RearSurfaceMemory()
        memory=self.surface_memory
        memory.zz,memory.xx=np.meshgrid(np.arange(-3.28,4.01,memory.cell),
            np.arange(-5.04,5.05,memory.cell),indexing='ij')

    def read(self,*args,**kwargs):
        road,pixels,heading=super().read(*args,**kwargs)
        extra=[]
        if len(road):
            # Work from the vehicle backwards and stop at the first unknown row.
            # Existing startup inference is never extended into the rear corridor.
            start=float(road[0,0])
            for z in np.arange(start-.16,-3.121,-.16):
                history=self.surface_memory.row(z)
                if history is None:break
                row,age=history
                extra.append(row)
            if extra:
                extra=np.asarray(extra[::-1]);road=np.r_[extra,road]
                heading=np.r_[np.full(len(extra),heading[0]),heading]
        near=self.last.get('nearfield_rows',[])
        backwards=[row for row in near if row['z']<=0]
        observed=(backwards and all(row['source']=='historical_material_observation' for row in backwards))
        observed_limit=float(road[0,0]) if len(road) and observed else None
        self.last['rear_history']=dict(rows=len(extra),rear_limit_m=float(road[0,0]) if len(road) else None,
            observed_limit_m=observed_limit,
            source='Previously camera-observed pavement transported by signed wheel motion and gyro; no new rear camera',
            direction='Nearest available direction prior; not a measured rear tangent')
        return road,pixels,heading
