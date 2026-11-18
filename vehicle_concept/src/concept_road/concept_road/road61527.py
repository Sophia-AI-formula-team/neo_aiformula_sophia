"""Runtime source extracted from the accepted V6.26 freeze: experiments/v615_recovery/road61527.py. """

import numpy as np

from concept_road.road615 import RawMaterialRoad

from concept_road.road61524 import RememberedRoad61524, current_lane_heading

class CapturedMaterialRoad(RawMaterialRoad):
    def __init__(self):
        super().__init__();self.heading_depth=np.empty(0);self.measured_heading=np.empty(0)

    def read(self,image,raw,lane_mask,speed=0.,yaw=0.,dt=.2,object_boxes=()):
        road,pixels,heading=super().read(image,raw,lane_mask,speed,yaw,dt,object_boxes)
        # RoadMap guided mode mutates the returned heading in place. Capture
        # before that mutation; do not run the temporal observer a second time.
        self.heading_depth=road[:,0].copy()
        self.measured_heading=heading.copy()
        return road,pixels,heading

class RememberedRoad61527(RememberedRoad61524):
    def __init__(self):
        super().__init__();self.material_road=CapturedMaterialRoad()

    def read(self,image,raw,lane,boxes=(),speed=0.,yaw=0.,dt=.2):
        road,pixels,heading=super().read(image,raw,lane,boxes,speed,yaw,dt)
        if image.max()<3:
            self.last['heading_guidance61527']=dict(source='transported_last_actual_direction',applied_rows=0)
            return road,pixels,heading
        material=self.material_road
        if len(road) and len(material.heading_depth):
            base=np.interp(road[:,0],material.heading_depth,material.measured_heading)
            heading,audit=current_lane_heading(road,base,self.last)
            audit.update(base_source='material_heading_before_B_envelope_override',
                material_direction_unknown=material.last.get('unknown_heading',True),
                material_depth_range=[float(material.heading_depth[0]),float(material.heading_depth[-1])],
                rule='Direction only: material base plus current lane pair in actual support; geometry and candidate-width guidance unchanged')
            self.last['heading_guidance61527']=audit
            if self.cached is not None:
                self.cached=(self.cached[0], heading.copy())
        return road,pixels,heading
