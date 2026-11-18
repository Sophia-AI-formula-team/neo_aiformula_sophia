"""Runtime source extracted from the accepted V6.26 freeze: experiments/v615_recovery/road6157.py. """

import numpy as np

from concept_road.road615 import RememberedRoad615

class RememberedRoad6157(RememberedRoad615):
    def __init__(self):
        super().__init__()
        self.cached = None
        self.blind_travel = 0.
        self.blind_turn = 0.

    def read(self, image, raw, lane, boxes=(), speed=0., yaw=0., dt=.2):
        if image.max() >= 3:
            result = super().read(image, raw, lane, boxes, speed, yaw, dt)
            if len(result[0]) >= 3:
                self.cached = (result[0].copy(), result[2].copy())
                self.blind_travel = self.blind_turn = 0.
            else:
                # An untransported cache cannot survive an invalid visible frame.
                self.cached = None
            return result
        self.blind_travel += abs(speed)*dt
        self.blind_turn += abs(yaw)*dt
        empty = np.zeros(image.shape[:2], bool)
        self.surface_memory.current_lane = empty
        self.surface_memory.update(empty, speed, yaw, dt, visible=empty)
        self.tracks = self.transport(speed,yaw,dt)
        self.material_road.history = []
        self.last = dict(tracks=self.tracks, lane_geometry_rows=[], camera_valid=False, nearfield_rows=[],
                         nearfield_markings=self.surface_memory.marking_points)
        if self.cached is None or self.blind_travel > 5.5 or self.blind_turn > 1.2:
            return np.empty((0,5)), [], np.empty(0)
        road, heading = self.cached
        angle = yaw*dt; c, s = np.cos(angle), np.sin(angle)
        dz = speed*dt*np.sinc(angle/np.pi)
        dx = speed*dt*(1-c)/angle if abs(angle)>1e-5 else 0.
        sides = []
        for side in [1,2]:
            z, x = road[:,0]-dz, road[:,side]-dx
            zz, xx = c*z+s*x, -s*z+c*x
            sigma = road[:,side+2]+.04*abs(speed)*dt+.08*abs(angle)*np.hypot(z,x)
            order = np.argsort(zz)
            sides.append((zz[order], xx[order], sigma[order]))
        lo, hi = max(sides[0][0].min(), sides[1][0].min()), min(sides[0][0].max(), sides[1][0].max())
        zz = np.unique(np.r_[sides[0][0],sides[1][0]])
        zz = zz[(zz>=lo)&(zz<=hi)]
        if len(zz)<3:
            self.cached = None
            return np.empty((0,5)),[],np.empty(0)
        newer = np.c_[zz, np.interp(zz,sides[0][0],sides[0][1]), np.interp(zz,sides[1][0],sides[1][1]),
                      np.interp(zz,sides[0][0],sides[0][2]), np.interp(zz,sides[1][0],sides[1][2])]
        newer_heading = np.interp(zz+dz,road[:,0],heading)-angle
        self.last.update(camera_valid=False, road_source='last_actual_view_transported_during_blackout',
                         blind_travel_m=self.blind_travel, blind_turn_rad=self.blind_turn)
        self.cached = (newer, newer_heading)
        return newer,[],newer_heading
