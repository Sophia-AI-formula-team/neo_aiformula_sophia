"""Runtime source extracted from the accepted V6.26 freeze: experiments/v615_recovery/road615.py. """

import cv2

import numpy as np

from concept_road.vision import CX, CY, FOCAL, HEIGHT

from concept_road.road import RoadMap

from concept_road.material import MaterialRoad

class RawMaterialRoad(MaterialRoad):
    # This observer uses the actual pixel endpoints below. Only its heading
    # consumes the polynomial fit, not its smoothed boundary positions.
    fit_boundaries = False

    def read(self, image, raw, lane_mask, speed=0., yaw=0., dt=.2, object_boxes=()):
        road, pixels, heading = super().read(image, raw, lane_mask, speed, yaw, dt, object_boxes)
        if len(road) != len(pixels):
            raise ValueError('Raw row/pixel pairing lost')
        audit = []
        for row, (left, right, v) in zip(road, pixels):
            row[1] = (left-CX)*row[0]/FOCAL
            row[2] = (right-CX)*row[0]/FOCAL
            # Same base measurement allowance as the original raw observer.
            # Polynomial residual belongs to heading quality, not a second
            # inward shift of this directly sampled support endpoint.
            row[3] = .045+.004*row[0]
            audit.append(dict(z=float(row[0]), left_source='fov_limit' if left<=3 else 'material_support',
                              right_source='fov_limit' if right>=image.shape[1]-4 else 'material_support'))
        self.last['raw_support_rows'] = audit
        self.last['boundary_rule'] = 'Raw material support; fit only guides heading. Segmentation allowance remains experimental.'
        return road, pixels, heading

class SurfaceMemory615:
    def __init__(self, cell=.08):
        self.cell = cell
        self.points = np.empty((0, 4))
        self.frames = []
        self.time = 0.
        self.travel = 0.
        self.turn = 0.
        self.current_lane = None
        self.marking_points = []
        # This origin is reset each run and integrates only wheel/gyro. It is
        # not the simulator pose and never uses its track or progress.
        self.pose = np.zeros(3)
        self.zz, self.xx = np.meshgrid(np.arange(-1.28, 4.01, cell),
                                     np.arange(-5.04, 5.05, cell), indexing='ij')

    def update(self, mask, speed, yaw, dt, visible=None):
        visible = mask if visible is None else np.asarray(visible, bool)
        if not 0 < dt <= .5 or not np.isfinite([speed, yaw]).all():
            self.frames = []
            self.pose[:] = 0.
            dt, speed, yaw = 0., 0., 0.
        angle = yaw*dt
        dz = speed*dt*np.sinc(angle/np.pi)
        dx = speed*dt*(1-np.cos(angle))/angle if abs(angle) > 1e-5 else 0.
        c, s = np.cos(self.pose[2]), np.sin(self.pose[2])
        self.pose[:2] += [c*dz-s*dx, s*dz+c*dx]
        self.pose[2] += angle
        self.time += dt
        self.travel += abs(speed)*dt
        self.turn += abs(angle)
        if visible.any():
            fresh = (mask.copy(), self.pose.copy(), self.time, self.travel, self.turn, visible.copy(), None if self.current_lane is None else self.current_lane.copy())
            if self.frames and np.linalg.norm(self.pose[:2]-self.frames[-1][1][:2]) < .03 and abs(self.pose[2]-self.frames[-1][1][2]) < .01:
                self.frames[-1] = fresh
            else:
                self.frames.append(fresh)
        self.frames = [f for f in self.frames if self.travel-f[3] < 5.5 and self.turn-f[4] < 1.2][-32:]
        c, s = np.cos(self.pose[2]), np.sin(self.pose[2])
        integrated_z = self.pose[0]+c*self.zz-s*self.xx
        integrated_x = self.pose[1]+s*self.zz+c*self.xx
        errors, ages = np.full(self.zz.shape, np.inf), np.full(self.zz.shape, np.inf)
        # Reproject into the ORIGINAL saved mask each time. Repeatedly rounding
        # transported point centres created fake gaps in a continuous surface.
        markings = {}
        for image_mask, pose, when, distance, rotation, visibility, lane in self.frames:
            dz, dx = integrated_z-pose[0], integrated_x-pose[1]
            c, s = np.cos(pose[2]), np.sin(pose[2])
            z, x = c*dz+s*dx, -s*dz+c*dx
            safe_z = np.maximum(z, .01)
            u = np.rint(CX+x*FOCAL/safe_z).astype(int)
            v = np.rint(CY+HEIGHT*FOCAL/safe_z).astype(int)
            valid = (z >= .95) & (z <= 4.) & (u >= 0) & (u < mask.shape[1]) & (v >= 0) & (v < mask.shape[0])
            support = np.zeros_like(valid)
            support[valid] = image_mask[v[valid], u[valid]]
            error = .06+.04*(self.travel-distance)+.08*(self.turn-rotation)*np.hypot(z, x)
            negative = np.zeros_like(valid)
            negative[valid] = visibility[v[valid], u[valid]] & ~image_mask[v[valid], u[valid]]
            negative &= error <= .30
            if lane is not None:
                marked = np.zeros_like(valid)
                marked[valid] = lane[v[valid], u[valid]]
                for iz, ix in zip(*np.nonzero(marked & (error <= .30))):
                    markings[(int(iz), int(ix))] = self.time-when
                for iz, ix in zip(*np.nonzero(negative)):
                    markings.pop((int(iz), int(ix)), None)
            errors[negative], ages[negative] = np.inf, np.inf
            # Erode each original observation by its own transport allowance BEFORE
            # union. This avoids assigning one old fringe cell's error to a
            # whole edge that is also supported by a newer, better observation.
            padded = np.pad(support.astype(np.uint8), 1)
            distance_to_unknown = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]*self.cell
            support &= distance_to_unknown > error+self.cell/2
            better = support & (error <= .30) & (error <= errors)
            errors[better], ages[better] = error[better], self.time-when
        self.marking_points = [[float(self.zz[k]), float(self.xx[k]), float(age)] for k, age in markings.items()]
        valid = np.isfinite(errors)
        self.points = np.c_[self.zz[valid], self.xx[valid], ages[valid], errors[valid]]
        return self.points

    def row(self, z):
        nearby = self.points[abs(self.points[:, 0]-z) <= .75*self.cell]
        if len(nearby) < 4:
            return None
        cells = np.unique(np.rint(nearby[:, 1]/self.cell).astype(int))
        groups = np.split(cells, np.flatnonzero(np.diff(cells) > 1)+1)
        # A disconnected group is never joined merely because both are road.
        group = min(groups, key=lambda g: max(g[0], -g[-1], 0))
        if len(group) < 4:
            return None
        selected = nearby[(nearby[:, 1] >= group[0]*self.cell-self.cell/2) &
                          (nearby[:, 1] <= group[-1]*self.cell+self.cell/2)]
        left = selected[selected[:, 1] <= (group[0]+1)*self.cell+1e-8]
        right = selected[selected[:, 1] >= (group[-1]-1)*self.cell-1e-8]
        return [z, group[0]*self.cell, group[-1]*self.cell,
                0., 0.], float(selected[:, 2].max())

class RememberedRoad615(RoadMap):
    def __init__(self):
        super().__init__('guided')
        self.surface_memory = SurfaceMemory615()
        self.material_road = RawMaterialRoad()

    def read(self, image, raw, lane, boxes=(), speed=0., yaw=0., dt=.2):
        self.surface_memory.current_lane = lane
        road, pixels, heading = super().read(image, raw, lane, boxes, speed, yaw, dt)
        pavement = self.material_road.pavement
        if pavement is None or len(road) < 3:
            self.surface_memory.update(np.zeros(image.shape[:2], bool), speed, yaw, dt)
            return road, pixels, heading
        # Match measured material support, without the base observer's occlusion
        # bridge/morphological fill. All object boxes remove current samples.
        chroma_squared = self.material_road.chroma_squared
        support = chroma_squared < 12**2
        # Ambiguous colour and detected occlusion are unknown, not a negative
        # road measurement. A clear current green verge does invalidate history.
        visible = support | (chroma_squared > 18**2)
        support[:int(CY)+2] = False
        visible[:int(CY)+2] = False
        for x0, y0, x1, y1 in boxes:
            support[max(0, int(y0)):min(image.shape[0], int(y1)+1),
                    max(0, int(x0)):min(image.shape[1], int(x1)+1)] = False
            visible[max(0, int(y0)):min(image.shape[0], int(y1)+1),
                    max(0, int(x0)):min(image.shape[1], int(x1)+1)] = False
        self.surface_memory.update(support, speed, yaw, dt, visible=visible)
        road = np.c_[road, road[:, 3]]
        near_rows, sources, near_heading = [], [], []
        # The first-frame blind approach must be marked inferred, otherwise a
        # front-only camera can never start. It retains cross-section width with
        # increasing error; the footprint check subtracts that error on each side.
        anchor = road[0]
        tangent = float(np.clip(np.tan(heading[0]), -1., 1.))
        for z in np.arange(-1.2, .951, .16):
            if z >= road[0, 0]:
                break
            historical = self.surface_memory.row(z)
            dz = anchor[0]-z
            inferred = [z, anchor[1]-dz*tangent, anchor[2]-dz*tangent, anchor[3]+.10*dz, anchor[4]+.10*dz]
            row, source, age = inferred, 'inferred_from_nearest_cross_section', None
            if historical is not None:
                measured, age = historical
                # A historical observed run can replace inference only where it
                # supports a complete vehicle width; sparse fragments stay logged.
                if measured[2]-measured[1] >= .76+measured[3]+measured[4]:
                    row, source = measured, 'historical_material_observation'
            near_rows.append(row)
            near_heading.append(heading[0])
            sources.append(dict(z=float(z), source=source, age_s=age,
                                allowance_left_m=float(row[3]), allowance_right_m=float(row[4])))
        self.last.update(version='V6.15', nearfield_rows=sources,
                         nearfield_cells=int(len(self.surface_memory.points)),
                         nearfield_markings=self.surface_memory.marking_points,
                         nearfield_history=self.surface_memory.points[self.surface_memory.points[:, 0] < .95].round(4).tolist(),
                         nearfield_note='Surface only. Current and predicted obstacles independently veto all paths. Startup inference is not an observation.',
                         rear_occupancy_observed=False)
        if near_rows:
            road = np.vstack((near_rows, road))
            heading = np.r_[near_heading, heading]
        return road, pixels, heading
