"""Runtime source extracted from the accepted V6.26 freeze: experiments/v616_decision/rear_memory.py. """

import cv2

import numpy as np

from concept_road.vision import CX, CY, FOCAL, HEIGHT

class RearSurfaceMemory:
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
        self.frames = [f for f in self.frames if self.travel-f[3] < 5.5 and self.turn-f[4] < 1.2]
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
