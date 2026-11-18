"""Runtime source extracted from the accepted V6.26 freeze: experiments/v617_regression/runtime_map.py. """

import math

import cv2

import numpy as np

CELL = .20

DEPTH_STEP = .20

def transform(points, pose):
    z, x, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + (z, x)

def lane_sections(evidence, raw=None, projection=None):
    tracks = evidence['tracks']
    current = evidence.get('current_lane')
    if current is None or current['score'] <= .45:
        current = None
    parts = []
    for band in evidence['allowed_bands']:
        left, right = [tracks[band[side]] for side in ('left', 'right')]
        lo = max(left['zmin'], right['zmin'])
        hi = min(left['zmax'], right['zmax'])
        if hi-lo < .8:
            continue
        depth = np.linspace(lo, hi, int(math.ceil((hi-lo)/DEPTH_STEP))+1)
        l, r = [np.polynomial.polynomial.polyval(depth, edge['coef']) for edge in (left, right)]
        valid = (r-l >= 1.2) & (r-l <= 6.5)
        # Require samples near the claimed section, so a long absent dash does
        # not give a distant fitted marking unlimited support.
        for edge in (left, right):
            points = np.asarray(edge['points'])
            valid &= np.min(abs(depth[:, None]-points[:, 0]), axis=1) <= 1.0
        for group in np.split(np.flatnonzero(valid), np.flatnonzero(np.diff(np.flatnonzero(valid))>1)+1):
            if len(group) < 2:
                continue
            z = depth[group]
            parts.append(dict(kind='ego' if band == current else 'adjacent',
                              rows=np.c_[z,l[group],r[group]].tolist(),
                              fit_error_m=max(left['error'], right['error']),
                              basis='two_supported_markings'))
    # B directly observes ego-lane pixels even if both markings cannot yet
    # define a band. Preserve these sparse runs; camera edges are not borders.
    if raw is not None and projection is not None:
        focal, cx, cy, height = projection
        for z in np.arange(1.2, 12.0, DEPTH_STEP):
            if any(p['kind']=='ego' and p['rows'][0][0]-.1 <= z <= p['rows'][-1][0]+.1 for p in parts):
                continue
            v = round(cy+height*focal/z)
            if not 1 <= v < raw.shape[0]-1:
                continue
            active = np.flatnonzero(raw[v-1:v+2].sum(axis=0) >= 2)
            for g in np.split(active, np.flatnonzero(np.diff(active)>1)+1):
                if len(g) < 5:
                    continue
                l, r = (g[[0,-1]]-cx)*z/focal
                parts.append(dict(kind='ego_mask', rows=[[z-.10,l,r],[z+.10,l,r]],
                                  fit_error_m=None, basis='projected_B_pixels'))
    return parts

class RuntimeMap:
    def __init__(self):
        self.pose = np.zeros(3)
        self.t = None
        self.motion = None
        self.cells = {}
        self.static = {}
        self.dynamic = {}
        self.path = []
        self.distance = 0.

    def _observe(self, parts):
        updates = {}
        for part in parts:
            a = np.asarray(part['rows'])
            polygon = np.vstack((a[:, :2], a[::-1][:,[0,2]]))
            polygon = transform(polygon, self.pose)
            grid = np.floor(polygon[:, ::-1]/CELL).astype(np.int32)
            low = grid.min(axis=0)
            size = grid.max(axis=0)-low+1
            canvas = np.zeros((size[1],size[0]), np.uint8)
            cv2.fillPoly(canvas, [grid-low], 1)
            kind = {'ego': 0, 'adjacent': 1, 'ego_mask': 2}[part['kind']]
            for iz, ix in zip(*np.nonzero(canvas)):
                k = (int(iz+low[1]),int(ix+low[0]))
                updates[k] = updates.get(k, 0) | (1<<kind)
        # At most one vote of each class per cell and frame. Keep per-class
        # counts so repeated observation and colour conflicts remain auditable.
        for key, bit in updates.items():
            state = self.cells.setdefault(key,[0,0,0,self.t])
            for k in range(3):
                if bit & (1<<k):
                    state[k] += 1
            state[3] = self.t
        return updates

    def _obstacles(self, objects, evidence):
        now = []
        if len(objects) != len(evidence['tracks']):
            raise ValueError('object and track counts differ')
        for o, track in zip(objects, evidence['tracks']):
            uid = track['id']
            static = float(o[25]) >= .5 and not track['dynamic_confirmed'] and uid not in self.dynamic
            state = np.asarray(o[26:30] if static else o[46:50])
            cov = np.asarray(o[30:46] if static else o[50:66]).reshape(4,4)
            c, s = np.cos(self.pose[2]), np.sin(self.pose[2])
            rot = np.array([[c,-s],[s,c]])
            pos = transform(state[:2], self.pose)
            age = track['last_observation_age_s']
            if age > 2. and not static:
                continue
            info = dict(id=uid, position=pos.tolist(), age_s=age, radius_m=float(o[4]),
                        motion='static' if static else 'moving' if track['dynamic_confirmed'] else 'uncertain')
            if static and age < .01 and track['hits'] >= 3:
                score = float(np.trace(cov[:2,:2]))
                if uid not in self.static or score < self.static[uid]['position_variance_m2']:
                    self.static[uid] = {**info, 'time':self.t, 'position_variance_m2':score}
            if not static:
                if track['dynamic_confirmed']:
                    self.static.pop(uid,None)
                ranges=[]
                for dt in (0.,.5,1.,1.5):
                    h=np.c_[np.eye(2),dt*np.eye(2)]
                    acc=np.asarray(track['acceleration_covariance'])
                    spread=rot@(h@cov@h.T+acc*dt**3/3)@rot.T
                    ranges.append(dict(in_s=dt, center=(pos+rot@state[2:]*dt).tolist(),
                                       covariance=spread.tolist()))
                info['ranges']=ranges
                if track['dynamic_confirmed'] and age < .21:
                    self.dynamic[uid] = {**info,'time':self.t}
            now.append(info)
        for released in evidence.get('released', []):
            self.static.pop(released['id'],None)
            self.dynamic.pop(released['id'],None)
        return now

    def update(self, sensor, parts, objects, obstacle_evidence):
        t = float(sensor['t'])
        speed = (sensor['leftRpm']+sensor['rightRpm'])*math.pi*sensor['wheelRadius']/60
        yaw_rate = float(sensor['gyro'])
        if not np.isfinite([t,speed,yaw_rate]).all():
            raise ValueError('Non-finite sensor sample')
        if self.t is not None:
            dt = t-self.t
            if not 0 < dt <= .5:
                raise ValueError('Unexpected interval in runtime map')
            distance, a = .5*(np.asarray(self.motion)+[speed,yaw_rate])*dt
            dz = distance*np.sinc(a/np.pi)
            dx = distance*(1-np.cos(a))/a if abs(a)>1e-7 else 0.
            self.pose[:2] = transform([dz,dx],self.pose)
            self.pose[2] += a
            self.distance += abs(distance)
        self.t, self.motion = t, (speed,yaw_rate)
        self.path.append([t,*self.pose.tolist()])
        updates = self._observe(parts)
        now = self._obstacles(objects,obstacle_evidence)
        return dict(t=t, pose=self.pose.tolist(), wheel_distance=self.distance,
                    parts=parts, cells=len(self.cells), changed=[[z,x,*self.cells[z,x]] for z,x in updates],
                    static=list(self.static.values()), dynamic=list(self.dynamic.values()), now=now)
