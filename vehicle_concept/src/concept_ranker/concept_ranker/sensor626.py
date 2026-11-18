"""Runtime source extracted from the accepted V6.26 freeze: experiments/v626_time/sensor626.py. """

import numpy as np

from concept_ranker.trajectory624 import encode as previous_encode

def encode(audit, road, heading, objects, speed, yaw):
    old = previous_encode(audit,road,heading,objects,speed,yaw)
    road, heading = np.asarray(road), np.asarray(heading)
    z, left, right = road[:,:3].T
    end = np.asarray(audit['paths'])[:,-1]
    def location(depth, x):
        l, r = np.interp(depth,z,left), np.interp(depth,z,right)
        relative = (2*x-l-r)/np.maximum(r-l,.5)
        return np.where((depth>=z[0]) & (depth<=z[-1]), np.clip(relative,-2.,2.), 0.)
    near, far = max(1.,z[0]), min(10.,z[-1])
    bend = np.interp(far,z,heading)-np.interp(near,z,heading) if far>near else 0.
    lateral = location(end[:,0],end[:,1])
    context = np.column_stack([np.full(len(end),location(near,0.)), lateral, np.full(len(end),bend), lateral*bend])
    return np.column_stack([old,context]).astype('float32')
