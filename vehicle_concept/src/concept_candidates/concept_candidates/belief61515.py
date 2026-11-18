"""Runtime source extracted from the accepted V6.26 freeze: experiments/v615_recovery/belief61515.py. """

import numpy as np

try:
    from . import trajectory_batch as batch
except ImportError:
    batch = None

from concept_road.vision import FOCAL, CX, CY, HEIGHT

from concept_feasibility.response61515 import advance

from concept_feasibility.geometry61514 import road_clearance, obstacle_clearance, path_obstacle_clearance

class CameraBelief:
    def __init__(self, objects, count):
        self.objects = objects
        self.p = np.asarray([np.tile(o[6:22].reshape(4, 4), (count, 1, 1)) for o in objects])
        self.updates = np.zeros(count, int)

    def advance(self, dt, time, z, x, heading, observe):
        if len(self.objects) == 0:
            return self.p
        f = np.eye(4); f[:2, 2:] = np.eye(2)*dt
        q = np.block([[np.eye(2)*dt**3/3, np.eye(2)*dt**2/2], [np.eye(2)*dt**2/2, np.eye(2)*dt]])*.12**2
        intensity = np.asarray([o[24] if len(o)>24 else .12 for o in self.objects])
        self.p = f@self.p@f.T+q*(intensity[:,None,None,None]/.12)**2
        if not observe:
            return self.p
        c, s = np.cos(heading), np.sin(heading)
        projected = []
        for o in self.objects:
            dz, dx = o[0]+o[2]*time-z, o[1]+o[3]*time-x
            forward, lateral = dz*c+dx*s, dx*c-dz*s
            safe = np.maximum(forward, .01)
            u, v = CX+FOCAL*lateral/safe, CY+HEIGHT*FOCAL/safe
            halfwidth = o[4]*FOCAL/safe
            projected.append((forward, lateral, u, v, halfwidth))
        for i, (o, projection) in enumerate(zip(self.objects, projected)):
            # Age and confirmation count are observed tracker metadata.
            if len(o) < 24 or o[22] > 1e-6 or o[23] < 3:
                continue
            forward, lateral, u, v, halfwidth = projection
            covariance = self.p[i, :, :2, :2]
            fvar = covariance[:,0,0]*c*c+2*covariance[:,0,1]*c*s+covariance[:,1,1]*s*s
            lvar = covariance[:,0,0]*s*s-2*covariance[:,0,1]*c*s+covariance[:,1,1]*c*c
            pad_z, pad_x = 2*np.sqrt(np.maximum(0., fvar)), 2*np.sqrt(np.maximum(0., lvar))
            safe_z = np.maximum(.1, forward-pad_z)
            pad_u = FOCAL*(pad_x/safe_z+abs(lateral)*pad_z/safe_z**2)
            bottom = CY+HEIGHT*FOCAL/safe_z
            visible = ((forward-pad_z > 1.05) & (forward+pad_z < 12) &
                       (u-halfwidth-pad_u > 2) & (u+halfwidth+pad_u < 638) & (bottom < 357))
            for j, (other_z, other_x, other_u, other_v, other_width) in enumerate(projected):
                if i != j:
                    visible &= ~((other_z > .1) & (other_z < forward) & (abs(other_u-u) < other_width+halfwidth))
            for k in np.flatnonzero(visible):
                dzdv = -forward[k]**2/(HEIGHT*FOCAL)
                jacobian = np.array([[0., dzdv], [forward[k]/FOCAL, lateral[k]/forward[k]*dzdv]])
                measurement = jacobian@np.diag([9., 9.])@jacobian.T+np.eye(2)*.04**2
                # Repeated images share calibration/segmentation bias. A .07m
                # floor prevents optimistic collapse from correlated views.
                measurement += np.eye(2)*.07**2
                rotation = np.array([[c[k], -s[k]], [s[k], c[k]]])
                measurement = rotation@measurement@rotation.T
                p = self.p[i, k]
                gain = p[:, :2]@np.linalg.inv(p[:2, :2]+measurement)
                h = np.zeros((2, 4)); h[:, :2] = np.eye(2)
                residual = np.eye(4)-gain@h
                self.p[i, k] = residual@p@residual.T+gain@measurement@gain.T
                self.updates[k] += 1
        return self.p

def braking_branch(obs, commands, first_yaw, stop_age, uncertainty=1., coast_seconds=.2, details=False, control_sequence=None, coast_sequence=None):
    """Execute one interval, then model observation loss and coast-to-stop.

    No hypothetical future measurement may shrink this branch's obstacles.
    It is a model-based contingency, not a physical safety certificate.
    """
    if batch is not None:
        return batch.braking(obs,commands,first_yaw,stop_age,uncertainty,coast_seconds,details,control_sequence,coast_sequence)
    n = len(commands)
    z, x, angle, travel, turn = [np.zeros(n) for _ in range(5)]
    speed, yaw = np.full(n, obs.speed), np.full(n, obs.yaw)
    zero_age = np.where(commands == 0., stop_age, 0.)
    _, road_now, known_now = road_clearance(obs.road, z, x, angle, uncertainty)
    _, obstacle_now = obstacle_clearance(obs.objects, z, x, angle, 0., uncertainty)
    valid = (road_now >= 0.) & (obstacle_now > 0.) & known_now
    stopped = np.zeros(n, bool)
    minimum = np.minimum(road_now, obstacle_now)
    paths, controls = [], []
    obstacle_states=[(z.copy(),x.copy(),angle.copy(),0.,travel.copy(),turn.copy())]
    obstacle_active=[np.ones(n,bool)]
    coast_steps = int(round(coast_seconds/.05))
    if coast_sequence is not None:
        coast_steps = coast_sequence.shape[1]
    last_drive = np.full(n,coast_steps-1)
    if control_sequence is not None:
        last_drive = np.max(np.where(abs(control_sequence[:,:,0])>1e-8,np.arange(control_sequence.shape[1]),-1),axis=1)
    for step in range(104):
        dt = .05
        future = (step+1)*dt
        drive = commands if step < coast_steps else np.zeros(n)
        steer = first_yaw if step < coast_steps else first_yaw*.65**((step-coast_steps)//4+1)
        if coast_sequence is not None:
            if step < coast_steps:
                drive,steer = coast_sequence[:,step,:].T
            else:
                steer = coast_sequence[:,-1,1]*.65**((step-coast_steps)//4+1)
        if control_sequence is not None:
            drive, steer = control_sequence[:, min(step, control_sequence.shape[1]-1), :].T
        controls.append(np.c_[drive, steer])
        old_speed, old_yaw = speed.copy(), yaw.copy()
        speed, yaw, zero_age = advance(speed, yaw, drive, steer, dt, zero_age)
        mid = angle+.25*(old_yaw+yaw)*dt
        z += .5*(speed+old_speed)*np.cos(mid)*dt
        x += .5*(speed+old_speed)*np.sin(mid)*dt
        angle += .5*(yaw+old_yaw)*dt
        travel += .5*(abs(speed)+abs(old_speed))*dt
        turn += .5*(abs(yaw)+abs(old_yaw))*dt
        _, road, known = road_clearance(obs.road, z, x, angle, uncertainty)
        road -= uncertainty*(.04*travel+.08*turn*1.16)
        obstacle_states.append((z.copy(),x.copy(),angle.copy(),future,travel.copy(),turn.copy()))
        obstacle_active.append(~stopped)
        valid &= stopped | ((road >= 0.) & known)
        minimum = np.minimum(minimum, np.where(stopped, 100., road))
        paths.append(np.c_[z.copy(), x.copy()])
        stopped |= (step > last_drive) & (abs(drive)<1e-8) & (speed < .04)
        if stopped.all():
            break
    _,obstacle=path_obstacle_clearance(obs.objects,obstacle_states,np.asarray(obstacle_active))
    valid &= obstacle>0.
    minimum=np.minimum(minimum,obstacle)
    if details:
        return valid & stopped, minimum, np.asarray(paths).transpose(1,0,2), np.asarray(controls).transpose(1,0,2)
    return valid & stopped, minimum
