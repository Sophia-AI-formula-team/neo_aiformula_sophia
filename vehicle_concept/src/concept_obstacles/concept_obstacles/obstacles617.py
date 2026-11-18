"""Runtime source extracted from the accepted V6.26 freeze: experiments/v617_regression/obstacles617.py. """

from dataclasses import dataclass

import cv2

import numpy as np

from concept_road.vision import FOCAL, CX, CY, HEIGHT

from concept_obstacles.silhouette615 import complete_box

from concept_obstacles.association61519 import assign

from concept_obstacles.clipped617 import censored_support

def overlap(a, b):
    intersection = np.maximum(0., np.minimum(a[2:], b[2:])-np.maximum(a[:2], b[:2])).prod()
    areas = [np.maximum(0., q[2:]-q[:2]).prod() for q in (a, b)]
    return float(intersection/max(min(areas), 1.)), float(intersection/max(sum(areas)-intersection, 1.))

def instances(prediction, shape=(360, 640), image=None):
    """A nested narrow fragment cannot become a second cone on its own.

    Group only near-contained, aligned boxes, not merely intersecting boxes.
    The outer box supplies geometry; all members can supply class confidence.
    """
    height, width = shape[:2]
    colored = None
    if image is not None:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        colored = (((hsv[:, :, 0] < 18) | (hsv[:, :, 0] > 170)) &
                   (hsv[:, :, 1] > 90) & (hsv[:, :, 2] > 45))
    def support(box):
        if colored is None:
            return None
        x0, y0, x1, y1 = np.rint(box).astype(int)
        result = np.zeros_like(colored)
        result[max(0, y0):min(height, y1+1), max(0, x0):min(width, x1+1)] = colored[max(0, y0):min(height, y1+1), max(0, x0):min(width, x1+1)]
        return result
    proposals = []
    for box, score, label in zip(prediction['boxes'], prediction['scores'], prediction['labels']):
        box = np.asarray(box, float)
        if label == 91 and score >= .15 and np.isfinite(box).all() and np.all(box[2:] > box[:2]):
            foot_height = HEIGHT
            foot_source = 'detector_box'
            if colored is not None:
                completed = complete_box(box, colored.astype(np.uint8), image)
                if completed is None:
                    continue
                box, shape_evidence = completed
                foot_source = shape_evidence['foot_source']
                # Camera mount .655m above body root; settled root is near -.05m.
                # Pavement is at -.049m, but these cone bases are placed at y=0.
                # Do NOT change the road projector's .655m height.
                foot_height = .605 if foot_source == 'observed_black_base' else .605-(.06-.05*.675)
            observed = support(box)
            # In these cone experiments, stand legs repeatedly scored as cones.
            # Requiring visible cone colour applies only to NEW measurements;
            # it never deletes an existing blind occupancy track.
            if observed is not None and observed.sum() < 3:
                continue
            if observed is not None:
                bottom = np.nonzero(observed)[0].max()
                # A high-scoring tiny cone inside a large background proposal
                # cannot lend that proposal an unsupported, much nearer foot.
                if box[3]-bottom > max(3., .15*(box[3]-box[1])):
                    continue
            proposals.append((box, float(score), foot_height, foot_source))
    groups = []
    for box, score, foot_height, foot_source in sorted(proposals, key=lambda b: -np.prod(b[0][2:]-b[0][:2])):
        match = None
        for group in groups:
            outer = group['box']
            axis = abs((box[0]+box[2]-outer[0]-outer[2])*.5)
            # A single cone's internal stripe stays close to its vertical axis.
            ioa, iou = overlap(outer, box)
            aligned = axis <= .25*(outer[2]-outer[0])
            vertical = abs((box[1]+box[3]-outer[1]-outer[3])*.5) <= .30*(outer[3]-outer[1])
            a, b = support(outer), support(box)
            same_pixels = a is not None and (a & b).sum()/max(1, min(a.sum(), b.sum())) >= .75
            if same_pixels or (aligned and (ioa >= .85 or (iou >= .4 and vertical))):
                match = group
                break
        if match is None:
            groups.append(dict(box=box, score=score, members=1, foot_height=foot_height, foot_source=foot_source))
        else:
            match['score'] = max(score, match['score'])
            match['members'] += 1
    result = []
    for group in groups:
        box = group['box']
        if box[3] <= CY+5:
            continue
        foot_height = group['foot_height']
        z = foot_height*FOCAL/(box[3]-CY)
        x = (.5*(box[0]+box[2])-CX)*z/FOCAL
        if not 0 < z < 12:
            continue
        clipped = [box[0] <= 2, box[1] <= 2, box[2] >= width-2, box[3] >= height-2]
        # Three pixels of foot jitter, plus a small ground-model allowance.
        dzdv = -z*z/(foot_height*FOCAL)
        jacobian = np.array([[0., dzdv], [z/FOCAL, x/z*dzdv]])
        covariance = jacobian@np.diag([3.**2, 3.**2])@jacobian.T+np.eye(2)*.04**2
        if image is not None:
            # The visible foot is the front edge of a 0.48m square base, not its
            # centre. Heading is not supplied: .24m correction plus .05m error
            # covers the .24..sqrt(2)*.24 orientation range approximately.
            factor = 1.+.24/max(np.hypot(z, x), .1)
            z, x = z*factor, x*factor
            covariance += np.eye(2)*.05**2
        result.append(dict(**group, point=np.array([z, x]), R=covariance, clipped=clipped))
    return result

@dataclass
class Track:
    number: int
    state: np.ndarray
    covariance: np.ndarray
    physical_size: np.ndarray
    age: float = 0.
    hits: int = 1
    source: str = 'new'
    score: float = 1.
    negative_views: int = 0
    motion_votes: int = 0
    dynamic_confirmed: bool = False
    process_noise_intensity: float = .6
    static_state: np.ndarray = None
    static_covariance: np.ndarray = None
    static_probability: float = .5
    motion_axis: np.ndarray = None
    travel_since_fix: float = 0.
    turn_since_fix: float = 0.
    maximum_radius: float = 0.
    odometry_budget: float = 0.
    # Presence of a clipped silhouette is not a new position measurement.
    # Keep its continuity separate from the age of the last complete foot.
    presence_age: float = 0.

class Occupancy617:
    def __init__(self):
        self.tracks = []
        self.next_id = 0
        self.last = {}
        self.recall = .8
        self.expected_new = 1.

    @staticmethod
    def image_box(track):
        z, x = track.state[:2]
        if z <= .1:
            return None
        u, v = CX+x*FOCAL/z, CY+HEIGHT*FOCAL/z
        w, h = track.physical_size*FOCAL/z
        return np.array([u-w/2, v-h, u+w/2, v])

    def update(self, detections, speed, yaw, dt, minimum_body_z=None, image=None):
        if not np.isfinite([speed, yaw, dt]).all() or not 0 < dt <= 1:
            raise ValueError('Invalid causal odometry')
        angle = yaw*dt
        c, s = np.cos(angle), np.sin(angle)
        rotation = np.array([[c, s], [-s, c]])
        motion = np.array([speed*dt*np.sinc(angle/np.pi), speed*dt*(1-c)/angle if abs(angle) > 1e-5 else 0.])
        f = np.eye(4)
        f[:2, 2:] = np.eye(2)*dt
        rotate = np.zeros((4, 4))
        rotate[:2, :2] = rotate[2:, 2:] = rotation
        # Continuous acceleration process; floor accounts for wheel/gyro drift.
        q = np.block([[np.eye(2)*dt**3/3, np.eye(2)*dt**2/2],
                      [np.eye(2)*dt**2/2, np.eye(2)*dt]])*.12**2
        for track in self.tracks:
            track.state = f@track.state
            track.state[:2] -= motion
            track.state = rotate@track.state
            axis = track.motion_axis
            acceleration = np.eye(2)*.6**2 if axis is None else (.12**2*np.eye(2)+(.6**2-.12**2)*np.outer(axis,axis))
            process = np.block([[acceleration*dt**3/3,acceleration*dt**2/2],
                                [acceleration*dt**2/2,acceleration*dt]])
            track.covariance = rotate@(f@track.covariance@f.T+process)@rotate.T
            if axis is not None:track.motion_axis=rotation@axis
            track.covariance[:2, :2] += np.eye(2)*(.01*abs(speed)*dt+.02*abs(angle))**2
            track.static_state = rotation@(track.static_state-motion)
            track.static_covariance = rotation@track.static_covariance@rotation.T
            track.travel_since_fix += abs(speed)*dt
            track.turn_since_fix += abs(angle)
            track.maximum_radius = max(track.maximum_radius,float(np.linalg.norm(track.static_state)))
            budget = .01*track.travel_since_fix+.02*track.turn_since_fix*track.maximum_radius
            # The same wheel/gyro scale bias persists through the blind interval;
            # sum its displacement before squaring, not independent step squares.
            track.static_covariance += np.eye(2)*(.02**2*dt+max(0.,budget**2-track.odometry_budget**2))
            track.odometry_budget = budget
            # Missing observations cannot increase stationary evidence.
            track.static_probability *= np.exp(-.005*dt)
            track.age += dt
            track.presence_age += dt
            track.source = 'remembered'
        associations = []
        matched, likelihood_audit, likelihood_prior = assign(self.tracks,detections,self.recall,self.expected_new)
        used_tracks, used_detections = set(), set()
        chosen = [(0,info['info']['cost'],i,j,info['maha'],0.) for i,j,info in matched]
        match_info = {(i,j):info for i,j,info in matched}
        # Incomplete boxes do not enter complete-foot Gaussian matching. Only
        # preserve presence when a substantial, credible predicted box overlaps.
        assigned_tracks={i for i,j,info in matched};assigned_detections={j for i,j,info in matched}
        presence=[]
        presence_info={}
        for i,track in enumerate(self.tracks):
            if i in assigned_tracks:continue
            for j,det in enumerate(detections):
                if j in assigned_detections or not any(det['clipped']):continue
                support=0.
                mode_support=[]
                for weight,point in [(track.static_probability,track.static_state),(1-track.static_probability,track.state[:2])]:
                    z,x=point
                    if z<=.1:
                        mode_support.append(False)
                        continue
                    u,v=CX+x*FOCAL/z,CY+HEIGHT*FOCAL/z
                    w,h=track.physical_size*FOCAL/z
                    ioa,iou=overlap(np.array([u-w/2,v-h,u+w/2,v]),det['box'])
                    compatible=ioa>=.5 and iou>=.12
                    mode_support.append(compatible)
                    if compatible:support+=weight*ioa
                if support>=.5:
                    presence.append((-support,i,j))
                    presence_info[i,j]=mode_support
        # Settle existing matches before considering uncertainty-expanded
        # matches. A new fallback must never steal an already explained box.
        for cost,i,j in sorted(presence):
            if i in assigned_tracks or j in assigned_detections:continue
            chosen.append((1,cost,i,j,0.,0.));assigned_tracks.add(i);assigned_detections.add(j)
        # A projected mean just outside the frame can still explain a clipped
        # fragment within the track's estimated uncertainty. Only a unique,
        # recently confirmed, still-unassigned track may claim it.
        censored_audit=[]
        censored=[]
        for j,det in enumerate(detections):
            if j in assigned_detections:continue
            possible=[]
            for i,track in enumerate(self.tracks):
                if i in assigned_tracks:continue
                match=censored_support(track,det,dt)
                if match is not None:possible.append((i,match))
            if len(possible)==1:
                i,match=possible[0]
                censored.append((-match['score'],i,j,match))
        for cost,i,j,match in sorted(censored,key=lambda item:item[:3]):
            if i in assigned_tracks or j in assigned_detections:continue
            chosen.append((1,cost,i,j,0.,0.));assigned_tracks.add(i);assigned_detections.add(j)
            presence_info[i,j]=match['mode_support'].tolist()
            censored_audit.append(dict(id=self.tracks[i].number,detection=j,score=match['score'],mode_scores=match['mode_scores']))
        for tier, cost, i, j, maha, axis_stat in chosen:
            used_tracks.add(i)
            used_detections.add(j)
            track, detection = self.tracks[i], detections[j]
            track.presence_age = 0.
            associations.append(dict(id=track.number,detection=j,mahalanobis2=maha,
                                     image_axis_stat=axis_stat if np.isfinite(axis_stat) else None,
                                     presence_only_gate=bool(tier),likelihood=None if tier else match_info[i,j]['info'],
                                     presence_mode_support=presence_info.get((i,j))))
            track.score = detection['score']
            track.negative_views = 0
            if detection['clipped'][0] or detection['clipped'][2]:
                # A side-cropped cone may still have a visible complete foot.
                # Update depth only; its cropped horizontal centre is biased.
                updated=[]
                supported=presence_info[i,j]
                for mode,(state,covariance) in enumerate([(track.static_state,track.static_covariance),(track.state,track.covariance)]):
                    variance=covariance[0,0]+detection['R'][0,0]
                    delta=detection['point'][0]-state[0]
                    if supported[mode] and not detection['clipped'][3] and delta*delta/variance<=6.63:
                        gain=covariance[:,0]/variance
                        state+=gain*delta
                        covariance-=np.outer(gain,covariance[0].copy())
                        updated.append(mode)
                if updated:
                    track.source='side_clipped_depth_only'
                    associations[-1]['depth_updated_modes']=updated
                    continue
            if any(detection['clipped']) or tier:
                track.source = 'clipped_or_outlier_presence_only'
                continue
            innovation = detection['point']-track.state[:2]
            # Use exactly the measurement covariance used for matching; a
            # different mode's residual cannot silently inflate this update.
            r = match_info[i,j]['R']
            static_innovation = detection['point']-track.static_state
            ss, sc = track.static_covariance+r, track.covariance[:2,:2]+r
            ls = -.5*(np.linalg.slogdet(ss)[1]+static_innovation@np.linalg.solve(ss,static_innovation))
            lc = -.5*(np.linalg.slogdet(sc)[1]+innovation@np.linalg.solve(sc,innovation))
            prior = np.clip(track.static_probability,1e-8,1-1e-8)
            mode_pd=match_info[i,j]['pd']
            log_odds = np.log(prior/(1-prior))+ls-lc+np.log(max(mode_pd[0],1e-300)/max(mode_pd[1],1e-300))
            track.static_probability = float(1/(1+np.exp(-np.clip(log_odds,-18,18))))
            ks = track.static_covariance@np.linalg.inv(ss)
            track.static_state += ks@static_innovation
            residual_s = np.eye(2)-ks
            track.static_covariance = residual_s@track.static_covariance@residual_s.T+ks@r@ks.T
            track.travel_since_fix=track.turn_since_fix=track.odometry_budget=0.
            track.maximum_radius=float(np.linalg.norm(track.static_state))
            gain = track.covariance[:, :2]@np.linalg.inv(track.covariance[:2, :2]+r)
            track.state += gain@innovation
            h = np.zeros((2, 4)); h[:, :2] = np.eye(2)
            residual = np.eye(4)-gain@h
            track.covariance = residual@track.covariance@residual.T+gain@r@gain.T
            track.physical_size = (detection['box'][2:]-detection['box'][:2])*detection['point'][0]/FOCAL
            track.age = 0.
            track.hits += 1
            track.source = 'complete_box_update'
            # Three full observations must reject zero velocity at the 2D
            # 99% chi-square level. Blind/clipped frames cannot add evidence.
            motion_stat = float(track.state[2:]@np.linalg.solve(track.covariance[2:,2:],track.state[2:]))
            track.motion_votes = track.motion_votes+1 if motion_stat > 9.21 else 0
            if track.motion_votes >= 3:
                track.dynamic_confirmed = True
            # Calibrated against separate V6.14.2 moving trials; intensity
            # units are m/s^(3/2), not a deterministic acceleration bound.
            track.process_noise_intensity = .6
            # Only actual observations supporting motion can establish its axis.
            # Keep the axis while blind; a reversal has the same covariance axis.
            if track.static_probability < .01 and np.linalg.norm(track.state[2:]) > .2:
                track.motion_axis = track.state[2:]/np.linalg.norm(track.state[2:])
        for j, detection in enumerate(detections):
            if j in used_detections or detection['score'] < .5:
                continue
            # A group already explained by an updated track cannot spawn a
            # second identity. Require image containment, not world proximity.
            if any(self.image_box(t) is not None and overlap(self.image_box(t), detection['box'])[0] >= .85
                   for index,t in enumerate(self.tracks) if index in used_tracks):
                continue
            point = detection['point'].copy()
            covariance = np.zeros((4, 4))
            covariance[:2, :2] = detection['R']
            covariance[2:, 2:] = np.eye(2)*.45**2
            if any(detection['clipped']):
                # First sight is an interval, never an invented accurate point.
                covariance[:2, :2] += np.eye(2)*.5**2
            size = (detection['box'][2:]-detection['box'][:2])*point[0]/FOCAL
            self.tracks.append(Track(self.next_id, np.r_[point, 0., 0.], covariance, size, score=detection['score'], static_state=point.copy(), static_covariance=covariance[:2,:2].copy(),maximum_radius=float(np.linalg.norm(point))))
            self.next_id += 1
        retained, objects, audit, released, inactive = [], [], [], [], []
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) if image is not None else None
        for track in self.tracks:
            z, x, vz, vx = track.state
            # A spatial release only, never a time-to-live removal while blind.
            # Use the same CV prediction as the collision check, including
            # cross covariance and acceleration noise. Never release by age.
            axis=track.motion_axis
            acceleration=np.eye(2)*.6**2 if axis is None else (.12**2*np.eye(2)+(.6**2-.12**2)*np.outer(axis,axis))
            future = np.linspace(0., 5., 101)
            variance = (track.covariance[0,0]+2*future*track.covariance[0,2]
                        +future**2*track.covariance[2,2]+acceleration[0,0]*future**3/3)
            front = float(np.max(z+vz*future+.34+.07+2*np.sqrt(np.maximum(0.,variance))))
            if hsv is not None and track.source == 'remembered':
                def contradicted(position,covariance):
                    zz,xx=position
                    if zz<=.1:return False
                    u,v=CX+xx*FOCAL/zz,CY+HEIGHT*FOCAL/zz
                    w,h=track.physical_size*FOCAL/zz
                    expected=np.array([u-w/2,v-h,u+w/2,v])
                    sigma_z,sigma_x=np.sqrt(np.diag(covariance)[:2])
                    if zz-2*sigma_z<=.95:return False
                    pad_x=2*FOCAL*(sigma_x/zz+abs(xx)*sigma_z/zz**2)
                    pad_y=2*HEIGHT*FOCAL*sigma_z/zz**2
                    a=expected-np.array([pad_x,pad_y,-pad_x,-pad_y])
                    if not (2<a[0]<a[2]<image.shape[1]-2 and 2<a[1]<a[3]<image.shape[0]-2):return False
                    x0,y0,x1,y1=np.rint(a).astype(int);roi=hsv[y0:y1+1,x0:x1+1]
                    red=((roi[:,:,0]<18)|(roi[:,:,0]>170))&(roi[:,:,1]>90)&(roi[:,:,2]>45)
                    grey=(roi[:,:,1]<60)&(roi[:,:,2]>40)
                    return bool(red.sum()<3 and grey.mean()>.85)
                both=(contradicted(track.state[:2],track.covariance[:2,:2]) and
                      contradicted(track.static_state,track.static_covariance))
                track.negative_views=track.negative_views+1 if both else 0
            if track.negative_views >= 3:
                released.append(dict(id=track.number, reason='three_visible_pavement_contradictions'))
                continue
            static_front = float(track.static_state[0]+.34+.07+2*np.sqrt(track.static_covariance[0,0]+.02**2*5))
            if minimum_body_z is not None and max(front,static_front) < min(-.88, minimum_body_z):
                # Freeze the coordinate frame for this regional test: no future
                # ego-motion error belongs in an object's own reachable set.
                # Excluding this frame does NOT justify permanent deletion.
                retained.append(track)
                inactive.append(dict(id=track.number,reason='outside_current_planning_region',
                                     age=track.age,cv_front=front,static_front=static_front))
                continue
            retained.append(track)
            prob = track.static_probability
            ms = np.r_[track.static_state,0.,0.]
            ps = np.zeros((4,4));ps[:2,:2]=track.static_covariance
            mean = prob*ms+(1-prob)*track.state
            ds,dc = ms-mean,track.state-mean
            pm = prob*(ps+np.outer(ds,ds))+(1-prob)*(track.covariance+np.outer(dc,dc))
            noise = .04+2*np.sqrt(np.linalg.eigvalsh(pm[:2,:2]).max())
            # 0:24 compatible summary; 25:68 explicit mode states/probability.
            objects.append([*mean,.34,noise,*pm.ravel(),track.age,track.hits,
                            np.sqrt(1-prob)*.6,prob,*ms,*ps.ravel(),*track.state,
                            *track.covariance.ravel(),.6,.02,.005,*acceleration.ravel(),
                            track.travel_since_fix,track.turn_since_fix,track.maximum_radius,track.odometry_budget])
            audit.append(dict(id=track.number, source=track.source, position=mean[:2].tolist(),
                              velocity=mean[2:].tolist(), error_m=float(noise),
                              last_observation_age_s=track.age, last_presence_age_s=track.presence_age, hits=track.hits, score=track.score,
                              dynamic_confirmed=track.dynamic_confirmed, motion_votes=track.motion_votes,
                              process_noise_intensity=track.process_noise_intensity,
                              static_probability=prob,static_position=track.static_state.tolist(),
                              cv_position=track.state[:2].tolist(),cv_velocity=track.state[2:].tolist(),
                              acceleration_covariance=acceleration.tolist(),odometry_budget_m=track.odometry_budget))
        self.tracks = retained
        self.last = dict(censored_association_candidates=censored_audit,association_candidates=likelihood_audit,association_prior=likelihood_prior,tracks=audit, released=released, inactive=inactive, associations=associations, groups=[dict(box=d['box'].tolist(), score=d['score'], members=d['members'], foot_source=d['foot_source'], foot_height_m=d['foot_height'], clipped=[bool(v) for v in d['clipped']]) for d in detections],
                         rule='Two-mode likelihood and explicit unmatched/new option; incomplete boxes presence-only; no TTL deletion',
                         limitation='Experimental pixel and motion error allowances, not a certified probability bound')
        return np.asarray(objects).reshape(-1, 77)
