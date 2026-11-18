"""Runtime source extracted from the accepted V6.26 freeze: experiments/v617_regression/curve_road617.py. """

import numpy as np

try:
    from . import trajectory_batch as batch
except ImportError:
    batch = None

from concept_road.v3_model import Observation, DEFAULT_PARAMETERS, BODY_HALF_WIDTH

from concept_feasibility.response61515 import advance, inverse_yaw

from concept_candidates.curve_progress import increment

from concept_candidates.curve_profile import evaluate_profile

from concept_candidates.belief61515 import CameraBelief, braking_branch

from concept_feasibility.geometry61514 import road_clearance, obstacle_clearance, path_obstacle_clearance

def preview_reason(row, road):
    """Explain whether this locally observed object extends the preview.

    Safety checks still consider every object. For a recently confirmed object
    whose stationary mode dominates, bound its likely position using its
    stationary covariance and compare that interval with the observed road.
    The 2-sigma term matches the existing per-time obstacle-risk scale.
    """
    z=float(row[0])
    if not .2 < z <= road[-1,0]-.4:return 'outside_observed_depth'
    stationary=len(row)>35 and row[23]>=3 and row[25]>.5
    if not stationary:return 'near_unconfirmed_or_moving' if z<9. else 'far_unconfirmed_or_moving'
    lateral=float(row[27]);variance=float(row[35])
    if not np.isfinite([lateral,variance]).all() or variance<0:return 'uncertain_static'
    margin=float(row[4])+BODY_HALF_WIDTH+2*np.sqrt(variance)
    left=float(np.interp(z,road[:,0],road[:,1])-np.interp(z,road[:,0],road[:,3]))
    col=4 if road.shape[1]>4 else 3
    right=float(np.interp(z,road[:,0],road[:,2])+np.interp(z,road[:,0],road[:,col]))
    return 'static_overlaps_road' if lateral+margin>=left and lateral-margin<=right else 'static_outside_road'

class RoadAwareCurvePlanner:
    def __init__(self, parameters=None):
        self.p = {**DEFAULT_PARAMETERS, **(parameters or {})}
        self.command = np.zeros(2)
        self.stop_age = 0.
        self.last = {}
        self.time = 0.
        self.reserved = None
        self.pending = None
        self.current_observation = None

    def no_observation(self):
        self.last = dict(reason='unobserved_road', candidates=0, feasible=0,
                         audit=dict(selected=None, table=[], paths=[]))
        if self.resume_contingency():
            return self.command.copy()
        self.command = np.array([0., self.command[1]*.65])
        self.last = dict(reason='unobserved_road', candidates=0, feasible=0,
                         recovery='reverse_unavailable_unknown_rear_and_unvalidated_response',
                         audit=dict(selected=None, table=[], paths=[]))
        return self.command.copy()

    def target_points(self, obs, look):
        road, p = obs.road, self.p
        lo, hi = [float(np.interp(look, road[:, 0], road[:, k])) for k in (1, 2)]
        guidance = getattr(obs, 'guidance', None)
        if guidance is not None and len(guidance) >= 3 and guidance[0, 0] <= look <= guidance[-1, 0]:
            a, b = [float(np.interp(look, guidance[:, 0], guidance[:, k])) for k in (1, 2)]
            lo, hi = max(lo, a), min(hi, b)
        # Body geometry and boundary uncertainty are distinct from obstacle gap.
        right_column = 4 if road.shape[1] > 4 else 3
        left_inset = BODY_HALF_WIDTH+p['uncertainty']*np.interp(look, road[:, 0], road[:, 3])
        right_inset = BODY_HALF_WIDTH+p['uncertainty']*np.interp(look, road[:, 0], road[:, right_column])
        lo, hi = lo+left_inset, hi-right_inset
        if lo > hi:
            return None
        targets = np.linspace(lo, hi, 9).tolist()+[float(np.clip(0., lo, hi))]
        openings = [(lo, hi)]
        for row in obs.objects:
            oz, ox, vz, vx, radius, noise = row[:6]
            if abs(oz-look) > 2.:
                continue
            at = ox+vx*oz/max(obs.speed, 1.)
            margin = radius+BODY_HALF_WIDTH+p['uncertainty']*noise
            newer = []
            for a, b in openings:
                if at-margin > a:
                    newer.append((a, min(b, at-margin)))
                if at+margin < b:
                    newer.append((max(a, at+margin), b))
            openings = [q for q in newer if q[1] > q[0]]
        extra = [(a+b)/2 for a, b in sorted(openings, key=lambda q: q[1]-q[0], reverse=True)[:5]]
        return np.sort(targets+extra+[targets[-1]]*(5-len(extra)))

    def step(self, obs):
        if not isinstance(obs, Observation):
            raise TypeError('Only local sensor Observation accepted')
        self.time += obs.dt
        self.current_observation = obs
        self.pending = None
        road, p = np.asarray(obs.road), self.p
        self.stop_age = self.stop_age+obs.dt if self.command[0] == 0 else 0.
        if len(road) < 3:
            return self.no_observation()
        if not np.all(np.isfinite(road)) or np.any(np.diff(road[:, 0]) <= 0):
            raise ValueError('Invalid local road')
        near_objects = [o[0] for o in obs.objects if preview_reason(o,road) in ('near_unconfirmed_or_moving','uncertain_static','static_overlaps_road')]
        look = max(3., 1.5+p['lookahead']*max(1., obs.speed))
        if near_objects:
            look = max(look, min(near_objects)+1.)
        look = float(min(look, road[-1, 0]-.4))
        targets = self.target_points(obs, look)
        if targets is None:
            return self.no_observation()
        speeds = np.array([0., .25, .55, .8, 1.])*p['pace']
        commands = np.tile(speeds, len(targets))
        targets = np.repeat(targets, len(speeds))
        count = len(commands)
        # End along the observed road direction. A near-zero lateral endpoint
        # may need an initial right bend followed by a left bend around a cone.
        slope = np.full(count,np.tan(np.interp(look,road[:,0],obs.heading)))
        aa, bb = (slope*look-2*targets)/look**3, (3*targets-slope*look)/look**2
        x, z, heading = np.zeros(count), np.zeros(count), np.zeros(count)
        speed, yaw = np.full(count, max(0., obs.speed)), np.full(count, obs.yaw)
        travel, turn = np.zeros(count), np.zeros(count)
        road_progress = np.zeros(count)
        heading_integral = np.zeros(count)
        reference_steps = np.zeros(count,int)
        camera_belief = CameraBelief(obs.objects, count)
        previous = self.command.copy()
        last_yaw = np.full(count, previous[1])
        zero_age = np.where(commands == 0, self.stop_age, 0.)
        horizon = float(np.clip((look+2.)/max(obs.speed, 1.2), 2., 5.))
        interval, substep = .2, .05
        steps = int(np.ceil(horizon/interval))
        horizon = steps*interval
        yaw_limit = p.get('yaw_limit', 1.55)
        if batch is not None:
            state, nominal_controls, geometric, road_lower, evidence, obstacle_lower, road_progress, reference_steps, heading_integral = batch.driven_path(
                obs,commands,slope,aa,bb,look,steps,previous,self.stop_age,p)
            first=nominal_controls[0,:,1].copy()
            paths=state[4::4,:,:2]
            z,x,heading,speed,yaw=state[-1,:,:5].T.copy()
        else:
            road_gap, road_lower, evidence = road_clearance(road, z, x, heading, p['uncertainty'])
            object_gap, obstacle_lower = obstacle_clearance(obs.objects, z, x, heading, 0., p['uncertainty'])
            geometric = np.minimum(road_gap, object_gap)
            paths, first, nominal_controls = [], None, []
            obstacle_states=[(z.copy(),x.copy(),heading.copy(),0.,travel.copy(),turn.copy())]
            # Replan short manoeuvres; the independent loss-of-observation branch
            # still checks all the way to a stop, using the actual covariance.
            for step in range(steps):
                pursuit = z+np.clip(1.+.7*speed, 1.2, 3.)
                q = np.minimum(pursuit, look)
                reference = aa*q**3+bb*q**2+np.maximum(pursuit-look, 0)*slope
                dz, dx = pursuit-z, reference-x
                lateral = dx*np.cos(heading)-dz*np.sin(heading)
                desired = p['steer_gain']*np.maximum(speed, .5)*2*lateral/(dz*dz+dx*dx+.01)-p['yaw_damping']*yaw
                command_dt = obs.dt if step == 0 else interval
                command_yaw = inverse_yaw(desired, commands,
                    lower=np.maximum(-yaw_limit,last_yaw-p['slew']*command_dt),
                    upper=np.minimum(yaw_limit,last_yaw+p['slew']*command_dt),
                    current_yaw=yaw,dt=command_dt)
                last_yaw = command_yaw
                if first is None:
                    first = command_yaw.copy()
                for sub in range(4):
                    nominal_controls.append(np.c_[commands,command_yaw])
                    old_speed, old_yaw = speed.copy(), yaw.copy()
                    old_z, old_x = z.copy(), x.copy()
                    speed, yaw, zero_age = advance(speed, yaw, commands, command_yaw, substep, zero_age)
                    mid_heading = heading+.25*(old_yaw+yaw)*substep
                    x += (old_speed+speed)*.5*np.sin(mid_heading)*substep
                    z += (old_speed+speed)*.5*np.cos(mid_heading)*substep
                    heading += (old_yaw+yaw)*.5*substep
                    # Reward travel along locally observed road direction, not the
                    # vehicle-forward axis. No map, GT station or new weight.
                    local_direction = np.interp((old_z+z)*.5, road[:,0], obs.heading)
                    delta=(z-old_z)*np.cos(local_direction)+(x-old_x)*np.sin(local_direction)
                    delta,used=increment(obs,old_z,old_x,z,x,delta)
                    road_progress += delta;reference_steps += used
                    heading_integral += (old_speed+speed)*.5*(mid_heading-local_direction)**2*substep
                    travel += (abs(old_speed)+abs(speed))*.5*substep
                    turn += (abs(old_yaw)+abs(yaw))*.5*substep
                    road_gap, road_bound, known = road_clearance(road, z, x, heading, p['uncertainty'])
                    # Today's road evidence must survive its future transport into
                    # the blind near field. Same odometry budget as surface memory;
                    # this is extra FUTURE travel/turn, not a second current erosion.
                    future_transport = .04*travel+.08*turn*1.16
                    road_bound -= p['uncertainty']*future_transport
                    future = step*interval+(sub+1)*substep
                    obstacle_states.append((z.copy(),x.copy(),heading.copy(),future,travel.copy(),turn.copy()))
                    geometric = np.minimum(geometric, road_gap)
                    road_lower = np.minimum(road_lower, road_bound)
                    evidence &= known
                paths.append(np.c_[z.copy(), x.copy()])
            object_gap,obstacle_lower=path_obstacle_clearance(obs.objects,obstacle_states)
            geometric=np.minimum(geometric,object_gap)
        dynamics = np.isfinite(speed) & np.isfinite(yaw) & (abs(first) <= yaw_limit+1e-9)
        # Single mask consumed by whitebox selection AND the learned interface.
        conditional = (road_lower >= 0.) & (obstacle_lower > 0.) & evidence & dynamics
        brake_ok, brake_lower, brake_paths, brake_controls = braking_branch(obs, commands, first, self.stop_age, p['uncertainty'], details=True)
        # Reuse the actual nominal steering sequence through the two-second
        # manoeuvre, then brake. Holding the first steering command is a
        # different trajectory and blocked otherwise executable turning paths.
        escape_ok, escape_lower, escape_paths, escape_controls = braking_branch(obs, commands, first, self.stop_age, p['uncertainty'], coast_seconds=2., details=True,
            coast_sequence=np.asarray(nominal_controls).transpose(1,0,2)[:,:40])
        feasible = conditional & (brake_ok | escape_ok)
        nominal_feasible = feasible.copy()
        profile = evaluate_profile(obs,escape_controls,horizon,self.stop_age,p['uncertainty'])
        retimed = (~feasible)&profile['valid']&escape_ok&(commands>0)
        # Same initial action and geometry slot; replace only an unexecutable
        # constant-drive schedule with its explicitly checked drive/stop schedule.
        path_array = np.asarray(paths).transpose(1,0,2)
        path_array[retimed] = profile['paths'][retimed]
        for original,key in [(road_lower,'road_lower'),(obstacle_lower,'obstacle_lower'),
            (geometric,'geometric'),(evidence,'evidence'),(dynamics,'dynamics'),
            (z,'z'),(x,'x'),(heading,'heading'),(speed,'speed'),(road_progress,'road_progress'),(heading_integral,'heading_integral'),(reference_steps,'reference_steps')]:
            original[retimed] = profile[key][retimed]
        conditional[retimed] = True
        feasible[retimed] = True
        self.pending = dict(brake_ok=brake_ok, brake_lower=brake_lower, brake_controls=brake_controls, escape_ok=escape_ok, escape_lower=escape_lower, escape_controls=escape_controls, escape_duration=horizon,retimed=retimed,profile_controls=profile['controls'],profile_lower=np.minimum(profile['road_lower'],profile['obstacle_lower']))
        robust = np.minimum(road_lower, obstacle_lower)
        sigma = (geometric-robust)/p['uncertainty']
        progress = road_progress/horizon
        road_heading = np.interp(z, road[:, 0], obs.heading)
        terminal_heading_cost = .8*speed*(heading-road_heading)**2
        heading_cost = .8*heading_integral/horizon
        smooth_cost = p['smoothness']*((first-previous[1])**2+.15*first**2)
        # Bounded secondary preference: tiny legal margins cannot create infinite stop cost.
        # It only ranks; a trained selector may choose ANY candidate in the mask.
        risk_cost = .35*np.clip((.5-robust)/.5, 0., 1.)**2
        scores = progress-heading_cost-smooth_cost-risk_cost
        pick = int(np.argmax(np.where(feasible, scores, -np.inf))) if feasible.any() else None
        table = np.c_[commands, first, targets, geometric, sigma, robust, feasible, scores,
                      progress, heading_cost, smooth_cost, risk_cost]
        if pick is None:
            self.command = np.array([0., previous[1]*.65])
        else:
            self.command = np.array([commands[pick], first[pick]])
        self.last = dict(reason='feasible' if pick is not None else 'no_feasible_trajectory', candidates=count,
                         feasible=int(feasible.sum()), feasible_forward=int(np.sum(feasible & (commands > 0))),
                         margin=None if pick is None else float(robust[pick]),
                         target=[look, 0. if pick is None else float(targets[pick])],
                         path=[] if pick is None else path_array[pick].round(4).tolist(),
                         recovery=('not_needed' if pick is not None and commands[pick] > 0 else
                                   'selected_deceleration' if np.any(feasible & (commands > 0)) else
                                   'no_forward_candidate_reverse_unavailable_unknown_rear_and_unvalidated_response'),
                         audit=dict(curve_progress=dict(reference=None if getattr(obs,'reference',None) is None else {key:(value.tolist() if isinstance(value,np.ndarray) else value) for key,value in obs.reference.items() if not key.startswith('_')},used_substeps=reference_steps.tolist(),total_substeps=steps*4,rule='Same-reference projection arc advance only within supported non-singular span; other substeps retain original projected travel'),heading_penalty_rule='0.8/H times speed-weighted squared direction error integrated over original H at .05s; stopped wait adds zero; safety tail beyond H excluded from score', terminal_heading_penalty=terminal_heading_cost.round(6).tolist(), schedule=['drive_2s_then_stop_and_wait' if flag else 'constant_drive' for flag in retimed],nominal_feasible=nominal_feasible.tolist(),retimed_profile_valid=profile['valid'].tolist(),profile_checked_seconds=profile['checked_seconds'],profile_rule='Only infeasible constant-drive schedules may use the existing two-second drive then complete stop; checked through original horizon and complete stop, including wait; same75 slots',progress_rule='Integral of each .05s displacement projected onto observed local road heading; m/s proxy, not map progress',path_rule='Cubic from current heading to target and observed terminal road heading', response_version='V6.15.15', inverse_rule='nearest reachable CAN output within yaw and slew interval', selected=pick, control_interval=interval, integration_substep=substep,
                                    constraint_interval=substep, horizon=horizon, threshold=0.,
                                    uncertainty_scale=p['uncertainty'], previous_command=previous.tolist(),
                                    columns=['speed', 'yaw', 'target_x', 'geometric_clearance', 'sigma', 'robust_clearance',
                                             'feasible', 'score', 'progress_score', 'heading_penalty', 'smooth_penalty', 'margin_penalty'],
                                    table=table.round(6).tolist(), paths=path_array.round(4).tolist(),
                                    conditional_path_feasible=conditional.tolist(),
                                    no_observation_brake_feasible=brake_ok.tolist(),
                                    no_observation_pass_then_stop_feasible=escape_ok.tolist(),
                                    pass_then_stop_lower=escape_lower.round(6).tolist(),
                                    brake_lower=brake_lower.round(6).tolist(),
                                    expected_camera_updates=camera_belief.updates.tolist(),
                                    road_lower=road_lower.round(6).tolist(), obstacle_lower=obstacle_lower.round(6).tolist(),
                                    evidence_ok=evidence.tolist(), dynamics_ok=dynamics.tolist(),
                                    rejected=dict(road=int((road_lower < 0).sum()), obstacle=int((obstacle_lower <= 0).sum()),
                                                  unobserved=int((~evidence).sum()), loss_of_camera_contingency=int((~(brake_ok|escape_ok)).sum()), dynamics=int((~dynamics).sum())),
                                    unique_commands=int(len(np.unique(table[:, :2].round(6), axis=0))),
                                    rule='Rank feasible only; empty feasible set has no selected candidate',
                                    obstacle_risk_limit=0.022750131948179195,
                                    obstacle_risk_rule='Sum of mode-weighted correlated-Gaussian rectangle conditional-supremum upper bounds at each predicted time; not a lap failure probability',
                                    geometric_limit='Full body at 0.05 s intervals; continuous-time between-step collision is not formally certified'))
        if pick is None:
            self.resume_contingency()
        return self.command.copy()

    def commit(self, index):
        if index is None or self.pending is None:
            return
        p = self.pending
        immediate = bool(p['brake_ok'][index])
        mode = 'immediate_stop' if immediate else 'pass_then_stop'
        controls = p['brake_controls' if immediate else 'escape_controls'][index].copy()
        lower = float(p['brake_lower' if immediate else 'escape_lower'][index])
        if p['retimed'][index]:
            mode = 'drive_then_stop_profile'
            controls = p['profile_controls'][index].copy()
            lower = float(p['profile_lower'][index])
        powered = np.flatnonzero(abs(controls[:,0])>1e-8)
        brake_delay = (int(powered[-1])+1)*.05 if len(powered) else 0.
        # Called after the whitebox/learned selector's actual final choice.
        self.reserved = dict(issued_at=self.time, index=int(index), mode=mode, controls=controls,
                             brake_at=self.time+brake_delay, stop_at=self.time+len(controls)*.05)
        self.last['contingency'] = dict(issued_at=self.time, selected=int(index), mode=mode,
                                       brake_at=self.reserved['brake_at'], stop_at=self.reserved['stop_at'],
                                       plan_end_at=self.reserved['stop_at'], stop_at_meaning='end of checked controls, including waiting; not measured first standstill',
                                       lower_m=lower, controls_005s=controls.tolist())

    def resume_contingency(self):
        if self.reserved is None or self.current_observation is None:
            return False
        reserve, obs = self.reserved, self.current_observation
        offset = int(round((self.time-reserve['issued_at'])/.05))
        if offset >= len(reserve['controls']) or len(obs.road)<3:
            self.reserved = None
            return False
        tail = reserve['controls'][offset:]
        if reserve['mode']=='drive_then_stop_profile':
            checked=evaluate_profile(obs,tail[None,:,:],len(tail)*.05,self.stop_age,self.p['uncertainty'])
            valid=checked['valid'];lower=np.minimum(checked['road_lower'],checked['obstacle_lower'])
            paths=checked['paths'];controls=checked['controls']
        else:
            valid, lower, paths, controls = braking_branch(obs, tail[:1,0], tail[:1,1], self.stop_age,
                                                       self.p['uncertainty'], details=True,
                                                       control_sequence=tail[None,:,:], coast_seconds=0.)
        if not valid[0]:
            self.last['contingency_rejected'] = dict(reason='remaining_plan_failed_current_evidence', lower_m=float(lower[0]))
            self.reserved = None
            return False
        self.command = tail[0].copy()
        if not self.last or 'audit' not in self.last:
            self.last = dict(audit=dict(selected=None,table=[],paths=[]))
        self.last.update(reason='executing_prior_contingency', contingency_execution=True,
                         recovery='previously_checked_pass_then_stop', path=paths[0].round(4).tolist(),
                         contingency=dict(issued_at=reserve['issued_at'], selected=reserve['index'], mode=reserve['mode'],
                                          brake_at=reserve['brake_at'], stop_at=reserve['stop_at'],
                                          consumed_steps=offset, remaining_steps=len(tail), rechecked_lower_m=float(lower[0])))
        return True
