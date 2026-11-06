"""Train a small feedback policy by racing simulated laps. No vehicle I/O."""
from dataclasses import dataclass

import numpy as np
from scipy.special import expit

from dynamics import DT, step
from course import PREVIEW


SPEED_FEATURES = (["bias", "speed", "abs_yaw_rate"] + [f"abs_middle_{d:g}m" for d in PREVIEW]
                  + [f"narrowing_{d:g}m" for d in PREVIEW])
STEER_FEATURES = ["bias", "yaw_rate"] + [f"middle_{d:g}m" for d in PREVIEW] + [
    f"narrowing_{d:g}m" for d in PREVIEW]
COUNT = len(SPEED_FEATURES) + len(STEER_FEATURES)
MAX_COMMON_RPM = 300.
MAX_DIFFERENTIAL_RPM = 120.  # each side can deviate this far from the average
WHEEL_LIMIT = 405.
HALF_LENGTH, HALF_WIDTH, BUFFER = 0.50, 0.36, 0.12  # conservative prototype footprint, metres


@dataclass(frozen=True)
class Conditions:
    speed_gain: float = 1.
    yaw_gain: float = 1.
    yaw_bias: float = 0.       # extra yaw per 100 RPM of mean command
    extra_delay: int = 0
    start_offset: float = 0.
    start_heading: float = 0.


class OnlineResponse:
    """Estimate speed gain and yaw drift solely from commands and resulting motion."""
    def __init__(self, count, base):
        self.base = base
        self.speed_delta = np.zeros(count)
        self.yaw_delta = np.zeros(count)

    def compensate(self, right, left):
        requested = (right+left)/2
        ratio = (self.base[0]+self.speed_delta)/self.base[0]
        common = np.clip(requested/ratio, 0, MAX_COMMON_RPM)
        difference = (right-left)/100
        nominal_turn = (np.where(difference >= 0, self.base[4], self.base[5])*difference
                        + self.base[6]*requested/100)
        # At the common RPM limit a weaker plant cannot recover nominal speed.
        # Scale yaw by achievable speed to retain the requested turn radius.
        achievable = np.divide(common*ratio, requested, out=np.ones_like(common), where=requested>1.)
        wanted = nominal_turn*achievable - (self.base[6]+self.yaw_delta)*common/100
        per_wheel = np.clip(50*wanted/np.where(wanted>=0, self.base[4], self.base[5]),
                            -MAX_DIFFERENTIAL_RPM, MAX_DIFFERENTIAL_RPM)
        return np.clip(common+per_wheel, 0, WHEEL_LIMIT), np.clip(common-per_wheel, 0, WHEEL_LIMIT)

    def observe(self, v, w, common, difference, measured_v, measured_w):
        pv, pw = step(v, w, common, difference, self.base)
        tau = np.where(self.base[0]*common >= v, self.base[1],
                       np.where(common<0.05, self.base[3], self.base[2]))
        hv = common*(1-np.exp(-DT/tau))
        hw = common*(1-np.exp(-DT/self.base[7]))
        # Normalized least-mean-squares updates. At rest the observations carry
        # no gain information, so these terms automatically vanish.
        self.speed_delta += .07*hv*(measured_v-pv-hv*self.speed_delta)/(.02+hv*hv)
        self.yaw_delta += .07*hw*(measured_w-pw-hw*self.yaw_delta)/(.02+hw*hw)
        self.speed_delta = np.clip(self.speed_delta, -.35*self.base[0], .35*self.base[0])
        self.yaw_delta = np.clip(self.yaw_delta, -.12, .12)


def commands(weights, observations, speed, yaw, constant_rpm=None):
    """Two trained heads select common drive and wheel difference from local observations."""
    ones = np.ones_like(speed)
    middle, opening = observations
    speed_features = np.column_stack((ones, speed/3., abs(yaw), abs(middle), 1-opening))
    steering_features = np.column_stack((ones, yaw, middle, 1-opening))
    split = len(SPEED_FEATURES)
    common = MAX_COMMON_RPM * expit(np.sum(weights[:, :split] * speed_features, axis=1))
    if constant_rpm is not None:
        common = np.full_like(common, constant_rpm)
    diff = MAX_DIFFERENTIAL_RPM * np.tanh(np.sum(weights[:, split:] * steering_features, axis=1))
    return np.clip(common + diff, 0, WHEEL_LIMIT), np.clip(common - diff, 0, WHEEL_LIMIT)


def simulate(course, model, weights, conditions=Conditions(), trace=False, constant_rpm=None, adapt=False):
    weights = np.atleast_2d(weights)
    if weights.shape[1] != COUNT or (trace and len(weights) != 1):
        raise ValueError("Expected one policy per row and a single policy for playback")
    n = len(weights)
    s, v, yaw = np.zeros((3, n))
    off = np.full(n, conditions.start_offset)
    err = np.full(n, conditions.start_heading)
    params = np.array(model["parameters"])
    estimate = OnlineResponse(n, params.copy()) if adapt else None
    params[0] *= conditions.speed_gain
    params[4:6] *= conditions.yaw_gain
    params[6] += conditions.yaw_bias
    delay = model["delay_steps"] + conditions.extra_delay
    queue = np.zeros((delay+1, 2, n))
    active = np.ones(n, dtype=bool)
    finish = np.full(n, np.inf)
    clearance_min = np.full(n, np.inf)
    history = [[0., 0., off[0], err[0], 0., 0., 0., 0., 0., 0.]] if trace else []
    limit_s = 2.5 * course.length
    for tick in range(int(np.ceil(limit_s / DT))):
        indices = course.indices(s)
        obs = course.view(s, off, err)
        right, left = commands(weights, obs, v, yaw, constant_rpm)
        if estimate is not None:
            right, left = estimate.compensate(right, left)
        queue[tick % len(queue)] = ((right+left)/200, (right-left)/100)
        common, difference = queue[(tick-delay) % len(queue)]
        v_next, yaw_next = step(v, yaw, common, difference, params)
        if estimate is not None:
            known_common, known_diff = queue[(tick-model["delay_steps"]) % len(queue)]
            estimate.observe(v, yaw, known_common, known_diff, v_next, yaw_next)
        avg_v, avg_yaw = 0.5*(v+v_next), 0.5*(yaw+yaw_next)
        bend = course.bend[indices]
        delta_s = DT * avg_v * np.cos(err) / np.maximum(0.30, 1-bend*off)
        next_s = s + delta_s
        next_off = off + DT * avg_v * np.sin(err)
        next_err = err + DT*avg_yaw - bend*delta_s
        clearance = course.clearance(next_s, next_off, next_err, HALF_LENGTH, HALF_WIDTH)
        clearance_min = np.where(active, np.minimum(clearance_min, clearance), clearance_min)
        permitted = (clearance >= BUFFER) & (abs(next_err) < np.pi/2)
        success = active & permitted & (next_s >= course.length)
        fraction = (course.length - s) / np.maximum(delta_s, 1e-9)
        finish[success] = DT * (tick + fraction[success])
        s, off, err, v, yaw = (np.where(active, new, old) for old, new in
                               ((s,next_s), (off,next_off), (err,next_err), (v,v_next), (yaw,yaw_next)))
        if trace:
            history.append([(tick+1)*DT, s[0], off[0], err[0], v[0], yaw[0], right[0], left[0],
                            estimate.speed_delta[0] if adapt else 0., estimate.yaw_delta[0] if adapt else 0.])
        active &= permitted & ~success
        if not np.any(active):
            break
    # A finisher beats every incomplete run. Among finishers only elapsed time matters.
    score = np.where(np.isfinite(finish), 2 + 1 - finish/limit_s, np.clip(s/course.length, 0, 1))
    return score, finish, clearance_min, np.asarray(history)


PRACTICE_SHIFTS = (Conditions(speed_gain=.92, yaw_bias=-.018, start_offset=.12),
                   Conditions(speed_gain=1.05, yaw_bias=.02, start_offset=-.12))


def practice(course, model, seed, generations, population, robust=False):
    if generations < 1 or population < 12:
        raise ValueError("Use at least one generation and twelve candidate policies")
    rng = np.random.default_rng(seed)
    mean = np.zeros(COUNT)
    spread = np.full(COUNT, 2.)
    best_score, best, first = -np.inf, mean.copy(), None
    logs = []
    for generation in range(generations):
        candidates = rng.normal(mean, spread, (population, COUNT))
        candidates[0] = best
        scores, times, _, _ = simulate(course, model, candidates)
        if robust:
            values = [scores]
            for condition in PRACTICE_SHIFTS:
                other_score, other_time, _, _ = simulate(course, model, candidates, condition, adapt=True)
                values.append(other_score)
                times = np.where(np.isfinite(other_time), times, np.inf)
            # Prioritize finishing all drills, then improve worst and mean time.
            scores = .85*np.min(values, axis=0) + .15*np.mean(values, axis=0)
        order = np.argsort(-scores, kind="stable")
        top = order[:max(5, population//6)]
        if scores[order[0]] > best_score:
            best_score = float(scores[order[0]])
            best = candidates[order[0]].copy()
        completed = times[np.isfinite(times)]
        if first is None and len(completed):
            first = candidates[order[0]].copy()
        mean = 0.20*mean + 0.80*candidates[top].mean(axis=0)
        spread = np.maximum(0.24, 0.25*spread + 0.75*candidates[top].std(axis=0))
        logs.append({"generation": generation+1, "trials": population, "finished": len(completed),
                     "best_time_s": float(completed.min()) if len(completed) else None,
                     "best_score": best_score})
        print(f"gen {generation+1:02d}   finished {len(completed):2d}/{population}"
              + (f"   {completed.min():.2f} s" if len(completed) else ""), flush=True)
    if first is None:
        raise RuntimeError("No simulated lap completed; increase practice budget or revise the model.")
    return first, best, logs
