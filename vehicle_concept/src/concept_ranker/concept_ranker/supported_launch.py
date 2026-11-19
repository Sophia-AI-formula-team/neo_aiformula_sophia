"""Allow a checked start when H12 prefers waiting on an observed clear corridor."""
import numpy as np

from concept_candidates.ranker617 import choose
from concept_road.v3_model import BODY_FRONT, BODY_HALF_WIDTH, BODY_REAR

# These are the stopped speed used by the candidate brake rollout, the margin
# above which its road-risk penalty is zero, and the near observer's row pitch.
STOPPED_MPS = .04
CLEAR_MARGIN_M = .5
NEAR_ROW_STEP_M = .16


def observed_corridor(near_z, visible_z, path):
    if visible_z is None:
        return False
    visible = np.asarray(visible_z, dtype=float)
    near = np.asarray(near_z, dtype=float)
    if (visible.shape != (2,) or near.ndim != 1 or not len(near)
            or not np.isfinite(visible).all() or not np.isfinite(near).all()):
        return False
    lo, hi = visible
    # A change in yaw can move the front corner further than BODY_FRONT.
    front_radius = np.hypot(BODY_FRONT, BODY_HALF_WIDTH)
    if hi <= lo or not (.0 < lo) or path[:, 0].max() + front_radius > hi:
        return False
    history = np.sort(near[near < lo])
    behind_body = np.flatnonzero(history <= -np.hypot(BODY_REAR, BODY_HALF_WIDTH))
    if not len(behind_body):
        return False
    # One missing 16 cm row doubles the spacing. A visible far-away patch
    # cannot bridge the gap from the front camera to the vehicle.
    samples = np.r_[history[behind_body[-1]:], lo]
    return bool(np.all(np.diff(samples) <= NEAR_ROW_STEP_M + 1e-6))


class SupportedLaunch:
    def __init__(self):
        self.last_command = None
        self.last_time = None

    def decide(self, pick, scores, allowed, baseline, audit, obs, evidence):
        last_command, last_time = self.last_command, self.last_time
        self.last_command = self.last_time = None
        if evidence is None:
            return pick, scores, None
        table = np.asarray(audit['table'])
        decision = {'h12_index': int(pick), 'applied': False}

        def keep(reason):
            decision['reason'] = reason
            return pick, scores, decision

        if table[pick, 0] != 0:
            return keep('h12_selected_motion')
        if not evidence.get('obstacle_clear') or len(obs.objects):
            return keep('obstacle_or_unresolved_detection')
        stamp = float(evidence['stamp_s'])
        applied = float(evidence['applied_speed_mps'])
        previous = np.asarray(audit['previous_command'])
        if not np.isfinite([stamp, applied, obs.speed, obs.dt]).all() or obs.speed < 0:
            return keep('invalid_or_reversing_motion')
        continuing = (last_command is not None and 0 < obs.dt <= .5
                      and abs(stamp - last_time - obs.dt) < 1e-4
                      and abs(applied - last_command[0]) <= 1e-5
                      and np.allclose(previous, last_command, rtol=0., atol=1e-5))
        if not continuing and not (0 <= obs.speed < STOPPED_MPS and abs(applied) < STOPPED_MPS):
            return keep('not_at_rest_or_continuing_applied_launch')
        if baseline is None or not allowed[baseline] or table[baseline, 0] <= 0:
            return keep('whitebox_prefers_hold')
        # Preserve the entire H12 order between moving options. If its top
        # forward option lacks evidence, a lower-ranked one does not replace it.
        moving = choose(scores, allowed & (table[:, 0] > 0))
        decision['forward_index'] = int(moving)
        if not (audit['nominal_feasible'][moving] and audit['no_observation_brake_feasible'][moving]):
            return keep('continuous_drive_or_immediate_brake_unavailable')
        if min(table[moving, 5], audit['brake_lower'][moving]) < CLEAR_MARGIN_M:
            return keep('insufficient_driving_or_braking_margin')
        path = np.asarray(audit['paths'][moving], dtype=float)
        if (not len(path) or not np.isfinite(path).all() or np.any(np.diff(path[:, 0]) < 0)
                or table[moving, 8] <= table[pick, 8] or path[-1, 0] <= 0):
            return keep('no_verified_forward_progress')
        if not observed_corridor(evidence['observed_near_z'], evidence.get('visible_z'), path):
            return keep('incomplete_measured_corridor')
        # Waiting is dominated here. Shift only the checked waiting choices as
        # one group so the exported scores explain the result without altering
        # the order between forward trajectories or their H12 corrections.
        waiting = allowed & (table[:, 0] == 0)
        final = scores.copy()
        shift = float(scores[waiting].max() - scores[moving] + 1e-6)
        final[waiting] -= shift
        self.last_time = stamp
        self.last_command = table[moving, :2].copy()
        decision.update(applied=True, reason='measured_clear_launch', waiting_score_reduction=shift,
                        continued=bool(continuing), h12_hold_score=float(scores[pick]),
                        h12_forward_score=float(scores[moving]))
        return moving, final, decision
