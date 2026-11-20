"""Historical 11/20 launch override for reproducing field logs in offline checks."""
from collections import Counter

import numpy as np

from concept_candidates.ranker617 import choose
from concept_road.v3_model import BODY_FRONT, BODY_HALF_WIDTH, BODY_REAR

# These are the stopped speed used by the candidate brake rollout, the margin
# above which its road-risk penalty is zero, and the near observer's row pitch.
STOPPED_MPS = .04
CLEAR_MARGIN_M = .5
NEAR_ROW_STEP_M = .16


def observed_path_limit(near_z, visible_z):
    if visible_z is None:
        return -np.inf
    visible = np.asarray(visible_z, dtype=float)
    near = np.asarray(near_z, dtype=float)
    if (visible.shape != (2,) or near.ndim != 1 or not len(near)
            or not np.isfinite(visible).all() or not np.isfinite(near).all()):
        return -np.inf
    lo, hi = visible
    # A change in yaw can move the front corner further than BODY_FRONT.
    front_radius = np.hypot(BODY_FRONT, BODY_HALF_WIDTH)
    if hi <= lo or lo <= 0:
        return -np.inf
    history = np.sort(near[near < lo])
    behind_body = np.flatnonzero(history <= -np.hypot(BODY_REAR, BODY_HALF_WIDTH))
    if not len(behind_body):
        return -np.inf
    # One missing 16 cm row doubles the spacing. A visible far-away patch
    # cannot bridge the gap from the front camera to the vehicle.
    samples = np.r_[history[behind_body[-1]:], lo]
    if np.any(np.diff(samples) > NEAR_ROW_STEP_M + 1e-6):
        return -np.inf
    return hi - front_radius


def certify_forward(moving, table, audit, stop, evidence):
    """Check every allowed forward candidate; retain the first failing reason."""
    nominal_brake = (np.asarray(audit['nominal_feasible'], dtype=bool)
                     & np.asarray(audit['no_observation_brake_feasible'], dtype=bool))
    margins = (table[:, 5] >= CLEAR_MARGIN_M) & (np.asarray(audit['brake_lower']) >= CLEAR_MARGIN_M)
    reasons = {}
    limit = observed_path_limit(evidence['observed_near_z'], evidence.get('visible_z'))
    certified = np.zeros_like(moving)
    for i in np.flatnonzero(moving):
        if not nominal_brake[i]:
            reason = 'continuous_drive_or_immediate_brake_unavailable'
        elif not margins[i]:
            reason = 'insufficient_driving_or_braking_margin'
        else:
            path = np.asarray(audit['paths'][i], dtype=float)
            if (not len(path) or not np.isfinite(path).all() or np.any(np.diff(path[:, 0]) < 0)
                    or table[i, 8] <= table[stop, 8] or path[-1, 0] <= 0):
                reason = 'no_verified_forward_progress'
            elif path[-1, 0] > limit:
                reason = 'incomplete_measured_corridor'
            else:
                certified[i] = True
                continue
        reasons[int(i)] = reason
    return certified, reasons


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
        moving = allowed & (table[:, 0] > 0)
        top = choose(scores, moving)
        certified, reasons = certify_forward(moving, table, audit, pick, evidence)
        if certified.any() and 'check_localized_cones' in evidence:
            indices = np.flatnonzero(certified)
            margins = np.asarray(evidence['check_localized_cones'](indices))
            if margins.shape != indices.shape:
                return keep('missing_localized_cone_trajectory_check')
            passes = np.isfinite(margins) & (margins >= CLEAR_MARGIN_M)
            decision['localized_cone_checked'] = len(indices)
            decision['localized_cone_clear'] = int(np.sum(passes))
            for idx in indices[~passes]:
                reasons[int(idx)] = 'localized_cone_on_drive_or_brake_path'
            certified[indices] = passes
        selected = choose(scores, certified)
        if selected is not None and 'localized_cone_checked' in decision:
            decision['localized_cone_selected_margin_m'] = float(margins[np.searchsorted(indices, selected)])
        decision.update(forward_index=top, h12_top_forward_score=float(scores[top]),
                        top_forward_rejection=reasons.get(top), certified_index=selected,
                        certified_forward_indices=np.flatnonzero(certified).tolist(),
                        rejection_counts=dict(Counter(reasons.values())))
        if selected is None:
            return keep(reasons[top])
        # Lexicographic order for this start only: certified first, then H12.
        # Export a score that agrees with that order. This common offset lets
        # the raw H12 scores be recovered exactly to reporting precision.
        others = allowed & ~certified
        final = scores.copy()
        shift = float(scores[others].max() - scores[certified].min() + 1e-6)
        final[others] -= shift
        self.last_time = stamp
        self.last_command = table[selected, :2].copy()
        decision.update(applied=True, reason='measured_clear_launch', waiting_score_reduction=shift,
                        noncertified_score_reduction=shift,
                        continued=bool(continuing), h12_hold_score=float(scores[pick]),
                        h12_forward_score=float(scores[selected]))
        return selected, final, decision
