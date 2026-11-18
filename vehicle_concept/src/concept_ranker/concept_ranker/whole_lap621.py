"""Runtime source extracted from the accepted V6.26 freeze: experiments/v617_regression/whole_lap621.py. """

import json

import numpy as np

from concept_candidates.ranker617 import choose

SCHEMA = 'V6.21 terminal lap scorer'

COEFFICIENTS = ('line', 'entry_lift', 'exit_acceleration')

BOUND = 1.4

def eligible_for_progress(allowed, table, audit):
    """Use a sustainable forward choice if one exists, then a checked advance."""
    forward = allowed & (table[:, 0] > 0)
    continuous = forward & np.asarray(audit['nominal_feasible'], bool)
    if continuous.any():
        return continuous
    return forward if forward.any() else allowed

def score_inputs(candidate, context, table, pace):
    # Road estimates at 1, 3, 5, 8 and 10 m. Each row ends with presence.
    road = context[:30].reshape(5, 6)
    angles = road[:, 2] * np.pi
    seen = road[:, 5] > 0
    near_bend = (angles[2] - angles[0]) if seen[[0, 2]].all() else 0.
    far_bend = (angles[4] - angles[2]) if seen[[2, 4]].all() else 0.
    if not seen[[0, 2, 4]].all():
        near_bend = far_bend = 0.
    entering = np.clip((abs(far_bend) - abs(near_bend)) / .16, 0., 1.)
    exiting = np.clip((abs(near_bend) - abs(far_bend)) / .16, 0., 1.)
    bend = np.tanh((near_bend + far_bend) / .22)
    width = float((road[3, 1] - road[3, 0]) * 6)
    center = float((road[3, 0] + road[3, 1]) * 3)
    lateral = np.tanh((candidate[:, 2] * 6 - center) / max(1., width * .5))
    speed_fraction = np.clip(table[:, 0] / pace, 0., 1.)
    return np.column_stack((
        lateral * bend,
        (1. - speed_fraction**2) * entering,
        speed_fraction**2 * exiting,
    )), (float(entering), float(exiting), float(bend))

class WholeLapSelector:
    def __init__(self, path, pace):
        data = json.loads(path.read_bytes())
        self.weights = np.asarray(data['weights'], dtype=float)
        if (data['schema'] != SCHEMA or self.weights.shape != (len(COEFFICIENTS),)
                or not np.isfinite(self.weights).all() or np.max(np.abs(self.weights)) > 4.):
            raise ValueError('Invalid matched-lap scoring weights')
        self.pace = pace

    def rank(self, features, allowed, baseline, audit):
        candidate, _, context = features
        table = np.asarray(audit['table'], float)
        signals, (entering, exiting, bend) = score_inputs(candidate, context, table, self.pace)
        components = signals * self.weights
        raw = components.sum(axis=1)
        adjustments = np.clip(raw - raw[baseline], -BOUND, BOUND)
        scores = table[:, 7]
        final = scores + adjustments
        eligible = eligible_for_progress(allowed, table, audit)
        pick = choose(final, eligible)
        if eligible[baseline] and final[baseline] >= final[pick] - 1e-8:
            pick = baseline
        return pick, dict(
            baseline=int(baseline), selected=int(pick), cap=BOUND,
            base_scores=scores.tolist(), score_adjustments=adjustments.tolist(),
            final_scores=final.tolist(), eligible=eligible.tolist(),
            first_control_changed=not np.allclose(table[pick, :2], table[baseline, :2], atol=1e-5, rtol=0),
            entry_signal=entering, exit_signal=exiting, curve_signal=bend,
            components_relative_to_whitebox=(components[pick]-components[baseline]).tolist(),
        )
