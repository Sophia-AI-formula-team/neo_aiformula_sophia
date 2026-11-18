"""Runtime source extracted from the accepted V6.26 freeze: experiments/v6251_junction/limits6251.py. """

import numpy as np

from concept_road.onboard625 import HALF_PAINT, LocalLimits

class JunctionLimits(LocalLimits):
    def update(self, road, tracks, lines, speed, yaw, dt):
        material = np.asarray(road)
        clipped, edge = super().update(road, tracks, lines, speed, yaw, dt)
        depth = clipped[:, 0]
        for side, col, sign in [('left', 1, 1.), ('right', 2, -1.)]:
            if side not in self.history:
                continue
            points, error = self.history[side]
            visible = (depth >= points[0, 0]) & (depth <= points[-1, 0])
            z = depth[visible]
            paint = sign * np.interp(z, points[:, 0], points[:, 1]) - HALF_PAINT
            ages = np.interp(z, points[:, 0], points[:, 2])
            paint_error = np.maximum(.04, error + .025 * ages)
            # The mean edge and the inward confidence bound each intersect both observations.
            bound = np.maximum(sign * material[visible, col] + material[visible, col + 2], paint + paint_error)
            clipped[visible, col + 2] = bound - sign * clipped[visible, col]
        return clipped, edge

def observed_direction(road, heading, tracks, edge):
    ordered = edge.get('ordered_paint')
    if not ordered:
        return heading, None
    by_id = {t['id']: t for t in tracks}
    triple = [by_id[i] for i in ordered]
    near = max(t['zmin'] for t in triple)
    far = min(t['zmax'] for t in triple)
    if far <= near:
        return heading, None

    divider = triple[1]
    coefficients = np.asarray(divider['coef'])
    z = road[:, 0]
    visible = (z >= near) & (z <= far)
    result = np.asarray(heading).copy()
    slope = np.polynomial.polynomial.polyder(coefficients)
    result[visible] = np.arctan(np.polynomial.polynomial.polyval(z[visible], slope))
    samples = np.linspace(near, far, int(np.ceil((far - near) / .08)) + 1)
    positions = np.polynomial.polynomial.polyval(samples, coefficients)
    reference = dict(points=np.c_[samples, positions], source='observed_two_lane_divider',
                     coef=coefficients, sigma=max(.04, float(divider['error'])), support=[near, far],
                     rule='Progress along a visible course marking; all lateral positions remain eligible.')
    return result, reference
