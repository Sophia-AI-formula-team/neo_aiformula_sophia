"""Position newly seen cone feet for a launch check; never create a live track."""
import numpy as np

from concept_obstacles.obstacles617 import (INITIAL_SPEED_SIGMA, UNKNOWN_ACCELERATION_SIGMA,
                                            Track, collision_row, projected_foot)
from concept_road.vision import FOCAL, HEIGHT, CX, CY


def launch_objects(detections, tracked, measurements):
    """None means unresolved, empty means clear, rows require trajectory checks."""
    if len(tracked):
        return None
    explained = set()
    risks = []
    for m in measurements:
        foot = np.asarray(m.get('foot_uv'), dtype=float)
        ground = np.asarray(m.get('ground_forward_right_m'), dtype=float)
        if (ground.shape != (2,) or foot.shape != (2,) or not np.isfinite([foot, ground]).all()
                or ground[0] <= 0 or foot[1] <= CY+5):
            return None
        z, x = ground
        projected = np.array([CX+FOCAL*x/z, CY+HEIGHT*FOCAL/z])
        if not np.allclose(projected, foot, atol=.01, rtol=0) or m.get('within_tracking_range') != (z < 12.):
            return None
        # Current runtimes record every original box belonging to this unique
        # foot. Historical bags still contain the measured foot even for a
        # subthreshold raw box deliberately omitted from the display overlay.
        for raw in m.get('source_boxes', []):
            raw = np.asarray(raw, dtype=float)
            if raw.shape != (4,) or not np.isfinite(raw).all():
                return None
            # A big foreground proposal can contain a tiny, much farther
            # cone. Its bottom must not claim a nearer foot than the measured
            # one beyond the detector's existing three-pixel error budget.
            if raw[3] > foot[1]+3. or not raw[0]-3. <= foot[0] <= raw[2]+3.:
                return None
            explained.add(tuple(raw))
        if z >= 12.:
            continue
        point, cov = projected_foot(z, x, HEIGHT, True)
        # Old recordings omitted clipping. Use the tracker's clipped-foot
        # uncertainty in that case. Never discount occupancy by class score.
        if m.get('clipped', True):
            cov += np.eye(2)*.5**2
        covariance = np.zeros((4, 4))
        covariance[:2, :2] = cov
        covariance[2:, 2:] = np.eye(2)*INITIAL_SPEED_SIGMA**2
        track = Track(0, np.r_[point, 0., 0.], covariance, np.zeros(2),
                      static_state=point, static_covariance=cov,
                      maximum_radius=float(np.linalg.norm(point)))
        row = np.asarray(collision_row(track, np.eye(2)*UNKNOWN_ACCELERATION_SIGMA**2))
        # Each hypothesis receives full weight: the risk sum upper-bounds
        # either unknown mode and aggregates all observed cones together.
        dynamic = row.copy()
        row[25], row[68], dynamic[25] = 1., 0., 0.
        risks.extend([row, dynamic])
    boxes = detections['boxes'][(detections['labels'] == 91) & (detections['scores'] >= .15)]
    for box in boxes:
        if not np.isfinite(box).all() or not np.all(box[2:] > box[:2]) or box[3] <= CY+5:
            return None
        if tuple(box) not in explained:
            return None
    return np.asarray(risks).reshape(-1, 77)
