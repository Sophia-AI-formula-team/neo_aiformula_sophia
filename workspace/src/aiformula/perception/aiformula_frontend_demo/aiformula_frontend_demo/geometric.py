"""Stage 2: semantic triangle hashing and robust point-to-curve registration."""
from collections import Counter, defaultdict, deque
from itertools import product
import math

import numpy as np

from .common import LANE_KEYS, lane_arrays, rotate, to_body, to_world, wrap


def features(lanes):
    center, left, right = lanes
    if len(center) < 7:
        return
    stride = max(1, len(center)//11)
    for ia in range(0, len(center)-3, stride):
        a = center[ia]
        for ib in range(ia+2, len(center), stride):
            b = center[ib]
            delta = b-a
            length = float(np.linalg.norm(delta))
            if not 1.9 <= length <= 5.8:
                continue
            u = delta/length
            for kind, curve in enumerate((left, right), 1):
                c = curve[(ia+ib)//2] - a
                normal = np.array((-u[1], u[0]))
                key = (kind, round(length/.6), round((c@u)/length/.20),
                       round((c@normal)/length/.20))
                yield key, a, b


def segments(groups, pose):
    """Keep the observed polylines separate at unmatched endpoints."""
    result = []
    for kind in range(3):
        curves = [to_body(group[kind], pose) for group in groups if len(group[kind]) >= 2]
        if not curves:
            result.append((np.empty((0, 2)), np.empty((0, 2))))
            continue
        a, b = np.vstack([c[:-1] for c in curves]), np.vstack([c[1:] for c in curves])
        good = (np.linalg.norm(b-a, axis=1) > .02) & (np.maximum(a[:, 0], b[:, 0]) >= -.25)
        result.append((a[good], b[good]))
    return result


def nearest_segment(points, seg):
    a, b = seg
    v = b-a
    length_squared = (v*v).sum(axis=1)
    diff = points[:, None] - a
    s = np.clip(np.einsum("pni,ni->pn", diff, v)/length_squared, 0., 1.)
    projected = a + s[..., None]*v
    error = points[:, None] - projected
    distance = np.einsum("pni,pni->pn", error, error)
    chosen = np.argmin(distance, axis=1)
    ii = np.arange(len(points))
    direction = v[chosen] / np.sqrt(length_squared[chosen, None])
    return projected[ii, chosen], np.sqrt(distance[ii, chosen]), direction


def evaluate(lanes, local_segments, x):
    jacobians, residuals, radial = [], [], []
    for points, seg in zip(lanes, local_segments):
        if not len(points) or not len(seg[0]):
            continue
        samples = points[::2]
        rotated = rotate(samples, x[2])
        estimate = rotated + x[:2]
        projected, distance, tangent = nearest_segment(estimate, seg)
        good = distance <= .65
        tangent = tangent[good]
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        deriv = np.column_stack((-rotated[good, 1], rotated[good, 0]))
        jacobians.append(np.column_stack((normal, (normal*deriv).sum(axis=1))))
        residuals.append((normal*(estimate[good]-projected[good])).sum(axis=1))
        radial.append(distance[good])
    if not jacobians:
        return np.empty((0, 3)), np.empty(0), np.empty(0)
    return np.vstack(jacobians), np.concatenate(residuals), np.concatenate(radial)


def comparison_rmse(lanes, segments_by_kind, correction):
    """Score all current samples, retaining a fixed penalty for distant points."""
    distances = []
    for points, edges in zip(lanes, segments_by_kind):
        if len(points) and len(edges[0]):
            query = rotate(points[::2], correction[2]) + correction[:2]
            _, radial, _ = nearest_segment(query, edges)
            distances.append(np.minimum(radial, .65))
    if not distances or sum(map(len, distances)) < 14:
        return None
    return float(np.sqrt(np.mean(np.concatenate(distances)**2)))


def refine(lanes, seg, seed):
    x = np.array(seed, dtype=float)
    penalty = np.diag((2.0, 1.0, 5.0))
    for _ in range(6):
        jac, res, radial = evaluate(lanes, seg, x)
        if len(res) < 14:
            return None
        loss = np.sqrt(np.minimum(1., .16/np.maximum(np.abs(res), 1e-6)))
        a = np.vstack((jac*loss[:, None], penalty))
        b = -np.r_[res*loss, penalty@x]
        step = np.linalg.lstsq(a, b, rcond=None)[0]
        x += np.clip(step, (-.20, -.20, -.035), (.20, .20, .035))
        if np.linalg.norm(x[:2]) > 1.0 or abs(x[2]) > .18:
            return None
        if np.linalg.norm(step) < 1e-4:
            break
    _, _, radial = evaluate(lanes, seg, x)
    if len(radial) < 14:
        return None
    rmse = float(np.sqrt(np.mean(radial**2)))
    score = rmse + .13*float(np.linalg.norm(x[:2])) + .25*abs(float(x[2])) - .001*min(50, len(radial))
    return score, rmse, len(radial), x


class GeometryStage:
    def __init__(self, history=12):
        self.history = deque(maxlen=history)
        self.last_stamp = None

    def index(self):
        result = defaultdict(list)
        for _, _, curves in self.history:
            for key, a, b in features(curves):
                result[key].append((a, b))
        return result

    def seeds(self, lanes, odom_pose):
        indexed = self.index()
        votes, hypotheses = Counter(), {}
        for (kind, length, u, v), a, b in features(lanes):
            direction = b-a
            yaw0 = math.atan2(direction[1], direction[0])
            for dl, du, dv in product((0, -1, 1), repeat=3):
                for old_a, old_b in indexed.get((kind, length+dl, u+du, v+dv), ()):
                    base_delta = old_b-old_a
                    yaw = wrap(math.atan2(base_delta[1], base_delta[0])-yaw0)
                    turn = wrap(yaw-odom_pose[2])
                    if abs(turn) > .17:
                        continue
                    world_xy = old_a-rotate(a, yaw)
                    xy = to_body(world_xy, odom_pose)
                    if abs(xy[0]) > .95 or abs(xy[1]) > .65:
                        continue
                    vec = np.r_[xy, turn]
                    vote = tuple(np.rint(vec/[.20, .14, .04]).astype(int))
                    votes[vote] += 1
                    if vote not in hypotheses:
                        hypotheses[vote] = vec
        selected = [(hypotheses[key], count) for key, count in votes.most_common(5)]
        return selected, len(votes)

    def step(self, obs, odom_pose):
        t = obs["stamp_ns"]
        if self.last_stamp is not None and t < self.last_stamp:
            self.history.clear()
        self.last_stamp = t
        live = lane_arrays(obs["corridor"])
        lanes = lane_arrays(obs.get("landmark_curves", obs["corridor"]))
        packet = dict(stamp_ns=t, body_frame=obs["body_frame"], corridor=obs["corridor"],
                      status="no_camera_corridor", hash_candidates=0, seed_votes=0,
                      fit_rmse_m=None, prior_rmse_m=None, fit_support=0, correction=[0., 0., 0.],
                      seed_offset=None, seeds_refined=0, selected_candidate_rmse_m=None,
                      map_keyframes=len(self.history), reference_points_blended=0,
                      map_lines_body={key: [] for key in LANE_KEYS})
        if not len(lanes[0]):
            return packet
        if odom_pose is None:
            packet["status"] = "odom_missing_camera_only"
            return packet
        odom_pose = np.asarray(odom_pose, dtype=float)
        if self.history and np.linalg.norm(self.history[-1][1][:2]-odom_pose[:2]) > 20.:
            self.history.clear()
        registered = odom_pose.copy()
        if self.history:
            old = [item[2] for item in self.history]
            seg = segments(old, odom_pose)
            prior = comparison_rmse(lanes, seg, np.zeros(3))
            if prior is not None:
                packet["prior_rmse_m"] = round(prior, 4)
            seeds, count = self.seeds(lanes, odom_pose)
            packet["hash_candidates"] = count
            packet["seeds_refined"] = len(seeds)
            fits = []
            for seed, votes in seeds:
                fit = refine(lanes, seg, seed)
                if fit is not None:
                    quality = comparison_rmse(lanes, seg, fit[3])
                    if quality is not None:
                        fits.append((quality, fit, votes, seed))
            packet["status"] = "odom_prior_camera_only"
            if fits:
                quality, fit, votes, seed = min(fits, key=lambda trial: (trial[0], trial[1][0]))
                _, rmse, support, delta = fit
                packet.update(fit_rmse_m=round(rmse, 4), fit_support=support, seed_votes=votes,
                              seed_offset=seed.round(5).tolist(),
                              selected_candidate_rmse_m=round(quality, 4))
                if rmse <= .30 and support >= 14 and prior is not None and prior-quality >= .006:
                    registered[:2] = to_world(delta[:2], odom_pose)
                    registered[2] = wrap(odom_pose[2]+delta[2])
                    packet.update(status="hash_refined", correction=delta.round(5).tolist())
            # Prior observations can move the reference gently within this frame's
            # observed corridor. They never move either of its live boundaries.
            if packet["status"] == "hash_refined":
                corrected_seg = segments(old, registered)
                center = live[0].copy()
                if len(corrected_seg[0][0]):
                    match, distance, _ = nearest_segment(center, corrected_seg[0])
                    use = (distance < .35) & (np.abs(match[:, 0]-center[:, 0]) < .24)
                    center[use, 1] += .35*(match[use, 1]-center[use, 1])
                    clipped = np.clip(center[:, 1], live[2][:, 1]+.20, live[1][:, 1]-.20)
                    center[:, 1] = clipped
                    packet["corridor"] = dict(obs["corridor"], center=center.round(4).tolist())
                    packet["reference_points_blended"] = int(use.sum())
        else:
            packet["status"] = "map_bootstrap"
        if not self.history or t-self.history[-1][0] > 750_000_000 or (
                np.linalg.norm(registered[:2]-self.history[-1][1][:2]) >= .80):
            self.history.append((t, registered.copy(), tuple(to_world(curve, registered) for curve in lanes)))
        packet["map_keyframes"] = len(self.history)
        packet["map_lines_body"] = {
            key: [to_body(item[2][i][::2], registered).round(3).tolist()
                  for item in list(self.history)[-8:]]
            for i, key in enumerate(LANE_KEYS)}
        return packet
