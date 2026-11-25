"""Find a connected full-width strip in the camera mask and extract its midpoint."""
import cv2
import numpy as np

from optimization_oa.planner import PlanResult


STATION_STEP = .20
LATERAL_STEP = .05
LATERAL_LIMIT = 7.
SHIFTS = np.arange(-4, 5)
SMOOTHING_M = (.10, .18, .28, .42, .60, .85)


def _half_width(config, slope):
    return (config.vehicle_width_m/2 + config.clearance_m) * np.hypot(1., slope)


def _visible_stations(camera, config):
    probe = np.arange(.05, min(config.max_lane_gap_m, config.lookahead_m)+.001, .025)
    grid = np.stack(np.broadcast_arrays(probe[:, None], [-_half_width(config, 0), 0,
                                                        _half_width(config, 0)]), axis=-1)
    uv, depth = camera.ground_to_image(grid)
    visible = ((depth > .05) & np.isfinite(uv).all(axis=-1) &
               (uv[..., 0] >= 0) & (uv[..., 0] <= camera.width-1) &
               (uv[..., 1] >= camera.height*camera.roi_top_ratio) &
               (uv[..., 1] <= camera.height-1))
    starts = np.flatnonzero(visible.all(axis=1))
    if not len(starts):
        return np.empty(0)
    near = probe[starts[0]]
    return np.arange(near, config.lookahead_m + .001, STATION_STEP/2)


def _ground_clearance(camera, allowed, solid, stations, lateral):
    x, y = np.meshgrid(stations, lateral, indexing="ij")
    uv, depth = camera.ground_to_image(np.stack((x, y), axis=-1))
    usable = (depth > .05) & np.isfinite(uv).all(axis=-1)
    u = np.rint(np.where(usable, uv[..., 0], -1)).astype(int)
    v = np.rint(np.where(usable, uv[..., 1], -1)).astype(int)
    usable &= ((u >= 0) & (u < camera.width) &
               (v >= camera.height*camera.roi_top_ratio) & (v < camera.height))
    columns = np.arange(len(lateral))
    free = np.zeros(usable.shape, bool)
    free[usable] = allowed[v[usable], u[usable]]
    left = np.maximum.accumulate(np.where(free, -1, columns), axis=1)
    right = np.minimum.accumulate(np.where(free, len(lateral), columns)[:, ::-1], axis=1)[:, ::-1]
    free_width = (np.minimum(columns-left, right-columns)-.5) * LATERAL_STEP
    if not solid.any():
        return free_width, None
    paint = np.zeros(usable.shape, bool)
    paint[usable] = solid[v[usable], u[usable]]
    left = np.maximum.accumulate(np.where(paint, columns, -1), axis=1)
    right = np.minimum.accumulate(np.where(paint, columns, len(lateral))[:, ::-1], axis=1)[:, ::-1]
    left_dist = np.where(left >= 0, columns-left, np.inf)
    right_dist = np.where(right < len(lateral), right-columns, np.inf)
    solid_distance = np.maximum(0., np.minimum(left_dist, right_dist)-.5)*LATERAL_STEP
    return free_width, solid_distance


def _reachable_bands(free, lateral, config, solid_distance=None, solid_weight=0.):
    """Stitch admissible transverse windows from the visible ego footprint forward."""
    slopes = SHIFTS*LATERAL_STEP/STATION_STEP
    widths = _half_width(config, slopes)
    headings = np.arctan(slopes)
    lengths = STATION_STEP*np.hypot(1., slopes)
    center_index = len(lateral)//2
    straight = len(SHIFTS)//2
    if free[0, center_index] < widths[straight]:
        return [], []
    costs = np.full((len(SHIFTS), len(lateral)), np.inf)
    costs[straight, center_index] = 0.
    parents, history = [], [costs]
    columns = np.arange(len(lateral))
    transitions = []
    for h, shift in enumerate(SHIFTS):
        src = columns - shift
        usable = (src >= 0) & (src < len(lateral))
        dest, src = columns[usable], src[usable]
        midpoint = dest + src
        choices = np.arange(max(0, h-1), min(len(SHIFTS), h+2))
        turn_ds = (lengths[choices]+lengths[h])/2
        turn_energy = (headings[choices]-headings[h])**2/turn_ds
        local_cost = config.length_weight*lengths[h]+config.smoothness_weight*turn_energy
        transitions.append((dest, src, midpoint//2, (midpoint+1)//2, choices, local_cost))
    for end in range(2, len(free), 2):
        following = np.full_like(costs, np.inf)
        parent = np.full(costs.shape, -1, dtype=np.int8)
        for h, (dest, src, mid0, mid1, choices, local_cost) in enumerate(transitions):
            spare = np.minimum.reduce((free[end, dest], free[end-2, src],
                                       free[end-1, mid0], free[end-1, mid1])) - widths[h]
            valid = spare >= 0
            target, source = dest[valid], src[valid]
            cross = costs[choices[:, None], source] + local_cost[:, None]
            if solid_distance is not None:
                touches = np.minimum.reduce((solid_distance[end, target], solid_distance[end-1, mid0[valid]],
                                              solid_distance[end-1, mid1[valid]])) <= widths[h]
                cross += solid_weight*lengths[h]*touches
            best = np.argmin(cross, axis=0)
            col = np.arange(len(target))
            active = np.isfinite(cross[best, col])
            target, best, col = target[active], best[active], col[active]
            following[h, target] = cross[best, col]
            parent[h, target] = choices[best]
        if not np.isfinite(following).any():
            break
        parents.append(parent)
        history.append(following)
        costs = following
    return parents, history


def _trace_windows(parents, depth, heading, column):
    columns = np.empty(depth+1, dtype=int)
    for row in range(depth, 0, -1):
        columns[row] = column
        heading, column = int(parents[row-1][heading, column]), column-SHIFTS[heading]
    columns[0] = column
    return columns


def _terminal_states(cost):
    order = np.argsort(cost, axis=None, kind="stable")
    order = order[np.isfinite(cost.ravel()[order])]
    if not len(order):
        return ()
    headings, columns = np.unravel_index(order, cost.shape)
    # Include a representative across the visible lateral extent. A single
    # cluster of cheap endpoints can otherwise hide the other side of an obstacle.
    group_width = int(round(.5/LATERAL_STEP))
    _, group_first = np.unique(columns//group_width, return_index=True)
    keep = np.union1d(np.arange(min(16, len(order))), group_first)
    return zip(headings[keep], columns[keep])


def _shape_candidates(x, offsets, config):
    ground_x = np.linspace(x[0], x[-1], max(3, int(np.ceil((x[-1]-x[0])/.04))+1))
    ds_x = ground_x[1]-ground_x[0]
    samples = np.array([np.interp(ground_x, x, row) for row in offsets])
    tail_slope = (offsets[:, -1]-offsets[:, -min(4, len(x))])/(x[-1]-x[-min(4, len(x))])
    t = np.clip((ground_x-ground_x[0])/.65, 0., 1.)
    anchor = 1.-t**3*(10.-15.*t+6.*t*t)
    curves = []
    for sigma_m in SMOOTHING_M:
        sigma = sigma_m/ds_x
        radius = int(np.ceil(4*sigma))
        reflected_start = np.pad(samples, ((0, 0), (radius, 0)), mode="reflect")
        tail = samples[:, -1:] + tail_slope[:, None]*np.arange(1, radius+1)*ds_x
        padded = np.concatenate((reflected_start, tail), axis=1)
        smoothed = cv2.GaussianBlur(padded, (2*radius+1, 1), sigmaX=sigma)
        filtered = smoothed[:, radius:radius+len(ground_x)]
        curves.append(filtered-filtered[:, :1]*anchor)
    offsets = np.vstack(curves)
    slope = np.gradient(offsets, ground_x, axis=1, edge_order=2)
    curvature = np.gradient(slope, ground_x, axis=1, edge_order=2)/(1.+slope*slope)**1.5
    ds = np.hypot(np.diff(ground_x), np.diff(offsets, axis=1))
    length = ds.sum(axis=1)
    energy = ((curvature[:, :-1]**2+curvature[:, 1:]**2)*ds/2).sum(axis=1)
    max_curvature = np.max(np.abs(curvature), axis=1)
    score = config.length_weight*length+config.smoothness_weight*energy
    score[max_curvature > config.max_curvature_inv_m] = np.inf
    return ground_x, offsets, slope, length, energy, max_curvature, score


def _checked_strip(camera, allowed, ground_x, center, slope, config):
    # Validate both physical boundaries and the complete raster footprint before
    # extracting the midpoint as the tracking path.
    half_width = _half_width(config, slope)
    edges = np.stack((np.column_stack((ground_x, center+half_width)),
                      np.column_stack((ground_x, center-half_width))))
    uv, depth = camera.ground_to_image(edges)
    if np.any(depth <= .05) or not np.isfinite(uv).all():
        return None
    vertices = np.rint(np.vstack((uv[0], uv[1, ::-1]))).astype(np.int32)
    if (np.any(vertices[:, 0] < 0) or np.any(vertices[:, 0] >= camera.width) or
            np.any(vertices[:, 1] < camera.height*camera.roi_top_ratio) or
            np.any(vertices[:, 1] >= camera.height)):
        return None
    if not allowed[vertices[:, 1], vertices[:, 0]].all():
        return None
    left, top = vertices.min(axis=0)
    right, bottom = vertices.max(axis=0) + 1
    raster = np.zeros((bottom-top, right-left), np.uint8)
    cv2.fillPoly(raster, [vertices-np.array([left, top], dtype=np.int32)], 255)
    if np.any((raster != 0) & ~allowed[top:bottom, left:right]):
        return None
    band = np.zeros(allowed.shape, np.uint8)
    band[top:bottom, left:right] = raster
    yaw = np.arctan(slope)
    path = np.column_stack((edges.mean(axis=0), yaw))
    center_uv, _ = camera.ground_to_image(path[:, :2])
    return path, center_uv, band


def _solid_contact(camera, band, solid):
    contact = (band != 0) & solid
    rows, cols = np.nonzero(contact)
    if not len(rows):
        return 0., 0
    occupied_rows, counts = np.unique(rows, return_counts=True)
    mean_col = np.bincount(rows, weights=cols, minlength=camera.height)[occupied_rows]/counts
    points = np.c_[mean_col, occupied_rows]
    bottom = camera.image_to_ground(points+[0., .5])
    top = camera.image_to_ground(points-[0., .5])
    return float(np.abs(top[:, 0]-bottom[:, 0]).sum()), len(rows)


def search_band(camera, allowed, config, solid=None, solid_weight=0.):
    """Check the complete raster strip on the native mask before returning a path."""
    result = PlanResult("no_feasible_band", allowed=allowed, obstacles=~allowed,
                        band=np.zeros(allowed.shape, np.uint8))
    stations = _visible_stations(camera, config)
    if not len(stations):
        result.reason = "no_visible_start"
        return result
    lateral = np.arange(-round(LATERAL_LIMIT/LATERAL_STEP),
                        round(LATERAL_LIMIT/LATERAL_STEP)+1)*LATERAL_STEP
    solid = np.zeros_like(allowed) if solid is None else solid & allowed
    free, solid_distance = _ground_clearance(camera, allowed, solid, stations, lateral)
    parents, history = _reachable_bands(free, lateral, config, solid_distance, solid_weight)
    x = stations[::2]
    result.metrics.update(visible_start_m=float(x[0]), candidates_checked=0,
                          target_end_m=float(x[-1]))
    if not history:
        result.reason = "start_footprint_blocked"
        return result
    first = max(1, int(np.searchsorted(x, config.min_lookahead_m)))
    if len(history) <= first:
        result.reason = "insufficient_visible_band"
        return result
    result.metrics["candidates_scored"] = 0
    for n in range(len(history)-1, first-1, -1):
        paths = np.array([_trace_windows(parents, n, h, col)
                          for h, col in _terminal_states(history[n])])
        if not len(paths):
            continue
        ground_x, offsets, slopes, length, energy, max_curvature, scores = _shape_candidates(
            x[:n+1], lateral[paths], config)
        result.metrics["candidates_scored"] += len(scores)
        best, best_score = None, np.inf
        for k in np.argsort(scores, kind="stable"):
            # The length/curvature score is a lower bound on the final score.
            if scores[k] >= best_score:
                break
            result.metrics["candidates_checked"] += 1
            checked = _checked_strip(camera, allowed, ground_x, offsets[k], slopes[k], config)
            if checked is None:
                continue
            contact_m, contact_px = _solid_contact(camera, checked[2], solid) if solid_distance is not None else (0., 0)
            score = scores[k]+solid_weight*contact_m
            if score < best_score:
                best, best_score = (k, checked, contact_m, contact_px), score
        if best is not None:
            k, checked, contact_m, contact_px = best
            result.path, result.center_uv, result.band = checked
            result.reason = "full_band" if n == len(x)-1 else "limited_band"
            result.metrics.update(length_m=float(length[k]), verified_end_m=float(x[n]),
                                  curvature_energy_inv_m=float(energy[k]),
                                  max_curvature_inv_m=float(max_curvature[k]),
                                  solid_overlap_px=contact_px, solid_contact_m=contact_m,
                                  solid_cost=float(solid_weight*contact_m),
                                  score=float(best_score), band_pixels=int(np.count_nonzero(result.band)))
            return result
    return result
