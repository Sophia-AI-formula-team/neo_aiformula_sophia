"""Local raster-island geometry proposals; never creates contour connectivity.

White connectivity is not invariant under sampling a thin painted ribbon.
Nearby islands can constrain a common shape without being joined in the map.
Every black center in the selected metric disk and finite tangent domain is
retained. Real gaps/nearby stripes that contradict a single band reject it.
"""
from fractions import Fraction

import numpy as np

from .quantized_ribbon import fit_label_curve


def _pca(points, cfg):
    if not cfg['normal_min_points'] <= len(points) <= cfg['max_patch_points']:
        return None
    origin = np.mean(points, axis=0)
    delta = points-origin
    covariance = delta.T@delta/len(points)
    values, vectors = np.linalg.eigh(covariance)
    if not (values[1] > 1.e-5 and values[0] <= cfg['normal_max_thickness_m']**2
            and values[0] <= cfg['normal_max_variance_ratio']*values[1]):
        return None
    tangent = vectors[:, 1]
    if tangent[0] < 0. or (tangent[0] == 0. and tangent[1] < 0.):
        tangent = -tangent
    return origin, tangent, np.array([-tangent[1], tangent[0]])


def _ordered_band(lower, upper, minimum, maximum):
    """Exact positive width over the finite binary64 domain, including vertex."""
    delta = [Fraction(float(a))-Fraction(float(b)) for a, b in zip(upper, lower)]
    lo, hi = Fraction(float(minimum)), Fraction(float(maximum))
    candidates = [lo, hi]
    if delta[2] != 0:
        vertex = -delta[1]/(2*delta[2])
        if lo <= vertex <= hi:
            candidates.append(vertex)
    return all(delta[0]+x*(delta[1]+x*delta[2]) > 0 for x in candidates)


def _pixel_candidates(pixel_xy, ordered, ordered_x, center, radius):
    """Conservative axis box; final original squared-distance test is separate.

    Twice the radius avoids a rounded box edge excluding a pixel admitted by
    the original floating squared distance. Preserve original raster order.
    This index never changes the disk, labels, seed order or fitting radii.
    """
    lower = np.searchsorted(ordered_x, center[0]-2.*radius, side='left')
    upper = np.searchsorted(ordered_x, center[0]+2.*radius, side='right')
    ids = ordered[lower:upper]
    ids = ids[np.abs(pixel_xy[ids, 1]-center[1]) <= 2.*radius]
    return np.sort(ids)


def island_ribbon_proposals(active, project, points, segments, white_pixels,
                           black_pixels, white_xy, black_xy, stable, contour_ids,
                           original_direction_valid, cfg, normal_cosine, intersect):
    """Return disjoint proposals for original faces; all topology is read-only.

    Deterministic near-to-far computational patches cover direction-invalid
    faces. Each seed tries only the existing radii, smallest first. A failed
    patch leaves the legacy estimate intact; no points/foreground mass are
    dropped. Each accepted side needs six real stable faces with the original
    PCA gates, from at least two original contours. Measured face tangents
    within the existing angle nominate side samples, not a physical cap proof.
    Nontraining raster faces may receive a proposal ONLY in their own original
    white/black bracket, whose two labels both participated in this band.

    These patch domains constrain computation, not observed map extent. No
    projection between disconnected pixels is exported or given a vote.
    """
    pending = stable & ~original_direction_valid
    if not np.any(pending) or len(np.unique(contour_ids[stable])) < 2:
        return []
    height, width = active.shape
    rows, columns = np.indices(active.shape)
    pixels = np.column_stack((columns.ravel(), rows.ravel()))
    pixel_xy, pixel_good = project(pixels)
    labels = active.ravel()
    usable = np.flatnonzero(pixel_good)
    ordered = usable[np.argsort(pixel_xy[usable, 0], kind='stable')]
    ordered_x = pixel_xy[ordered, 0]
    pixel_slot = np.full(len(pixels), -1, dtype=np.int64)
    last_candidates = np.empty(0, dtype=np.int64)
    white_ids = white_pixels[:, 1]*width+white_pixels[:, 0]
    black_ids = black_pixels[:, 1]*width+black_pixels[:, 0]
    face_delta = segments[:, 1]-segments[:, 0]
    face_length = np.linalg.norm(face_delta, axis=1)
    good_face = stable & np.isfinite(face_length) & (face_length > 0.)
    face_tangent = np.zeros_like(face_delta)
    face_tangent[good_face] = face_delta[good_face]/face_length[good_face, None]
    radii, radius = [], cfg['normal_radius_m']
    while radius < cfg['normal_max_radius_m']:
        radii.append(radius)
        radius = min(2.*radius, cfg['normal_max_radius_m'])
    radii.append(radius)
    claimed = np.zeros(len(points), dtype=bool)
    seeds = np.flatnonzero(pending)
    seeds = seeds[np.argsort(np.sum(points[seeds]*points[seeds], axis=1), kind='stable')]
    result = []
    for seed in seeds:
        if not pending[seed] or claimed[seed]:
            continue
        center = white_xy[seed]
        candidates = _pixel_candidates(pixel_xy, ordered, ordered_x, center, radii[-1])
        if not len(candidates):
            continue
        pixel_slot[last_candidates] = -1
        pixel_slot[candidates] = np.arange(len(candidates))
        last_candidates = candidates
        white_slots, black_slots = pixel_slot[white_ids], pixel_slot[black_ids]
        candidate_xy, candidate_labels = pixel_xy[candidates], labels[candidates]
        with np.errstate(invalid='ignore', over='ignore'):
            pixel_distance2 = np.sum((candidate_xy-center)**2, axis=1)
            face_distance2 = np.sum((points-center)**2, axis=1)
        # Bound repeated failed work without deleting a legacy observation.
        pending[face_distance2 <= radii[0]**2] = False
        for radius in radii:
            disk = pixel_distance2 <= radius*radius
            whites = np.flatnonzero(disk & candidate_labels)
            chart = _pca(candidate_xy[whites], cfg)
            if chart is None:
                continue
            origin, tangent, normal = chart
            local = (candidate_xy-origin)@np.column_stack((tangent, normal))
            smin, smax = np.min(local[whites, 0]), np.max(local[whites, 0])
            domain = disk & (local[:, 0] >= smin) & (local[:, 0] <= smax)
            blacks = np.flatnonzero(domain & ~candidate_labels)
            if (not len(blacks) or len(whites)+len(blacks) > 200000
                    or np.any(local[blacks, 1] == 0.)):
                continue
            low_black = blacks[local[blacks, 1] < 0.]
            high_black = blacks[local[blacks, 1] > 0.]
            if not len(low_black) or not len(high_black):
                continue
            black_h = local[np.maximum(black_slots, 0), 1]
            face_region = (good_face & (white_slots >= 0) & (black_slots >= 0)
                           & domain[np.maximum(white_slots, 0)]
                           & domain[np.maximum(black_slots, 0)])
            parallel = face_region & (np.abs(face_tangent@tangent) >= normal_cosine)
            side_faces = [np.flatnonzero(parallel & (black_h < 0.)),
                          np.flatnonzero(parallel & (black_h > 0.))]
            side_pca = [_pca(points[ids], cfg) for ids in side_faces]
            if any(fit is None or len(np.unique(contour_ids[ids])) < 2
                   for fit, ids in zip(side_pca, side_faces)):
                continue
            in_labels = local[whites].copy()
            in_labels[:, 0] /= radius
            outside_low, outside_high = local[low_black].copy(), local[high_black].copy()
            outside_low[:, 0] /= radius
            outside_high[:, 0] /= radius
            negative_inside = in_labels*np.array([1., -1.])
            lower_fit = fit_label_curve(negative_inside, outside_low*np.array([1., -1.]))
            upper_fit = fit_label_curve(in_labels, outside_high)
            if lower_fit.coefficients is None or upper_fit.coefficients is None:
                continue
            lower, upper = -lower_fit.coefficients, upper_fit.coefficients
            lower_s, upper_s = smin/radius, smax/radius
            if not _ordered_band(lower, upper, lower_s, upper_s):
                continue
            ids = np.flatnonzero(face_region & ~claimed & ~original_direction_valid
                                 & (black_h != 0.))
            if not len(ids):
                continue
            lower_side = black_h[ids] < 0.
            # A computational patch is not permission to borrow direction
            # support from 2*r away. Every actual support face must also lie
            # within this target's original radius. Keep the patch fit, or
            # reject the target; never trim supports until a test passes.
            supported = np.zeros(len(ids), dtype=bool)
            for side, support in enumerate(side_faces):
                selected = np.flatnonzero(lower_side if side == 0 else ~lower_side)
                if len(selected):
                    delta = points[ids[selected], None, :]-points[support][None, :, :]
                    supported[selected] = np.all(np.sum(delta*delta, axis=2) <= radius*radius, axis=1)
            ids, lower_side = ids[supported], lower_side[supported]
            if not len(ids):
                continue
            origins = np.tile(origin, (len(ids), 1))
            normals = np.tile(normal, (len(ids), 1))
            coefficients = np.tile(upper, (len(ids), 1))
            bounds = np.tile([lower_s, upper_s], (len(ids), 1))
            normals[lower_side] *= -1.
            coefficients[lower_side] = lower*np.array([-1., 1., -1.])
            bounds[lower_side] = [-upper_s, -lower_s]
            positions, fitted, good = intersect(white_xy[ids], black_xy[ids], origins,
                normals, coefficients, np.full(len(ids), radius), bounds,
                np.ones(len(ids), dtype=bool))
            # The chart only parameterizes the polynomial. The published
            # direction and its angle gate come from the real side-face PCA.
            side_normals = np.asarray([fit[2] for fit in side_pca]).copy()
            for side in range(2):
                outward = -normal if side == 0 else normal
                if side_normals[side]@outward < 0.:
                    side_normals[side] *= -1.
            normals = side_normals[np.where(lower_side, 0, 1)]
            good &= np.sum(fitted*normals, axis=1) >= normal_cosine
            counts = np.where(lower_side, len(side_faces[0]), len(side_faces[1]))
            # Successful label model owns only these original two-label faces.
            # Original direction-valid faces are never claimed or overwritten.
            result.append((ids, normals, positions, fitted, good, counts, radius))
            claimed[ids] = True
            pending[ids] = False
            break
    return result
