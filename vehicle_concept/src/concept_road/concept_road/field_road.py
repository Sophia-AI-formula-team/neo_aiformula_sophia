"""Sensor-supported road sections for the field camera, in the canonical ground view."""

from dataclasses import dataclass
import math

import cv2
import numpy as np

from .field_scan import _section
from .vision import CX, CY, FOCAL, HEIGHT


@dataclass
class FieldRoadResult:
    road: np.ndarray
    heading: np.ndarray
    guidance: np.ndarray
    reference: object
    audit: dict


def _transport(rows, speed, yaw, dt):
    """Transport the two measured sides independently, then use their common depth."""
    angle = yaw * dt
    dz = speed * dt * np.sinc(angle / np.pi)
    dx = speed * dt * .5 * angle * np.sinc(.5 * angle / np.pi) ** 2
    c, s = math.cos(angle), math.sin(angle)
    new = []
    for col in (1, 2):
        z = c * (rows[:, 0] - dz) + s * (rows[:, col] - dx)
        x = -s * (rows[:, 0] - dz) + c * (rows[:, col] - dx)
        if np.any(np.diff(z) <= 0):
            return None
        new.append((z, x))
    lo, hi = max(v[0][0] for v in new), min(v[0][-1] for v in new)
    z = np.unique(np.r_[lo, rows[(rows[:, 0] > lo) & (rows[:, 0] < hi), 0], hi])
    if hi <= lo or len(z) < 2:
        return None
    return np.column_stack((z, *(np.interp(z, depth, x) for depth, x in new)))


def _connect_edge_samples(current, edges, labels, seed, green):
    """Follow a measured verge through gaps, checking the actual image at each step."""
    for side in (0, 1):
        col = 1 + side
        direction = 1 if side == 0 else -1  # points towards the road
        anchor = current[edges[:, side]]
        if len(anchor) < 3 or np.ptp(anchor[:, 0]) < .7:
            continue
        for i, row in enumerate(current):
            if edges[i, side]:
                continue
            z = row[0]
            # Three close measured samples define a local tangent. No correction
            # far beyond the last measured edge or across a change in its direction.
            nearest = anchor[np.argsort(abs(anchor[:, 0] - z))[:5]]
            if min(abs(nearest[:, 0] - z)) > 2.1 or np.ptp(nearest[:, 0]) < .35:
                continue
            line = np.polynomial.polynomial.polyfit(nearest[:, 0], nearest[:, col], 1)
            if np.max(abs(nearest[:, col] - np.polynomial.polynomial.polyval(nearest[:, 0], line))) > .20:
                continue
            # Stored anchors are at the outside of legal paint. Compare colour
            # across the paint centre, then recover the outside edge exactly once.
            predicted = np.polynomial.polynomial.polyval(z, line) + direction*.055
            scale = FOCAL/z
            center = int(round(CX + scale*predicted))
            v = round(CY + HEIGHT*scale)
            pad, width = max(2, round(.07*scale)), max(3, round(.32*scale))
            reach = int(.27*scale)
            if not pad+width+reach < center < seed.shape[1]-pad-width-reach:
                continue
            # Compare patches beside the expected boundary instead of treating
            # an interrupted white line or a failed B section as the outer edge.
            us = np.arange(center-reach, center+reach+1)
            prefix = np.r_[0., np.cumsum(green[v], dtype=float)]
            left = (prefix[us-pad] - prefix[us-pad-width]) / width
            right = (prefix[us+pad+width] - prefix[us+pad]) / width
            inside, outside = (right, left) if side == 0 else (left, right)
            contrast = outside-inside
            scores = contrast - .13 * abs(us-center) / max(1, reach)
            plausible = (outside > .038) & (contrast > .029)
            if not np.any(plausible):
                continue
            best = int(np.argmax(np.where(plausible, scores, -np.inf)))
            u = int(us[best])
            interior = u + direction * np.arange(pad, max(pad+2, int(.9*scale)))
            interior = interior[(interior >= 0) & (interior < seed.shape[1])]
            if len(interior) < width or np.mean(seed[v, interior]) < .10:
                continue
            x = (u-CX)/scale - direction*.055
            if direction*(row[3-col]-x) <= .28:
                # The entire selected B island lies on the non-road side.
                # Reject the section instead of moving its opposite boundary.
                row[1:3] = np.nan
                continue
            row[col] = x
            labels[i][side] = 'connected_material_edge'
            edges[i, side] = True


class FieldRoad:
    """A fresh observer is needed for each run. Inputs contain no map or world pose."""

    max_memory_s = .65

    def __init__(self):
        self._history = []

    def update(self, image, raw, lane, road, heading, motion_speed, motion_yaw, dt, *, nearfield_rows=()):
        old = np.asarray(road, dtype=float)
        if old.size == 0:
            old = np.empty((0, 5))
        if old.ndim != 2 or old.shape[1] < 4 or (len(old) > 1 and np.any(np.diff(old[:, 0]) <= 0)):
            raise ValueError('expected depth-sorted road rows')
        if image.shape[:2] != raw.shape or raw.shape != lane.shape:
            raise ValueError('field road inputs must share the canonical camera grid')
        if not np.isfinite([motion_speed, motion_yaw, dt]).all() or not 0 < dt <= self.max_memory_s:
            self._history.clear()
        else:
            history = []
            for previous, age in self._history:
                if age + dt <= self.max_memory_s:
                    points = _transport(previous, motion_speed, motion_yaw, dt)
                    if points is not None:
                        history.append((points, age + dt))
            self._history = history

        # Canonical pixels already contain the real camera -> nominal ground warp.
        # Three rows suppress isolated paint pixels without erasing thin distant lines.
        raw_u8 = np.asarray(raw, dtype=np.uint8)
        paint_u8 = np.asarray(lane, dtype=np.uint8)
        vote = raw_u8.astype(np.float32)
        seed_rows = cv2.blur(vote, (3, 3)) >= .45
        # Small holes in a well supported body of B pixels are frequent under
        # headlamps. At near rows use a wider sample in camera pixels.
        near_image = slice(int(CY + HEIGHT * FOCAL / 2.5), None)
        near_vote = cv2.blur(vote[near_image], (39, 15))
        seed_rows[near_image] |= near_vote >= .45
        paint_rows = cv2.dilate(paint_u8, np.ones((3, 1), np.uint8)) != 0
        rgb = np.asarray(image, dtype=np.float32)
        green = (rgb[:, :, 1] - rgb[:, :, 2]) / np.maximum(rgb[:, :, 1] + rgb[:, :, 2], 32.)
        green = cv2.blur(green, (3, 3))
        blue = (rgb[:, :, 0] - rgb[:, :, 1]) / np.maximum(rgb[:, :, 0] + rgb[:, :, 1], 32.)
        blue = cv2.blur(blue, (3, 3))
        light = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        patch = np.s_[int(CY)+15::6, 12:-12:6]
        samples = green[patch][seed_rows[patch] & ~paint_rows[patch]]
        if len(samples) >= 30:
            # The B head may spill onto turf even in dark scenes. Compare to
            # pavement in the same exposure; a few green pixels do not open a lane.
            green_limit = max(.052, float(np.percentile(samples, 20)) + .057)
            turf = (cv2.blur(green, (13, 5)) > green_limit).astype(np.uint8)
            broad_turf = cv2.morphologyEx(turf, cv2.MORPH_OPEN, np.ones((5, 29), np.uint8))
            # A broad connected segmentation near the car is positive evidence.
            # In that case decide the outer edge at the paint/material transition
            # below, rather than removing dim, green-tinted asphalt pixel by pixel.
            near_coverage = cv2.blur(vote[near_image], (1, 11)).mean(axis=1)
            broad_turf[near_image] &= (near_coverage[:, None] < .52)
            seed_rows &= (broad_turf == 0) | paint_rows
            # When B calls a uniformly saturated green area road, there is
            # no asphalt colour anchor in this view. Do not learn turf as the
            # pavement reference just because it occupies most of the frame.
            if np.percentile(samples, 10) > .15:
                seed_rows &= green < .10
        seed_rows &= (light >= 9) & (light <= 248)
        measured, target, edges, explanations = [], [], [], []
        previous_u = CX
        depths = np.linspace(.95, 12., 45)
        for z in depths:
            v = round(CY + HEIGHT * FOCAL / z)
            if not 0 <= v < len(raw_u8):
                continue
            row_seed = seed_rows[v]
            if z <= 2.3:
                # A water drop or a dark patch may split just the nearest scan
                # into several islands. Close only holes bracketed by current B
                # pixels and supported at the same lateral position on the next
                # visible ground slice. Exterior paint still wins afterwards.
                dv = .25
                next_v = round(CY + HEIGHT*FOCAL/(z+dv))
                us = np.arange(len(row_seed))
                next_u = np.rint(CX + (us-CX)*z/(z+dv)).astype(int)
                across = cv2.morphologyEx(row_seed[None, :].astype(np.uint8), cv2.MORPH_CLOSE,
                             np.ones((1, max(3, int(.70*FOCAL/z)) | 1), np.uint8),
                             borderType=cv2.BORDER_CONSTANT, borderValue=0)[0]
                row_seed = row_seed | ((across != 0) & seed_rows[next_v, next_u] &
                                       (light[v] > 10) & (light[v] < 245))
            result = _section(z, row_seed, paint_rows[v], green[v], blue[v], light[v], previous_u)
            if result is None:
                continue
            bounds, sides, middle, sources = result
            measured.append((z, *bounds))
            target.append(middle)
            edges.append(sides)
            explanations.append(sources)
            previous_u = CX + middle * FOCAL / z
        current = np.array(measured, dtype=float).reshape(-1, 3)
        edges = np.array(edges, dtype=bool).reshape(-1, 2)
        if len(current):
            _connect_edge_samples(current, edges, explanations, seed_rows, green)
            valid = np.all(np.isfinite(current), axis=1)
            current, edges = current[valid], edges[valid]
            target = np.asarray(target)[valid]
            explanations = [label for label, ok in zip(explanations, valid) if ok]
        # Horizontal scanlines alone can latch onto an unrelated distant island.
        # Keep the closest continuous component, with at most one missing scanline.
        if len(current):
            breaks = np.flatnonzero(np.diff(current[:, 0]) > 2.01 * (depths[1]-depths[0]))
            if len(breaks):
                end = breaks[0] + 1
                current, edges, target, explanations = current[:end], edges[:end], target[:end], explanations[:end]
        visible_z = None
        if len(current) >= 3 and current[0, 0] <= 1.21:
            gaps = np.flatnonzero(np.diff(current[:, 0]) > 1.01*(depths[1]-depths[0]))
            last = int(gaps[0]) if len(gaps) else len(current)-1
            if last >= 1:
                visible_z = [float(current[0, 0]), float(current[last, 0])]
        audit = dict(source='field_camera', observed_rows=len(current), exterior_rows=np.sum(edges, axis=0).tolist(),
                     direct_span_m=current[[0, -1], 0].tolist() if len(current) else [],
                     visible_z=visible_z, observed_bounds=[[round(float(z), 2), *s]
                     for z, s in zip(current[:, 0], explanations)],
                     memory_rows=0, footprint_rows=0, local_rows=[])
        empty = FieldRoadResult(np.empty((0, 5)), np.empty(0), np.empty((0, 4)), None, audit)
        history = self._history[:]
        if len(current) >= 3:
            # Only directly observed sections refresh memory; never refresh transported rows.
            self._history.insert(0, (current.copy(), 0.))
            self._history = self._history[:16]
        else:
            current = np.empty((0, 3))
        if not len(current) and not history:
            return empty

        # A failed legacy material fit supplies no rows. Sensor support can
        # still produce a road on the ordinary camera sampling grid.
        z = old[:, 0] if len(old) else np.r_[np.arange(-.16, .95, .16), depths]
        upstream_history = {round(float(row['z']), 3) for row in nearfield_rows
                            if row.get('source') == 'historical_material_observation'}
        backwards = [row for row in nearfield_rows if float(row['z']) <= 0.]
        rear_observed = bool(backwards) and all(round(float(row['z']), 3) in upstream_history for row in backwards)
        result_rows = []
        result_sources = []
        for j, depth in enumerate(z):
            previous = next((segment for segment, age in history
                             if segment[0, 0] <= depth <= segment[-1, 0]), None)
            if len(current) and current[0, 0] <= depth <= current[-1, 0]:
                left, right = (np.interp(depth, current[:, 0], current[:, col]) for col in (1, 2))
                nearest_depth = float(np.min(np.abs(current[:, 0]-depth)))
                source = 'camera' if nearest_depth <= .51*(depths[1]-depths[0]) else 'camera_gap_bridge'
                sigma = .045 + .004 * max(0, depth)
            elif previous is not None:
                left, right = (np.interp(depth, previous[:, 0], previous[:, col]) for col in (1, 2))
                source = 'motion_memory'
                audit['memory_rows'] += 1
                sigma = .08
            elif len(old) and depth < .95 and (round(float(depth), 3) in upstream_history or
                                  (depth < -1.2 and rear_observed)):
                left, right = old[j, 1:3]
                sigma = max(old[j, 3], old[j, -1])
                source = 'upstream_motion_memory'
                audit['memory_rows'] += 1
            elif -.17 <= depth < .95 and len(current) and current[0, 0] <= 1.21:
                # Bridge only from the current contact patch to the nearest direct
                # section. The width may open on a straight. An unrelated fragment
                # from the upstream observer is not a hard physical bound.
                blend = np.clip(depth / current[0, 0], 0, 1)
                origin_left = min(-.42, max(-1., current[0, 1]))
                origin_right = max(.42, min(1., current[0, 2]))
                left = origin_left + blend * (current[0, 1] - origin_left)
                right = origin_right + blend * (current[0, 2] - origin_right)
                sigma = .05 + .025 * (1-blend)
                source = 'footprint_bridge'
                audit['footprint_rows'] += 1
            else:
                continue
            if left >= right:
                continue
            result_rows.append([depth, left, right, sigma, sigma])
            result_sources.append(source)
        if len(result_rows) < 3:
            return empty
        rows = np.asarray(result_rows)
        # Interpolation across a large unobserved depth gap would give it a false
        # clearance. Keep only the closest connected component of final sections.
        gaps = np.flatnonzero(np.diff(rows[:, 0]) > .55)
        if len(gaps):
            end = gaps[0] + 1
            rows, result_sources = rows[:end], result_sources[:end]
        if len(rows) < 3:
            return empty
        audit['local_rows'] = [[round(float(z), 2), source] for z, source in zip(rows[:, 0], result_sources)]
        visible = current if len(current) else history[0][0]
        mids = np.asarray(target) if len(current) else .5 * (visible[:, 1] + visible[:, 2])
        # Estimate local direction from the observed centre of the B lane. The
        # entire supported road, including a corroborated adjacent lane, remains legal.
        fit = visible[:, 0] <= min(8., visible[-1, 0])
        if fit.sum() < 3:
            return FieldRoadResult(rows, np.zeros(len(rows)), rows[:, :4].copy(), None, audit)
        zz, xx = visible[fit, 0], mids[fit]
        reference_source = 'field_camera_lane' if len(current) else 'field_motion_memory'
        if len(current):
            # The middle of a lane clipped by the image border rotates with
            # the camera's field of view. Prefer a continuous measured edge
            # for direction; B still identifies the occupied lane and offset.
            fits = []
            for side in (0, 1):
                support = edges[:, side] & fit
                if np.sum(support) < 6 or np.ptp(current[support, 0]) < 2.:
                    continue
                edge_z, edge_x = current[support, 0], current[support, 1+side]
                curve = np.polynomial.polynomial.polyfit(edge_z, edge_x, 2)
                for _ in range(2):
                    delta = abs(edge_x-np.polynomial.polynomial.polyval(edge_z, curve))
                    weight = np.minimum(1., .14/np.maximum(delta, 1e-6))
                    curve = np.polynomial.polynomial.polyfit(edge_z, edge_x, 2, w=weight)
                delta = abs(edge_x-np.polynomial.polynomial.polyval(edge_z, curve))
                inliers = delta < .20
                if inliers.sum() < 6 or np.mean(inliers) < .72:
                    continue
                e = current[support][inliers]
                own = np.asarray(target)[support][inliers]
                offset = float(np.median(own-e[:, 1+side]))
                score = len(e) * np.ptp(e[:, 0]) / (.12+np.median(delta[inliers]))
                fits.append((score, e[:, 0], e[:, 1+side]+offset))
            if fits:
                _, zz, xx = max(fits, key=lambda item: item[0])
                reference_source = 'field_camera_edge_tangent'
        coef = np.polynomial.polynomial.polyfit(zz, xx, 2)
        residual = xx - np.polynomial.polynomial.polyval(zz, coef)
        keep = abs(residual - np.median(residual)) <= max(.15, 2.8 * np.median(abs(residual)))
        if keep.sum() >= 3:
            coef = np.polynomial.polynomial.polyfit(zz[keep], xx[keep], 2)
        angle = np.arctan(coef[1] + 2 * coef[2] * np.clip(rows[:, 0], zz[0], zz[-1]))
        points_z = np.linspace(zz[0], zz[-1], int(np.ceil((zz[-1] - zz[0])/.08)) + 1)
        ref = dict(points=np.c_[points_z, np.polynomial.polynomial.polyval(points_z, coef)],
                   source=reference_source, coef=coef,
                   sigma=max(.07, float(np.median(abs(residual)))), support=[zz[0], zz[-1]],
                   rule='Measured lane direction over observed distance; any supported lane is legal.')
        return FieldRoadResult(rows, angle, rows[:, :4].copy(), ref, audit)
