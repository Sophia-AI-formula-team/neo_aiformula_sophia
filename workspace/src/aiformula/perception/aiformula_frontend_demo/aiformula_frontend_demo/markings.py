"""Classify longitudinal paint tracks from frame pixels and metric support."""
from dataclasses import dataclass
from itertools import combinations

import cv2
import numpy as np


def spans(row):
    changes = np.diff(np.r_[False, row, False].astype(np.int8))
    return list(zip(np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)))


def classify_paint(visible, present, spacing):
    """Treat an occlusion as missing evidence, never as a paint gap."""
    visible = np.asarray(visible, dtype=bool)
    present = np.asarray(present, dtype=bool) & visible
    if len(present) >= 3:
        a = present.copy()
        a[1:-1] |= present[:-2] & present[2:] & visible[1:-1]
        present = a
    runs = [(i, j) for i, j in spans(present) if (j-i)*spacing >= .50]
    # Require two actual painted runs, not merely a break in the model's output.
    for (_, a), (b, _) in zip(runs, runs[1:]):
        gap = visible[a:b] & ~present[a:b]
        if (b-a)*spacing >= .75 and len(gap) and np.all(gap):
            return "dashed", present
    # One visible painted run can be one segment of a long dash.
    return "unknown", present


def link_tracks(paint, drive, lateral, spacing):
    """Join narrow detections across forward scans with a gated lateral prediction."""
    tracks = []
    for i, (row, road) in enumerate(zip(paint, drive)):
        near_road = cv2.dilate(road[None].astype(np.uint8), np.ones((1, 11), np.uint8))[0]
        hits = [(a+b-1)//2 for a, b in spans(row)
                if 0.05 <= lateral[min(b, len(lateral)-1)]-lateral[a] <= .48
                and near_road[(a+b-1)//2]]
        available = set(range(len(hits)))
        links = []
        for idx, samples in enumerate(tracks):
            last_i, y = samples[-1]
            gap = (i-last_i)*spacing
            if not 0 < gap <= 4.5:
                continue
            if len(samples) >= 4:
                before_i, before_y = samples[-4]
                slope = np.clip((y-before_y)/((last_i-before_i)*spacing), -.40, .40)
            else:
                slope = 0.
            predicted = y + slope*gap
            for j, column in enumerate(hits):
                error = abs(lateral[column]-predicted)
                if error <= min(.72, .21+.15*gap) and abs(lateral[column]-y) <= .25+.55*gap:
                    links.append((error+.04*gap, idx, j))
        seen = set()
        for _, idx, j in sorted(links):
            if idx not in seen and j in available:
                tracks[idx].append((i, lateral[hits[j]]))
                seen.add(idx)
                available.remove(j)
        tracks.extend([[(i, lateral[hits[j]])] for j in sorted(available)])
    return tracks


def longitudinal_contrast(pixels, drive, valid, blocked):
    """Find locally brighter narrow paint between supported pavement samples."""
    light = cv2.cvtColor(pixels, cv2.COLOR_BGR2GRAY).astype(np.int16)
    sat = cv2.cvtColor(pixels, cv2.COLOR_BGR2HSV)[:, :, 1]
    usable = drive & valid & ~blocked
    pad = 7
    paved = np.pad(usable, ((0, 0), (pad, pad)))
    tones = np.pad(light, ((0, 0), (pad, pad)), constant_values=255)
    width = light.shape[1]

    # Probe both sides of the stripe at about 0.3 m on the projected grid.
    def flank(offsets):
        samples = [np.where(paved[:, pad+d:pad+d+width], tones[:, pad+d:pad+d+width], 255)
                   for d in offsets]
        return np.minimum.reduce(samples)

    brighter_than = np.maximum(flank((-7, -6, -5)), flank((5, 6, 7)))
    return valid & ~blocked & (sat <= 100) & (light >= 82) & (light-brighter_than >= 18)


@dataclass
class Marking:
    kind: str
    rows: np.ndarray
    lateral: np.ndarray
    present: np.ndarray
    side: int

    def detail(self, x):
        return dict(kind=self.kind, side="left" if self.side > 0 else "right",
                    first_x_m=round(float(x[self.rows[0]]), 3),
                    last_x_m=round(float(x[self.rows[-1]]), 3),
                    painted_samples=int(self.present.sum()))


def marking_tracks(paint, drive, obstacle, valid, pixels, stations, lateral):
    """Trace paint supported by segmentation or local contrast along the current image."""
    local_paint = longitudinal_contrast(pixels, drive, valid, obstacle)
    paint = paint | local_paint
    spacing = float(stations[1]-stations[0])
    dy = float(lateral[1]-lateral[0])
    light = cv2.cvtColor(pixels, cv2.COLOR_BGR2GRAY).astype(np.float32)
    hsv = cv2.cvtColor(pixels, cv2.COLOR_BGR2HSV)
    n = len(lateral)
    pad = max(4, round(.45/dy))
    results = []
    for track in link_tracks(paint, drive, lateral, spacing):
        if len(track) < 6:
            continue
        i, ys = np.asarray(track).T
        i = i.astype(int)
        if stations[i[-1]]-stations[i[0]] < 2.50:
            continue
        xs = stations[i]
        fit = np.polyfit(xs, ys, min(2, len(xs)-1))
        errors = np.abs(np.polyval(fit, xs)-ys)
        if np.quantile(errors, .80) > .30:
            continue
        rows = np.arange(i[0], i[-1]+1)
        curve = np.polyval(fit, stations[rows])
        column = np.rint((curve-lateral[0])/dy).astype(int)
        good = (column >= pad) & (column < n-pad)
        safe = np.clip(column, pad, n-pad-1)
        offsets = np.arange(-2, 3)
        target = safe[:, None]+offsets
        flank = safe[:, None]+np.r_[np.arange(-pad, -4), np.arange(5, pad+1)]
        if flank.shape[1] == 0:
            continue
        ix = rows[:, None]
        nearby_drive = np.any(drive[ix, flank], axis=1) | np.any(drive[ix, target], axis=1)
        seen = good & valid[rows, safe] & nearby_drive & ~np.any(obstacle[ix, target], axis=1)
        track_light = light[ix, target]
        near_light = np.median(light[ix, flank], axis=1)
        contrast = track_light-near_light[:, None]
        hue, chroma = hsv[ix, target, 0], hsv[ix, target, 1]
        white = ((track_light >= 95) & (contrast >= 15) & (chroma < 108)) | local_paint[ix, target]
        yellow = (hue >= 13) & (hue <= 39) & (chroma >= 60) & (contrast >= 7)
        # Limit broad light transitions such as shadows or highlighted pavement.
        flank_spread = np.abs(np.median(light[ix, flank[:, :flank.shape[1]//2]], axis=1) -
                              np.median(light[ix, flank[:, flank.shape[1]//2:]], axis=1))
        count = np.count_nonzero(white | yellow, axis=1)
        present = seen & (count >= 1) & ((flank_spread < 65) | (count >= 2))
        kind, present = classify_paint(seen, present, spacing)
        # Ignore weak longitudinal fragments as well as transverse ground paint.
        if present.sum() < 3 or (kind == "unknown" and present.sum()*spacing < .75):
            continue
        results.append(Marking(kind, rows, curve, present, 1 if curve[0] >= 0 else -1))
    return results


def extend_visible_white_tracks(tracks, valid, drive, obstacles, pixels, stations, lateral):
    """Grow a longitudinal observation only through consecutive visible paint."""
    light = cv2.cvtColor(pixels, cv2.COLOR_BGR2GRAY).astype(np.int16)
    chroma = cv2.cvtColor(pixels, cv2.COLOR_BGR2HSV)[:, :, 1]
    white = (light >= 100) & (chroma <= 95) & valid & ~obstacles
    dy = float(lateral[1]-lateral[0])
    dx = float(stations[1]-stations[0])
    narrow = []
    for i, mask in enumerate(white):
        hits = []
        for a, b in spans(mask):
            # A nearby wall, cone or opaque vehicle is not a supported paint stroke.
            if a < 3 or b+3 > len(lateral) or not .05 <= (b-a)*dy <= .40:
                continue
            flanks = np.r_[np.arange(a-3, a), np.arange(b, b+3)]
            if not drive[i, flanks].any():
                continue
            if np.median(light[i, a:b])-np.median(light[i, flanks]) >= 17:
                hits.append(float((lateral[a]+lateral[b-1])/2))
        narrow.append(hits)

    def grow(rows, values, direction):
        y = float(values[-1])
        slope = np.clip((y-values[0])/max(1, len(values)-1), -.25, .25)
        found = []
        for row in range(int(rows[-1])+direction, len(stations) if direction>0 else -1, direction):
            if not narrow[row]:
                break
            estimate = y+slope
            match = min(narrow[row], key=lambda value: abs(value-estimate))
            if abs(match-estimate) > .22 or abs(match-y) > max(.24, dx):
                break
            found.append((row, match))
            slope = .55*slope+.45*(match-y)
            y = match
        return found

    for track in tracks:
        matched = [narrow[i] and min(abs(c-y) for c in narrow[i]) <= .16
                   for i, y, presence in zip(track.rows, track.lateral, track.present) if presence]
        if not matched or sum(bool(m) for m in matched) < .55*len(matched):
            continue
        begin = grow(track.rows[:4][::-1], track.lateral[:4][::-1], -1)
        end = grow(track.rows[-4:], track.lateral[-4:], 1)
        if not begin and not end:
            continue
        additions = begin[::-1]+end
        # Interpolate internal observed paint centres and the measured extensions.
        old_rows, old_lateral = track.rows, track.lateral
        rows = np.r_[old_rows, [r for r,y in additions]].astype(int)
        ys = np.r_[old_lateral, [y for r,y in additions]]
        order = np.argsort(rows)
        track.rows, track.lateral = rows[order], ys[order]
        track.present = np.r_[track.present, np.ones(len(additions), bool)][order]


def marking_masks(tracks, stations, camera):
    masks = {key: np.zeros((camera.height, camera.width), np.uint8)
             for key in ("solid", "dashed", "unknown")}
    for track in tracks:
        xy = np.column_stack((stations[track.rows], track.lateral))
        uv, depth = camera.ground_to_image(xy)
        valid = np.isfinite(uv).all(axis=1) & (depth > .05)
        valid &= track.present
        for i, j in spans(valid):
            path = np.rint(np.clip(uv[i:j], -30000, 30000)).astype(np.int32)
            if len(path) == 1:
                cv2.circle(masks[track.kind], tuple(path[0]), 2, 1, -1)
            else:
                cv2.polylines(masks[track.kind], [path], False, 1, 4)
    return masks


def visible_dashed_pixels(frame, trace, camera):
    """Expand a confirmed dash across its visible width and remove bare pavement."""
    rows = np.flatnonzero(trace.any(axis=1))
    result = np.zeros_like(trace)
    if not len(rows):
        return result
    _, sat, value = cv2.split(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
    light = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)[rows]
    try:
        _, scale, _ = camera.image_row_calibration(rows)
    except ValueError:
        return trace & ((sat <= 100) & (value >= 179)).astype(np.uint8)
    # Use a wider horizontal sample in the near field to look past both edges.
    kernel_sizes = np.array([9, 17, 33, 65, 129, 257])
    bins = np.searchsorted(kernel_sizes, .65/np.maximum(scale, 1e-6)).clip(0, len(kernel_sizes)-1)
    for index in np.unique(bins):
        chosen = bins == index
        width = int(kernel_sizes[index])
        samples = light[chosen]
        base = cv2.morphologyEx(samples, cv2.MORPH_OPEN, np.ones((1, width), np.uint8))
        lateral_support = cv2.dilate(trace[rows[chosen]], np.ones((1, width//2+1), np.uint8)) != 0
        pixels = (samples >= 82) & (samples.astype(np.int16)-base >= 13)
        result[rows[chosen]] = lateral_support & pixels & (sat[rows[chosen]] <= 100)
    # Keep paint components attached to a traced stripe and reject isolated asphalt texture.
    _, labels, stats, _ = cv2.connectedComponentsWithStats(result)
    anchor = cv2.dilate(trace, np.ones((5, 9), np.uint8)) != 0
    linked = np.bincount(labels[anchor], minlength=len(stats)) >= 2
    linked[0] = False
    return (linked[labels] & (stats[labels, cv2.CC_STAT_AREA] >= 5)).astype(np.uint8)


def double_white_mask(paint, drive, obstacles, valid, pixels, stations, lateral, camera):
    """Find two near-parallel white strokes using a dark gap visible in the image."""
    mask = np.zeros((camera.height, camera.width), np.uint8)
    if not paint.any():
        return mask, 0
    dx = float(stations[1]-stations[0])
    dy = float(lateral[1]-lateral[0])
    light = cv2.cvtColor(pixels, cv2.COLOR_BGR2GRAY).astype(np.int16)
    saturation = cv2.cvtColor(pixels, cv2.COLOR_BGR2HSV)[:, :, 1]
    hint = cv2.dilate(paint.astype(np.uint8), np.ones((3, 13), np.uint8)) != 0
    white = (light >= 96) & (saturation <= 85) & valid & hint & ~obstacles
    tracks = [np.asarray(t, dtype=float) for t in link_tracks(white, drive, lateral, dx)
              if (t[-1][0]-t[0][0])*dx >= 3.0]

    pairs = 0
    for a, b in combinations(tracks, 2):
        rows, ia, ib = np.intersect1d(a[:, 0].astype(int), b[:, 0].astype(int),
                                     assume_unique=True, return_indices=True)
        if len(rows) < 2 or (rows[-1]-rows[0])*dx < 3.0:
            continue
        # One dash on each track cannot be joined across gaps in either stroke.
        span = rows[-1]-rows[0]+1
        if len(rows) < .85*span or np.any(np.diff(rows)*dx > .50):
            continue
        paired = np.stack((a[ia, 1], b[ib, 1]))
        if np.median(paired[0]-paired[1]) > 0:
            paired = paired[::-1]
        distance = paired[1]-paired[0]
        if (np.any(distance < .16) or np.any(distance > .55) or
                np.percentile(distance, 90)-np.percentile(distance, 10) > .14):
            continue
        columns = np.rint((paired-lateral[0])/dy).astype(int)
        mid = np.rint(np.mean(columns, axis=0)).astype(int)
        side_brightness = np.minimum(light[rows, columns[0]], light[rows, columns[1]])
        dark_gap = valid[rows, mid] & ~white[rows, mid] & ((side_brightness-light[rows, mid]) >= 16)
        if np.mean(dark_gap) < .80:
            continue

        # Fill the interval between the two outer borders over their jointly
        # observed extent, including short raster gaps between samples.
        x = stations[rows]
        x_all = stations[rows[0]:rows[-1]+1]
        curves = np.stack([np.polyval(np.polyfit(x, y, 2), x_all) for y in paired])
        if np.any(curves[1]-curves[0] < .10):
            continue
        left = np.column_stack((x_all, curves[1]+.12))
        right = np.column_stack((x_all, curves[0]-.12))
        xy = np.vstack((left, right[::-1]))
        uv, depth = camera.ground_to_image(xy)
        if np.any(depth <= .05) or not np.isfinite(uv).all():
            continue
        pixels_on_screen = np.rint(np.clip(uv, -30000, 30000)).astype(np.int32)
        cv2.fillPoly(mask, [pixels_on_screen], 1)
        pairs += 1
    return mask, pairs
