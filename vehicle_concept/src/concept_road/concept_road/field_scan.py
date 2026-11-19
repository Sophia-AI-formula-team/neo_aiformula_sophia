"""Interpret the observed paint and pavement on one horizontal camera scanline."""
import math

import cv2
import numpy as np

from .vision import CX, FOCAL


def _runs(mask):
    changes = (np.flatnonzero(mask[1:] != mask[:-1]) + 1).tolist()
    if mask[0]:
        changes.insert(0, 0)
    if mask[-1]:
        changes.append(len(mask))
    return [(a, end - 1) for a, end in zip(changes[::2], changes[1::2])]


def _prefix(row):
    output = np.empty(len(row) + 1, dtype=np.float64)
    output[0] = 0.
    np.cumsum(row, dtype=np.float64, out=output[1:])
    return output


def _mean_range(prefix, a, b):
    """Prefix sums allow material comparisons even when markings are wide."""
    a = max(0, a)
    b = min(len(prefix) - 1, b)
    return (prefix[b] - prefix[a]) / (b - a) if b > a else np.nan


def _asphalt_end(first, direction, step, paint, brightness, green_sum, blue_sum,
                 inside_green, inside_blue, base_light):
    """Visit each patch outwards, keeping only a contiguous extension beyond the paint."""
    width = len(paint)
    last = -1
    low_light = max(14., .52 * base_light)
    high_light = min(240., 1.8 * base_light + 10.)
    v = first
    while 0 <= v < width:
        end = v + direction * step
        lo, hi = min(v, end), max(v, end)
        if lo < 0 or hi >= width:
            break
        if (abs((green_sum[hi] - green_sum[lo]) / step - inside_green) > .036 or
                abs((blue_sum[hi] - blue_sum[lo]) / step - inside_blue) > .045):
            break
        total = 0
        for i in range(lo, hi):
            if paint[i]:
                return last
            total += int(brightness[i])
        if not low_light < total / step < high_light:
            break
        last = v
        v = end
    return last


def _section(z, seed, paint, green, blue, brightness, expected_u):
    scale = FOCAL / z
    # Join small segmentation holes and paint interruptions, but never whole lanes.
    gap = max(3, round(.18 * scale)) | 1
    joined = cv2.morphologyEx(seed[None, :].astype(np.uint8), cv2.MORPH_CLOSE,
                              np.ones((1, gap), np.uint8))[0]
    spans = [(a, b) for a, b in _runs(joined) if b - a >= max(4, .28 * scale)]
    if not spans:
        return None
    # A disconnected prediction on grass must not displace the main lane.
    a, b = max(spans, key=lambda ab: ab[1] - ab[0] -
               .45 * max(ab[0] - expected_u, expected_u - ab[1], 0))
    middle = .5 * (a + b)
    # Green excess uses G-R, normalized by local brightness. Night lighting can
    # make both asphalt and turf green. Compare across each marking in THIS row.
    prefix = _prefix(green)
    blue_prefix = _prefix(blue)
    seed_prefix = _prefix(seed)
    margin = max(2, round(.07 * scale))
    width = max(4, round(.27 * scale))
    bounds = [a, b]
    exterior = [False, False]
    sources = ['road_head', 'road_head']
    markings = list(_runs(paint))
    for side in (0, 1):
        direction = -1 if side == 0 else 1
        boundary = None
        extension = None
        for start, end in markings:
            u = .5 * (start + end)
            if direction * (u - middle) <= 0 or end - start > .55 * scale:
                continue
            left = (start - margin - width, start - margin)
            right = (end + margin + 1, end + margin + width + 1)
            inside, outside = (right, left) if side == 0 else (left, right)
            inside_green = _mean_range(prefix, *inside)
            outside_green = _mean_range(prefix, *outside)
            if not (math.isfinite(inside_green) and math.isfinite(outside_green)):
                continue
            inside_blue = _mean_range(blue_prefix, *inside)
            outside_blue = _mean_range(blue_prefix, *outside)
            outside_b = _mean_range(seed_prefix, *outside)
            inside_b = _mean_range(seed_prefix, *inside)
            green_transition = outside_green > .037 and outside_green - inside_green > .028
            color_transition = (outside_b < .22 and inside_b > .55 and
                                math.hypot(outside_green-inside_green, outside_blue-inside_blue) > .105)
            if not (green_transition or color_transition):
                # Crossing an apparent lane divider requires asphalt on both
                # sides and the divider at the edge of the segmented lane.
                if abs(u-bounds[side]) > max(gap, .3*scale) or inside_b < .5:
                    continue
                if (abs(outside_green-inside_green) > .032 or
                        abs(outside_blue-inside_blue) > .035):
                    continue
                # Stop at a colour/brightness change, another marking, black
                # image padding, or the last pixel actually measured.
                step = max(3, int(.14*scale))
                near_u = start if side == 0 else end
                u0 = near_u + direction*(margin+1)
                inner = slice(max(0,inside[0]),min(len(seed),inside[1]))
                base_light = float(np.mean(brightness[inner]))
                if not 17 < base_light < 237:
                    continue
                last = _asphalt_end(int(u0), direction, step, paint.view(np.uint8), brightness,
                                    prefix, blue_prefix, inside_green, inside_blue, base_light)
                if last >= 0:
                    extension = last
                continue
            # The asphalt immediately behind the paint must actually connect to
            # the predicted lane. A remote painted island is not an extension.
            if u < a or u > b:
                near = min(max(u, a), b)
                lo, hi = sorted((int(u) + direction * -margin, int(near)))
                gap_green = _mean_range(prefix, lo, hi)
                if abs(u-near) > 4.5*scale or (np.isfinite(gap_green) and
                                                gap_green > inside_green + .033):
                    continue
            # When B itself spans a marking, require visible material change.
            # When B ends before the marking, the connected asphalt check above applies.
            if inside_b < .12 and (a <= u <= b):
                continue
            if boundary is None or direction * (u - boundary) > 0:
                boundary = u
        if boundary is not None:
            # Legal contact with the outer edge of the paint is included.
            bounds[side] = boundary + direction * .055 * scale
            exterior[side] = True
            sources[side] = 'paint_material_edge'
        elif extension is not None and direction*(extension-bounds[side]) > 0:
            bounds[side] = extension
            sources[side] = 'paint_connected_asphalt'
    if bounds[1] - bounds[0] < .28 * scale:
        return None
    x = (np.asarray(bounds) - CX) / scale
    lane_middle = (min(max(middle, bounds[0]), bounds[1]) - CX) / scale
    return x, exterior, lane_middle, sources
