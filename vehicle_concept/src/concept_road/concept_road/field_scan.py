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


def _pavement_strip(lo, hi, step, paint, brightness, green_sum, blue_sum, base_g, base_b, light):
    """Check every short block between two observed stripes, including the last one."""
    count = hi - lo
    while lo < hi:
        end = min(hi, lo + step)
        n = end-lo
        if (abs((green_sum[end]-green_sum[lo])/n-base_g) > .036 or
                abs((blue_sum[end]-blue_sum[lo])/n-base_b) > .045):
            return False
        total = 0
        for j in range(lo, end):
            if paint[j] or brightness[j] < 9:
                return False
            total += int(brightness[j])
        if not max(14., .52*light) < total/n < min(240., 1.8*light+10):
            return False
        lo = end
    return count > 0


def _two_lane_span(z, side, near, u, last, ordered_x, markings, seed, paint, brightness, g, b):
    """Verify a tracked divider and far verge against the current image."""
    if ordered_x is None:
        return False
    scale = FOCAL/z
    exterior = ordered_x[2*side]
    divider = ordered_x[1]
    if not (math.isfinite(exterior) and math.isfinite(divider)):
        return False
    direction = 2*side-1
    if abs(u-(CX+scale*exterior)) > .20*scale:
        return False
    lo, hi = sorted((near,u))
    separating = [(a,h) for a,h in markings if lo < (a+h)*.5 < hi and abs((a+h)*.5-u) > .15*scale]
    if len(separating) != 1:
        return False
    a,h = separating[0]
    if h-a > .30*scale or abs((a+h)*.5-(CX+scale*divider)) > .20*scale:
        return False
    step, pad = max(3,round(.14*scale)), max(2,round(.055*scale))
    # Check the two material strips independently, from the seed to the
    # divider and from the divider to the outer paint. Only recognized paint
    # may interrupt the pavement. Require B support beyond the divider.
    interior = int(near)-direction*step
    if not (0 <= interior < len(seed) and 0 <= last < len(seed)):
        return False
    first_strip, second_strip = ((interior,a-pad),(h+pad+1,last)) if side else (
                                (h+pad+1,interior+1),(last,a-pad))
    if min(first_strip[1]-first_strip[0], second_strip[1]-second_strip[0]) < step:
        return False
    if np.count_nonzero(seed[second_strip[0]:second_strip[1]]) < .28*scale:
        return False
    low, high = sorted((int(near)-direction*step,int(near)))
    light = float(np.mean(brightness[low:high]))
    # Dilation can absorb black image padding or shadow along a strip just beside
    # the divider. The excluded stripe itself still needs valid RGB.
    guard_lo, guard_hi = max(0,a-pad), min(len(seed),h+pad+1)
    if np.min(brightness[guard_lo:guard_hi]) < 9:
        return False
    if not 17. < light < 237.:
        return False
    base_g, base_b = _mean_range(g,low,high), _mean_range(b,low,high)
    return (_pavement_strip(first_strip[0],first_strip[1],step,paint.view(np.uint8),brightness,g,b,base_g,base_b,light) and
            _pavement_strip(second_strip[0],second_strip[1],step,paint.view(np.uint8),brightness,g,b,base_g,base_b,light))


def _section(z, seed, paint, green, blue, brightness, expected_u, ordered_x=None, partial_bounds=None):
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
        across_divider = False
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
            across = False
            if u < a or u > b:
                near = min(max(u, a), b)
                lo, hi = sorted((int(u) + direction * -margin, int(near)))
                gap_green = _mean_range(prefix, lo, hi)
                if np.isfinite(gap_green) and gap_green > inside_green + .033:
                    continue
                if abs(u-near) > 4.5*scale:
                    last = (start-margin) if side else (end+margin+1)
                    if not green_transition or not _two_lane_span(z, side, near, u, last, ordered_x, markings, seed,
                                                                  paint, brightness, prefix, blue_prefix):
                        continue
                    across = True
            # When B itself spans a marking, require visible material change.
            # When B ends before the marking, the connected asphalt check above applies.
            if inside_b < .12 and (a <= u <= b):
                continue
            if boundary is not None and across and direction*(u-boundary) > 0:
                continue
            if boundary is None or direction * (u - boundary) > 0 or across_divider and not across:
                boundary, across_divider = u, across
        if boundary is not None:
            # Legal contact with the outer edge of the paint is included.
            bounds[side] = boundary + direction * .055 * scale
            exterior[side] = True
            sources[side] = 'two_lane_material_edge' if across_divider else 'paint_material_edge'
        elif extension is not None and direction*(extension-bounds[side]) > 0:
            bounds[side] = extension
            sources[side] = 'paint_connected_asphalt'
        # A cropped verge can prevent a colour comparison. Propose only the
        # farthest *observed B pixel* in a connected patch past the divider.
        # The caller still requires two true material edges in nearby scans.
        if not exterior[side] and partial_bounds is not None and ordered_x is not None:
            outer_u = CX + scale*ordered_x[side*2]
            divider_u = CX + scale*ordered_x[1]
            if not (math.isfinite(outer_u) and math.isfinite(divider_u)):
                continue
            for start, end in markings:
                u = .5*(start+end)
                if abs(u-outer_u) > .20*scale or direction*(u-divider_u) <= 0:
                    continue
                for first, last in spans:
                    near_patch, far_patch = (first,last) if side else (last,first)
                    if (abs(near_patch-divider_u) > .25*scale or
                            direction*(far_patch-bounds[side]) <= 0 or
                            direction*(u-far_patch) < .12*scale):
                        continue
                    if _two_lane_span(z, side, (b if side else a), u, far_patch, ordered_x,
                                      markings, seed, paint, brightness, prefix, blue_prefix):
                        partial_bounds[side] = (far_patch-CX)/scale
                        break
    if bounds[1] - bounds[0] < .28 * scale:
        return None
    x = (np.asarray(bounds) - CX) / scale
    lane_middle = (min(max(middle, bounds[0]), bounds[1]) - CX) / scale
    return x, exterior, lane_middle, sources
