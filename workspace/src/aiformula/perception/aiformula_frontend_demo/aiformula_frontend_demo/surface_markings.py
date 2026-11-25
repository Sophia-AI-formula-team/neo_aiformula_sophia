from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Stroke:
    label: int
    center: np.ndarray
    axis: np.ndarray
    length: float
    width: float
    polygon: np.ndarray

    @property
    def across_vehicle(self):
        return abs(self.axis[1]) >= .82


def road_on_both_sides(camera, paving, line, normal, width):
    offsets = np.array([-.24, -.10, .10, .24])
    offsets += np.sign(offsets) * width / 2
    ground = line[:, None, :] + offsets[None, :, None] * normal
    uv, depth = camera.ground_to_image(ground)
    finite = np.isfinite(uv).all(axis=-1) & (depth > .05)
    u, v = np.moveaxis(np.rint(np.where(finite[..., None], uv, -1)).astype(int), -1, 0)
    valid = finite & (u >= 0) & (u < camera.width) & (v >= 0) & (v < camera.height)
    found = np.zeros(valid.shape, dtype=bool)
    found[valid] = paving[v[valid], u[valid]]
    return found[:, :2].any(axis=1) & found[:, 2:].any(axis=1)


def pavement_strokes(candidate, paving, camera):
    _, labels, stats, _ = cv2.connectedComponentsWithStats(candidate.astype(np.uint8))
    strokes = []
    for index, (u, v, w, h, area) in enumerate(stats[1:], 1):
        if area < 12 or area > .09 * candidate.size or max(w, h) < 7:
            continue
        local = (labels[v:v+h, u:u+w] == index).astype(np.uint8)
        contours, _ = cv2.findContours(local, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        outline = max(contours, key=cv2.contourArea).reshape(-1, 2) + [u, v]
        # Fit physical paint dimensions, including the extent of a one-pixel stroke.
        corners = outline[:, None, :] + np.array([[-.5, -.5], [.5, -.5], [-.5, .5], [.5, .5]])
        try:
            ground = camera.image_to_ground(corners.reshape(-1, 2))
        except ValueError:
            continue
        if not np.isfinite(ground).all() or ground[:, 0].min() <= 0. or ground[:, 0].max() > 16.:
            continue
        rect = cv2.minAreaRect(ground.astype(np.float32))
        center = np.asarray(rect[0])
        length, width = rect[1]
        heading = rect[2]
        if length < width:
            length, width = width, length
            heading += 90
        if length > 8. or not .035 <= width <= .62:
            continue
        angle = np.deg2rad(heading)
        axis = np.array([np.cos(angle), np.sin(angle)])
        if length < max(.90, 3 * width):
            # A paint stroke cut off by the bottom of the image has an unknown
            # full length. Check the visible length and width along the road.
            if v+h != camera.height:
                continue
            if abs(axis[0]) < .82 and abs(axis[1]) >= .82:
                length, width = width, length
                axis = np.array([-axis[1], axis[0]])
            if abs(axis[0]) < .82 or length < .10 or not .035 <= width <= .28:
                continue
        normal = np.array([-axis[1], axis[0]])
        samples = center + np.linspace(-.40, .40, 9)[:, None] * length * axis
        if np.count_nonzero(road_on_both_sides(camera, paving, samples, normal, width)) < 5:
            continue
        strokes.append(Stroke(index, center, axis, length, width, cv2.boxPoints(rect)))
    return labels, strokes


def crosswalk_groups(strokes):
    """Group a repeated row of parallel bars, including lateral zebra layouts."""
    used = set()
    for seed in strokes:
        if seed.label in used:
            continue
        normal = np.array([-seed.axis[1], seed.axis[0]])
        aligned = []
        for stroke in strokes:
            delta = stroke.center - seed.center
            if (stroke.label not in used and abs(stroke.axis @ seed.axis) >= .965 and
                    .65 <= stroke.length / seed.length <= 1.55 and
                    .55 <= stroke.width / seed.width <= 1.82 and
                    abs(delta @ seed.axis) <= min(.65, .23 * seed.length) and abs(delta @ normal) <= 6.):
                aligned.append((float(delta @ normal), stroke))
        aligned.sort(key=lambda item: item[0])
        runs = []
        for item in aligned:
            if runs:
                prev_at, prev_stroke = runs[-1][-1]
                step = item[0] - prev_at
                gap = step - (prev_stroke.width + item[1].width) / 2
                previous_steps = np.diff([at for at, _ in runs[-1]])
                regular = not len(previous_steps) or .65 <= step / np.median(previous_steps) <= 1.55
                if .06 <= gap <= .85 and step <= 1.22 and regular:
                    runs[-1].append(item)
                    continue
            runs.append([item])
        for run in runs:
            if len(run) >= 3 and any(stroke.label == seed.label for _, stroke in run):
                group = [stroke for _, stroke in run]
                used.update(stroke.label for stroke in group)
                yield group
                break


def surface_markings(frame, road_support, obstacles, camera):
    """Find transverse paint and zebra regions in the current calibrated image."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    sat, value = hsv[:, :, 1], hsv[:, :, 2]
    local_light = cv2.medianBlur(value, 11)
    contrast = value.astype(np.int16) - local_light
    clear = obstacles == 0
    paving = (road_support != 0) & clear & (sat <= 110) & (value < 176)
    candidate = (sat <= 90) & clear & ((value >= 179) | ((value >= 130) & (contrast >= 25)))
    candidate[:int(np.ceil(camera.height * camera.roi_top_ratio))] = False
    labels, strokes = pavement_strokes(candidate, paving, camera)

    groups = list(crosswalk_groups(strokes))
    stripe_ids = {stroke.label for group in groups for stroke in group}
    transverse = [stroke for stroke in strokes if stroke.across_vehicle and stroke.label not in stripe_ids]
    longitudinal = [stroke for stroke in strokes if abs(stroke.axis[0]) >= .82 and
                    stroke.width <= .28 and stroke.label not in stripe_ids]
    markings = []
    for group in groups:
        polygon = cv2.convexHull(np.vstack([stroke.polygon for stroke in group])).reshape(-1, 2)
        markings.append(dict(kind="crosswalk", stripes=len(group), polygon_m=polygon.astype(float).round(3).tolist()))
    for stroke in transverse:
        ends = stroke.center + np.array([-.5, .5])[:, None] * stroke.length * stroke.axis
        markings.append(dict(kind="transverse", width_m=round(stroke.width, 3), points_m=ends.round(3).tolist()))

    classified = {"crosswalk": [stroke for group in groups for stroke in group],
                  "transverse": transverse, "longitudinal": longitudinal}
    masks = {kind: np.zeros(candidate.shape, np.uint8) for kind in classified}
    # Restore only visible paint pixels with nearby camera-supported pavement on both sides.
    for kind, candidates in classified.items():
        for stroke in candidates:
            vv, uu = np.nonzero(labels == stroke.label)
            xy = camera.image_to_ground(np.column_stack((uu, vv)))
            normal = np.array([-stroke.axis[1], stroke.axis[0]])
            on_axis = stroke.center + ((xy-stroke.center) @ stroke.axis)[:, None] * stroke.axis
            supported = road_on_both_sides(camera, paving, on_axis, normal, stroke.width)
            masks[kind][vv[supported], uu[supported]] = 1

    # Include mixed-color boundary pixels only next to supported paint and inside
    # the road segmentation. A missed raster edge can block the full vehicle strip.
    paint_edges = (road_support != 0) & clear & (sat <= 95) & (value >= 132) & (value < 187)
    claimed = np.logical_or.reduce(list(masks.values()))
    neighborhood = np.ones((3, 3), np.uint8)
    for mask in masks.values():
        if mask.any():
            edge = paint_edges & ~claimed & (cv2.dilate(mask, neighborhood) != 0)
            mask[edge] = 1
            claimed |= edge
    return masks, markings
