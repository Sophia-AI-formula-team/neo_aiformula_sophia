"""Stage 1: infer semantic masks from camera pixels and project the corridor."""
import cv2
import numpy as np

from optimization_oa.geometry import GroundCamera
from .common import LANE_KEYS, encode_mask
from .markings import double_white_mask, extend_visible_white_tracks, marking_masks, marking_tracks, spans, visible_dashed_pixels
from .surface_markings import surface_markings

RULE_SET = "single-frame-ground-markings-v12"


def bottom_edge_candidates(road_margin):
    """Weak, compact holes at the bottom of the 640-pixel model view."""
    weak = np.zeros(road_margin.shape, dtype=bool)
    depth = min(6, road_margin.shape[0])
    if not np.any((road_margin[-depth:] <= 0) & (road_margin[-depth:] >= -.025)):
        return weak
    road = road_margin > 0
    height, width = road.shape
    _, labels, stats, _ = cv2.connectedComponentsWithStats((~road).astype(np.uint8))
    kernel = np.ones((5, 5), np.uint8)
    for index, (x, y, w, h, area) in enumerate(stats[1:], 1):
        if y < height-depth or y+h != height or area > 80 or x == 0 or x+w == width:
            continue
        component = labels == index
        rim = (cv2.dilate(component.astype(np.uint8), kernel) != 0) & ~component
        if np.count_nonzero(road & rim) >= .90*np.count_nonzero(rim):
            margins = road_margin[component]
            # A borderline fragment can flag a compact component for
            # re-inference even when some of its pixels scored lower.
            if margins.max() >= -.025 and margins.min() >= -.25:
                weak |= component
            else:
                weak |= component & (road_margin >= -.025)
    return weak


class Yolop:
    def __init__(self, model_path, score=.30, threads=3, provider="auto"):
        import onnxruntime as ort
        available = ort.get_available_providers()
        use_dml = provider == "dml" or (provider == "auto" and "DmlExecutionProvider" in available)
        if provider not in ("auto", "cpu", "dml") or (use_dml and "DmlExecutionProvider" not in available):
            raise ValueError(f"Unavailable ONNX Runtime provider {provider!r}: {available}")
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        options.enable_mem_pattern = not use_dml
        providers = ["DmlExecutionProvider", "CPUExecutionProvider"] if use_dml else ["CPUExecutionProvider"]
        self.session = ort.InferenceSession(str(model_path), options, providers=providers)
        self.provider = self.session.get_providers()[0]
        if provider == "dml" and self.provider != "DmlExecutionProvider":
            raise RuntimeError("ONNX Runtime could not initialize DirectML")
        self.input = self.session.get_inputs()[0].name
        self.score = score

    def _predict(self, canvas, outputs):
        image = canvas.astype(np.float32)/255
        image -= np.array([.485, .456, .406], np.float32)
        image /= np.array([.229, .224, .225], np.float32)
        inputs = {self.input: np.ascontiguousarray(image.transpose(2, 0, 1)[None])}
        return self.session.run(outputs, inputs)

    def infer(self, bgr):
        h, w = bgr.shape[:2]
        scale = min(640/w, 640/h)
        sw, sh = round(w*scale), round(h*scale)
        left, top = (640-sw)//2, (640-sh)//2
        canvas = np.full((640, 640, 3), 114, np.uint8)
        resized = cv2.resize(bgr, (sw, sh), interpolation=cv2.INTER_AREA)
        canvas[top:top+sh, left:left+sw] = resized[:, :, ::-1]
        det, drivable, marking = self._predict(canvas, ["det_out", "drive_area_seg", "lane_line_seg"])
        bottom, right = top+sh, left+sw
        raw_road = drivable[0, 1, top:bottom, left:right] - drivable[0, 0, top:bottom, left:right]
        raw_marking = marking[0, 1, top:bottom, left:right] > marking[0, 0, top:bottom, left:right]
        recovery = bottom_edge_candidates(raw_road) if bottom < canvas.shape[0] else np.zeros_like(raw_marking)
        if recovery.any():
            # Re-evaluate only the proposed fragments. Keep the original scores elsewhere.
            mirrored = cv2.copyMakeBorder(canvas[top:bottom, left:right], 0, canvas.shape[0]-bottom,
                                          0, 0, cv2.BORDER_REFLECT_101)
            canvas[bottom:, left:right] = mirrored[sh:]
            alternative, = self._predict(canvas, ["drive_area_seg"])
            agreement = (alternative[0, 1, top:bottom, left:right] -
                         alternative[0, 0, top:bottom, left:right]) >= .25
            recovery &= agreement
        masks = [cv2.resize(mask.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST)
                 for mask in (raw_road > 0, raw_marking, recovery)]
        rows = det.reshape(-1, det.shape[-1])
        class_score = rows[:, 5:].max(axis=1)
        scores = rows[:, 4] * class_score
        valid = scores >= self.score
        if not valid.any():
            return masks[0], masks[1], [], masks[2]
        rows, scores = rows[valid, :4], scores[valid]
        scale_xy = np.array((sw/w, sh/h), dtype=float)
        centers = (rows[:, :2] - [left, top])/scale_xy
        wh = rows[:, 2:4]/scale_xy
        xy0 = np.maximum(centers - wh/2, 0.)
        xy1 = np.minimum(centers + wh/2, [w-1, h-1])
        boxes = np.column_stack((xy0, np.maximum(0., xy1 - xy0)))
        keep = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), self.score, .45)
        out = []
        for i in np.asarray(keep).reshape(-1).astype(int):
            if min(boxes[i, 2:]) < 3:
                continue
            out.append(dict(xyxy=np.r_[xy0[i], xy1[i]].round(2).tolist(),
                            score=round(float(scores[i]), 3), label="vehicle"))
        return masks[0], masks[1], out, masks[2]


def tolerant_center(xs, ys):
    fit = np.polyfit(xs, ys, min(2, len(xs)-1))
    residual = np.abs(np.polyval(fit, xs)-ys)
    keep = residual <= max(.24, 2.5*np.median(residual))
    if keep.sum() >= 5:
        fit = np.polyfit(xs[keep], ys[keep], min(2, keep.sum()-1))
    return np.clip(np.polyval(fit, xs), ys-.32, ys+.32)


def enclosed_asphalt_holes(frame, road, protected, camera=None):
    """Only recover speckles enclosed by pavement with matching local color."""
    h, w = road.shape
    holes = np.zeros_like(road, dtype=bool)
    _, labels, stats, _ = cv2.connectedComponentsWithStats((~road).astype(np.uint8))
    for index, (x, y, width, height, area) in enumerate(stats[1:], 1):
        if area > 24 or x == 0 or y == 0 or x+width >= w or y+height >= h:
            continue
        cols = slice(x-1, x+width+1)
        rows = slice(y-1, y+height+1)
        local = labels[rows, cols] == index
        blocked = protected[rows, cols]
        if np.any(local & blocked):
            continue
        if area > 12:
            # A few extra pixels are acceptable only when the calibrated
            # ground footprint is still a texture-sized speckle.
            if camera is None:
                continue
            corners = np.array([[x-.5, y-.5], [x+width-.5, y+height-.5],
                                [x+width-.5, y-.5], [x-.5, y+height-.5]])
            try:
                ground = camera.image_to_ground(corners)
            except ValueError:
                continue
            extent = np.ptp(ground, axis=0)
            if not np.isfinite(extent).all() or np.linalg.norm(extent) > .055:
                continue
        ring = (cv2.dilate(local.astype(np.uint8), np.ones((3, 3), np.uint8)) != 0) & ~local
        adjacent = road[rows, cols] & ~blocked
        if not ring.any() or not np.all(adjacent[ring]):
            continue
        colors = frame[rows, cols].astype(float)
        reference = np.median(colors[ring], axis=0)
        delta = colors[local]-reference
        worst = np.max(np.abs(delta), axis=1)
        if area <= 12:
            matches = np.max(worst) <= 20
        else:
            # Allow local texture variation in mostly matching dark pavement.
            matches = (np.percentile(worst, 85) <= 20 and np.max(worst) <= 40 and
                       np.max(colors[local]) <= 135)
        if matches:
            holes[rows, cols] |= local
    return holes


def asphalt_and_paint(frame, road_support, marking, masks, obstacles, camera):
    """Confirm dark asphalt and white pavement markings from the current RGB frame."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    sat, value = hsv[:, :, 1], hsv[:, :, 2]
    h, w = value.shape
    seed = np.s_[round(.68*h):, round(.22*w):round(.78*w)]
    patch = value[seed][road_support[seed] & ~obstacles[seed] & (sat[seed] <= 105) &
                        (value[seed] <= 170)]
    # Adapt within a fixed dark-to-medium-gray range for auto exposure and shade.
    brightness = float(np.clip(np.percentile(patch, 85)+28, 96, 170)) if len(patch) else 118.
    local_value = cv2.medianBlur(value, 11).astype(np.int16)
    relative_light = value.astype(np.int16)-local_value
    longitudinal = masks["solid"] | masks["dashed"] | masks["unknown"]
    painted_hint = (marking != 0) | (longitudinal != 0)
    # Mild bright-gray texture on asphalt is not itself white paint.
    strong_white = (sat <= 95) & (value >= 187)
    white = strong_white | ((sat <= 95) & (value >= max(135, brightness+3)) &
                            ((relative_light >= 18) | painted_hint))
    dark = ((cv2.medianBlur(value, 3) <= brightness) &
            (cv2.medianBlur(sat, 3) <= 105) & ~strong_white)
    asphalt = road_support & dark & ~obstacles
    tone_veto = ~road_support | obstacles | strong_white | (masks["double_white"] != 0)
    asphalt |= enclosed_asphalt_holes(frame, asphalt, tone_veto, camera)

    # A bright patch can be paint if marking evidence and pavement on both
    # sides support a thin strip at ground scale. A vehicle box has priority.
    confirmed = cv2.dilate(longitudinal, np.ones((5, 13), np.uint8)) != 0
    segmentation = cv2.dilate(marking.astype(np.uint8), np.ones((5, 9), np.uint8)) != 0
    evidence = confirmed | segmentation
    far_uv, depth = camera.ground_to_image(np.array([[18., -4.], [18., 4.]]))
    top = int(np.ceil(camera.height*camera.roi_top_ratio))
    if np.all(depth > .05) and np.isfinite(far_uv).all():
        top = max(top, int(np.ceil(far_uv[:, 1].max())))
    rows = np.arange(max(0, top), h)
    paint = np.zeros_like(asphalt)
    if not len(rows):
        return asphalt, paint, brightness
    try:
        _, scale, _ = camera.image_row_calibration(rows)
    except ValueError:
        return asphalt, paint, brightness
    columns = np.arange(w)
    paving = asphalt[rows]
    left = np.maximum.accumulate(np.where(paving, columns, -1), axis=1)
    right = np.minimum.accumulate(np.where(paving, columns, w)[:, ::-1], axis=1)[:, ::-1]
    bounded = (left >= 0) & (right < w) & ((right-left-1)*scale[:, None] <= .40)
    paint[rows] = white[rows] & evidence[rows] & bounded & ~obstacles[rows]
    return asphalt, paint, brightness


class CorridorExtractor:
    def __init__(self, near=2.0, far=13., lane_width=3.35):
        self.stations = np.arange(near, far+.01, .25)
        if len(self.stations) < 7 or lane_width <= 0:
            raise ValueError("invalid semantic grid or nominal lane width")
        self.lateral = np.arange(-8., 8.01, .055)
        forward, sideways = np.meshgrid(self.stations, self.lateral, indexing="ij")
        self.grid = np.stack((forward, sideways), axis=-1)
        self.lane_width = lane_width
        self.projection_key, self.uv = None, None

    def sample(self, camera):
        key = (camera.width, camera.height, camera.projection.tobytes(), camera.roi_top_ratio,
               camera.origin.tobytes(), camera.rotation.tobytes())
        if self.projection_key != key:
            pixels, depth = camera.ground_to_image(self.grid)
            valid = np.isfinite(pixels).all(axis=-1) & (depth > .05)
            u = np.rint(np.where(valid, pixels[..., 0], -1)).astype(int)
            v = np.rint(np.where(valid, pixels[..., 1], -1)).astype(int)
            valid &= (u >= 0) & (u < camera.width) & (v >= camera.height*camera.roi_top_ratio) & (v < camera.height)
            self.uv = u, v, valid
            self.projection_key = key
        return self.uv

    def extract(self, frame, road, marking, obstacles, camera, edge_support):
        ground_masks, ground_marks = surface_markings(frame, road, obstacles, camera)
        surface_paint = (ground_masks["transverse"] | ground_masks["crosswalk"]) != 0
        ground_longitudinal = ground_masks.pop("longitudinal") != 0
        u, v, valid = self.sample(camera)
        drive, paint, blocked = [np.zeros(valid.shape, bool) for _ in range(3)]
        drive[valid], paint[valid], blocked[valid] = (
            image[v[valid], u[valid]] != 0 for image in (road, marking, (obstacles != 0) | surface_paint))
        paint &= ~blocked
        pixels = np.zeros((*valid.shape, 3), np.uint8)
        pixels[valid] = frame[v[valid], u[valid]]
        supported_paint = paint.copy()
        supported_paint[valid] |= ground_longitudinal[v[valid], u[valid]]
        tracks = marking_tracks(supported_paint, drive, blocked, valid, pixels, self.stations, self.lateral)
        extend_visible_white_tracks(tracks, valid, drive, blocked, pixels, self.stations, self.lateral)
        masks = marking_masks(tracks, self.stations, camera)
        for mask in masks.values():
            mask[surface_paint] = 0
        masks.update(ground_masks)
        masks["double_white"], double_white_pairs = double_white_mask(
            paint, drive, blocked, valid, pixels, self.stations, self.lateral, camera)
        protected = (masks["double_white"] | obstacles) != 0
        base_road = road != 0
        edge_candidate = (edge_support != 0) & ~base_road & ~protected
        interior = enclosed_asphalt_holes(frame, base_road, protected, camera)
        asphalt, white_paint, brightness = asphalt_and_paint(
            frame, base_road | edge_candidate | interior, marking, masks, obstacles != 0, camera)
        white_paint |= (surface_paint | ground_longitudinal) & ~protected
        road_surface = asphalt | white_paint
        edge_repaired = edge_candidate & asphalt
        # Restore visible paint thickness near a classified track. A paint
        # pixel assigned to a hard restriction cannot be reclassified here.
        paint = white_paint & ~protected & ~surface_paint
        for kind in ("dashed", "solid", "unknown"):
            unassigned = (masks["dashed"] | masks["solid"] | masks["unknown"]) == 0
            around = cv2.dilate(masks[kind], np.ones((5, 13), np.uint8)) != 0
            masks[kind] |= (paint & around & unassigned).astype(np.uint8)
        dashed = (visible_dashed_pixels(frame, masks["dashed"], camera) != 0) & ~protected & ~surface_paint
        masks["dashed"] = (dashed & (masks["solid"] == 0)).astype(np.uint8)
        masks["unknown"][masks["dashed"] != 0] = 0
        unassigned = (masks["dashed"] | masks["solid"] | masks["unknown"]) == 0
        masks["unknown"] |= (ground_longitudinal & ~protected & unassigned).astype(np.uint8)
        drive[valid] = road_surface[v[valid], u[valid]]

        guidance = [[] for _ in self.stations]
        for track in tracks:
            for i, y in zip(track.rows[track.present], track.lateral[track.present]):
                guidance[i].append(float(y))

        road_for_rows = cv2.morphologyEx(drive.astype(np.uint8), cv2.MORPH_CLOSE,
                                         np.ones((1, 7), np.uint8)).astype(bool)
        rows = []
        anchor = 0.
        for i, (x, surface) in enumerate(zip(self.stations, road_for_rows)):
            ranges = [(float(self.lateral[a]), float(self.lateral[b-1])) for a, b in spans(surface)
                      if b-a >= 10 and np.any(valid[i, a:b])]
            nearby = [item for item in ranges if item[0]-.40 <= anchor <= item[1]+.40]
            if nearby:
                right, left = min(nearby, key=lambda item: abs(np.clip(anchor, *item)-anchor))
                if left-right >= .65 and right-.40 <= anchor <= left+.40:
                    active = float(np.clip(anchor, right+.15, left-.15))
                    left_marks = [p for p in guidance[i] if active+.35 < p <= min(left+.08, active+self.lane_width)]
                    right_marks = [p for p in guidance[i] if max(right-.08, active-self.lane_width) <= p < active-.35]
                    ref_left = min(left_marks, default=min(left, active+self.lane_width/2))
                    ref_right = max(right_marks, default=max(right, active-self.lane_width/2))
                    middle = float(np.clip((ref_left+ref_right)/2, right+.20, left-.20))
                    rows.append((x, middle, left, right, ref_left, ref_right))
                    anchor = .7*active + .3*middle
                    continue
            if rows and x-rows[-1][0] > .5:
                break

        if len(rows) < 7:
            empty = {key: [] for key in LANE_KEYS}
            return empty, empty, tracks, masks, road_surface, edge_repaired, white_paint, brightness, double_white_pairs, ground_marks
        xs, center, left, right, ref_left, ref_right = np.asarray(rows, dtype=float).T
        # Filter boundary wiggles by moving them inward only. Preserve the
        # measured bounds where smoothing would otherwise widen the corridor.
        refined_left = np.minimum(left, tolerant_center(xs, left))
        refined_right = np.maximum(right, tolerant_center(xs, right))
        tight = (refined_left-refined_right) < .55
        left, right = np.where(tight, left, refined_left), np.where(tight, right, refined_right)
        center = np.clip(tolerant_center(xs, center), right+.20, left-.20)
        corridor = dict(zip(LANE_KEYS, (np.column_stack((xs, values)).round(4).tolist()
                                      for values in (center, left, right))))
        landmarks = dict(zip(LANE_KEYS, (np.column_stack((xs, values)).round(4).tolist()
                                       for values in (center, ref_left, ref_right))))
        return corridor, landmarks, tracks, masks, road_surface, edge_repaired, white_paint, brightness, double_white_pairs, ground_marks


def orange_cones(bgr, road, roi):
    """Propose narrow high-saturation orange upright obstacles on the test track."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    warm = cv2.inRange(hsv, (0, 118, 103), (24, 255, 255))
    warm[:int(roi*bgr.shape[0]), :] = 0
    warm = cv2.morphologyEx(warm, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(warm)
    occupied = cv2.dilate(road.astype(np.uint8), np.ones((9, 13), np.uint8))
    out = []
    for k in range(1, count):
        x, y, w, h, area = stats[k]
        if (h < 6 or w < 3 or area < 14 or not .78 <= h/w <= 5.5 or
                not .22 <= area/(w*h) <= .95 or h > .40*bgr.shape[0]):
            continue
        bottom = min(bgr.shape[0]-1, y+h+1)
        if not np.any(occupied[bottom, x:x+w]):
            continue
        local = labels[y:y+h, x:x+w] == k
        if np.count_nonzero(local[:max(1, h//3)]) > 1.15*np.count_nonzero(local[-max(1, h//3):]):
            continue
        out.append(dict(xyxy=[int(x), int(y), int(x+w-1), int(y+h-1)],
                        score=None, label="orange_cone_candidate"))
    return out



class SemanticStage:
    def __init__(self, model, near=2., far=13., lane_width=3.35, cone_candidates=False):
        self.cone_candidates = bool(cone_candidates)
        self.model = model
        self.extractor = CorridorExtractor(near, far, lane_width)

    def step(self, bgr, stamp_ns, image_frame, body_frame, calibration):
        camera = GroundCamera(**calibration)
        if bgr.shape[:2] != (camera.height, camera.width):
            raise ValueError("camera calibration and image dimensions disagree")
        road, marking, boxes, edge_support = self.model.infer(bgr)
        if self.cone_candidates:
            boxes.extend(orange_cones(bgr, road, camera.roi_top_ratio))
        obstacles = np.zeros_like(road)
        for box in boxes:
            u0, v0, u1, v1 = np.rint(box["xyxy"]).astype(int)
            cv2.rectangle(obstacles, (u0, v0), (u1, v1), 1, -1)
        (corridor, landmarks, tracks, masks, surface, edge_repaired, white_paint, brightness,
         double_white_pairs, ground_marks) = self.extractor.extract(bgr, road, marking, obstacles, camera, edge_support)
        policy = surface & ~(masks["double_white"] | obstacles).astype(bool)
        return dict(
            stamp_ns=int(stamp_ns), image_frame=image_frame, body_frame=body_frame,
            rule_set=RULE_SET, calibration=calibration, corridor=corridor,
            landmark_curves=landmarks, boxes=boxes,
            markings=[track.detail(self.extractor.stations) for track in tracks] + ground_marks,
            marking_support=sum(int(track.present.sum()) for track in tracks),
            edge_repaired_px=int(edge_repaired.sum()),
            edge_repaired_png=encode_mask(edge_repaired) if edge_repaired.any() else None,
            road_png=encode_mask(surface), drivable_png=encode_mask(policy),
            white_paint_px=int(white_paint.sum()), asphalt_value_limit=round(brightness, 1),
            solid_png=encode_mask(masks["solid"]), dashed_png=encode_mask(masks["dashed"]),
            transverse_png=encode_mask(masks["transverse"]), crosswalk_png=encode_mask(masks["crosswalk"]),
            double_white_pairs=double_white_pairs, double_white_png=encode_mask(masks["double_white"]),
            unknown_png=encode_mask(masks["unknown"]), obstacle_png=encode_mask(obstacles),
            status="ok" if corridor["center"] else "corridor_missing")
