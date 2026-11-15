"""Optional current-frame identity check for model light detections (v2).

No model weights, object detections, range, state history or simulator truth are
changed. A model object's full box is never treated as the LED's physical size.
"""
import math


def _field(value, name, default=None):
    return value.get(name, default) if isinstance(value, dict) else getattr(value, name, default)


def _finite(value, length):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        return None
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item) for item in value):
        return None
    return [float(item) for item in value]


def _camera(calibration, image_shape):
    if (not isinstance(calibration, dict) or not isinstance(image_shape, (list, tuple))
            or len(image_shape) not in (2, 3)):
        return None
    height, width = image_shape[:2]
    if any(isinstance(value, bool) or not isinstance(value, int) or not 16 <= value <= 8192 for value in (height, width)):
        return None
    k, rotation = _finite(calibration.get("K"), 9), _finite(calibration.get("R"), 9)
    if k is None or rotation is None or k[0] <= 0 or k[4] <= 0:
        return None
    if abs(k[3]) > 1e-9 or k[6:] != [0., 0., 1.] or not 0 < k[2] < width or not 0 < k[5] < height:
        return None
    for row in range(3):
        for other in range(3):
            dot = sum(rotation[3*row+j]*rotation[3*other+j] for j in range(3))
            if abs(dot-(1. if row == other else 0.)) > 1e-5:
                return None
    a, b, c, d, e, f, g, h, i = rotation
    if abs(a*(e*i-f*h)-b*(d*i-f*g)+c*(d*h-e*g)-1.) > 1e-5:
        return None
    return k, rotation


def _box(value):
    box = _finite(_field(value, "box_xyxy"), 4)
    return box if box is not None and box[0] < box[2] and box[1] < box[3] else None


def _iou(a, b):
    intersection = max(0., min(a[2], b[2])-max(a[0], b[0])) * max(0., min(a[3], b[3])-max(a[1], b[1]))
    union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
    return intersection/union if union > 0 else 0.


def _colored_pixel_evidence(image_bgr, box, state, camera):
    """Use A4 color_semantics thresholds; require every pixel cell below horizon.

    A model box can include a post: only the observed state-colored pixels are
    evaluated. Too few pixels never authorize rejecting the model detection.
    """
    result = dict(available=False, reason="no_current_rgb", color=state,
                  colored_pixels=0, minimum_pixels=None, all_below_horizon=False,
                  maximum_ray_up_with_half_pixel=None)
    if image_bgr is None:
        return result
    import cv2
    import numpy as np
    if (not isinstance(image_bgr, np.ndarray) or image_bgr.dtype != np.uint8
            or image_bgr.ndim != 3 or image_bgr.shape[2] != 3):
        return result
    height, width = image_bgr.shape[:2]
    left, top = min(max(math.floor(box[0]), 0), width), min(max(math.floor(box[1]), 0), height)
    right, bottom = min(max(math.ceil(box[2]), 0), width), min(max(math.ceil(box[3]), 0), height)
    crop = image_bgr[top:bottom, left:right]
    if crop.size == 0:
        return result
    hue, saturation, value = cv2.split(cv2.cvtColor(crop, cv2.COLOR_BGR2HSV))
    vivid = (saturation >= 75) & (value >= 65)
    color = ((hue <= 9) | (hue >= 170)) if state == "red" else ((hue >= 38) & (hue <= 90))
    ys, xs = np.nonzero(vivid & color)
    count, minimum = len(xs), max(3, int(crop.shape[0]*crop.shape[1]*.002))
    result.update(available=True, reason="insufficient_color_pixels", colored_pixels=count,
                  minimum_pixels=minimum)
    if count < minimum:
        return result
    k, rotation = camera
    py = (ys+top-k[5])/k[4]
    px = (xs+left-k[2]-k[1]*py)/k[0]
    # Affine ray-up permits the conservative maximum over each +/-0.5 pixel
    # cell. Even one cell touching above the horizon preserves the model.
    dx = rotation[6]/k[0]
    dy = rotation[7]/k[4]-rotation[6]*k[1]/(k[0]*k[4])
    maximum = float(np.max(rotation[6]*px+rotation[7]*py+rotation[8])) + .5*(abs(dx)+abs(dy))
    below = maximum <= 0.
    result.update(reason="all_colored_pixel_cells_below_horizon" if below else "colored_pixel_at_or_above_horizon",
                  maximum_ray_up_with_half_pixel=maximum, all_below_horizon=below)
    return result


def filter_model_light_identity(model_state, detections, rgb_candidates, calibration, image_shape,
                                image_bgr=None):
    """Combine v1 cone conflicts and explicit current-pixel impossibility.

    A current, same-color calibrated RGB panel candidate overrides the conflict.
    The caller must pass candidates already checked by panel_candidates using
    this frame's camera. Missing/invalid calibration preserves the model input.
    """
    result = dict(effective_model_state=model_state, enabled=True, version=2,
                  rejected=[], conflicts=[], evaluated=[], reason="no_rejected_model_light",
                  policy="v1_cone_conflict_or_all_current_color_pixel_cells_below_horizon_without_rgb_identity")
    camera = _camera(calibration, image_shape)
    if camera is None:
        result["reason"] = "invalid_calibration_preserve_model"
        return result
    k, rotation = camera
    lights = [item for item in detections or () if _field(item, "class_id") == 10]
    cones = [item for item in detections or () if _field(item, "class_id") == 91]
    retained = []
    for light in lights:
        state, box = _field(light, "traffic_light_state"), _box(light)
        score = _field(light, "score")
        if box is None or _finite([score], 1) is None:
            retained.append(state)
            continue
        if not (0 <= box[0] < box[2] <= image_shape[1] and 0 <= box[1] < box[3] <= image_shape[0]):
            retained.append(state)
            continue
        if state not in ("red", "green"):
            retained.append(state)
            continue
        conflicts = [cone for cone in cones if _box(cone) is not None
                     and _finite([_field(cone, "score")], 1) is not None
                     and _field(cone, "score") > score and _iou(box, _box(cone)) >= .95]
        x, y = (box[0]+box[2])/2., (box[1]+box[3])/2.
        optical_y = (y-k[5])/k[4]
        optical_x = (x-k[2]-k[1]*optical_y)/k[0]
        ray_up = sum(rotation[6+j]*value for j, value in enumerate((optical_x, optical_y, 1.)))
        rgb_identity = False
        for candidate in rgb_candidates or ():
            candidate_box = _finite(candidate.get("bbox"), 4)
            if candidate_box is not None and candidate.get("state") == state:
                cx, cy = (candidate_box[0]+candidate_box[2])/2., (candidate_box[1]+candidate_box[3])/2.
                if box[0]-1 <= cx <= box[2]+1 and box[1]-1 <= cy <= box[3]+1:
                    rgb_identity = True
                    break
        pixels = _colored_pixel_evidence(image_bgr, box, state, camera)
        v1_reject = bool(conflicts) and ray_up <= 0. and not rgb_identity
        pixel_reject = pixels["all_below_horizon"] and not rgb_identity
        evidence = dict(box_xyxy=box, state=state, score=score,
                        cone_score=max((_field(item, "score") for item in conflicts), default=None),
                        maximum_iou=max((_iou(box, _box(item)) for item in conflicts), default=None),
                        center_ray_up=ray_up, current_rgb_identity=rgb_identity,
                        v1_conflict_reject=v1_reject, colored_pixel_reject=pixel_reject,
                        colored_pixel_evidence=pixels)
        result["evaluated"].append(evidence)
        if conflicts:
            result["conflicts"].append(evidence)
        if v1_reject or pixel_reject:
            reasons = []
            if v1_reject:
                reasons.append("conflicting_cone_below_horizon_without_rgb_identity")
            if pixel_reject:
                reasons.append("all_current_color_pixel_cells_below_horizon_without_rgb_identity")
            result["rejected"].append(dict(evidence, reasons=reasons))
        else:
            retained.append(state)
    if result["rejected"]:
        result["effective_model_state"] = "red" if "red" in retained else "green" if "green" in retained else "unknown"
        result["reason"] = "rejected_conflicting_model_light"
    return result
