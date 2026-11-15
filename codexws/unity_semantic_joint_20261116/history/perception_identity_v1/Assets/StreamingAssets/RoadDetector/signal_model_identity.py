"""Optional current-frame identity check for conflicting model light detections.

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


def filter_model_light_identity(model_state, detections, rgb_candidates, calibration, image_shape):
    """Remove only a stronger cone's same-object light below the camera horizon.

    A current, same-color calibrated RGB panel candidate overrides the conflict.
    The caller must pass candidates already checked by panel_candidates using
    this frame's camera. Missing/invalid calibration preserves the model input.
    """
    result = dict(effective_model_state=model_state, enabled=True, rejected=[], conflicts=[],
                  reason="no_rejected_model_light", policy="higher_score_cone_iou_0.95_below_horizon_without_rgb_identity")
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
        conflicts = [cone for cone in cones if _box(cone) is not None
                     and _finite([_field(cone, "score")], 1) is not None
                     and _field(cone, "score") > score and _iou(box, _box(cone)) >= .95]
        if not conflicts or state not in ("red", "green"):
            retained.append(state)
            continue
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
        evidence = dict(box_xyxy=box, state=state, score=score,
                        cone_score=max(_field(item, "score") for item in conflicts),
                        maximum_iou=max(_iou(box, _box(item)) for item in conflicts),
                        center_ray_up=ray_up, current_rgb_identity=rgb_identity)
        result["conflicts"].append(evidence)
        if ray_up <= 0. and not rgb_identity:
            result["rejected"].append(dict(evidence, reason="conflicting_cone_below_horizon_without_rgb_identity"))
        else:
            retained.append(state)
    if result["rejected"]:
        result["effective_model_state"] = "red" if "red" in retained else "green" if "green" in retained else "unknown"
        result["reason"] = "rejected_conflicting_model_light"
    return result
