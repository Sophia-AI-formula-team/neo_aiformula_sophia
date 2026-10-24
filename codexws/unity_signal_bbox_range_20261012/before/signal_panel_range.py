"""Current-image flag-panel range from calibrated optical rays.

The panel center and camera heights must share one road datum. Defaults describe
the saved simulator geometry, not measured hardware. World positions, signal
truth, route coordinates and previous-frame ranges are never inputs.
"""
import math


LED_HEIGHT_M = 0.286  # Saved DisplayFace mesh extent is +/-0.143 m.


def _finite_values(value, length):
    try:
        if len(value) != length:
            return None
        values = [float(item) for item in value]
        return values if all(math.isfinite(item) for item in values) else None
    except (TypeError, ValueError, OverflowError):
        return None


def _multiply(matrix, vector):
    return [sum(matrix[3 * row + column] * vector[column] for column in range(3))
            for row in range(3)]


def project_panel_geometry(bbox, calibration, image_shape,
                         panel_height_above_road_m=0.95,
                         camera_height_above_road_m=0.6750197659,
                         front_offset_m=0.216):
    """Validate and project an observed bbox without a temporal/color decision.

    ``R`` maps optical coordinates to base_link (x forward, y left, z up), and
    ``t`` is the camera origin in base_link. ``range_m`` starts at that camera
    origin; ``distance_m`` additionally adds t.x and subtracts the front offset.
    This is longitudinal point depth, not Euclidean/slant distance or distance
    to a world-coordinate stop line. The caller owns age/sequence validation.
    """
    result = dict(valid=False, distance_m=None, range_m=None, reason="invalid_input",
                  source="current_rgb_panel_center_ray", uncertainty_m=None)

    def invalid(reason):
        result["reason"] = reason
        return result

    if not isinstance(calibration, dict):
        return invalid("invalid_calibration")
    k = _finite_values(calibration.get("K"), 9)
    rotation = _finite_values(calibration.get("R"), 9)
    translation = _finite_values(calibration.get("t"), 3)
    if k is None or rotation is None or translation is None:
        return invalid("nonfinite_or_missing_calibration")
    try:
        if len(image_shape) not in (2, 3):
            return invalid("invalid_image_shape")
        height, width = image_shape[:2]
        if any(isinstance(n, bool) or int(n) != n or n < 16 or n > 8192 for n in (height, width)):
            return invalid("invalid_image_shape")
        if len(image_shape) == 3 and image_shape[2] != 3:
            return invalid("invalid_image_channels")
        height, width = int(height), int(width)
    except (TypeError, ValueError, OverflowError):
        return invalid("invalid_image_shape")
    if (k[0] <= 0 or k[4] <= 0 or abs(k[3]) > 1e-9 or abs(k[6]) > 1e-9
            or abs(k[7]) > 1e-9 or abs(k[8] - 1) > 1e-9
            or not 0 < k[2] < width or not 0 < k[5] < height):
        return invalid("invalid_intrinsics_or_image_size")
    # 拒绝错误轴序/缩放矩阵；有效外参旋转可以保留俯仰和横滚。
    for row in range(3):
        for other in range(3):
            dot = sum(rotation[3 * row + j] * rotation[3 * other + j] for j in range(3))
            if abs(dot - (1 if row == other else 0)) > 1e-5:
                return invalid("invalid_rotation")
    a, b, c, d, e, f, g, h, i = rotation
    if abs(a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g) - 1) > 1e-5:
        return invalid("invalid_rotation")
    dimensions = _finite_values((panel_height_above_road_m, camera_height_above_road_m, front_offset_m), 3)
    if dimensions is None or min(dimensions) < 0:
        return invalid("invalid_height_or_front_offset")
    panel_height, camera_height, front = dimensions
    vertical_difference = panel_height - camera_height
    if vertical_difference <= 0:
        return invalid("panel_center_not_above_camera")
    box = _finite_values(bbox, 4)
    if box is None:
        return invalid("invalid_panel_bbox")
    left, top, right, bottom = box
    if not (0 < left < right < width and 0 < top < bottom < height):
        return invalid("cropped_or_invalid_panel_bbox")
    if min(right - left, bottom - top) < 3:
        return invalid("panel_too_small")
    center_x, center_y = (left + right) / 2, (top + bottom) / 2

    def depth_at(vertical_pixel):
        optical_y = (vertical_pixel - k[5]) / k[4]
        optical_x = (center_x - k[2] - k[1] * optical_y) / k[0]
        ray = _multiply(rotation, (optical_x, optical_y, 1.0))
        if ray[2] <= 1e-3 * math.sqrt(sum(v * v for v in ray)):
            return None
        scale = vertical_difference / ray[2]
        point = [scale * v for v in ray]
        return point if point[0] > 0 and all(math.isfinite(v) for v in point) else None

    point = depth_at(center_y)
    if point is None:
        return invalid("ray_below_or_near_horizon")
    distance = translation[0] + point[0] - front
    if distance <= 0:
        return invalid("panel_at_or_behind_vehicle_front")
    # 已知LED高度只校验同一观测是否自洽；不从模型含支架的大框估距。
    inverse_rotation = [rotation[3 * column + row] for row in range(3) for column in range(3)]
    endpoint_pixels = []
    for offset in (-LED_HEIGHT_M / 2, LED_HEIGHT_M / 2):
        optical = _multiply(inverse_rotation, (point[0], point[1], point[2] + offset))
        if optical[2] <= 0:
            return invalid("panel_height_projection_invalid")
        endpoint_pixels.append(k[4] * optical[1] / optical[2] + k[5])
    expected_height = abs(endpoint_pixels[1] - endpoint_pixels[0])
    observed_height = bottom - top
    if abs(observed_height - expected_height) > max(2.0, 0.30 * expected_height):
        return invalid("panel_height_inconsistent")
    bounds = [depth_at(center_y + delta) for delta in (-0.5, 0.5)]
    if any(bound is None for bound in bounds):
        return invalid("pixel_uncertainty_reaches_horizon")
    depth_bounds = sorted(bound[0] for bound in bounds)
    uncertainty = max(abs(bound - point[0]) for bound in depth_bounds)
    result.update(valid=True, distance_m=distance, range_m=point[0], reason="calibrated_current_panel",
                  uncertainty_m=uncertainty, range_interval_m=depth_bounds,
                  front_interval_m=[translation[0] + bound - front for bound in depth_bounds],
                  uncertainty_definition="bbox_center_plus_minus_half_pixel_only; excludes body tilt and height calibration error",
                  center_pixel=[center_x, center_y], observed_height_px=observed_height,
                  expected_height_px=expected_height,
                  calibration_provenance=dict(
                      method="optical_ray_to_base_link_then_known_center_height_plane",
                      panel_center_above_common_road_datum_m=panel_height,
                      camera_above_common_road_datum_m=camera_height,
                      vertical_difference_m=vertical_difference, led_height_m=LED_HEIGHT_M,
                      front_offset_m=front, camera_forward_offset_m=translation[0],
                      default_dimensions="saved_simulator_scene_prefab_and_settled_camera; not measured hardware",
                      assumptions="base_link up follows road normal; panel upright; calibrated intrinsics match image_shape",
                      image_shape=[height, width]))
    return result


def estimate_panel_range(evidence, calibration, image_shape,
                         panel_height_above_road_m=0.95,
                         camera_height_above_road_m=0.6750197659,
                         front_offset_m=0.216):
    """Return current range only after fresh, same-color two-frame confirmation."""
    def invalid(reason):
        return dict(valid=False, distance_m=None, range_m=None, reason=reason,
                    source="current_rgb_panel_center_ray", uncertainty_m=None)

    if not isinstance(evidence, dict) or evidence.get("fresh") is not True:
        return invalid("no_fresh_panel_evidence")
    state = evidence.get("state")
    panel = evidence.get("panel")
    if state not in ("red", "green") or not isinstance(panel, dict):
        return invalid("no_confirmed_colored_panel")
    if panel.get("state") != state:
        return invalid("panel_state_mismatch")
    confirmed = panel.get("confirmed_frames")
    if not isinstance(confirmed, int) or isinstance(confirmed, bool) or confirmed < 2:
        return invalid("panel_not_confirmed")
    return project_panel_geometry(panel.get("bbox"), calibration, image_shape,
                                  panel_height_above_road_m,
                                  camera_height_above_road_m, front_offset_m)
