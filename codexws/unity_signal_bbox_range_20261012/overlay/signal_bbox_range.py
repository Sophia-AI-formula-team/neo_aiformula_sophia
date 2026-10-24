"""Opt-in current confirmed luminous-panel bbox range; no tracking or control.

Reference pixels and depth must describe this same panel, optical camera, and
resolution. The supplied simulator preset is theoretical, not measured. The
reported interval covers pixel quantization only, not total measurement error.
"""
import json
import math
from functools import lru_cache
from pathlib import Path

SOURCE = "current_confirmed_panel_bbox_height_v1"


def _invalid(reason):
    return dict(valid=False, distance_m=None, range_m=None, uncertainty_m=None,
                target_reached=None, source=SOURCE, reason=reason)


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _vector(value, length):
    return isinstance(value, (list, tuple)) and len(value) == length and all(_number(x) for x in value)


def validate_config(config):
    if not isinstance(config, dict) or type(config.get("enabled")) is not bool:
        raise ValueError("enabled_must_be_boolean")
    if not config["enabled"]:
        return
    if config.get("schema") != "signal_bbox_range/v1":
        raise ValueError("invalid_schema")
    for name in ("reference_depth_m", "reference_height_px", "min_height_px",
                 "min_width_px", "target_height_px", "height_error_px"):
        if not _number(config.get(name)) or config[name] <= 0:
            raise ValueError("invalid_" + name)
    if config["height_error_px"] < 1 or config["height_error_px"] >= config["min_height_px"]:
        raise ValueError("invalid_pixel_error_or_minimum")
    if not _number(config.get("front_offset_m")) or config["front_offset_m"] < 0:
        raise ValueError("invalid_front_offset")
    for name in ("image_width", "image_height"):
        if type(config.get(name)) is not int or not 16 <= config[name] <= 8192:
            raise ValueError("invalid_" + name)
    camera = config.get("camera")
    if not isinstance(camera, dict) or any(not _vector(camera.get(k), n) for k, n in (("K", 9), ("R", 9), ("t", 3))):
        raise ValueError("invalid_camera")
    k = camera["K"]
    # This first version deliberately supports the existing level rectified
    # camera only: optical depth equals base_link forward displacement.
    if (camera["R"] != [0, 0, 1, -1, 0, 0, 0, -1, 0]
            or k[0] <= 0 or k[4] <= 0 or k[1] != 0 or k[3] != 0
            or k[6:] != [0, 0, 1] or not 0 < k[2] < config["image_width"]
            or not 0 < k[5] < config["image_height"]):
        raise ValueError("unsupported_camera")
    if config["target_height_px"] < config["min_height_px"]:
        raise ValueError("target_below_minimum_box_size")
    target_depth = config["reference_depth_m"] * config["reference_height_px"] / config["target_height_px"]
    if not math.isfinite(target_depth) or target_depth + camera["t"][0] - config["front_offset_m"] <= 0:
        raise ValueError("invalid_target_depth")
    if not isinstance(config.get("calibration_provenance"), str) or not config["calibration_provenance"].strip():
        raise ValueError("missing_calibration_provenance")


def estimate_bbox_range(evidence, calibration, image_shape, config):
    """One current observation. No cache, fallback, color change or command."""
    try:
        validate_config(config)
    except ValueError as error:
        return _invalid("invalid_config:" + str(error))
    if not config["enabled"]:
        return _invalid("bbox_mode_disabled")
    if not isinstance(evidence, dict) or evidence.get("fresh") is not True:
        return _invalid("no_fresh_panel_evidence")
    state, panel = evidence.get("state"), evidence.get("panel")
    if state not in ("red", "green") or not isinstance(panel, dict):
        return _invalid("no_confirmed_colored_panel")
    if panel.get("state") != state:
        return _invalid("panel_state_mismatch")
    if type(panel.get("confirmed_frames")) is not int or panel["confirmed_frames"] < 2:
        return _invalid("panel_not_confirmed")
    if (not isinstance(image_shape, (list, tuple)) or len(image_shape) not in (2, 3)
            or any(type(n) is not int for n in image_shape)
            or list(image_shape[:2]) != [config["image_height"], config["image_width"]]
            or len(image_shape) == 3 and image_shape[2] != 3):
        return _invalid("image_size_or_channels_mismatch")
    if not isinstance(calibration, dict):
        return _invalid("camera_mismatch")
    for key, size in (("K", 9), ("R", 9), ("t", 3)):
        values = calibration.get(key)
        # K is serialized through Unity float32; allow only rounding-sized
        # differences from the repository decimal intrinsics.
        tolerance = 2e-5 if key == "K" else 1e-6
        if not _vector(values, size) or any(abs(a-b) > tolerance for a, b in zip(values, config["camera"][key])):
            return _invalid("camera_mismatch")
    box = panel.get("bbox")
    if not _vector(box, 4):
        return _invalid("invalid_panel_bbox")
    left, top, right, bottom = box
    if not (0 < left < right < config["image_width"] and 0 < top < bottom < config["image_height"]):
        return _invalid("cropped_or_invalid_panel_bbox")
    height, width = bottom-top, right-left
    if height < config["min_height_px"] or width < config["min_width_px"]:
        return _invalid("panel_too_small")
    coefficient = config["reference_depth_m"] * config["reference_height_px"]
    offset = config["camera"]["t"][0] - config["front_offset_m"]
    depth = coefficient / height
    distance = depth + offset
    error = config["height_error_px"]
    bounds = [coefficient/(height+error), coefficient/(height-error)]
    if not all(math.isfinite(v) for v in (depth, distance, *bounds)) or distance <= 0 or bounds[0]+offset <= 0:
        return _invalid("panel_at_or_behind_vehicle_front")
    target_height = config["target_height_px"]
    return dict(valid=True, source=SOURCE, reason="calibrated_current_bbox_height",
                distance_m=distance, range_m=depth,
                uncertainty_m=max(depth-bounds[0], bounds[1]-depth),
                range_interval_m=bounds, front_interval_m=[v+offset for v in bounds],
                observed_height_px=height, observed_width_px=width,
                target_height_px=target_height, target_reached=height >= target_height,
                target_front_distance_m=coefficient/target_height+offset,
                uncertainty_definition="bbox_height_plus_minus_configured_pixels_only; excludes pose, detection and calibration error",
                calibration_provenance=config["calibration_provenance"])


@lru_cache(maxsize=1)
def _runtime_config():
    # Restart the worker after changing config. A missing file retains the
    # original behavior; a present broken configuration never falls back.
    path = Path(__file__).with_name("signal_bbox_range.json")
    if not path.exists():
        return {"enabled": False}
    try:
        config = json.loads(path.read_text(encoding="utf-8-sig"))
        validate_config(config)
        return config
    except (OSError, ValueError) as error:
        return {"config_error": str(error)}


def estimate_panel_range(evidence, calibration, image_shape):
    """Drop-in distance-only worker wiring; existing consumer stays unchanged."""
    config = _runtime_config()
    if "config_error" in config:
        return _invalid("invalid_config:" + config["config_error"])
    if config["enabled"]:
        return estimate_bbox_range(evidence, calibration, image_shape, config)
    from signal_panel_range import estimate_panel_range as legacy_estimate
    return legacy_estimate(evidence, calibration, image_shape)
