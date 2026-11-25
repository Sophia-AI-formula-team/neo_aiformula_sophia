"""Select a camera-space band directly from the current semantic mask."""
import numpy as np

from optimization_oa.geometry import GroundCamera
from optimization_oa.planner import PlannerConfig

from .band_search import search_band
from .common import decode_mask
from .semantic import RULE_SET


def policy_layers(semantic):
    if semantic.get("rule_set") != RULE_SET:
        raise ValueError("semantic observation uses obsolete lane rules")
    road = decode_mask(semantic["road_png"]) != 0
    obstacle = decode_mask(semantic["obstacle_png"]) != 0
    double_white = decode_mask(semantic["double_white_png"]) != 0
    return {"off_road": ~road, "double_white": double_white, "obstacle": obstacle}


class OaStage:
    def __init__(self, width, clearance=.10, lookahead=11.5, max_lane_gap=4., origin_x=0.,
                 length_weight=1., curvature_weight=.25, solid_weight=.70):
        self.origin_x = float(origin_x)
        self.solid_weight = float(solid_weight)
        if not np.isfinite(self.solid_weight) or self.solid_weight < 0:
            raise ValueError("solid line penalty must be finite and nonnegative")
        self.config = PlannerConfig(vehicle_width_m=width, clearance_m=clearance,
                                    lookahead_m=lookahead, min_lookahead_m=min(2.5, lookahead),
                                    max_lane_gap_m=max_lane_gap, length_weight=length_weight,
                                    smoothness_weight=curvature_weight, reference_weight=0.)
        self.config.validate()

    def step(self, semantic):
        metadata = dict(stamp_ns=semantic["stamp_ns"], body_frame=semantic["body_frame"],
                        path=[], metrics={}, band_checked=False)
        calibration = dict(semantic["calibration"])
        origin = np.array(calibration["origin"], dtype=float)
        origin[0] -= self.origin_x
        calibration["origin"] = origin
        camera = GroundCamera(**calibration)
        layers = policy_layers(semantic)
        allowed = decode_mask(semantic["drivable_png"]) != 0
        shape = (camera.height, camera.width)
        if allowed.shape != shape or any(layer.shape != shape for layer in layers.values()):
            raise ValueError("semantic mask does not match camera")
        if np.any(allowed & np.logical_or.reduce(list(layers.values()))):
            raise ValueError("semantic allowed area overlaps a forbidden layer")
        solid = decode_mask(semantic["solid_png"]) != 0
        unknown = decode_mask(semantic["unknown_png"]) != 0
        if solid.shape != shape or unknown.shape != shape:
            raise ValueError("longitudinal marking masks do not match camera")
        result = search_band(camera, allowed, self.config, solid | unknown, self.solid_weight)
        valid = result.band > 0
        if len(result.path):
            hits = {key: int(np.count_nonzero(valid & layer)) for key, layer in layers.items()}
            if not valid.any() or np.any(valid & ~allowed):
                raise RuntimeError("Selected path band violates the semantic occupancy constraints")
            result.path[:, 0] += self.origin_x
            metadata.update(path=result.path.round(5).tolist(), band_checked=True, policy_hits=hits,
                            uncertain_mark_overlap_px=int(np.count_nonzero(valid & unknown)),
                            crossable_dash_overlap_px=int(np.count_nonzero(
                                valid & (decode_mask(semantic["dashed_png"]) != 0))))
        metadata.update(status=result.reason, metrics=result.metrics)
        return metadata, result
