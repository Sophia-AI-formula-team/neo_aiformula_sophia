"""Synthetic integration: rendered masks -> production consensus -> real route.

Not a real-road accuracy, ROS/DDS, VectorNav transport or hardware test. Analytic
lane truth is used ONLY by the simulated camera renderer; the mapper receives
the current binary image and current pose, never the truth lane point array.
"""

import math

import numpy as np

from lane_mapping_lya_reference.mapping_core import (
    GroundLookup, SparseConsensusMap, make_transform, transform_local_points,
)
from lane_mapping_lya_reference.route import build_route
from lane_mapping_lya_reference.controller import ClosedRouteController


def _render_mask(lane_truth, pose):
    """A flat 1 m high camera, optical right/down/forward, 640 x 360."""
    x, y, yaw = pose
    delta = lane_truth - np.array([x, y])
    forward = math.cos(yaw) * delta[:, 0] + math.sin(yaw) * delta[:, 1]
    left = -math.sin(yaw) * delta[:, 0] + math.cos(yaw) * delta[:, 1]
    in_front = forward > 1.0  # Other points are outside this camera's image.
    forward, left = forward[in_front], left[in_front]
    u = np.rint(320.0 - 254.0 * left / forward).astype(int)
    v = np.rint(180.0 + 254.0 / forward).astype(int)
    inside = (u >= 0) & (u < 640) & (v >= 0) & (v < 360)
    mask = np.zeros((360, 640), dtype=np.uint8)
    mask[v[inside], u[inside]] = 255
    return mask


def test_current_masks_generate_consensus_usable_as_a_real_closed_route():
    # These are the shipped mapping defaults, not relaxed test-only thresholds.
    lookup = GroundLookup(
        640, 360, [254.0, 0.0, 320.0, 0.0, 254.0, 180.0, 0.0, 0.0, 1.0],
        make_transform([0.0, 0.0, 1.0], [-0.5, 0.5, -0.5, 0.5]), 0.5)
    mapper = SparseConsensusMap(
        0.1, 5, 1, max_candidate_cells=200000, max_confirmed_cells=100000,
        candidate_ttl_s=30.0)
    radius = 12.0
    angle = np.linspace(-math.pi / 2, 3 * math.pi / 2, 5001, endpoint=False)
    lane_truth = np.vstack([
        np.column_stack((r * np.cos(angle), radius + r * np.sin(angle)))
        for r in (radius - 1.2, radius + 1.2)
    ])
    arrived_trace = []
    for index, angle in enumerate(np.linspace(-math.pi / 2, 3 * math.pi / 2, 1001)):
        # Append just the current pose; the final route is built after the lap.
        pose = (radius * math.cos(angle), radius + radius * math.sin(angle),
                angle + math.pi / 2)
        arrived_trace.append(pose)
        mask = _render_mask(lane_truth, pose)
        local_xy, sensitivity, _ = lookup.project_mask_with_sensitivity(mask, 127)
        world_xy = transform_local_points(local_xy, *pose)
        candidates = mapper.points_to_unique_keys(world_xy)
        reliable = mapper.points_to_unique_keys(world_xy[sensitivity <= 0.1])
        mapper.update(candidates, reliable, stamp_ns=1000000000 + index * 50000000)
        if index < 4:
            # Five observations cannot exist before the fifth image arrives.
            assert not mapper.confirmed_keys

    map_xy, votes = mapper.confirmed_points_and_votes()
    assert len(map_xy) > 1000
    assert np.all(votes >= 5)
    assert mapper.capacity_rejected_cells == 0
    assert mapper.confirmation_capacity_rejections == 0
    # No lane_truth is passed here: use only the actual fused map and trace.
    route = build_route(np.asarray(arrived_trace), map_xy)
    assert route["diagnostics"]["valid"] is True
    assert route["diagnostics"]["prior_support_fraction"] == 1.0
    assert route["diagnostics"]["final_support_fraction"] == 1.0
    assert route["diagnostics"]["min_clearance_m"] > 0.5
    assert 74.0 < route["diagnostics"]["route_length_m"] < 77.0
    # Independent consumer validation includes tangent, curvature and the seam.
    controller = ClosedRouteController(route)
    assert 74.0 < controller.length < 77.0
