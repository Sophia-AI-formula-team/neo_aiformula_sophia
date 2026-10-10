"""A distant paint connector must not fabricate local overlap support.

This is a synthetic admission counterexample, not a vehicle-quality benchmark.
There are three independent local line cross-sections, only one of which exists
in the frozen anchor. A connector outside every reliable point's local support
circle changes global topology but must not make the two wrong lines half-price.
All production thresholds and real 40 ms matcher clocks remain unchanged.
"""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

from lane_mapping_lya_reference.mask_localization import (
    LaneMapLocalizer, _FrozenIndex, _configuration, _voxel_means,
)


def remote_bridge_scene(joined, angle=0.):
    x = 1. + np.arange(46) * .11
    arms = [np.column_stack((x, np.full(len(x), y))) for y in (-.3, .3)]
    good = np.column_stack((x[:27], np.full(27, 2.)))
    scan = np.vstack((arms[0][:21], arms[1][:21], good))
    bridge = np.column_stack((np.full(3, x[-1]), [-.15, 0., .15]))
    support = np.vstack((*arms, good, *([bridge] if joined else [])))
    anchor_x = np.arange(0., 8., .11)
    anchor = np.column_stack((anchor_x, np.full(len(anchor_x), 2.)))
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    return anchor @ rotation.T, scan @ rotation.T, support @ rotation.T, bridge @ rotation.T


@pytest.mark.parametrize("joined", [False, True])
@pytest.mark.parametrize("angle", [0., .13])
def test_one_matching_line_cannot_pass_by_joining_two_wrong_lines_far_away(joined, angle):
    anchor, scan, support, bridge = remote_bridge_scene(joined, angle)
    matcher = LaneMapLocalizer(anchor)
    cfg = matcher.config
    assert cfg["runtime_budget_ms"] == 40.
    assert cfg["min_overlap_ratio"] == .55
    assert len(scan) == 69 and 27 >= cfg["min_inliers"]
    assert np.min(np.linalg.norm(scan[:, None, :] - bridge[None, :, :], axis=2)) > cfg["normal_max_radius_m"]
    result = matcher.match(scan, [0., 0., 0.], 1, commit=False,
                           scan_support_xy=support, scan_sensitivity=np.full(len(scan), .01))
    assert not result.accepted, result.metrics
    assert result.reason == "insufficient_overlap", result.metrics
    assert result.metrics["scan_line_points"] == 69
    assert result.metrics["scan_geometric_rejections"] == 0
    # Each reliable cross-section has one pixel per independently visible line.
    # The remote U bridge must not reduce 42 wrong votes into 21 wrong votes.
    assert result.metrics["scan_feature_weight_sum"] == pytest.approx(69.)
    assert result.metrics["overlap_ratio"] == pytest.approx(27. / 69.)
    assert result.metrics["unweighted_overlap_ratio"] == pytest.approx(27. / 69.)
    np.testing.assert_array_equal(matcher.correction_se2, np.zeros(3))
    assert matcher.last_accepted_stamp_ns is None
    assert matcher._pending_result is None
    assert matcher._pending_data is None


def test_remote_connector_does_not_change_current_reliable_local_geometry():
    cfg = _configuration(None)
    indexes, queries = [], []
    for joined in (False, True):
        _, scan, support, _ = remote_bridge_scene(joined)
        cloud = _voxel_means(support, cfg["scan_voxel_m"])
        current = _FrozenIndex(cloud, cfg)
        found = current.nearest(scan, valid_only=False)
        assert np.all(found >= 0)
        assert np.all(current.normal_valid[found])
        indexes.append(current)
        queries.append(found)
    for name in ("normal_valid", "normals", "line_points", "normal_radii",
                 "normal_thickness", "normal_position_support"):
        np.testing.assert_array_equal(getattr(indexes[0], name)[queries[0]],
                                      getattr(indexes[1], name)[queries[1]], err_msg=name)
    # Confirm the intended global-topology perturbation actually happened.
    assert indexes[0].components[queries[0][0]] != indexes[0].components[queries[0][21]]
    assert indexes[1].components[queries[1][0]] == indexes[1].components[queries[1][21]]
