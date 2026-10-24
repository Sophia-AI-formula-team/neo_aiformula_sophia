"""Known metric transforms: geometry quality, not just admission counts.

These synthetic paint masks/rigid transforms are deterministic development
oracles, not camera/vehicle or 40 ms runtime acceptance.
"""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

from lane_mapping_lya_reference.mapping_core import GroundLookup
from lane_mapping_lya_reference.paint_geometry import BoundaryConsensus, extract_boundary_observation
from lane_mapping_lya_reference.boundary_localization import BoundaryLaneLocalizer, FrozenBoundaryIndex
from lane_mapping_lya_reference.mask_localization import _configuration, _rotation


def ribbon(curvature=0., width=.44):
    size = (200,400)
    row,col = np.indices(size)
    x, y = col*.02, 2.-row*.02
    mid = curvature*(x-4.)**2/2.
    mask = (((np.abs(y-mid-1.) < width/2.) | (np.abs(y-mid+1.) < width/2.))
            & (x > .3) & (x < 7.7)).astype(np.uint8)*255
    transform = np.diag([1.,-1.,-1.,1.])
    transform[2,3] = 2.
    camera = np.array([[100.,0.,0.],[0.,100.,100.],[0.,0.,1.]])
    lookup = GroundLookup(400,200,camera,transform,.5)
    xy,sensitivity,_ = lookup.project_mask_with_sensitivity(mask,127)
    consensus = BoundaryConsensus()
    for k in range(5):
        obs = extract_boundary_observation(mask,lookup,[0.,0.,1.],1000000000+k*100000000)
        assert consensus.commit(consensus.prepare(obs))
    return xy,sensitivity,obs,consensus.snapshot(),lookup,mask


@pytest.mark.parametrize("curvature", [-.06,0.,.06])
@pytest.mark.parametrize("error", [-1.,1.])
def test_recovers_real_signed_lateral_yaw_errors_on_curved_and_straight_paint(curvature,error):
    xy,sensitivity,obs,reference,_,_ = ribbon(curvature)
    true_pose = np.array([.2,error*.04,error*.01])
    rotation = _rotation(true_pose[2])
    local = (xy-true_pose[:2]) @ rotation
    inverse = np.r_[-true_pose[:2] @ rotation, -true_pose[2]]
    current = obs.transformed(inverse)
    from dataclasses import replace
    current = replace(current,source_stamp_ns=1500000000)
    target = BoundaryLaneLocalizer(reference,{"runtime_budget_ms":2000.})
    result = target.match(local,[.2,0.,0.],1500000000,scan_support_xy=local,
                          scan_sensitivity=sensitivity,boundary_observation=current)
    assert result.accepted,result.reason
    assert abs(result.pose_xyyaw[1]-true_pose[1]) < .005, result.pose_xyyaw
    assert abs(result.pose_xyyaw[2]-true_pose[2]) < .002, result.pose_xyyaw
    assert result.metrics["observable_rank"] == 2
    assert result.metrics["boundary_unassigned_mass"] >= -1e-12
    assert result.metrics["scan_feature_weight_sum"] == result.metrics["boundary_original_foreground_mass"]


@pytest.mark.parametrize("width", [.06,.22,.44])
def test_no_false_pose_for_identical_paint_of_different_widths(width):
    xy,sensitivity,obs,reference,_,_ = ribbon(.06,width)
    from dataclasses import replace
    current = replace(obs,source_stamp_ns=1500000000)
    target = BoundaryLaneLocalizer(reference,{"runtime_budget_ms":2000.})
    result = target.match(xy,[0.,0.,0.],1500000000,scan_support_xy=xy,
                          scan_sensitivity=sensitivity,boundary_observation=current)
    assert result.accepted,result.reason
    assert abs(result.pose_xyyaw[1]) < .005,result.pose_xyyaw
    assert abs(result.pose_xyyaw[2]) < .002,result.pose_xyyaw


def test_signed_nearest_never_matches_the_other_side_of_thin_paint():
    _,_,_,reference,_,_ = ribbon(0.,.06)
    index = FrozenBoundaryIndex(reference,_configuration(None))
    ids = np.flatnonzero(index.normal_valid)
    assert len(ids) > 24
    actual = index.nearest(index.anchor[ids],index.normals[ids])
    np.testing.assert_array_equal(actual,ids)
    opposite = index.nearest(index.anchor[ids],-index.normals[ids])
    matched = opposite >= 0
    assert np.all(opposite[matched] != ids[matched])
    assert np.all(np.sum(index.normals[ids[matched]]*index.normals[opposite[matched]],axis=1)<0.)


def test_reference_finite_spans_are_not_filled_across_a_gap():
    from lane_mapping_lya_reference.paint_geometry import BoundaryReference
    reference = BoundaryReference(
        np.array([[0.,0.]]),np.array([[0.,1.]]),
        np.array([[[-1.,0.],[-.5,0.]],[[.5,0.],[1.,0.]]]),np.array([0,0]),
        np.array([5]),np.array([5]),np.array([0]),1000000000)
    index = FrozenBoundaryIndex(reference,_configuration(None))
    np.testing.assert_array_equal(index.covered(np.array([[-.7,.02],[0.,.02],[.7,.02]]),np.array([0,0,0])),
                                  [True,False,True])
