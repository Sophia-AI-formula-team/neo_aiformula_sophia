"""Verify the DDS test inputs geometrically, even on a host without ROS."""
import hashlib
import ast
import csv
import importlib.util
import io
import json
import math
from pathlib import Path

import numpy as np

from lane_mapping_lya_reference.mapping_core import GroundLookup
from lane_mapping_lya_reference.mask_localization import LaneMapLocalizer


def fixture_module():
    path = Path(__file__).with_name("runtime_smoke.py")
    spec = importlib.util.spec_from_file_location("visual_runtime_fixture", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_repeat_pixels_use_same_circle_and_calibration_as_saved_anchor():
    module = fixture_module()
    camera, extrinsic = module.repeat_calibration("test")
    mask = np.frombuffer(module.repeat_mask_pixels("test"), dtype=np.uint8).reshape(
        camera["height"], camera["width"])
    lookup = GroundLookup(camera["width"], camera["height"], camera["matrix"],
                          np.asarray(extrinsic["matrix"]), 0.1)
    xy, sensitivity, _ = lookup.project_mask_with_sensitivity(mask, 127)
    radius = np.hypot(xy[:, 0], xy[:, 1] - 12.)
    assert len(xy) >= 60
    assert np.all(sensitivity <= 0.1)
    assert np.max(np.minimum(abs(radius - 10.8), abs(radius - 13.2))) <= 0.065


def test_repeat_fixture_contains_hash_checked_consensus_and_complete_camera(tmp_path):
    module = fixture_module()
    directory = tmp_path / "bundle"
    manifest, _ = module.synthetic_bundle(directory, "test_map")
    metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["data_sha256"]["consensus.csv"] == hashlib.sha256(
        (directory / "consensus.csv").read_bytes()).hexdigest()
    assert manifest["metadata_sha256"] == hashlib.sha256(
        (directory / "metadata.json").read_bytes()).hexdigest()
    assert metadata["camera_calibration"]["frame_id"] == "test_camera"
    assert metadata["static_extrinsic"]["parent_frame"] == "test_base"
    assert metadata["camera_calibration"]["distortion"] == [0.] * 5


def test_dds_fixture_passes_real_bundle_loader_and_lane_matcher(tmp_path):
    # Execute the production pure loader unchanged; ROS construction is not
    # involved and this test is deliberately not claimed as DDS evidence.
    source = Path(__file__).parents[1] / "lane_mapping_lya_reference" / "follower_node.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    tree.body = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                 and node.name == "load_repeat_context"]
    namespace = dict(np=np, Path=Path, csv=csv, io=io, json=json, math=math,
                     hashlib=hashlib, GroundLookup=GroundLookup,
                     LaneMapLocalizer=LaneMapLocalizer)
    exec(compile(tree, str(source), "exec"), namespace)
    module = fixture_module()
    directory = tmp_path / "bundle"
    manifest, _ = module.synthetic_bundle(directory, "test_map")
    metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
    params = dict(visual_max_anchor_points=100000, visual_max_image_pixels=2500000,
                  visual_matcher_config_json="{}", refinement_publish_rate_hz=1.0)
    context = namespace["load_repeat_context"](directory / "bundle.json", manifest, metadata, params)
    mask = np.frombuffer(module.repeat_mask_pixels("test"), dtype=np.uint8).reshape(360, 640)
    points, sensitivity, _ = context["lookup"].project_mask_with_sensitivity(mask, context["threshold"])
    result = context["matcher"].match(points[sensitivity <= context["reliable_sensitivity"]],
                                      [0., 0., 0.], 1000000000)
    assert result.accepted, result.reason
    assert np.linalg.norm(result.pose_xyyaw[:2]) < 0.1
