"""Small, ROS-free checks for the offline replay's input/output boundaries."""

import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


@pytest.fixture
def replay():
    path = Path(__file__).parents[1] / "scripts" / "mask_localization_replay.py"
    spec = importlib.util.spec_from_file_location("mask_localization_replay_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_static_transform_uses_only_supplied_past_edges(replay):
    a = np.eye(4)
    a[:3, 3] = [1, 2, 3]
    b = np.eye(4)
    b[:3, 3] = [0.5, 0.0, 0.0]
    assert np.allclose(replay.resolve_static_transform({("base", "mount"): a,
                                                        ("mount", "camera"): b},
                                                       "base", "camera"), a @ b)
    with pytest.raises(ValueError, match="no camera-to-base"):
        replay.resolve_static_transform({("base", "mount"): a}, "base", "camera")


def test_mask_decode_keeps_step_padding_out_of_geometry(replay):
    message = SimpleNamespace(encoding="mono8", width=2, height=2, step=4,
                              data=bytes([0, 255, 99, 99, 255, 0, 88, 88]))
    assert replay.mask_array(message).tolist() == [[0, 255], [255, 0]]


def test_mask_decode_rejects_truncated_and_nonbinary_inputs(replay):
    message = SimpleNamespace(encoding="rgb8", width=2, height=2, step=2, data=b"1234")
    with pytest.raises(ValueError, match="encoding"):
        replay.mask_array(message)
    message.encoding, message.data = "mono8", b"12"
    with pytest.raises(ValueError, match="truncated"):
        replay.mask_array(message)


def test_metrics_are_valid_json_even_for_rejected_infinite_residual(replay):
    clean = replay.clean_metrics({"residual": float("inf"), "values": np.array([1., np.nan])})
    assert clean == {"residual": None, "values": [1., None]}


def test_renderer_cannot_see_lanes_behind_camera(replay):
    behind = np.asarray([[-3., 0.5], [-2., -0.5]])
    assert not replay.render_synthetic_mask(behind, np.zeros(3)).any()


def test_map_quality_exposes_loss_of_coverage(replay):
    truth = np.asarray([[0., 0.], [1., 0.], [2., 0.]])
    filtered = replay.map_quality(truth[:1], truth)
    assert filtered["point_count"] == 1
    assert filtered["points_within_20cm_fraction"] == 1.0
    assert filtered["truth_within_20cm_coverage_fraction"] == pytest.approx(1. / 3.)
    empty = replay.map_quality(np.empty((0, 2)), truth)
    assert empty["point_distance_to_truth_m"] is None
    assert empty["truth_within_20cm_coverage_fraction"] == 0.0


def test_nearest_distances_are_evaluation_only_and_correct(replay):
    points = np.asarray([[1., 1.], [3., 4.]])
    reference = np.asarray([[0., 0.]])
    assert replay.nearest_distances(points, reference) == pytest.approx([2. ** .5, 5.])


def test_file_never_imports_ros_or_writes_source_bag(replay):
    source = Path(replay.__file__).read_text(encoding="utf-8")
    module = ast.parse(source, feature_version=(3, 8))
    imported = []
    for node in ast.walk(module):
        if isinstance(node, ast.Import):
            imported.extend(item.name for item in node.names)
        elif isinstance(node, ast.ImportFrom):
            imported.append(node.module)
    assert "rclpy" not in imported
    assert '"?mode=ro"' in source
    assert 'PRAGMA query_only=ON' in source
    assert 'LaneMapLocalizer(anchor_xy)' in source
    assert 'fuse(map_builder, local_xy, sensitivity, prior, source)' in source
    assert '"refinement_reused_for_localization": False' in source


def test_recorded_pipeline_defaults_match_runtime_age_contract(replay):
    assert replay.replay_bag.__defaults__ == (60.0, 500.0)
    source = Path(replay.__file__).read_text(encoding="utf-8")
    assert "latest_not_after(source, sequence, 150000000)" in source
    assert "anchor_update_after_freeze" in source


def test_rendered_mask_known_drift_improves_true_position_not_only_fit(replay, monkeypatch, tmp_path):
    # Exercise the exact GroundLookup -> first-lap consensus -> frozen matcher
    # path. Skip only plot generation and expensive independent map scoring.
    monkeypatch.setattr(replay, "plot_synthetic", lambda *args: None)
    monkeypatch.setattr(replay, "plot_map_quality", lambda *args: None)
    monkeypatch.setattr(replay, "map_quality", lambda points, truth: {"point_count": len(points)})
    report = replay.synthetic_experiment(tmp_path)
    assert report["anchor_cells"] > 1000
    # Wall-clock budget remains the production 40 ms; allow occasional CI
    # scheduling rejections without relaxing it or asserting portable timing.
    assert report["accepted_frames"] >= 128
    assert report["accepted_position_rmse_m"] < 0.9 * report["same_accepted_subset_prior_position_rmse_m"]
    errors = report["error_components"]
    assert errors["accepted_frames_corrected_lateral_rmse_m"] < 0.08
    assert abs(errors["accepted_frames_corrected_longitudinal_rmse_m"]
               - errors["accepted_frames_prior_longitudinal_rmse_m"]) < 0.02
    assert report["map_quality"]["accepted_matched_refinement_3vote"]["point_count"] > 1000
    assert report["anchor_update_after_freeze"] is False
