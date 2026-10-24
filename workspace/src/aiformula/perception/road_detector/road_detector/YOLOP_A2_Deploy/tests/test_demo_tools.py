from argparse import ArgumentTypeError
from pathlib import Path

import cv2
import numpy as np
import pytest

from tools.demo import (
    DemoInputError,
    _discover_images,
    _positive_integer,
    _probability,
    _publish_staged_output,
    _validate_output_target,
    _validate_source_output_boundary,
)
from tools.demo_board import build_demo_board


def test_probability_rejects_values_outside_closed_unit_interval() -> None:
    assert _probability("0") == 0.0
    assert _probability("1") == 1.0
    with pytest.raises(ArgumentTypeError, match="between 0 and 1"):
        _probability("1.1")
    with pytest.raises(ArgumentTypeError, match="between 0 and 1"):
        _probability("nan")


def test_positive_integer_rejects_zero() -> None:
    assert _positive_integer("3") == 3
    with pytest.raises(ArgumentTypeError, match="positive"):
        _positive_integer("0")


def test_discover_images_is_recursive_and_stable(tmp_path: Path) -> None:
    nested = tmp_path / "nested"
    nested.mkdir()
    (nested / "b.PNG").write_bytes(b"image")
    (tmp_path / "a.jpg").write_bytes(b"image")
    (tmp_path / "notes.txt").write_text("ignored", encoding="utf-8")

    assert [path.name for path in _discover_images(tmp_path)] == ["a.jpg", "b.PNG"]


def test_source_and_output_directories_must_be_disjoint(tmp_path: Path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    with pytest.raises(DemoInputError, match="must be disjoint"):
        _validate_source_output_boundary(source, source / "results")


def test_source_file_cannot_live_inside_replaced_output(tmp_path: Path) -> None:
    output = tmp_path / "output"
    source = output / "lane_mask" / "source.png"
    source.parent.mkdir(parents=True)
    source.write_bytes(b"image")
    with pytest.raises(DemoInputError, match="must be disjoint"):
        _validate_source_output_boundary(source, output)


def test_output_with_unmanaged_entries_is_not_replaced(tmp_path: Path) -> None:
    output = tmp_path / "output"
    output.mkdir()
    (output / "keep.txt").write_text("user file", encoding="utf-8")
    with pytest.raises(DemoInputError, match="unmanaged entries"):
        _validate_output_target(output)


def test_staged_output_replaces_stale_managed_results(tmp_path: Path) -> None:
    output = tmp_path / "output"
    stale = output / "lane_mask"
    stale.mkdir(parents=True)
    (stale / "old.png").write_bytes(b"old")
    (output / "summary.json").write_text("{}", encoding="utf-8")

    staging = tmp_path / "staging"
    current = staging / "lane_mask"
    current.mkdir(parents=True)
    (current / "new.png").write_bytes(b"new")
    (staging / "summary.json").write_text('{"images": []}', encoding="utf-8")

    _validate_output_target(output)
    _publish_staged_output(staging, output)

    assert not (output / "lane_mask" / "old.png").exists()
    assert (output / "lane_mask" / "new.png").read_bytes() == b"new"
    assert not staging.exists()


def test_publish_restores_previous_output_when_staging_is_missing(
    tmp_path: Path,
) -> None:
    output = tmp_path / "output"
    output.mkdir()
    (output / "summary.json").write_text('{"old": true}', encoding="utf-8")
    missing_staging = tmp_path / "missing-staging"

    with pytest.raises(FileNotFoundError):
        _publish_staged_output(missing_staging, output)

    assert (output / "summary.json").read_text(encoding="utf-8") == '{"old": true}'


def test_demo_board_is_built_from_current_outputs(tmp_path: Path) -> None:
    source = tmp_path / "source.jpg"
    image = np.full((80, 120, 3), 90, dtype=np.uint8)
    assert cv2.imwrite(str(source), image)
    stem = "0000_source"
    for directory in ("combined_overlay", "object_overlay"):
        path = tmp_path / directory
        path.mkdir()
        assert cv2.imwrite(str(path / f"{stem}.jpg"), image)
    lane_directory = tmp_path / "lane_mask"
    obstacle_directory = tmp_path / "obstacle_mask"
    lane_directory.mkdir()
    obstacle_directory.mkdir()
    lane = np.zeros((80, 120), dtype=np.uint8)
    obstacle = np.zeros_like(lane)
    lane[:, 20:24] = 255
    obstacle[40:70, 70:110] = 255
    assert cv2.imwrite(str(lane_directory / f"{stem}.png"), lane)
    assert cv2.imwrite(str(obstacle_directory / f"{stem}.png"), obstacle)

    board_path = build_demo_board(
        tmp_path,
        [
            {
                "source": str(source),
                "traffic_light_state": "unknown",
                "detections": [{"semantic_label": "red_traffic_cone"}],
            }
        ],
    )

    board = cv2.imread(str(board_path), cv2.IMREAD_COLOR)
    assert board is not None
    assert board.shape[0] > 400
    assert board.shape[1] == 1280
