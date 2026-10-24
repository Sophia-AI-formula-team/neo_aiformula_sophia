from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
import numpy as np


TILE_WIDTH = 640
TILE_HEIGHT = 400
LABEL_HEIGHT = 42


def build_demo_board(
    demo_root: Path,
    images: list[dict[str, Any]],
    *,
    max_images: int = 8,
) -> Path:
    if not images:
        raise ValueError("cannot build a demo board without image results")
    if max_images < 1:
        raise ValueError("max_images must be positive")

    selected = images[:max_images]
    tiles = [
        _result_tile(demo_root, index, payload)
        for index, payload in enumerate(selected)
    ]
    tiles.append(_mask_tile(demo_root, selected))
    if len(tiles) % 2:
        tiles.append(np.full_like(tiles[0], 24))

    rows = [
        np.hstack((tiles[index], tiles[index + 1]))
        for index in range(0, len(tiles), 2)
    ]
    body = np.vstack(rows)
    title_height = 66
    board = np.full(
        (body.shape[0] + title_height, body.shape[1], 3),
        16,
        dtype=np.uint8,
    )
    board[title_height:] = body
    cv2.putText(
        board,
        f"YOLOP_A2 | lane segmentation + object detection | {len(images)} images",
        (24, 43),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (245, 245, 245),
        2,
        cv2.LINE_AA,
    )

    board_path = demo_root / "demo_board.jpg"
    if not cv2.imwrite(
        str(board_path),
        board,
        [cv2.IMWRITE_JPEG_QUALITY, 92],
    ):
        raise OSError(f"failed to save demo board: {board_path}")
    return board_path


def _result_tile(
    demo_root: Path,
    index: int,
    payload: dict[str, Any],
) -> np.ndarray:
    source = Path(str(payload["source"]))
    output_kind = (
        "object_overlay"
        if payload.get("traffic_light_state") in {"red", "green"}
        else "combined_overlay"
    )
    image_path = demo_root / output_kind / f"{index:04d}_{source.stem}.jpg"
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"failed to read demo overlay: {image_path}")

    detections = payload.get("detections", [])
    labels = list(
        dict.fromkeys(
            str(detection.get("semantic_label", ""))
            for detection in detections
            if isinstance(detection, dict) and detection.get("semantic_label")
        )
    )
    detail = ", ".join(labels[:2]) if labels else "no object detections"
    return _labeled_tile(image, f"{source.stem}: {detail}")


def _mask_tile(
    demo_root: Path,
    images: list[dict[str, Any]],
) -> np.ndarray:
    selected: tuple[np.ndarray, np.ndarray] | None = None
    for index, payload in enumerate(images):
        source = Path(str(payload["source"]))
        stem = f"{index:04d}_{source.stem}.png"
        lane = cv2.imread(str(demo_root / "lane_mask" / stem), cv2.IMREAD_GRAYSCALE)
        obstacle = cv2.imread(
            str(demo_root / "obstacle_mask" / stem),
            cv2.IMREAD_GRAYSCALE,
        )
        if lane is None or obstacle is None:
            raise ValueError(f"failed to read demo masks for {source}")
        selected = (lane, obstacle)
        if np.count_nonzero(lane) and np.count_nonzero(obstacle):
            break

    if selected is None:
        raise ValueError("demo mask outputs are empty")
    lane, obstacle = selected
    lane_color = np.zeros((*lane.shape, 3), dtype=np.uint8)
    lane_color[lane > 0] = (40, 40, 235)
    obstacle_color = np.zeros((*obstacle.shape, 3), dtype=np.uint8)
    obstacle_color[obstacle > 0] = (0, 165, 255)
    half_width = TILE_WIDTH // 2
    content_height = TILE_HEIGHT - LABEL_HEIGHT
    content = np.hstack(
        (
            _fit_image(lane_color, half_width, content_height),
            _fit_image(obstacle_color, TILE_WIDTH - half_width, content_height),
        )
    )
    return _labeled_tile(
        content,
        "Independent masks: lane (left) | obstacles (right)",
    )


def _labeled_tile(image: np.ndarray, label: str) -> np.ndarray:
    tile = np.full((TILE_HEIGHT, TILE_WIDTH, 3), 24, dtype=np.uint8)
    tile[LABEL_HEIGHT:] = _fit_image(
        image,
        TILE_WIDTH,
        TILE_HEIGHT - LABEL_HEIGHT,
    )
    cv2.putText(
        tile,
        _fit_label(label, TILE_WIDTH - 28),
        (14, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.68,
        (245, 245, 245),
        1,
        cv2.LINE_AA,
    )
    return tile


def _fit_image(image: np.ndarray, width: int, height: int) -> np.ndarray:
    if image.size == 0:
        raise ValueError("cannot place an empty image on the demo board")
    canvas = np.full((height, width, 3), 24, dtype=np.uint8)
    source_height, source_width = image.shape[:2]
    scale = min(width / source_width, height / source_height)
    target_width = max(1, int(round(source_width * scale)))
    target_height = max(1, int(round(source_height * scale)))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(image, (target_width, target_height), interpolation=interpolation)
    left = (width - target_width) // 2
    top = (height - target_height) // 2
    canvas[top : top + target_height, left : left + target_width] = resized
    return canvas


def _fit_label(label: str, maximum_width: int) -> str:
    candidate = label
    while candidate:
        width = cv2.getTextSize(
            candidate,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            1,
        )[0][0]
        if width <= maximum_width:
            return candidate
        candidate = candidate[:-4].rstrip() + "..."
    return "..."
