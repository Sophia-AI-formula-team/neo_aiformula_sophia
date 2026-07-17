#!/usr/bin/env python3
import json
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
MASK_PATH = ROOT / "codex_road_mask.png"
REPORT_PATH = ROOT / "projection_audit.json"

# Values read from the active remote process/config and TF tree.
FX = 254.391622
FY = 254.391622
CX = 330.013020833
CY = 181.149637858
TRANSLATION = np.array([0.055, 0.060, 0.540], dtype=np.float64)
QUATERNION_XYZW = np.array([-0.496, 0.496, -0.504, 0.504], dtype=np.float64)


def quaternion_matrix(quaternion):
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def main():
    mask = cv2.imread(str(MASK_PATH), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise RuntimeError("missing mask: " + str(MASK_PATH))
    rows, cols = np.nonzero(mask)
    pixels = np.stack([cols, rows, np.ones_like(cols)], axis=0).astype(np.float64)
    inverse_k = np.linalg.inv(
        np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]], dtype=np.float64)
    )
    camera_rays = inverse_k @ pixels
    vehicle_rays = quaternion_matrix(QUATERNION_XYZW) @ camera_rays
    scales = -TRANSLATION[2] / vehicle_rays[2]
    points = TRANSLATION.reshape(3, 1) + vehicle_rays * scales
    forward = scales >= 0.0
    in_roi = (
        forward
        & (points[0] >= 0.0)
        & (points[0] <= 10.0)
        & (points[1] >= -4.0)
        & (points[1] <= 4.0)
        & np.all(np.isfinite(points), axis=0)
    )
    left = cols < CX
    report = {
        "mask_size": [int(mask.shape[1]), int(mask.shape[0])],
        "mask_nonzero_pixels": int(mask.astype(bool).sum()),
        "positive_ground_intersections": int(forward.sum()),
        "projected_points_inside_active_roi": int(in_roi.sum()),
        "left_half_inside_roi": int((in_roi & left).sum()),
        "right_half_inside_roi": int((in_roi & ~left).sum()),
        "intrinsics": {"fx": FX, "fy": FY, "cx": CX, "cy": CY},
        "base_to_camera_translation": TRANSLATION.tolist(),
        "base_to_camera_quaternion_xyzw": QUATERNION_XYZW.tolist(),
    }
    REPORT_PATH.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
