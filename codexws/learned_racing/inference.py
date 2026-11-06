"""Run the portable trained driver once on a local corridor. Prints right/left target RPM."""
import argparse
import json
from pathlib import Path

import numpy as np

from course import PREVIEW, encode_edges
from driving import COUNT, commands


def wheels(driver, left_m, right_m, speed_mps, yaw_rad_s):
    left, right = np.asarray(left_m, dtype=float), np.asarray(right_m, dtype=float)
    weights = np.asarray(driver["weights"], dtype=float)
    width = float(driver["edge_scale_m"])
    if driver.get("schema") != 1 or driver["preview_m"] != PREVIEW.tolist() or weights.shape != (COUNT,):
        raise ValueError("Incompatible driving policy or section distances")
    if left.shape != PREVIEW.shape or right.shape != PREVIEW.shape or np.any(left <= right):
        raise ValueError("Provide five ordered local left/right boundary pairs, left positive")
    if width <= 0 or speed_mps < 0 or not np.isfinite([*weights, *left, *right, speed_mps, yaw_rad_s, width]).all():
        raise ValueError("Forward speed and boundary measurements must be finite and valid")
    sides = encode_edges(left[None, :], right[None, :], width)
    r, l = commands(weights[None, :], sides, np.array([speed_mps]), np.array([yaw_rad_s]))
    return {"right_rpm": float(r[0]), "left_rpm": float(l[0])}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--driver", type=Path, default=Path(__file__).parent/"demo/driver.json")
    parser.add_argument("--left", nargs=5, type=float, required=True)
    parser.add_argument("--right", nargs=5, type=float, required=True)
    parser.add_argument("--speed", type=float, required=True)
    parser.add_argument("--yaw", type=float, required=True)
    args = parser.parse_args()
    driver = json.loads(args.driver.read_text(encoding="utf8"))
    print(json.dumps(wheels(driver, args.left, args.right, args.speed, args.yaw)))
