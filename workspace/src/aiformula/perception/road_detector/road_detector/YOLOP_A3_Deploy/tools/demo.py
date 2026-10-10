"""Run one image through the public A3 entrypoint and save both task outputs."""
import argparse
import json
from pathlib import Path
import sys

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from yolop_a3_runtime import PerceptionModel


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--weights", type=Path, default=ROOT / "weights/shared_best.pth")
    parser.add_argument("--device", default="auto", choices=("auto", "cpu", "cuda"))
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("Choose a new output directory; existing results are preserved")
    image = cv2.imread(str(args.image))
    if image is None:
        raise ValueError(f"Unreadable input image: {args.image}")
    result = PerceptionModel(args.weights, device=args.device).predict(image)
    args.output.mkdir(parents=True)
    for name, pixels in dict(rgb=image, lane_mask=result.lane_mask,
                             obstacle_mask=result.obstacle_mask,
                             lane_overlay=result.lane_overlay,
                             object_overlay=result.object_overlay,
                             combined_overlay=result.combined_overlay).items():
        if not cv2.imwrite(str(args.output / f"{name}.png"), pixels):
            raise OSError(f"Cannot write output image: {name}")
    (args.output / "detections.json").write_text(
        json.dumps(result.detections_payload(), indent=2), encoding="utf-8")
    print(f"A3 outputs saved: {args.output.resolve()}")


if __name__ == "__main__":
    main()
