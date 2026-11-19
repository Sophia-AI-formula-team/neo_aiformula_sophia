"""Offline comparison. Load only one model at a time; no ROS topics or camera."""
import argparse
from collections import Counter
import gc
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from yolop_a3_runtime import PerceptionModel


def signature(result):
    fields = ("lane_mask", "lane_overlay", "obstacle_mask", "object_overlay")
    return {**{k: hashlib.sha256(getattr(result, k).tobytes()).hexdigest() for k in fields},
            "boxes_and_scores": [d.to_dict() for d in result.detections],
            "light": result.traffic_light_state, "red": result.red_pixels_count,
            "status": result.temporal_status}


def run(paths, weights, device):
    cuda = torch.device(device).type == "cuda"
    references, errors, classes = {}, [], Counter()
    blocks = []
    for block, lean in enumerate((False, True, True, False)):
        rt = PerceptionModel(weights, device=device, lean_runtime=lean)
        first = cv2.imread(str(paths[0]))
        for _ in range(8):
            rt.predict(first, combined_overlay=not lean)
        if cuda:
            torch.cuda.synchronize(device)
            torch.cuda.reset_peak_memory_stats(device)
        elapsed = []
        for i, path in enumerate(paths):
            img = cv2.imread(str(path))
            if img is None:
                raise ValueError(f"Unreadable image: {path}")
            start = time.perf_counter()
            result = rt.predict(img, combined_overlay=not lean)
            if cuda:
                torch.cuda.synchronize(device)
            elapsed.append((time.perf_counter() - start) * 1000)
            obs = signature(result)
            if not block:
                references[i] = obs
                classes.update(d.class_name for d in result.detections)
            else:
                changed = [k for k, v in obs.items() if references[i][k] != v]
                if changed:
                    errors.append(dict(block=block, frame=i, fields=changed))
        blocks.append(dict(lean=lean, ms=elapsed, p50_p95_p99=np.percentile(elapsed,[50,95,99]).tolist(),
                           torch_peak_allocated_bytes=torch.cuda.max_memory_allocated(device) if cuda else None))
        del rt
        gc.collect()
        if cuda:
            torch.cuda.empty_cache()
    return dict(images=[p.name for p in paths], classes=classes, mismatches=errors, blocks=blocks,
                weights_sha256=hashlib.sha256(weights.read_bytes()).hexdigest(), torch=torch.__version__,
                device=torch.cuda.get_device_name(device) if cuda else device)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", nargs="+", required=True, type=Path)
    parser.add_argument("--weights", default=ROOT/"weights/shared_best.pth", type=Path)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--out", required=True, type=Path)
    args=parser.parse_args()
    paths=[]
    for p in args.images:
        if p.is_dir():
            paths.extend(sorted(f for f in p.iterdir() if f.suffix.lower() in {".png",".jpg",".jpeg"}))
        else:
            paths.append(p)
    if not paths:
        parser.error("No images found")
    torch.set_num_threads(2)
    cv2.setNumThreads(1)
    data=run(paths,args.weights,args.device)
    args.out.write_text(json.dumps(data,indent=1),encoding="utf8")
    for block in data["blocks"]:
        print("lean",block["lean"],"P50/P95/P99 ms",np.round(block["p50_p95_p99"],2),flush=True)
    print("images",len(paths),"mismatches",len(data["mismatches"]))
    return bool(data["mismatches"])


if __name__=="__main__":
    sys.exit(main())
