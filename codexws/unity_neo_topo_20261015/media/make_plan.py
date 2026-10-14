"""Seal one completed genuine Unity run; no rendering and no acceptance inference."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil

from media_common import (RENDERER, RENDERER_SHA, SCHEMA, beneath, check_policy,
                          digest, read_json, referenced_files, rows, schedule, write_json)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--name", choices=("INNER", "OUTER"), required=True)
    parser.add_argument("--versions", required=True, type=Path)
    parser.add_argument("--policy", required=True, type=Path)
    parser.add_argument("--output-directory", required=True, type=Path)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--speed", type=int, choices=(1, 2, 4), default=4)
    parser.add_argument("--renderer", type=Path, default=RENDERER,
                        help="Verbatim pinned renderer; defaults to packaged shared/recording_renderer.py")
    parser.add_argument("--ffmpeg", type=Path,
                        help="FFmpeg executable; alternatively AIFORMULA_FFMPEG or ffmpeg on PATH")
    args = parser.parse_args()
    source = args.source.resolve()
    if args.output_directory.exists() or args.plan.exists():
        raise FileExistsError("Use a new output directory and plan")
    policy = read_json(args.policy)
    check_policy(source, policy)
    frames, trace, motion, events = [rows(beneath(source, name)) for name in
                                    ("frames.jsonl", "trace.jsonl", "motion.jsonl", "events.jsonl")]
    if len(frames) < 2 or frames[-1]["wall"] <= frames[0]["wall"] or not trace or not motion:
        raise ValueError("Actual completed camera, detection and motion records required")
    for row in trace + motion:
        if row.get("semantic") is not None and not isinstance(row["semantic"], dict):
            raise ValueError("Semantic record must be an object or null")
        status = (row.get("semantic") or {}).get("status") or {}
        if status.get("traffic_policy") not in (None, "stop_wait"):
            raise ValueError("Mixed semantic traffic mode in recording")
        approach = status.get("signal_approach") or {}
        if approach.get("enabled") is True:
            raise ValueError("Signal approach must remain disabled in stop_wait recording")
    names = {"frames.jsonl", "trace.jsonl", "motion.jsonl", "events.jsonl", "semantic_ticks.jsonl", "result.json"}
    names.update(check["file"] for check in policy["checks"])
    names.update(referenced_files(source, frames, trace))
    hashes = {name: digest(beneath(source, name)) for name in sorted(names)}
    renderer = args.renderer.resolve()
    if digest(renderer) != RENDERER_SHA:
        raise RuntimeError("Shared renderer changed; review required")
    ffmpeg_value = args.ffmpeg or os.environ.get("AIFORMULA_FFMPEG") or shutil.which("ffmpeg")
    if not ffmpeg_value:
        raise ValueError("Specify --ffmpeg, AIFORMULA_FFMPEG or install ffmpeg on PATH")
    ffmpeg = Path(ffmpeg_value).resolve()
    if not ffmpeg.is_file():
        raise FileNotFoundError(ffmpeg)
    versions = read_json(args.versions)
    if not isinstance(versions, dict) or not versions:
        raise ValueError("Actual source-version manifest required")
    plan = {"schema": SCHEMA, "source": str(source), "name": args.name,
            "output_directory": str(args.output_directory.resolve()),
            "output_filename": f"{args.name.lower()}_neo_stop_wait.mp4",
            "renderer": str(renderer), "renderer_sha256": RENDERER_SHA,
            "ffmpeg": str(ffmpeg), "ffmpeg_sha256": digest(ffmpeg),
            "fps": 15, "width": 1280, "height": 960, "unobscured_observer_rectangle": [0, 0, 1280, 720],
            "policy": policy, "policy_file": str(args.policy.resolve()), "policy_sha256": digest(args.policy),
            "versions": versions, "versions_file": str(args.versions.resolve()), "versions_sha256": digest(args.versions),
            "source_sha256": hashes, "recorded_result": read_json(beneath(source, "result.json")),
            "segments": schedule(frames[0]["wall"], frames[-1]["wall"], events, motion, args.speed),
            "scripts_sha256": {name: digest(Path(__file__).parent / name) for name in
                               ("make_plan.py", "media_common.py", "build_video.py")},
            "acceptance_inferred_by_renderer": False, "status": "sealed; encoding and visual QA not run"}
    args.plan.parent.mkdir(parents=True, exist_ok=True)
    write_json(args.plan, plan)
    print(str(args.plan.resolve()))


if __name__ == "__main__":
    main()
