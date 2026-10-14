"""Render a sealed stop_wait run under the root's bounded process supervisor."""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
from fractions import Fraction
import re
import subprocess
import time

from media_common import SCHEMA, beneath, check_policy, digest, latest, read_json, rows, write_json


def verify_sources(plan):
    source = Path(plan["source"])
    for name, expected in plan["source_sha256"].items():
        if digest(beneath(source, name)) != expected:
            raise RuntimeError(f"Recorded source changed: {name}")
    for key in ("renderer", "ffmpeg", "policy", "versions"):
        path = plan[key + "_file"] if key in ("policy", "versions") else plan[key]
        if digest(path) != plan[key + "_sha256"]:
            raise RuntimeError(f"Pinned dependency changed: {path}")
    for name, expected in plan["scripts_sha256"].items():
        if digest(Path(__file__).parent / name) != expected:
            raise RuntimeError(f"Reviewed media script changed: {name}")
    check_policy(source, plan["policy"])


def load_renderer(plan):
    spec = importlib.util.spec_from_file_location("neo_recording_shared_renderer", plan["renderer"])
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if (module.WIDTH, module.HEIGHT, module.MAIN_HEIGHT) != (1280, 960, 720):
        raise RuntimeError("Shared renderer layout changed")
    return module


def short(value, absent="--"):
    if value is None:
        return absent
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, dict):
        return json.dumps(value, separators=(",", ":"), ensure_ascii=True)
    return str(value)


def first(value, names):
    for name in names:
        if isinstance(value, dict) and name in value:
            return value[name]
    return None


class Recording:
    def __init__(self, plan, renderer):
        self.plan, self.r = plan, renderer
        self.trial = renderer.Trial(Path(plan["source"]))
        self.trace_times = [row["wall"] for row in self.trial.trace]
        self.events = rows(beneath(plan["source"], "events.jsonl"))
        self.event_times = [row["wall"] for row in self.events]
        self.configuration = read_json(beneath(plan["source"], "configuration.json"))
        self.result = plan["recorded_result"]
        self.versions_label = " | ".join(f"{key}={short(value)}" for key, value in plan["versions"].items()
                                        if isinstance(value, (str, int, float))) or "see sealed source manifest"

    def compose(self, wall, speed):
        r = self.r
        frame, motion, fi, mi = self.trial.at(wall)
        detection, di = latest(self.trial.trace, self.trace_times, wall)
        event, ei = latest(self.events, self.event_times, wall)
        sem = motion.get("semantic") or {}
        status = sem.get("status") or {}
        motion_elapsed = wall - motion["wall"] if motion else 0
        runtime = r.semantic_runtime(motion, motion_elapsed)
        detection_elapsed = wall - detection["wall"] if detection else None
        age = r.finite(detection.get("age"))
        age = age + detection_elapsed if age is not None and detection_elapsed is not None else None
        capture = detection.get("junction_capture") or {}
        capture_epoch = capture.get("epoch", detection.get("epoch"))
        ui_epoch = motion.get("epoch")
        stale_epoch = capture_epoch is not None and ui_epoch is not None and capture_epoch != ui_epoch
        canvas = r.Image.new("RGB", (1280, 960), r.BACKGROUND)
        r.paste_contained(canvas, self.trial.image(frame["path"]), (0, 0, 1280, 720))
        draw = r.ImageDraw.Draw(canvas)
        for x in (320, 640, 960):
            draw.line((x, 720, x, 960), fill=r.GRID, width=2)

        perception_label = "A3 lanes + RGB signal helper" if self.configuration.get("signal_panel_rgb") is True else "A3 recorded overlay"
        r.text(draw, (10, 727), perception_label + (" [STALE]" if stale_epoch else ""), "small", width=300)
        if detection.get("overlay"):
            r.paste_contained(canvas, self.trial.image(detection["overlay"]), (8, 750, 312, 921))
        else:
            r.text(draw, (10, 812), "No detection received", "small", r.MUTED, 300)
        r.text(draw, (10, 925), f"{self.plan['name']} | {speed}x | wall {wall:.2f}s | age {r.number(age)}s", "tiny", r.WHITE, 300)
        r.text(draw, (10, 943), f"seq {detection.get('sequence', '--')} | capture e{capture_epoch} / state e{ui_epoch}", "tiny", r.ORANGE if stale_epoch else r.MUTED, 300)

        lane_row = dict(motion)
        lane_row["candidate"] = detection.get("candidate")
        lane_row["semantic_result_age"] = runtime["age"]
        # Display-only copied state: a held or old-generation response remains gray.
        if stale_epoch:
            lane_row["semantic_running"] = False
        r.draw_lane(draw, 320, lane_row, None if stale_epoch else age)

        stage = motion.get("stage") or detection.get("stage") or motion.get("phase") or frame.get("phase")
        case_kind = motion.get("case_kind") or detection.get("case_kind") or self.result.get("case_kind")
        traffic = first(motion, ("traffic_stop_required",))
        if traffic is None:
            traffic = sem.get("traffic_stop_required", status.get("traffic_stop_required"))
        command_gate = motion.get("command_gate", sem.get("command_gate"))
        gate_label = first(command_gate, ("reason", "state", "mode")) if isinstance(command_gate, dict) else command_gate
        scenario = first(motion, ("signal_truth", "light", "signal_state", "scenario_light"))
        detected = detection.get("detected_light")
        color = r.MUTED if runtime["frozen"] else r.PURPLE
        r.text(draw, (650, 727), "Semantic | " + runtime["label"], "title", color, 300)
        field_rows = [
            (f"stop_wait/{short(status.get('traffic_mode'))} | {short(case_kind)}", r.WHITE),
            (f"Stage: {short(stage)}", r.WHITE),
            (f"stop_required: {short(traffic)}", r.ORANGE if traffic is True else color),
            (f"Detected: {short(detected)} | {short(detection.get('detected_light_source'))}", r.MUTED),
            (f"Gate: {'DIAGNOSTIC HOLD / ' if motion.get('diagnostic_hold') else ''}{short(gate_label)}", r.ORANGE if motion.get("diagnostic_hold") else color),
            (f"Scenario lamp: {short(scenario)} (evaluation only)", r.MUTED),
            (f"LYA v/w: {r.number(sem.get('controller_v'))} / {r.number(sem.get('controller_w'))}", r.PINK),
            (f"Applied v/w: {r.number(motion.get('applied_v'))} / {r.number(motion.get('applied_w'))}", r.WHITE),
            (f"Actual v/w: {r.number(motion.get('actual_v'))} / {r.number(motion.get('actual_w'))}", r.CYAN),
            (f"generation {short(motion.get('worker_generation'))} | tick {short(sem.get('sequence'))}", r.MUTED),
            (f"semantic age: {r.number(runtime['age'])}s", r.MUTED),
            (f"Event: {short(first(event, ('kind', 'event', 'type')))}", r.MUTED),
        ]
        for index, (value, fill) in enumerate(field_rows):
            r.text(draw, (650, 750 + index * 17), value, "tiny", fill, 300)

        closure = motion.get("topo_closure") or status.get("closure") or {}
        persistence = motion.get("persistence") or status.get("persistence") or {}
        restore_event = motion.get("restore_event") or sem.get("restore_event") or {}
        fusion = status.get("map_fusion") or {}
        r.text(draw, (970, 727), "Semantic topo + persistence", "title", color, 300)
        fields = [
            ("Closure: " + short(first(closure, ("state", "status", "reason"))), color),
            ("Blocker: " + short(closure.get("blocker")), r.ORANGE if closure.get("blocker") else r.MUTED),
            ("Samples: " + short(closure.get("recorded_samples")), r.MUTED),
            ("Travel / start gap: " + r.number(closure.get("recorded_distance_m")) + " / " + r.number(closure.get("distance_to_start_m")) + "m", r.MUTED),
            ("Persistence: " + short(first(persistence, ("state", "status", "reason"))), color),
            ("Saved samples: " + short(persistence.get("saved_samples")), r.MUTED),
            ("Resume gap: " + r.number(persistence.get("resume_distance_m")) + "m | err: " + short(persistence.get("error")), color),
            ("OSM fusion: " + short(fusion.get("available")) + " | " + short(fusion.get("reason")), r.MUTED),
            ("Restore event: " + short(restore_event.get("observed")) + " @ tick " + short(restore_event.get("tick_sequence")), color),
        ]
        for index, (value, fill) in enumerate(fields):
            r.text(draw, (970, 750 + index * 17), value, "tiny", fill, 300)
        r.lines(draw, (970, 907), "Source: " + self.versions_label, 300, 3, style="tiny", fill=r.MUTED)
        if wall >= self.trial.end - 2:
            shutdown = self.result.get("finalShutdownReceipt") or {}
            acknowledgement = shutdown.get("acknowledgement") or {}
            checkpoint = acknowledgement.get("persistence") or {}
            cleanup_errors = self.result.get("cleanupErrors") or []
            failed_cleanup = bool(cleanup_errors) or acknowledgement.get("checkpoint_ok") is False or shutdown.get("clean") is False
            top = 906 if failed_cleanup else 923
            draw.rectangle((0, top, 1280, 960), fill=(25, 31, 39))
            r.text(draw, (10, top + 2), f"{self.plan['name']} | recorded accepted={short(self.result.get('accepted'))} | topo closed={short(self.result.get('firstLapClosed'))} | {speed}x | {short(case_kind)}", "small", r.WHITE, 1260)
            r.text(draw, (10, top + 20), "Recorded reason: " + short(self.result.get("reason")), "tiny", r.ORANGE, 1260)
            if failed_cleanup:
                terminal_failure = "Checkpoint: " + short(checkpoint.get("state")) + " / " + short(checkpoint.get("error"))
                terminal_failure += " | cleanup: " + "; ".join(str(item) for item in cleanup_errors)
                r.text(draw, (10, top + 37), terminal_failure, "tiny", r.ORANGE, 1260)
        return canvas, {"source_wall": wall, "playback_speed": speed, "observer_index": fi,
                        "observer_file": frame["path"], "observer_wall": frame["wall"],
                        "motion_index": mi, "motion_wall": motion.get("wall"), "detection_index": di,
                        "detection_wall": detection.get("wall"), "sequence": detection.get("sequence"),
                        "overlay": detection.get("overlay"), "capture_epoch": capture_epoch, "state_epoch": ui_epoch,
                        "stale_epoch": stale_epoch, "perception_age": age, "semantic_age": runtime["age"],
                        "semantic_running": motion.get("semantic_running"), "worker_generation": motion.get("worker_generation"),
                        "worker_boot_id": motion.get("worker_boot_id", sem.get("worker_boot_id")),
                        "stage": stage, "case_kind": case_kind, "event_index": ei,
                        "traffic_stop_required": traffic, "topo_closure": closure, "persistence": persistence,
                        "restore_event": restore_event}


def decode_audit(stderr, expected):
    header = stderr.split("Stream mapping:", 1)[0]
    codecs = re.findall(r"(?m)^\s*Stream #0:\d+[^\n]*?\bVideo:\s*([^\s,(]+)", header)
    if codecs != ["h264"]:
        raise RuntimeError(f"Unexpected decoded codec {codecs}")
    timing = re.findall(r"\[Parsed_showinfo_[^\]]+\]\s+config in time_base:\s*(\d+/\d+),\s*frame_rate:\s*(\d+/\d+)", stderr)
    if not timing or len(set(timing)) != 1:
        raise RuntimeError("Missing/changing decoded timing")
    time_base, rate = map(Fraction, timing[0])
    if time_base <= 0 or rate != 15:
        raise RuntimeError("Incorrect decoded frame rate")
    parsed = re.findall(r"\[Parsed_showinfo_[^\]]+\]\s+n:\s*(\d+)\s+pts:\s*(-?\d+)\b[^\n]*?\bfmt:\s*(\S+)[^\n]*?\bs:\s*(\d+)x(\d+)\b", stderr)
    if len(parsed) != expected:
        raise RuntimeError(f"Full decode count {len(parsed)} != {expected}")
    for index, (number, pts, fmt, width, height) in enumerate(parsed):
        if int(number) != index or (fmt, int(width), int(height)) != ("yuv420p", 1280, 960) or int(pts) * time_base != Fraction(index, 15):
            raise RuntimeError(f"Decoded frame {index} timing/format mismatch")
    return {"frame_count": expected, "codec": "h264", "format": "yuv420p", "width": 1280, "height": 960,
            "time_base": str(time_base), "frame_rate": str(rate), "every_frame_index_and_pts_checked": True,
            "pts_rule": "exact n/15 seconds", "decode_resize_fps_pixfmt_forced": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--permit-encoding", action="store_true")
    args = parser.parse_args()
    if not args.permit_encoding or os.environ.get("AIFORMULA_NEO_MEDIA_BOUNDED") != "1":
        raise RuntimeError("Root must explicitly run this under its bounded owned process supervisor")
    started = time.monotonic()
    plan = read_json(args.plan)
    if plan.get("schema") != SCHEMA:
        raise RuntimeError("Unexpected plan schema")
    verify_sources(plan)
    r = load_renderer(plan)
    recording = Recording(plan, r)
    segments = plan["segments"]
    if not segments or segments[0]["start"] != recording.trial.start or segments[-1]["end"] != recording.trial.end:
        raise RuntimeError("Schedule does not cover actual camera extent")
    for index, segment in enumerate(segments):
        if segment["speed"] not in (1, 2, 4) or segment["end"] <= segment["start"] or (index and segment["start"] != segments[index - 1]["end"]):
            raise RuntimeError("Schedule contains a gap, overlap or unsupported rate")
    output = Path(plan["output_directory"])
    output.mkdir(parents=True, exist_ok=False)
    destination = output / plan["output_filename"]
    command = [plan["ffmpeg"], "-hide_banner", "-loglevel", "warning", "-n", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "1280x960", "-r", "15", "-i", "pipe:0", "-an", "-c:v", "libx264", "-threads", "4", "-preset", "fast", "-crf", "19", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(destination)]
    count, output_segments, samples = 0, [], set()
    pending_event_times = sorted({event["wall"] for event in recording.events
                                  if recording.trial.start <= event["wall"] <= recording.trial.end})
    next_event = 0
    with (output / "encode.log").open("xb") as log, (output / "timeline.jsonl").open("x", encoding="utf-8") as timeline:
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=log)
        try:
            for segment in segments:
                frames = math.ceil((segment["end"] - segment["start"]) * 15 / segment["speed"])
                offset = count
                samples.update((offset, offset + frames // 2, offset + frames - 1))
                for local in range(frames):
                    wall = min(segment["end"], segment["start"] + local * segment["speed"] / 15)
                    image, mapping = recording.compose(wall, segment["speed"])
                    process.stdin.write(image.tobytes())
                    while next_event < len(pending_event_times) and pending_event_times[next_event] <= wall:
                        samples.update((max(0, count - 1), count))
                        next_event += 1
                    mapping.update(output_frame=count, output_time=count / 15)
                    timeline.write(json.dumps(mapping, allow_nan=False) + "\n")
                    count += 1
                output_segments.append(dict(segment, output_start_frame=offset, output_frames=frames))
            # Include the exact last camera sample rather than silently truncating it.
            image, mapping = recording.compose(recording.trial.end, segments[-1]["speed"])
            process.stdin.write(image.tobytes())
            mapping.update(output_frame=count, output_time=count / 15)
            timeline.write(json.dumps(mapping, allow_nan=False) + "\n")
            samples.add(count)
            count += 1
        finally:
            process.stdin.close()
        if process.wait() != 0:
            raise RuntimeError("FFmpeg encode failed; partial output retained")
    samples.update(range(0, count, 15 * 15))
    qa = output / "qa"
    qa.mkdir()
    expression = "+".join(f"eq(n\\,{index})" for index in sorted(samples))
    decode_command = [plan["ffmpeg"], "-hide_banner", "-v", "info", "-nostats", "-xerror", "-i", str(destination), "-progress", "pipe:1", "-map", "0:v:0", "-vf", "showinfo", "-f", "null", os.devnull, "-map", "0:v:0", "-vf", "select=" + expression, "-vsync", "0", "-compression_level", "1", str(qa / "decoded_%04d.png")]
    decoded = subprocess.run(decode_command, capture_output=True, text=True)
    (output / "decode.log").write_text(decoded.stderr, encoding="utf-8")
    if decoded.returncode:
        raise RuntimeError("Full decode failed; diagnostic log retained")
    progress = dict(line.split("=", 1) for line in decoded.stdout.splitlines() if "=" in line)
    if progress.get("progress") != "end" or int(progress.get("frame", -1)) != count:
        raise RuntimeError("Full decode progress did not finish at exact expected frame count")
    audit = decode_audit(decoded.stderr, count)
    images = sorted(qa.glob("decoded_*.png"))
    if len(images) != len(samples):
        raise RuntimeError("Decoded QA sample count mismatch")
    contact = r.Image.new("RGB", (1280, math.ceil(len(images) / 2) * 492), r.BACKGROUND)
    sample_rows = []
    for index, (path, number) in enumerate(zip(images, sorted(samples))):
        with r.Image.open(path) as actual:
            if actual.size != (1280, 960):
                raise RuntimeError("Decoded QA sample resolution mismatch")
            preview = actual.convert("RGB").resize((640, 480), r.Image.Resampling.LANCZOS)
        x, y = index % 2 * 640, index // 2 * 492
        contact.paste(preview, (x, y))
        r.text(r.ImageDraw.Draw(contact), (x + 4, y + 480), f"DECODED frame {number} | {number / 15:.3f}s", "tiny", r.WHITE, 630)
        sample_rows.append({"path": str(path), "output_frame": number, "sha256": digest(path)})
    contact.save(output / "decoded_contact.jpg", quality=94)
    verify_sources(plan)
    report = {"schema": SCHEMA, "state": "encoded and fully decoded; visual QA pending", "video": str(destination),
              "video_sha256": digest(destination), "bytes": destination.stat().st_size, "frames": count,
              "duration_seconds": count / 15, "segments": output_segments, "exact_final_sample_added": True,
              "observer_unobscured_rectangle": [0, 0, 1280, 720], "observer_fraction": 0.75,
              "plan_sha256": digest(args.plan), "source_sha256": plan["source_sha256"],
              "source_hashes_before_after_equal": True, "versions": plan["versions"], "policy": plan["policy"],
              "recorded_result": recording.result, "acceptance_inferred_by_renderer": False,
              "source_clock": "recorded wall; latest sample at or before display time", "source_rows_rewritten": False,
              "rgb_and_overlay": "saved Texture2D previews; no exact original RGB-input byte claim",
              "bbox_distance_used": False, "slam_source_used": False, "new_inference": False,
              "full_decode": audit, "decode_command": decode_command, "encode_command": command,
              "decoded_samples": sample_rows, "visual_qa": "pending", "bounded_process_receipt": "supplied by root supervisor",
              "elapsed_seconds": time.monotonic() - started}
    write_json(output / "verification.json", report)
    print(json.dumps({key: report[key] for key in ("state", "video", "video_sha256", "frames", "duration_seconds")}, indent=2))


if __name__ == "__main__":
    main()
