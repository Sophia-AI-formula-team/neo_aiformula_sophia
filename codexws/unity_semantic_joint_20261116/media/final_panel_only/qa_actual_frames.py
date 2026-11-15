"""Read back six transition images from the final MP4; never render or encode."""
import hashlib
import json
import time
from fractions import Fraction
from pathlib import Path

import av

ROOT = Path(__file__).resolve().parent
RENDER = ROOT / "encode_v1" / "render"
DEST = ROOT / "qa_decoded"
LABELS = {
    225: "inner_red_stopped", 346: "inner_green_visible_still_waiting",
    351: "inner_native_green_resume_delay", 359: "inner_green_driving",
    449: "inner_final_two_laps", 450: "outer_independent_start",
    675: "outer_red_stopped", 804: "outer_green_visible_still_waiting",
    808: "outer_native_green_resume_delay", 817: "outer_green_driving",
    899: "outer_final_two_laps",
}


def sha(path):
    return hashlib.file_digest(Path(path).open("rb"), "sha256").hexdigest()


def write(path, data):
    with path.open("x", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False, allow_nan=False)
        f.write("\n")


def main():
    started = time.perf_counter()
    receipt = json.loads((RENDER / "receipt.json").read_text(encoding="utf-8"))
    video = RENDER / "semantic_joint.mp4"
    before = sha(video)
    assert receipt["complete_decode"] and receipt["output_frames"] == 900
    assert before == receipt["video_sha256"]
    rows = [json.loads(line) for line in (RENDER / "output_frames.jsonl").read_text(encoding="utf-8").splitlines()]
    assert len(rows) == 900
    DEST.mkdir(exist_ok=False)
    wanted = {i for i in LABELS if not (RENDER / f"decoded_{i:04d}.png").exists()}
    saved = []
    with av.open(str(video), "r") as container:
        stream = container.streams.video[0]
        stream.codec_context.thread_count = 1
        for i, frame in enumerate(container.decode(stream)):
            if i in wanted:
                assert frame.pts * frame.time_base == Fraction(i, 15)
                path = DEST / f"decoded_{i:04d}.png"
                frame.to_image().save(path)
                saved.append(i)
            if i >= max(wanted):
                break
    assert set(saved) == wanted
    samples = []
    event_index = []
    for case_index in (1, 2):
        case_rows = [r for r in rows if r["case_index"] == case_index]
        source = Path(case_rows[0]["case_source"])
        chosen = [i for i in LABELS if rows[i]["case_index"] == case_index]
        indices = {rows[i]["motion_index"] for i in chosen if rows[i]["motion_index"] >= 0}
        motion = {}
        with (source / "motion.jsonl").open(encoding="utf-8-sig") as f:
            for n, line in enumerate(f):
                if n in indices:
                    motion[n] = json.loads(line)
        for i in chosen:
            row = rows[i]
            m = motion.get(row["motion_index"], {})
            semantic = m.get("semantic") or {}
            status = semantic.get("status") or {}
            panel = m.get("signal_panel") or {}
            path = RENDER / f"decoded_{i:04d}.png"
            if not path.exists():
                path = DEST / path.name
            samples.append(dict(label=LABELS[i], image=str(path), image_sha256=sha(path),
                                frame_journal=row,
                                measured={k: m.get(k) for k in ("t", "wall", "x", "z", "actual_v", "actual_w", "applied_v", "applied_w", "distance", "completed_laps", "signal_truth", "detected_light", "semantic_runtime_status")},
                                native_status={k: status.get(k) for k in ("traffic_mode", "traffic_reason", "odom_reference_ready", "map_fusion", "topology_mapping")},
                                native_command={k: semantic.get(k) for k in ("v", "w")},
                                signal_panel={k: panel.get(k) for k in ("state", "source", "fresh", "model_state", "effective_model_state", "signal_policy")}))
        for line in (source / "events.jsonl").read_text(encoding="utf-8-sig").splitlines():
            event = json.loads(line)
            match = next((r for r in case_rows if r["source_wall"] >= event["wall"]), case_rows[-1])
            event_index.append(dict(case_index=case_index, event=event["event"], stage=event.get("stage"),
                                    source_sim_time=event["t"], source_wall=event["wall"],
                                    first_output_at_or_after=match["output_time"], output_frame=match["output_frame"],
                                    sampled_source_wall=match["source_wall"]))
    after = sha(video)
    assert before == after
    write(DEST / "receipt.json", dict(schema="semantic_joint_video_transition_readback/v1",
          video=str(video), video_sha256=after, video_unchanged=True, action="decode only; no composition or encoding",
          source_full_decode_receipt=str(RENDER / "receipt.json"), decoder_thread_count=1,
          newly_decoded_frames=saved, reused_original_decoded_frames=sorted(set(LABELS)-wanted),
          elapsed_sec=time.perf_counter()-started, samples=sorted(samples, key=lambda x: x["frame_journal"]["output_frame"])))
    write(ROOT / "event_time_index.json", dict(schema="semantic_joint_video_event_index/v1", video_sha256=after,
          timeline="source wall -> first sampled video frame at or after; asynchronous inputs retain their own recorded ages", events=event_index))
    print(json.dumps({"receipt":str(DEST / "receipt.json"), "new_images":saved, "seconds":time.perf_counter()-started}))


if __name__ == "__main__":
    main()
