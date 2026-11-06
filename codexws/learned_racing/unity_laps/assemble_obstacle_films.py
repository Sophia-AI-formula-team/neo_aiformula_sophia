"""Join scored Unity clips without re-encoding, verify each frame, and publish a local player."""
import argparse
import hashlib
import json
import subprocess
import sys
from fractions import Fraction
from html import escape
from pathlib import Path
from zlib import crc32

import av


def inspect(path, poster=None, poster_second=0):
    marks = []
    with av.open(str(path)) as video:
        stream = video.streams.video[0]
        fps = stream.average_rate
        poster_frame = round(poster_second * fps)
        for n, frame in enumerate(video.decode(stream)):
            if abs(float(frame.pts * frame.time_base - n / fps)) > .002:
                raise ValueError(f"Discontinuous time in {path} at frame {n}")
            if poster and n == poster_frame:
                frame.to_image().save(poster, quality=92)
            small = frame.reformat(120, 68, format="gray").to_ndarray()
            marks.append(crc32(small.tobytes()))
        if not marks or (poster and not (0 <= poster_frame < len(marks))):
            raise ValueError(f"Invalid video or still time for {path}")
        return fps, (stream.width, stream.height), stream.codec_context.extradata, marks


def join(paths, fps, counts, titles, output):
    boundaries = [0]
    for count in counts:
        boundaries.append(boundaries[-1] + count)
    chapters = [dict(id=i, start=boundaries[i], end=boundaries[i+1], time_base=1/fps,
                     metadata={"title": title}) for i, title in enumerate(titles)]
    temp = output.with_suffix(".part.mp4")
    with av.open(str(temp), "w", options={"movflags": "+faststart"}) as reel:
        reel.set_chapters(chapters)
        target = None
        for begin, path in zip(boundaries, paths):
            with av.open(str(path)) as segment:
                source = segment.streams.video[0]
                if target is None:
                    target = reel.add_stream_from_template(source)
                for packet in segment.demux(source):
                    if packet.dts is None:
                        continue
                    shift = Fraction(begin) / fps / packet.time_base
                    if shift.denominator != 1:
                        raise ValueError("Source timestamps do not align")
                    packet.pts += int(shift)
                    packet.dts += int(shift)
                    packet.stream = target
                    reel.mux(packet)
    temp.replace(output)


def publish(folder, render=False, sources=None):
    cases = json.loads((folder / "cases.json").read_bytes())
    videos = [folder / (c["id"] + ".mp4") for c in cases]
    counts, expected, rows = [], [], []
    (folder / "posters").mkdir(exist_ok=True)
    signature = None
    for case, path in zip(cases, videos):
        root = folder.parent / case["captureRoot"]
        trajectory = root / "runs" / (case["trial"] + ".json")
        if render:
            raw = (sources or {}).get(case["captureRoot"], root) / ".work"
            subprocess.run([sys.executable, "-B", str(Path(__file__).with_name("three_view.py")),
                            "--trajectory", str(trajectory), "--frames", str(raw / "frames" / case["trial"]),
                            "--a4", str(raw / "camera" / case["trial"] / "a4_samples"),
                            "--start", str(case["start"]), "--end", str(case["end"]),
                            "--title", case["title"], "--referee-inset", "--out", str(path)], check=True)
            subprocess.run([sys.executable, "-B", str(Path(__file__).with_name("verify_three_view.py")),
                            "--receipt", str(path.with_suffix(".json")), "--frames", str(raw / "frames" / case["trial"]),
                            "--a4", str(raw / "camera" / case["trial"] / "a4_samples"),
                            "--out", str(path.with_suffix(".verification.json"))], check=True)
        receipt = json.loads(path.with_suffix(".json").read_bytes())
        if (receipt["trajectory_sha256"] != hashlib.sha256(trajectory.read_bytes()).hexdigest()
                or receipt["run"] != case["trial"] or receipt["start_s"] != case["start"]
                or receipt.get("end_s", receipt.get("end_s_exclusive")) != case["end"]
                or not receipt.get("referee_inset")
                or any(receipt["missing_camera_output_frames"].values())):
            raise ValueError(f"Clip sources or start time differ from the case: {path}")
        poster = "posters/" + case["id"] + ".jpg"
        fps, size, codec, marks = inspect(path, folder / poster, case["posterSecond"])
        if signature and signature != (fps, size, codec):
            raise ValueError(f"Different video settings in {path}")
        signature = (fps, size, codec)
        if receipt["encoded"]["frames"] != len(marks):
            raise ValueError(f"Frame count differs from capture report: {path}")
        rows.append(dict(**case, video=path.name, poster=poster, sourceReceipt=path.with_suffix(".json").name,
                         trajectory=Path("..",case["captureRoot"],"runs",trajectory.name).as_posix(),
                         reelSecond=float(len(expected) / fps), seconds=float(len(marks)/fps),
                         frames=len(marks), sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        counts.append(len(marks))
        expected.extend(marks)
    reel = folder / "all-scenes.mp4"
    join(videos, fps, counts, [c["title"] for c in cases], reel)
    rate, pixels, _, observed = inspect(reel)
    if (fps, size, expected) != (rate, pixels, observed):
        raise ValueError("Joined film does not reproduce the source frames")
    with av.open(str(reel)) as video:
        chapters = video.chapters()
        if len(chapters) != len(rows):
            raise ValueError("Chapter metadata differs from the playlist")
        for chapter, row in zip(chapters, rows):
            actual = float(chapter["start"] * chapter["time_base"])
            if abs(actual - row["reelSecond"]) >= 1 / fps:
                raise ValueError("Chapter timestamp mismatch")
    log = dict(fps=float(fps), width=size[0], height=size[1], checkedFrames=len(expected),
               seconds=float(len(expected)/fps), reelSha256=hashlib.sha256(reel.read_bytes()).hexdigest(), scenes=rows)
    (folder / "verification.json").write_text(json.dumps(log, ensure_ascii=False, indent=2), encoding="utf8")
    controls = []
    for i, row in enumerate(rows):
        controls.append(f'<article><button class="choice" data-index="{i}" aria-pressed="false">'
                        f'<img src="{row["poster"]}" alt=""><span><b>{escape(row["title"])}</b>'
                        f'<small>{row["seconds"]:.1f} 秒</small></span></button>'
                        f'<p>{escape(row["description"])}</p><a href="{row["video"]}">单段 MP4</a></article>')
    template = Path(__file__).with_name("obstacle_player.html").read_text(encoding="utf8")
    template = template.replace("{{POSTER}}", rows[0]["poster"]).replace("{{SCENES}}", "".join(controls))
    data = json.dumps(rows, ensure_ascii=False).replace("<", "\\u003c")
    (folder / "index.html").write_text(template.replace("{{DATA}}", data), encoding="utf8")
    print(f"{len(videos)} chapters, {len(expected)} decoded frames, {log['seconds']:.1f} s")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument("--render",action="store_true",help="Render individual clips from captured camera JPGs first")
    parser.add_argument("--capture-root",nargs=2,action="append",default=[],metavar=("NAME","PATH"),
                        help="Use captures from another local worktree, e.g. scenario_shots D:/.../scenario_shots")
    args = parser.parse_args()
    publish(args.folder, args.render, {name:Path(path) for name,path in args.capture_root})
