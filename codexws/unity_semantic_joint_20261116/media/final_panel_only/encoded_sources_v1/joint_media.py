"""Evidence-only video composer for AIFormulaJointTopoTrial recordings.

No planner, inference, route fitting or acceptance computation runs here.
"""
from __future__ import annotations

import argparse
import bisect
from collections import OrderedDict
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
from fractions import Fraction

SCHEMA = "aiformula_semantic_joint_media/v1"
COMBINED_SCHEMA = "aiformula_semantic_joint_combined_media/v1"
WIDTH, HEIGHT, FPS = 1920, 1080, 15
BG = (17, 23, 31)
WHITE, GRAY, GRID = (235, 241, 247), (164, 179, 195), (49, 62, 77)
CYAN, GREEN, ORANGE = (69, 205, 248), (127, 220, 150), (255, 178, 84)
PINK, PURPLE, RED = (255, 121, 175), (191, 155, 255), (255, 105, 108)
TABLES = ("frames.jsonl", "trace.jsonl", "motion.jsonl", "events.jsonl")
EVENTS = {"physical_lap_completed", "package_reference_closed", "physical_panel_red",
          "package_observed_red", "package_detour_entered", "measured_leftward_motion",
          "package_detour_completed", "trial_stop", "red_detour_failed",
          "completed_detour_waiting_at_red", "physical_panel_green_for_second_lap_completion",
          "motion_control_started", "render_settings_changed"}


def finite(value):
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def number(value, digits=2):
    result = finite(value)
    return "--" if result is None else f"{result:.{digits}f}"


def short(value):
    if value is None:
        return "unrecorded"
    if isinstance(value, (dict, list)):
        return json.dumps(value, ensure_ascii=True, separators=(",", ":"))
    return str(value).lower() if isinstance(value, bool) else str(value)


def identity_guard_configuration(configuration):
    """Display recorded new/legacy field names; never supply a control setting."""
    names = ("signal_panel_model_identity_guard", "signal_panel_reject_conflicting_below_horizon_lights")
    recorded = {name: configuration[name] for name in names if name in configuration}
    values = list(recorded.values())
    valid = bool(values) and all(type(value) is bool for value in values)
    conflict = valid and len(set(values)) > 1
    return dict(fields=recorded, enabled=values[0] if valid and not conflict else None,
                conflict=conflict, source="recorded configuration only")


def signal_input_configuration(configuration):
    """Describe the recorded input policy without interpreting a model vote."""
    flag = configuration.get("signal_panel_only")
    policy = configuration.get("signal_policy")
    panel_policy = policy == "calibrated_rgb_panel_only"
    return dict(panel_only=flag is True and panel_policy,
                recorded_panel_only=flag, recorded_policy=policy, recorded_policy_field="signal_policy",
                conflict=(panel_policy and flag is False) or (flag is True and policy not in (None, "calibrated_rgb_panel_only")))


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")


def source_file(root, name):
    path = (Path(root) / str(name)).resolve()
    path.relative_to(Path(root).resolve())
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def pyav_identity():
    """Fingerprint the already installed encoder; never fetch/install anything."""
    import av
    codec = av.codec.Codec("libx264", "w")
    if "yuv420p" not in {f.name for f in codec.video_formats}:
        raise ValueError("Installed libx264 does not support yuv420p")
    root = Path(av.__file__).resolve().parent
    files = {root/"__init__.py", root/"about.py", *root.rglob("*.pyd"),
             *(root.parent/"av.libs").glob("*.dll")}
    return dict(kind="pyav", version=av.__version__, module=str(Path(av.__file__).resolve()),
                library_versions={k: list(v) for k, v in av.library_versions.items()},
                codec="libx264", pixel_format="yuv420p", thread_count=1,
                files_sha256={str(p): digest(p) for p in sorted(files)})


def rows(path, allow_empty=False, projection=None):
    result = []
    with Path(path).open(encoding="utf-8-sig") as stream:
        for line_no, line in enumerate(stream, 1):
            if not line.strip():
                continue
            row = json.loads(line)
            wall = finite(row.get("wall")) if isinstance(row, dict) else None
            if wall is None or (result and wall < result[-1]["wall"]):
                raise ValueError(f"Invalid/nonmonotone wall time: {path}:{line_no}")
            result.append(projection(row) if projection else row)
    if not result and not allow_empty:
        raise ValueError(f"Empty required table: {path}")
    return result


def point(value):
    if isinstance(value, dict):
        value = [value.get("x"), value.get("y")]
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        x, y = finite(value[0]), finite(value[1])
        if x is not None and y is not None:
            return x, y
    return None


def points(values):
    return [p for value in (values or []) if (p := point(value)) is not None]


def latest(data, times, wall):
    index = bisect.bisect_right(times, wall) - 1
    return (data[index], index) if index >= 0 else ({}, -1)


def schedule(start, end, events, motion, target):
    """Keep all time intervals. Reserve slower windows around recorded events."""
    windows = [(start, min(end, start + 1), "capture_start"),
               (max(start, end - 1), end, "capture_end")]
    for row in events:
        label = row.get("event", row.get("kind"))
        if label in EVENTS and start <= row["wall"] <= end:
            windows.append((max(start, row["wall"] - 1.5), min(end, row["wall"] + 2.5), label))
    protected = []
    for event in events:
        if event.get("event") in {"red_detour_failed", "completed_detour_waiting_at_red"}:
            sim_end = finite(event.get("t"))
            candidates = [r for r in motion if r["wall"] <= event["wall"] and
                          finite(r.get("t")) is not None and sim_end is not None and r["t"] >= sim_end-12]
            low = candidates[0]["wall"] if candidates else event["wall"]-12
            low, high = max(start, low), min(end, event["wall"])
            if high > low:
                protected.append((low, high))
    prior = None
    for row in motion:
        status = (row.get("semantic") or {}).get("status") or {}
        state = (status.get("traffic_mode"), status.get("odom_reference_ready"),
                 (status.get("topology_mapping") or {}).get("available"))
        if prior is not None and state != prior and start <= row["wall"] <= end:
            windows.append((max(start, row["wall"] - 1), min(end, row["wall"] + 2), "package_state_change"))
        prior = state
    cuts = sorted({start, end, *(p for a, b, _ in windows for p in (a, b)),
                   *(p for a, b in protected for p in (a, b))})
    result = []
    for a, b in zip(cuts, cuts[1:]):
        reasons = sorted({label for low, high, label in windows if low < b and high > a})
        if b > a:
            locked = any(low < b and high > a for low, high in protected)
            result.append(dict(start=a, end=b, event_window=bool(reasons), protected_red_wait=locked, reasons=reasons))
    duration = end - start
    target = min(target, duration)
    locked_length = sum(s["end"]-s["start"] for s in result if s["protected_red_wait"])
    # Keep the full red-wait observation at 1x even if this makes the film exceed
    # the approximate 60-second target. Never remove a failed interval to fit.
    target = max(target, locked_length + min(12, duration-locked_length))
    slow = sum(s["end"]-s["start"] for s in result if s["event_window"] and not s["protected_red_wait"])
    fast = max(0, duration - slow - locked_length)
    budget = max(target-locked_length, 1e-9)
    if duration <= target:
        slow_speed = fast_speed = 1.0
    elif fast <= 1e-9:
        slow_speed = fast_speed = max(1.0, slow / budget)
    else:
        slow_budget = min(slow, budget * .72)
        slow_speed = max(1.0, slow / max(slow_budget, 1e-9))
        fast_speed = max(slow_speed, fast / max(budget - slow / slow_speed, 1e-9))
    for segment in result:
        segment["speed"] = 1.0 if segment["protected_red_wait"] else slow_speed if segment["event_window"] else fast_speed
    return result


def prepare(args):
    root = args.source.resolve()
    def schedule_only(row):
        if row.get("semantic"):
            row["semantic"] = {"status": row["semantic"].get("status") or {}}
        for key in ("candidate", "detections", "signal_panel"):
            row.pop(key, None)
        return row
    data = {name: rows(source_file(root, name), name == "events.jsonl", schedule_only) for name in TABLES}
    config = read_json(source_file(root, "configuration.json"))
    result = read_json(source_file(root, "result.json"))
    if config.get("experiment_mode") != "semantic_joint_1115":
        raise ValueError("Only semantic_joint_1115 recordings are accepted")
    if config.get("bbox_enabled") is not False or config.get("neo_stop_wait_enabled") is not False:
        raise ValueError("Bbox and old neo stop_wait modes must explicitly be disabled")
    frames = data["frames.jsonl"]
    if any(row.get("source") != "actual_first_person_detector_input" for row in frames):
        raise ValueError("Every frame must identify actual detector.Input first person capture")
    image_start, image_end = frames[0]["wall"], frames[-1]["wall"]
    start = min(0.0, *(row["wall"] for table in data.values() for row in table))
    end = max(*(row["wall"] for table in data.values() for row in table), finite(result.get("wallSeconds")) or 0)
    if end <= start:
        raise ValueError("At least two distinct capture times are required")
    names = set(TABLES) | {"configuration.json", "result.json"}
    names.update(name for name in ("semantic_ticks.jsonl", "terminal_sample.json", "play_after.json", "red_wait_snapshot.json")
                 if (root / name).is_file())
    for table in data.values():
        for row in table:
            names.update(str(row[key]) for key in ("rgb", "overlay") if row.get(key))
    names.update(row["path"] for row in frames)
    display_events, transition = [], None
    if args.render_transition_receipt:
        transition_receipt = read_json(args.render_transition_receipt)
        transition = json.loads(transition_receipt["result"])
        observed = next((row for row in data["motion.jsonl"]
                         if (finite(row.get("absolute_sim_time")) or -1) >= transition["sim_time"]), None)
        if observed is None:
            raise ValueError("Render transition is not covered by the recorded motion clock")
        transition["first_observed_wall"] = observed["wall"]
        display_events.append(dict(wall=observed["wall"], event="render_settings_changed",
                                   source="separate MCP receipt; first following recorded motion sample"))
    segments = schedule(start, end, sorted(data["events.jsonl"]+display_events, key=lambda row: row["wall"]), data["motion.jsonl"], args.seconds)
    duration = sum((s["end"] - s["start"]) / s["speed"] for s in segments)
    plan = dict(schema=SCHEMA, name=args.name or config.get("case") or root.name,
                source=str(root), width=WIDTH, height=HEIGHT, fps=FPS,
                first_wall=start, last_wall=end, target_seconds=args.seconds,
                first_image_wall=image_start, last_image_wall=image_end,
                duration_seconds=duration, frame_count=math.ceil(duration * FPS), segments=segments,
                configuration=config, recorded_result=result,
                script=str(Path(__file__).resolve()), script_sha256=digest(__file__),
                source_sha256={name: digest(source_file(root, name)) for name in sorted(names)},
                notes=["All captured intervals retained; source samples held, never interpolated",
                       "Map acceptance is displayed from recorded result, never inferred",
                       "Internal graph nodes are estimates, not observed internal road mapping",
                       "Source may stop early or fail; requested two laps is not an acceptance assertion"])
    if transition:
        plan["render_transition"] = transition
        plan["display_events"] = display_events
        plan["render_transition_receipt"] = str(args.render_transition_receipt.resolve())
        plan["render_transition_receipt_sha256"] = digest(args.render_transition_receipt)
    if args.backend == "pyav":
        plan["encoder_backend"] = pyav_identity()
    else:
        plan["encoder_backend"] = {"kind": "ffmpeg"}
    if args.ffmpeg and args.backend == "ffmpeg":
        plan["ffmpeg"] = str(args.ffmpeg.resolve())
        plan["ffmpeg_sha256"] = digest(args.ffmpeg)
    if args.evaluation_surface:
        surface = read_json(args.evaluation_surface)
        if (surface.get("schema") != "semantic_joint_evaluation_surface/v1" or
                surface.get("coordinate_frame") != "unity_world_xz" or surface.get("enters_control") is not False):
            raise ValueError("Expected a display-only Unity world-xz surface")
        if surface.get("live_mcp_verified") is not True:
            raise ValueError("Evaluation surface requires completed live MCP transform verification")
        plan["evaluation_surface"] = str(args.evaluation_surface.resolve())
        plan["evaluation_surface_sha256"] = digest(args.evaluation_surface)
    write_json(args.plan, plan)
    print(json.dumps({"plan": str(args.plan), "duration": duration, "frames": plan["frame_count"],
                      "accepted": result.get("accepted"), "completedLaps": result.get("completedLaps")}))


def verify(plan):
    if plan.get("schema") not in (SCHEMA, COMBINED_SCHEMA) or digest(__file__) != plan["script_sha256"]:
        raise ValueError("Plan schema or sealed renderer differs")
    if plan["schema"] == COMBINED_SCHEMA:
        if len(plan.get("cases", [])) != 2:
            raise ValueError("A combined plan must contain exactly two cases")
        if (plan.get("width"), plan.get("height"), plan.get("fps")) != (WIDTH, HEIGHT, FPS):
            raise ValueError("Combined output format differs")
        cursor, sources, results = 0, set(), []
        for index, case in enumerate(plan["cases"]):
            if digest(case["plan_file"]) != case["plan_sha256"]:
                raise ValueError("Sealed child plan changed")
            child = read_json(case["plan_file"])
            if child.get("schema") != SCHEMA:
                raise ValueError("Nested combined plans are not supported")
            verify(child)
            if ((child["width"], child["height"], child["fps"]) != (WIDTH, HEIGHT, FPS)
                    or child.get("encoder_backend") != plan.get("encoder_backend")
                    or case["frame_count"] != child["frame_count"]
                    or case["name"] != child["name"] or case["source"] != child["source"]
                    or case["case_index"] != index+1
                    or case["first_output_frame"] != cursor
                    or case["last_output_frame"] != cursor+child["frame_count"]-1
                    or case["first_output_time"] != cursor/FPS
                    or case["end_output_time_exclusive"] != (cursor+child["frame_count"])/FPS
                    or case["duration_seconds"] != child["frame_count"]/FPS
                    or case["scheduled_seconds"] != child["duration_seconds"]):
                raise ValueError("Combined case metadata differs from its sealed plan")
            if child["source"] in sources:
                raise ValueError("Two distinct recorded cases are required")
            sources.add(child["source"])
            results.append(dict(case_index=index+1, name=child["name"], recorded_result=child["recorded_result"]))
            cursor += child["frame_count"]
        if (plan["frame_count"] != cursor or plan["duration_seconds"] != cursor/FPS
                or plan["encoder_backend"].get("kind") != "pyav"):
            raise ValueError("Combined timeline or encoder differs")
        if plan["recorded_result"] != {"case_results": results, "combined_acceptance": "not_computed"}:
            raise ValueError("Combined results differ from the two recorded case results")
        return
    for name, expected in plan["source_sha256"].items():
        if digest(source_file(plan["source"], name)) != expected:
            raise ValueError(f"Sealed recording changed: {name}")
    if plan.get("ffmpeg") and digest(plan["ffmpeg"]) != plan["ffmpeg_sha256"]:
        raise ValueError("Sealed FFmpeg executable changed")
    if plan.get("evaluation_surface") and digest(plan["evaluation_surface"]) != plan["evaluation_surface_sha256"]:
        raise ValueError("Sealed evaluation surface changed")
    if plan.get("render_transition_receipt") and digest(plan["render_transition_receipt"]) != plan["render_transition_receipt_sha256"]:
        raise ValueError("Sealed render transition receipt changed")
    backend = plan.get("encoder_backend") or {"kind": "ffmpeg"}
    if backend["kind"] == "pyav" and pyav_identity() != backend:
        raise ValueError("Installed PyAV encoder differs from the sealed backend")


def combine(args):
    cases, backend, cursor, results = [], None, 0, []
    for index, path in enumerate(args.plans):
        path = path.resolve()
        child = read_json(path)
        if child.get("schema") != SCHEMA:
            raise ValueError("Combine accepts two sealed single-case plans only")
        verify(child)
        if (child.get("encoder_backend") or {}).get("kind") != "pyav":
            raise ValueError("Combined output requires the explicit PyAV backend")
        if backend is not None and backend != child["encoder_backend"]:
            raise ValueError("Cases use different pinned encoders")
        backend = child["encoder_backend"]
        count = child["frame_count"]
        if not isinstance(count, int) or isinstance(count, bool) or count < 1:
            raise ValueError("Child frame count must be a positive integer")
        cases.append(dict(case_index=index+1, name=child["name"], source=child["source"],
                          plan_file=str(path), plan_sha256=digest(path), frame_count=count,
                          first_output_frame=cursor, last_output_frame=cursor+count-1,
                          first_output_time=cursor/FPS, end_output_time_exclusive=(cursor+count)/FPS,
                          duration_seconds=count/FPS, scheduled_seconds=child["duration_seconds"]))
        results.append(dict(case_index=index+1, name=child["name"], recorded_result=child["recorded_result"]))
        cursor += count
    plan = dict(schema=COMBINED_SCHEMA, name=args.name, cases=cases, width=WIDTH, height=HEIGHT, fps=FPS,
                frame_count=cursor, duration_seconds=cursor/FPS, encoder_backend=backend,
                script=str(Path(__file__).resolve()), script_sha256=digest(__file__),
                recorded_result={"case_results": results, "combined_acceptance": "not_computed"},
                notes=["Two independent cases in the supplied order; direct cut, no crossfade or invented continuous drive",
                       "Each child schedule retained unchanged; protected events may extend the approximate total",
                       "One output encoding and one complete decode; no intermediate encoded videos"])
    verify(plan)
    write_json(args.plan, plan)
    print(json.dumps({"plan": str(args.plan), "duration": plan["duration_seconds"],
                      "frames": cursor, "cases": [{"name": c["name"], "seconds": c["duration_seconds"]} for c in cases]}))


def case_for_frame(plan, index):
    """Return the sealed case and its local frame index; never wrap across cuts."""
    if not 0 <= index < plan["frame_count"]:
        raise IndexError(index)
    for case in plan["cases"]:
        if case["first_output_frame"] <= index <= case["last_output_frame"]:
            return case, index-case["first_output_frame"]
    raise ValueError("Combined timeline contains a gap")


def case_frame_metadata(plan, index):
    case, local = case_for_frame(plan, index)
    boundaries = []
    if local == 0:
        boundaries.append("start")
    if local == case["frame_count"]-1:
        boundaries.append("end")
    return dict(case_index=case["case_index"], case_count=len(plan["cases"]),
                case_name=case["name"], case_source=case["source"],
                case_plan_sha256=case["plan_sha256"], case_output_frame=local,
                case_output_time=local/FPS, case_boundary=boundaries)


def source_time(plan, output_time):
    remaining = min(max(0, output_time), plan["duration_seconds"])
    for segment in plan["segments"]:
        length = (segment["end"] - segment["start"]) / segment["speed"]
        if remaining < length:
            return segment["start"] + remaining * segment["speed"], segment["speed"]
        remaining -= length
    last = plan["segments"][-1]
    return plan["last_wall"], last["speed"]


class Renderer:
    def __init__(self, plan, display_prefix=""):
        from PIL import Image, ImageDraw, ImageFont, ImageOps
        self.Image, self.ImageDraw, self.ImageOps = Image, ImageDraw, ImageOps
        self.plan, self.root = plan, Path(plan["source"])
        self.display_prefix = display_prefix
        self.guard_configuration = identity_guard_configuration(plan["configuration"])
        self.signal_configuration = signal_input_configuration(plan["configuration"])
        curves = {}
        def compact_motion(row):
            for key in ("candidate", "detections", "signal_panel"):
                row.pop(key, None)
            geometry = (row.get("semantic") or {}).get("map_data") or {}
            for key in ("learned_samples", "reference_points", "odom_reference_points", "detour_points"):
                if geometry.get(key):
                    curve = tuple(tuple(p) for p in geometry[key])
                    geometry[key] = curves.setdefault(curve, curve)
            return row
        def compact_trace(row):
            row.pop("semantic", None)  # Semantic truth is read from the motion table.
            return row
        self.tables = {name: rows(source_file(self.root, name), name == "events.jsonl",
                       compact_motion if name == "motion.jsonl" else compact_trace if name == "trace.jsonl" else None)
                       for name in TABLES}
        self.times = {name: [row["wall"] for row in table] for name, table in self.tables.items()}
        self.image_sources = {row["rgb"]: row for row in self.tables["trace.jsonl"] if row.get("rgb")}
        self.surface = read_json(plan["evaluation_surface"]) if plan.get("evaluation_surface") else None
        self.cache = OrderedDict()
        self.fonts = {}
        for size in (14, 16, 18, 22, 26):
            for candidate in ("C:/Windows/Fonts/YuGothM.ttc", "C:/Windows/Fonts/segoeui.ttf",
                              "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
                if Path(candidate).is_file():
                    self.fonts[size] = ImageFont.truetype(candidate, size)
                    break
            else:
                self.fonts[size] = ImageFont.load_default(size=size)
        self.odom_points = [self.odom(row) for row in self.tables["motion.jsonl"]]
        bounds_points = [p for p in self.odom_points if p is not None]
        for row in self.tables["motion.jsonl"]:
            m = (row.get("semantic") or {}).get("map_data") or {}
            if m.get("frame_id") == "odom":
                bounds_points.extend(points(m.get("reference_points")))
                bounds_points.extend(points(m.get("odom_reference_points")))
        if self.surface:
            for mesh in self.surface["meshes"]:
                if mesh["category"] in ("green", "paint"):
                    for ring in mesh["boundaries"]:
                        bounds_points.extend((z, -x) for x, z in points(ring))
        self.bounds = self.extent(bounds_points)

    @staticmethod
    def odom(row):
        # Recorded Unity-to-worker coordinate conversion: odom=(world z,-world x).
        x, z = finite(row.get("x")), finite(row.get("z"))
        return (z, -x) if x is not None and z is not None else point((row.get("semantic") or {}).get("odom"))

    @staticmethod
    def extent(values):
        if not values:
            return -1, 1, -1, 1
        xs, ys = zip(*values)
        size = max(max(xs)-min(xs), max(ys)-min(ys), 2) * 1.12
        cx, cy = (max(xs)+min(xs))/2, (max(ys)+min(ys))/2
        return cx-size/2, cx+size/2, cy-size/2, cy+size/2

    def text(self, draw, xy, value, size=18, color=WHITE, width=None):
        value = str(value)
        if width:
            while value and draw.textlength(value, font=self.fonts[size]) > width:
                value = value[:-2] + "…" if len(value) > 2 else ""
        draw.text(xy, value, font=self.fonts[size], fill=color)

    def wrapped(self, draw, xy, value, width, size=16, color=WHITE, limit=2):
        lines, line = [], ""
        for word in str(value).split():
            proposed = (line + " " + word).strip()
            if line and draw.textlength(proposed, font=self.fonts[size]) > width:
                lines.append(line)
                line = word
            else:
                line = proposed
        if line:
            lines.append(line)
        for index, line in enumerate(lines[:limit]):
            self.text(draw, (xy[0], xy[1]+index*22), line, size, color, width)

    def image(self, name):
        if name not in self.cache:
            with self.Image.open(source_file(self.root, name)) as source:
                self.cache[name] = source.convert("RGB")
            while len(self.cache) > 8:
                self.cache.popitem(last=False)
        self.cache.move_to_end(name)
        return self.cache[name]

    def paste(self, canvas, name, rect):
        if not name:
            return
        l, t, r, b = rect
        img = self.ImageOps.contain(self.image(name), (r-l, b-t))
        canvas.paste(img, (l+(r-l-img.width)//2, t+(b-t-img.height)//2))

    def chart(self, draw, rect, bounds):
        l, t, r, b = rect
        xmin, xmax, ymin, ymax = bounds
        scale = min((r-l)/(xmax-xmin), (b-t)/(ymax-ymin))
        cx, cy = (l+r)/2, (t+b)/2
        def project(p):
            return cx+(p[0]-(xmin+xmax)/2)*scale, cy-(p[1]-(ymin+ymax)/2)*scale
        for i in range(5):
            draw.line((l+(r-l)*i/4, t, l+(r-l)*i/4, b), fill=GRID)
            draw.line((l, t+(b-t)*i/4, r, t+(b-t)*i/4), fill=GRID)
        draw.rectangle(rect, outline=GRAY)
        return project

    @staticmethod
    def line(draw, values, project, color, width=2, dashed=False):
        pixels = [project(p) for p in values]
        for a, b in zip(pixels, pixels[1:]):
            if not dashed:
                draw.line((*a, *b), fill=color, width=width)
            else:
                length = math.dist(a, b)
                for offset in range(0, math.ceil(length), max(10, math.ceil(length/1000))):
                    f, g = offset/max(length, 1e-9), min(offset+5, length)/max(length, 1e-9)
                    draw.line((a[0]+(b[0]-a[0])*f, a[1]+(b[1]-a[1])*f,
                               a[0]+(b[0]-a[0])*g, a[1]+(b[1]-a[1])*g), fill=color, width=width)

    def graph(self, draw, m, project):
        nodes = {str(n.get("id")): n for n in (m.get("graph_nodes") or []) if point(n) is not None}
        for edge in m.get("graph_edges") or []:
            if not isinstance(edge, list) or len(edge) != 2:
                continue
            a, b = nodes.get(str(edge[0])), nodes.get(str(edge[1]))
            if a is not None and b is not None:
                estimated = "internal" in str(a.get("kind")) or "internal" in str(b.get("kind"))
                self.line(draw, [point(a), point(b)], project, ORANGE if estimated else GRAY, 2, estimated)
        for node in nodes.values():
            x, y = project(point(node))
            color = ORANGE if "internal" in str(node.get("kind")) else WHITE
            draw.ellipse((x-3, y-3, x+3, y+3), fill=color)

    def topo(self, draw, sem, motion_index):
        m, status = sem.get("map_data") or {}, sem.get("status") or {}
        self.text(draw, (1300, 50), "Semantic topology | recorded package output", 22)
        subtitle = "odom [m] | Unity road surface / evaluation only" if self.surface else "odom [m]  x=Unity z, y=-Unity x"
        self.text(draw, (1300, 80), subtitle, 16, GRAY, 600)
        project = self.chart(draw, (1300, 112, 1900, 550), self.bounds)
        if self.surface:
            # Exported points already include the saved/live-verified scene
            # transform. Only exchange axes; never fit the background to a route.
            # Draw boundaries only: filling rings would incorrectly fill holes.
            for mesh in self.surface["meshes"]:
                if mesh["category"] not in ("green", "paint"):
                    continue  # AsphaltApron is visual ground, not a road boundary.
                color = (38, 74, 55) if mesh["category"] == "green" else (83, 95, 105)
                for ring in mesh["boundaries"]:
                    self.line(draw, [(z, -x) for x, z in points(ring)], project, color, 2 if mesh["category"] == "green" else 1)
        if m.get("frame_id") == "odom":
            self.line(draw, points(m.get("odom_reference_points")), project, GRAY, 3)
            self.line(draw, points(m.get("reference_points")), project, GREEN, 3)
            if m.get("graph_frame") == "odom":
                self.graph(draw, m, project)
            self.line(draw, points(m.get("detour_points")), project, PINK, 4)
            for raw in m.get("learned_samples") or []:
                p = point(raw)
                if p:
                    x, y = project(p)
                    excluded = len(raw) > 3 and raw[3] is True
                    draw.ellipse((x-1, y-1, x+1, y+1), fill=ORANGE if excluded else PURPLE)
        history = [p for p in self.odom_points[:motion_index+1] if p is not None]
        self.line(draw, history, project, CYAN, 2)
        if history:
            x, y = project(history[-1])
            draw.ellipse((x-6, y-6, x+6, y+6), fill=WHITE, outline=BG, width=2)
        if not m:
            self.text(draw, (1330, 270), "No semantic map output recorded", 22, ORANGE, 540)
        self.text(draw, (1300, 558), "Fixed scale; geometry appears only when recorded", 16, GRAY, 600)
        legends = [(CYAN, "Actual driven trajectory"), (GREEN, "Current package reference"),
                   (GRAY, "First closed odom reference"), (PURPLE, "Learned samples (excluded: orange)"),
                   (ORANGE, "Dashed: estimated internal topology"), (PINK, "Package detour (not a driven path)")]
        for i, (color, label) in enumerate(legends):
            y = 584+i*26
            draw.line((1300, y+9, 1320, y+9), fill=color, width=3)
            self.text(draw, (1330, y), label, 16, color, 390)
        if m.get("graph_frame") == "official_unaligned" and m.get("graph_nodes"):
            draw.rectangle((1720, 578, 1910, 752), fill=BG, outline=ORANGE)
            self.text(draw, (1728, 582), "Official / NOT ALIGNED", 14, ORANGE, 176)
            inset = self.chart(draw, (1738, 610, 1898, 741), self.extent(points(m["graph_nodes"])))
            self.graph(draw, m, inset)
        elif m.get("graph_frame"):
            self.text(draw, (1740, 594), "Graph frame:", 16, GRAY, 170)
            self.text(draw, (1740, 620), m["graph_frame"], 18, WHITE, 170)

    def lane(self, draw, candidate, sem, stale, perception_stale=False, packet_state="present"):
        self.text(draw, (418, 774), "LLP points + LYA output", 22)
        project = self.chart(draw, (462, 824, 720, 1030), (-5, 5, 0, 10))
        def local(raw):
            p = point(raw)
            return (-p[1], p[0]) if p and 0 <= p[0] <= 10 and abs(p[1]) <= 5 else None
        for cloud in candidate.get("raw_clouds") or []:
            for raw in cloud:
                p = local(raw)
                if p:
                    draw.point(project(p), fill=GRAY)
        for cloud, color in zip(candidate.get("clouds") or [], (CYAN, ORANGE, GREEN)):
            self.line(draw, [p for raw in cloud if (p := local(raw))], project, GRAY if perception_stale else color)
        for values, color, radius in ((candidate.get("selected_points"), GRAY if perception_stale else WHITE, 3),
                                      (sem.get("selected_points"), GRAY if stale else PURPLE, 5)):
            for raw in values or []:
                p = local(raw)
                if p:
                    x, y = project(p)
                    draw.ellipse((x-radius, y-radius, x+radius, y+radius), outline=color, width=2)
        p = local(sem.get("target"))
        if p:
            x, y = project(p)
            draw.ellipse((x-8, y-8, x+8, y+8), outline=GRAY if stale else PINK, width=3)
        labels = [("Mode", candidate.get("mode")), ("Supported", candidate.get("supported")),
                  ("LLP", candidate.get("selection_source")), ("LYA target", sem.get("target")),
                  ("LLP ABC", candidate.get("selected_points"))]
        for i, (label, value) in enumerate(labels):
            self.text(draw, (734, 820+i*42), label, 14, GRAY, 140)
            rendered = "no packet" if packet_state in ("null", "stopped_tail_null") and label != "LYA target" else short(value)
            self.text(draw, (734, 838+i*42), rendered, 14, WHITE, 140)
        self.text(draw, (418, 1040), "forward [m]; screen right = vehicle right", 16, GRAY, 450)
        if packet_state in ("null", "stopped_tail_null"):
            self.text(draw, (418, 800), "No current LLP packet" + (" (stopped tail)" if packet_state == "stopped_tail_null" else " (recorded null)"), 16, ORANGE, 450)
        elif stale or perception_stale:
            label = "Held/stale: " + ("LLP input " if perception_stale else "") + ("semantic output" if stale else "")
            self.text(draw, (418, 800), label, 16, ORANGE, 450)

    def compose(self, wall, speed):
        frame, fi = latest(self.tables["frames.jsonl"], self.times["frames.jsonl"], wall)
        motion, mi = latest(self.tables["motion.jsonl"], self.times["motion.jsonl"], wall)
        trace, ti = latest(self.tables["trace.jsonl"], self.times["trace.jsonl"], wall)
        event, ei = latest(self.tables["events.jsonl"], self.times["events.jsonl"], wall)
        sem = motion.get("semantic") or {}
        status, m = sem.get("status") or {}, sem.get("map_data") or {}
        age = finite(motion.get("semantic_result_age"))
        age = age + wall-motion["wall"] if age is not None else None
        stale = age is not None and age > .75
        perception_age = finite(trace.get("age"))
        perception_age = perception_age + wall-trace["wall"] if perception_age is not None else None
        signal_panel = trace.get("signal_panel") or {}
        identity_filter = signal_panel.get("model_identity_filter") or {}
        image_source = self.image_sources.get(frame.get("rgb") or frame.get("path"), {})
        image_age = finite(image_source.get("age"))
        image_age = image_age + wall-image_source["wall"] if image_age is not None else None
        canvas = self.Image.new("RGB", (WIDTH, HEIGHT), BG)
        # This is the existing detector overlay, with the raw input shown below.
        main = frame.get("overlay") or frame.get("rgb") or frame.get("path")
        self.paste(canvas, main, (0, 42, 1280, 762))
        self.paste(canvas, frame.get("rgb") or frame.get("path"), (8, 810, 392, 1026))
        draw = self.ImageDraw.Draw(canvas)
        if not main:
            self.text(draw, (290, 340), "No recorded first-person frame yet", 26, GRAY, 900)
            self.text(draw, (290, 382), "Startup interval; no image synthesized", 22, GRAY, 900)
        panel_only = self.signal_configuration["panel_only"]
        helper = " + calibrated RGB panel" if panel_only else " + RGB signal helper" if self.plan["configuration"].get("signal_panel_rgb") else ""
        self.text(draw, (14, 6), f"{self.display_prefix}{self.plan['name']} | first-person overlay{helper} | {speed:.2f}x | wall {wall:.2f}s", 22, WHITE, 1260)
        self.topo(draw, sem, mi)
        for x in (400, 890, 1350):
            draw.line((x, 762, x, 1080), fill=GRID, width=2)
        draw.line((0, 762, 1920, 762), fill=GRID, width=2)
        self.text(draw, (12, 774), "Original detect.Input", 22)
        self.text(draw, (12, 1034), f"seq {frame.get('sequence')} | sim {number(frame.get('frame_sim_time'))}s | age {number(image_age)}s", 16, ORANGE if image_age is not None and image_age > .75 else GRAY, 380)
        overlay_note = "Existing overlay; no extra image annotations" if frame.get("overlay") else "Overlay unavailable; raw input shown"
        if identity_filter.get("enabled") is True:
            overlay_note = "Raw overlay unchanged | identity adapter ON"
        if panel_only:
            overlay_note = "Raw overlay unchanged | RGB panel control only"
        transition = self.plan.get("render_transition")
        if transition:
            after = (finite(motion.get("absolute_sim_time")) or 0) >= transition["sim_time"]
            suffix = "after" if after else "before"
            overlay_note = f"Render: FPS {transition['fps_'+suffix]} / vSync {transition['vsync_'+suffix]} | MIXED RUN"
        self.text(draw, (12, 1056), overlay_note, 14, GRAY, 380)
        packet_state = "not_yet_recorded" if "candidate" not in trace else "null" if trace["candidate"] is None else "present"
        if packet_state == "null" and frame.get("phase") == "stopped_tail":
            packet_state = "stopped_tail_null"
        self.lane(draw, trace.get("candidate") or motion.get("candidate") or {}, sem, stale,
                  perception_age is not None and perception_age > .75, packet_state)
        self.text(draw, (906, 774), "Semantic traffic + control", 22)
        identity_label = "ON" if identity_filter.get("enabled") is True else "OFF" if identity_filter.get("enabled") is False else "unrecorded"
        configured = self.guard_configuration["enabled"]
        if configured is not None:
            identity_label += " (cfg " + ("ON" if configured else "OFF") + ")"
        elif self.guard_configuration["conflict"]:
            identity_label += " (cfg CONFLICT)"
        identity_reason = identity_filter.get("reason")
        if any(item.get("reason") == "conflicting_cone_below_horizon_without_rgb_identity"
               for item in identity_filter.get("rejected") or []):
            identity_reason = "cone overlap + below horizon + no RGB ID"
        fields = [("Traffic", status.get("traffic_mode")), ("Reason", status.get("traffic_reason")),
                  ("Control light / raw model", f"{short(motion.get('detected_light'))} / {short(signal_panel.get('model_state'))}"),
                  ("Filtered model / scene lamp", f"{short(signal_panel.get('effective_model_state'))} / {short(motion.get('signal_truth'))} (eval only)"),
                  ("ID "+identity_label, identity_reason),
                  ("LYA v / w", f"{number(sem.get('controller_v'))} / {number(sem.get('controller_w'))}"),
                  ("Native semantic v / w", f"{number(sem.get('semantic_v'))} / {number(sem.get('semantic_w'))}"),
                  ("Applied v / w", f"{number(motion.get('applied_v'))} / {number(motion.get('applied_w'))}"),
                  ("Actual v / w", f"{number(motion.get('actual_v'))} / {number(motion.get('actual_w'))}"),
                  ("Blocked / detour nodes", f"{short(status.get('blocked_edge'))} / {short(status.get('detour_node_ids'))}"),
                  ("Semantic age", f"{number(age)}s" + (" HELD / STALE" if stale else "")),
                  ("Transport gate", sem.get("transport_gate_reason"))]
        if panel_only:
            # An excluded model vote is not a missing control observation. The
            # actual aggregate state comes from the recorded RGB panel output.
            fields[2:5] = [
                ("Signal", "calibrated RGB panel; raw model display-only"),
                ("Control / scene lamp", f"{short(motion.get('detected_light'))} / {short(motion.get('signal_truth'))} (scene: eval only)"),
                ("Raw model (no model vote)", signal_panel.get("model_state")),
            ]
        elif self.signal_configuration["conflict"]:
            fields[4] = ("Signal config", "CONFLICT: see sealed recorded configuration")
        for i, (label, value) in enumerate(fields):
            self.text(draw, (906, 808+i*22), f"{label}: {short(value)}", 14 if panel_only and i == 2 else 16, ORANGE if i == 4 and identity_filter.get("rejected") and not panel_only else GRAY if i == 3 else WHITE, 430)
        result = self.plan["recorded_result"]
        red_failed = any(e.get("event") == "red_detour_failed" for e in self.tables["events.jsonl"][:ei+1])
        green_recovery = any(e.get("event") == "physical_panel_green_for_second_lap_completion"
                             for e in self.tables["events.jsonl"][:ei+1])
        self.text(draw, (1366, 774), "Recorded evidence | not a map acceptance claim", 22, WHITE, 540)
        terminal = wall >= self.plan["last_wall"] - 3 or event.get("event") == "trial_stop"
        accepted = result.get("accepted") if terminal else None
        acceptance = short(accepted) if terminal else "PENDING (final receipt shown at end)"
        fields = [("Final receipt accepted", acceptance), ("Stage / laps", f"{short(motion.get('stage'))} / {short(motion.get('completed_laps'))}"),
                  ("Odom reference ready", status.get("odom_reference_ready")),
                  ("Fusion / topology available", f"{short((status.get('map_fusion') or {}).get('available'))} / {short((status.get('topology_mapping') or {}).get('available'))}"),
                  ("Red detour / green recovery", f"{'FAILED' if red_failed else 'no failure recorded'} / {green_recovery}"),
                  ("Learned / reference / OSM nodes", f"{len(m.get('learned_samples') or [])} / {len(m.get('reference_points') or [])} / {len(m.get('graph_nodes') or [])}"),
                  ("Fusion reason", (status.get("map_fusion") or {}).get("reason")),
                  ("Final reason" if terminal else "Latest event", result.get("reason") if terminal else event.get("event")),
                  ("Mode", "1115 traffic; bbox OFF; old stop_wait OFF"),
                  ("Limits", "Ideal simulated odom; internal nodes are estimates")]
        offset = 0
        for i, (label, value) in enumerate(fields):
            color = (GREEN if accepted is True else RED if accepted is False else ORANGE) if i == 0 else GRAY if i >= 8 else WHITE
            if label == "Final reason":
                self.wrapped(draw, (1366, 810+i*25), f"{label}: {short(value)}", 540, 16, color)
                offset = 22
            else:
                self.text(draw, (1366, 810+i*25+offset), f"{label}: {short(value)}", 16, color, 540)
        receipt = dict(source_wall=wall, playback_speed=speed, frame_index=fi,
                       frame_wall=frame.get("wall"), rgb=frame.get("rgb", frame.get("path")), overlay=frame.get("overlay"),
                       frame_sequence=frame.get("sequence"), frame_sim_time=frame.get("frame_sim_time"),
                       motion_index=mi, motion_wall=motion.get("wall"), trace_index=ti, event_index=ei,
                       semantic_sequence=sem.get("sequence"), worker_boot_id=sem.get("worker_boot_id"),
                       semantic_age=age, perception_age=perception_age, image_age=image_age, stale=stale, stage=motion.get("stage"),
                       llp_packet_state=packet_state,
                       effective_control_light=motion.get("detected_light"), raw_model_light=signal_panel.get("model_state"),
                       filtered_model_light=signal_panel.get("effective_model_state"),
                       identity_filter_enabled=identity_filter.get("enabled"),
                       identity_filter_reason=identity_filter.get("reason"),
                       identity_filter_policy=identity_filter.get("policy"),
                       identity_guard_configuration=self.guard_configuration,
                       signal_input_configuration=self.signal_configuration,
                       signal_observation_policy=signal_panel.get("signal_policy"),
                       rejected_model_light_count=len(identity_filter.get("rejected") or []),
                       graph_frame=m.get("graph_frame"), trial_accepted_display=accepted,
                       evaluation_surface=bool(self.surface),
                       render_transition_applied=bool(transition and (finite(motion.get("absolute_sim_time")) or 0) >= transition["sim_time"]),
                       red_detour_failed_display=red_failed, green_recovery_display=green_recovery,
                       odom_reference_ready=status.get("odom_reference_ready"), traffic_mode=status.get("traffic_mode"))
        return canvas, receipt


class CombinedRenderer:
    """Load one immutable recorded case at a time and keep its local clock."""
    def __init__(self, plan):
        self.plan, self.case_index, self.renderer, self.child = plan, None, None, None

    def compose_output(self, index):
        case, local = case_for_frame(self.plan, index)
        if self.case_index != case["case_index"]:
            self.renderer = None  # Release the prior large recording before loading the next.
            self.child = read_json(case["plan_file"])
            self.renderer = Renderer(self.child, display_prefix=f"[{case['case_index']}/2] ")
            self.case_index = case["case_index"]
        wall, speed = source_time(self.child, local/FPS)
        if local == self.child["frame_count"]-1:
            wall = self.child["last_wall"]
        canvas, receipt = self.renderer.compose(wall, speed)
        receipt.update(case_frame_metadata(self.plan, index))
        return canvas, receipt


def render(args):
    plan = read_json(args.plan)
    verify(plan)
    combined = plan["schema"] == COMBINED_SCHEMA
    renderer = CombinedRenderer(plan) if combined else Renderer(plan)
    if args.preview:
        if args.preview.exists():
            raise FileExistsError(args.preview)
        if combined:
            index = min(max(0, math.floor(args.at*FPS)), plan["frame_count"]-1)
            canvas, receipt = renderer.compose_output(index)
            receipt.update(output_frame=index, output_time=index/FPS)
            wall = receipt["source_wall"]
        else:
            wall, speed = source_time(plan, args.at)
            canvas, receipt = renderer.compose(wall, speed)
        canvas.save(args.preview)
        write_json(args.preview.with_suffix(".json"), receipt)
        print(json.dumps({"preview": str(args.preview), "source_wall": wall}))
        return
    if not args.permit_encoding or os.environ.get("AIFORMULA_JOINT_MEDIA_BOUNDED") != "1":
        raise RuntimeError("Encoding requires --permit-encoding and root-owned bounded supervisor contract")
    backend = (plan.get("encoder_backend") or {}).get("kind", "ffmpeg")
    if backend == "ffmpeg" and not plan.get("ffmpeg"):
        raise ValueError("Prepare a plan with --ffmpeg for encoding")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    video = output / "semantic_joint.mp4"
    if backend == "pyav":
        render_pyav(plan, renderer, output, video, args.plan)
        return
    log = output / "ffmpeg.log"
    command = [plan["ffmpeg"], "-nostdin", "-v", "warning", "-f", "rawvideo", "-pix_fmt", "rgb24",
               "-s", f"{WIDTH}x{HEIGHT}", "-r", str(FPS), "-i", "pipe:0", "-an", "-c:v", "libx264",
               "-threads", "1", "-preset", "fast", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(video)]
    process = None
    with log.open("wb") as error_log, (output / "output_frames.jsonl").open("x", encoding="utf-8") as mapping:
        try:
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=error_log)
            for index in range(plan["frame_count"]):
                wall, speed = source_time(plan, index/FPS)
                if index == plan["frame_count"]-1:
                    wall = plan["last_wall"]
                canvas, receipt = renderer.compose(wall, speed)
                receipt.update(output_frame=index, output_time=index/FPS)
                process.stdin.write(canvas.tobytes())
                mapping.write(json.dumps(receipt, allow_nan=False) + "\n")
                if index in {0, plan["frame_count"]//2, plan["frame_count"]-1}:
                    canvas.save(output / f"composed_{index:04d}.png")
            process.stdin.close()
            if process.wait(timeout=120):
                raise RuntimeError(f"FFmpeg failed; see {log}")
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=10)
    decode = subprocess.run([plan["ffmpeg"], "-nostdin", "-v", "error", "-xerror", "-threads", "1",
                             "-i", str(video), "-f", "null", "-"], capture_output=True, text=True, timeout=180)
    if decode.returncode:
        raise RuntimeError("Complete decode failed: " + decode.stderr)
    verify(plan)
    write_json(output / "receipt.json", dict(schema=SCHEMA, video=str(video), video_sha256=digest(video),
                plan_sha256=digest(args.plan), source_hashes_before_after=True, complete_decode=True,
                output_frames=plan["frame_count"], output_fps=FPS, recorded_result=plan["recorded_result"],
                visual_qa="PENDING; composed samples are not decoded video frames",
                bounded_supervisor="Caller contract only; root must attach owned process receipt"))
    print(json.dumps({"video": str(video), "frames": plan["frame_count"], "complete_decode": True}))


def render_pyav(plan, renderer, output, video, plan_path):
    import av
    combined = plan["schema"] == COMBINED_SCHEMA
    samples = {0, plan["frame_count"]//2, plan["frame_count"]-1}
    if combined:
        samples = {point for case in plan["cases"] for point in
                   (case["first_output_frame"], case["first_output_frame"]+case["frame_count"]//2, case["last_output_frame"])}
        write_json(output/"case_boundaries.json", dict(schema=COMBINED_SCHEMA,
                   plan_sha256=digest(plan_path), global_time_base=f"1/{FPS}", cases=plan["cases"]))
    with av.open(str(video), "w", format="mp4", options={"movflags": "+faststart"}) as container:
        stream = container.add_stream("libx264", rate=FPS, options={"preset": "fast", "crf": "20", "threads": "1"})
        stream.width, stream.height, stream.pix_fmt = WIDTH, HEIGHT, "yuv420p"
        stream.codec_context.thread_count = 1
        stream.codec_context.time_base = Fraction(1, FPS)
        with (output/"output_frames.jsonl").open("x", encoding="utf-8") as mapping:
            for index in range(plan["frame_count"]):
                if combined:
                    canvas, receipt = renderer.compose_output(index)
                else:
                    wall, speed = source_time(plan, index/FPS)
                    if index == plan["frame_count"]-1:
                        wall = plan["last_wall"]
                    canvas, receipt = renderer.compose(wall, speed)
                receipt.update(output_frame=index, output_time=index/FPS)
                mapping.write(json.dumps(receipt, allow_nan=False)+"\n")
                frame = av.VideoFrame.from_image(canvas)
                frame.pts, frame.time_base = index, Fraction(1, FPS)
                for packet in stream.encode(frame):
                    container.mux(packet)
                if index in samples:
                    canvas.save(output/f"composed_{index:04d}.png")
            for packet in stream.encode():
                container.mux(packet)
    decoded = 0
    with av.open(str(video), "r") as container, (output/"decoded_frames.jsonl").open("x", encoding="utf-8") as log, (output/"output_frames.jsonl").open(encoding="utf-8") as mapping:
        stream = container.streams.video[0]
        stream.codec_context.thread_count = 1
        if len(container.streams.video) != 1 or stream.codec_context.name != "h264":
            raise RuntimeError("Encoded file is not one H.264 video stream")
        for index, frame in enumerate(container.decode(stream)):
            if frame.width != WIDTH or frame.height != HEIGHT or frame.format.name != "yuv420p":
                raise RuntimeError("Decoded dimensions or pixel format differ")
            pts = frame.pts * frame.time_base if frame.pts is not None else None
            if pts != Fraction(index, FPS):
                raise RuntimeError(f"Decoded frame {index} has unexpected PTS: {pts}")
            recorded = json.loads(next(mapping))
            if recorded.get("output_frame") != index or recorded.get("output_time") != index/FPS:
                raise RuntimeError("Output frame journal index/time differs from decoded PTS")
            case_data = case_frame_metadata(plan, index) if combined else {}
            if any(recorded.get(key) != value for key, value in case_data.items()):
                raise RuntimeError(f"Output frame {index} case/boundary differs from the sealed combined plan")
            log.write(json.dumps(dict(index=index, pts=frame.pts, time_base=str(frame.time_base),
                                      pts_seconds=str(pts), **case_data))+"\n")
            if index in samples:
                frame.to_image().save(output/f"decoded_{index:04d}.png")
            decoded += 1
        if next(mapping, None) is not None:
            raise RuntimeError("Output frame journal has extra rows after complete decode")
    if decoded != plan["frame_count"]:
        raise RuntimeError(f"Decoded {decoded} frames; expected {plan['frame_count']}")
    verify(plan)
    write_json(output/"receipt.json", dict(schema=plan["schema"], video=str(video), video_sha256=digest(video),
                plan_sha256=digest(plan_path), source_hashes_before_after=True, complete_decode=True,
                every_decoded_frame_pts_checked=True, output_frames=decoded, output_fps=FPS,
                encoder_backend=plan["encoder_backend"], recorded_result=plan["recorded_result"],
                case_boundaries_readback_checked=combined,
                case_boundaries_sha256=digest(output/"case_boundaries.json") if combined else None,
                cases=plan.get("cases"),
                visual_qa="PENDING; decoded_*.png are actual decoded video frames",
                bounded_supervisor="Root must attach owned Job process receipt"))
    print(json.dumps({"video": str(video), "frames": decoded, "complete_decode": True, "backend": "pyav"}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare", help="Seal completed recording and schedule; no image decoding/encoding")
    prep.add_argument("--source", type=Path, required=True)
    prep.add_argument("--plan", type=Path, required=True)
    prep.add_argument("--name")
    prep.add_argument("--seconds", type=float, default=60)
    prep.add_argument("--ffmpeg", type=Path)
    prep.add_argument("--backend", choices=("ffmpeg", "pyav"), default="ffmpeg")
    prep.add_argument("--evaluation-surface", type=Path)
    prep.add_argument("--render-transition-receipt", type=Path)
    join = sub.add_parser("combine", help="Seal two single-case plans for one sequential PyAV encoding")
    join.add_argument("--plans", type=Path, nargs=2, required=True, metavar=("INNER_PLAN", "OUTER_PLAN"))
    join.add_argument("--plan", type=Path, required=True)
    join.add_argument("--name", default="INNER_AND_OUTER")
    run = sub.add_parser("render", help="Preview one frame, or explicitly encode under root supervisor")
    run.add_argument("--plan", type=Path, required=True)
    run.add_argument("--preview", type=Path)
    run.add_argument("--at", type=float, default=0, help="Preview position on output timeline, seconds")
    run.add_argument("--output", type=Path)
    run.add_argument("--permit-encoding", action="store_true")
    args = parser.parse_args()
    if args.command == "prepare":
        if not math.isfinite(args.seconds) or args.seconds < 5:
            parser.error("--seconds must be finite and at least 5")
        prepare(args)
    elif args.command == "combine":
        combine(args)
    else:
        if not args.preview and not args.output:
            parser.error("render requires --preview or --output")
        render(args)


if __name__ == "__main__":
    main()
