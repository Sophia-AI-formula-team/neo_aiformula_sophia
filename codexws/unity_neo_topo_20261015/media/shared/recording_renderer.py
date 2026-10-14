"""Compose recorded Unity observer/YOLOP frames and aligned diagnostic telemetry.

No rendering, inference, route generation, simulation, or controller runs here.
Each trial supplies frames.jsonl and trace.jsonl with one common wall clock.
"""
from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import subprocess
from typing import Any

from PIL import Image, ImageDraw, ImageFont, ImageOps

WIDTH, HEIGHT, MAIN_HEIGHT = 1280, 960, 720
FPS = 15
PANEL_WIDTH = WIDTH // 4
BACKGROUND = (15, 21, 28)
MUTED = (165, 180, 193)
WHITE = (236, 242, 246)
GRID = (46, 58, 70)
CYAN, ORANGE, GREEN = (65, 202, 255), (255, 170, 80), (120, 225, 138)
PINK, PURPLE = (255, 105, 159), (190, 150, 255)
RED = (255, 96, 96)


def finite(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def load_jsonl(path: Path) -> list[dict]:
    rows = []
    with path.open(encoding="utf-8-sig") as source:
        for line_no, line in enumerate(source, 1):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict) or finite(value.get("wall")) is None:
                raise ValueError(f"{path}:{line_no}: object with finite wall timestamp required")
            if rows and float(value["wall"]) < float(rows[-1]["wall"]):
                raise ValueError(f"{path}:{line_no}: wall time moved backwards")
            rows.append(value)
    if not rows:
        raise ValueError(f"No records in {path}")
    return rows


def point(value: Any) -> tuple[float, float] | None:
    if isinstance(value, dict):
        value = [value.get("x"), value.get("y")]
    if not isinstance(value, (tuple, list)) or len(value) < 2:
        return None
    x, y = finite(value[0]), finite(value[1])
    return (x, y) if x is not None and y is not None else None


def digest(path: Path) -> str:
    with path.open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def font(size: int, bold: bool = False):
    # Yu Gothic includes Latin and Japanese/Chinese diagnostic case labels.
    names = ["C:/Windows/Fonts/YuGothB.ttc" if bold else "C:/Windows/Fonts/YuGothM.ttc",
             "C:/Windows/Fonts/segoeuib.ttf" if bold else "C:/Windows/Fonts/segoeui.ttf",
             "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"]
    for name in names:
        if Path(name).is_file():
            return ImageFont.truetype(name, size)
    return ImageFont.load_default(size=size)


FONTS = {"title": font(17, True), "body": font(15), "small": font(13), "tiny": font(11)}


def text(draw, xy, value, style="body", fill=WHITE, width=None):
    rendered = str(value)
    face = FONTS[style]
    if width is not None:
        while rendered and draw.textlength(rendered, font=face) > width:
            rendered = rendered[:-2] + "…" if len(rendered) > 2 else ""
    draw.text(xy, rendered, fill=fill, font=face)


def lines(draw, xy, value, width, max_lines=2, style="small", fill=MUTED):
    words = str(value).replace("\n", " ").split()
    result, row = [], ""
    for word in words:
        trial = (row + " " + word).strip()
        if row and draw.textlength(trial, font=FONTS[style]) > width:
            result.append(row)
            row = word
        else:
            row = trial
    if row:
        result.append(row)
    for index, row in enumerate(result[:max_lines]):
        if index == max_lines - 1 and len(result) > max_lines:
            row += "…"
        text(draw, (xy[0], xy[1] + index * 18), row, style, fill, width)


def number(value, precision=2):
    v = finite(value)
    return f"{v:.{precision}f}" if v is not None else "--"


def light_color(value):
    state = str(value or "unknown").lower()
    return RED if state == "red" else GREEN if state == "green" else MUTED


def semantic_runtime(row, elapsed_since_sample=0):
    """Distinguish a retained response from a currently running worker."""
    running = row.get("semantic_running")
    have_snapshot = bool(row.get("semantic"))
    age = finite(row.get("semantic_result_age"))
    if age is not None:
        age += max(0, finite(elapsed_since_sample) or 0)
    if running is False:
        label = "FROZEN / OFF" if have_snapshot else "OFF"
        frozen = have_snapshot
    elif running is True:
        label = "RUNNING / STALE" if age is not None and age > .75 else "RUNNING"
        frozen = age is not None and age > .75
    else:
        label = "runtime unrecorded"
        frozen = False
    return dict(label=label, frozen=frozen, age=age, status=row.get("semantic_runtime_status"),
                running=running, runtimeRecorded=isinstance(running, bool))


def stopping_diagnostics(row):
    """Expose recorded perception, controller goal and evaluation as separate facts."""
    semantic = row.get("semantic") or {}
    status = semantic.get("status") or {}
    approach = status.get("signal_approach") or {}
    observation = row.get("signal_range") or {}
    profile = approach.get("profile") or {}
    present = bool(observation or approach) or "actual_front_gap_m" in row
    if "signal_range" in row:
        distance = finite(observation.get("distance_m"))
        uncertainty = finite(observation.get("uncertainty_m"))
        observation_age = finite(row.get("age"))
    else:
        distance = finite(approach.get("distance_m"))
        uncertainty = finite(approach.get("uncertainty_m"))
        observation_age = finite(profile.get("age_sec"))
    valid = False if observation.get("valid") is False else profile.get("valid")
    return dict(present=present, distance=distance, uncertainty=uncertainty,
                source=observation.get("source") or observation.get("method") or "recorded visual range",
                age=observation_age, valid=valid,
                reason=profile.get("reason"), lower=finite(profile.get("distance_lower_m")),
                target=finite(approach.get("target_stop_distance_m")),
                speed_limit=finite(profile.get("speed_limit_mps")),
                evaluation_gap=finite(row.get("actual_front_gap_m")),
                frozen=semantic_runtime(row)["frozen"])


def chart(draw, rect, xbounds, ybounds, label_x, label_y):
    left, top, right, bottom = rect
    for index in range(5):
        fraction = index / 4
        x = left + fraction * (right-left)
        y = bottom - fraction * (bottom-top)
        draw.line((x, top, x, bottom), fill=GRID)
        draw.line((left, y, right, y), fill=GRID)
    draw.rectangle(rect, outline=MUTED)
    text(draw, (left, bottom+3), f"{xbounds[0]:.1f}", "tiny", MUTED)
    end = f"{xbounds[1]:.1f}"
    text(draw, (right-draw.textlength(end, font=FONTS["tiny"]), bottom+3), end, "tiny", MUTED)
    text(draw, (left-3, top-15), f"{ybounds[1]:.1f} {label_y}", "tiny", MUTED)
    text(draw, (left+50, bottom+3), label_x, "tiny", MUTED)

    def project(x, y):
        return (left + (x-xbounds[0])/(xbounds[1]-xbounds[0])*(right-left),
                bottom - (y-ybounds[0])/(ybounds[1]-ybounds[0])*(bottom-top))
    return project


class Trial:
    def __init__(self, directory: Path, synthetic=False):
        self.directory = directory.resolve()
        self.frames = load_jsonl(self.directory / "frames.jsonl")
        self.trace = load_jsonl(self.directory / "trace.jsonl")
        motion_path = self.directory / "motion.jsonl"
        self.motion = load_jsonl(motion_path) if motion_path.is_file() else self.trace
        self.have_motion = motion_path.is_file()
        for row in self.trace + (self.motion if self.have_motion else []):
            for key in ("candidate", "semantic"):
                if row.get(key) is not None and not isinstance(row[key], dict):
                    raise ValueError(f"{directory}: {key} must be an object or null")
        self.frame_times = [float(row["wall"]) for row in self.frames]
        self.trace_times = [float(row["wall"]) for row in self.trace]
        self.motion_times = [float(row["wall"]) for row in self.motion]
        self.start, self.end = self.frame_times[0], self.frame_times[-1]
        if len(self.frames) < 2 or self.end <= self.start:
            raise ValueError(f"{directory}: two distinct captured frame times required")
        if self.trace_times[-1] < self.start or self.trace_times[0] > self.end:
            raise ValueError(f"{directory}: frames and trace have no overlapping wall clock")
        self.name = directory.name
        self.synthetic = synthetic
        result_path = self.directory / "result.json"
        self.result = json.loads(result_path.read_text(encoding="utf-8-sig")) if result_path.is_file() else None
        if self.result is not None and not isinstance(self.result, dict):
            raise ValueError(f"{directory}: result.json must contain an object")
        self._terminal_overlay = None
        self._terminal_overlay_key = None
        self._cache = {}
        self.loaded_files = set()
        self.world_points = [(float(row["x"]), float(row["z"])) for row in self.motion
                             if finite(row.get("x")) is not None and finite(row.get("z")) is not None]
        self.bounds = self.trajectory_bounds()
        for row in self.frames:
            self.resolve_image(row["path"])
            if row.get("overlay"):
                self.resolve_image(row["overlay"])

    def resolve_image(self, value):
        path = (self.directory / str(value)).resolve()
        if not path.is_relative_to(self.directory) or not path.is_file():
            raise ValueError(f"Image missing or outside trial directory: {value}")
        return path

    def image(self, value):
        path = self.resolve_image(value)
        self.loaded_files.add(str(path.relative_to(self.directory)))
        if path not in self._cache:
            with Image.open(path) as source:
                self._cache[path] = source.convert("RGB")
            while len(self._cache) > 6:
                self._cache.pop(next(iter(self._cache)))
        return self._cache[path]

    def trajectory_bounds(self):
        if not self.world_points:
            return (-1, 1), (-1, 1)
        xs, zs = zip(*self.world_points)
        # Equal scale; fixed bounds for the complete recorded trial avoid zoom jumps.
        cx, cz = (min(xs)+max(xs))/2, (min(zs)+max(zs))/2
        span = max(max(xs)-min(xs), max(zs)-min(zs), 8) + 4
        return (cx-span/2, cx+span/2), (cz-span/2, cz+span/2)

    def at(self, wall):
        fi = max(0, bisect.bisect_right(self.frame_times, wall)-1)
        ti = bisect.bisect_right(self.motion_times, wall)-1
        return self.frames[fi], self.motion[ti] if ti >= 0 else {}, fi, ti

    def summary(self):
        values = [finite(row.get("actual_v")) for row in self.motion]
        values = [value for value in values if value is not None]
        return dict(name=self.name, source=str(self.directory), sourceWallStart=self.start,
                    sourceWallEnd=self.end, sourceWallDuration=self.end-self.start,
                    sourceFrames=len(self.frames), telemetryRows=len(self.trace),
                    motionRows=len(self.motion), motionSource="motion.jsonl" if self.have_motion else "trace.jsonl",
                    sourceResult=self.result,
                    maxSourceFrameGap=max(b-a for a, b in zip(self.frame_times, self.frame_times[1:])),
                    minRecordedSpeed=min(values) if values else None,
                    maxRecordedSpeed=max(values) if values else None,
                    traceSha256=digest(self.directory/"trace.jsonl"),
                    motionSha256=digest(self.directory/"motion.jsonl") if self.have_motion else None,
                    framesSha256=digest(self.directory/"frames.jsonl"),
                    inputImagesRead=sorted(self.loaded_files), synthetic=self.synthetic)


def paste_contained(canvas, source, rect):
    left, top, right, bottom = rect
    resized = ImageOps.contain(source, (right-left, bottom-top), Image.Resampling.LANCZOS)
    canvas.paste(resized, (left+(right-left-resized.width)//2, top+(bottom-top-resized.height)//2))


def corridor_diagnostics(candidate):
    """Describe recorded geometry provenance; never manufacture a lane/width."""
    corridor = candidate.get("corridor") or {}
    debug = candidate.get("corridor_debug") or {}
    if not isinstance(corridor, dict):
        corridor = {}
    if not isinstance(debug, dict):
        debug = {}
    clouds = candidate.get("clouds") or []
    inferred_left = corridor.get("source") == "current_left_identity_checked_native"
    inferred = (inferred_left or corridor.get("source") == "current_right_inferred"
                or debug.get("target_type") == "inferred_center_from_current_right")
    paired = (debug.get("target_type") == "observed_center" and len(clouds) >= 2
              and bool(clouds[0]) and bool(clouds[1]))
    calibration = debug.get("calibration") or {}
    if not isinstance(calibration, dict):
        calibration = {}
    quality = debug.get("pair_quality") or {}
    boundary = debug.get("raw_left_source" if inferred_left else "raw_right_source") or {}
    identity = debug.get("left_identity_match") or {}
    width = corridor.get("width_m", debug.get("width_m", quality.get("width_m")))
    capture_age = corridor.get("calibration_age_at_capture", debug.get("calibration_age_at_capture",
                            calibration.get("calibration_age_at_capture")))
    limit = corridor.get("max_calibration_age_sec", debug.get("calibration_max_age_sec",
                         calibration.get("calibration_max_age_sec")))
    return dict(present=bool(corridor or debug), inferred=inferred, inferred_left=inferred_left, paired=paired,
                side="left" if inferred_left else "right",
                label="ESTIMATED center from CURRENT left" if inferred_left else
                      "ESTIMATED center from CURRENT right" if inferred else
                      "MEASURED L+R / derived center" if paired else "No supported current corridor",
                boundary_source=boundary.get("kind"), width=finite(width),
                native_left_width=finite(corridor.get("native_left_width_m", debug.get("native_left_width_m"))),
                identity_accepted=identity.get("accepted"),
                capture_age=finite(capture_age), limit=finite(limit))


def draw_lane(draw, x, row, age):
    candidate = row.get("candidate") or {}
    semantic = row.get("semantic") or {}
    runtime = semantic_runtime(row)
    unsupported = candidate.get("supported") is False
    corridor = corridor_diagnostics(candidate)
    title = "LLP UNSUPPORTED + LYA" if unsupported else "LLP + LYA | estimated" if corridor["inferred"] else "LLP + LYA points / target"
    text(draw, (x+10, MAIN_HEIGHT+8), title, "title", ORANGE if unsupported else WHITE, 300)
    # Provenance and width timing remain entirely inside the lower LLP panel.
    area = ((x+99, MAIN_HEIGHT+60, x+221, MAIN_HEIGHT+182) if corridor["present"]
            else (x+84, MAIN_HEIGHT+53, x+236, MAIN_HEIGHT+205))
    project = chart(draw, area, (-5, 5), (0, 10), "right [m]", "forward [m]")
    if corridor["present"]:
        text(draw, (x+10, MAIN_HEIGHT+29), corridor["label"], "tiny",
             ORANGE if corridor["inferred"] or unsupported else CYAN, 300)
        source = ("Current " + corridor["side"] + ": " + str(corridor["boundary_source"] or "unrecorded")) if corridor["inferred"] else (
                 "Current left + right boundaries" if corridor["paired"] else str(candidate.get("selection_reason") or "unsupported"))
        if corridor["inferred_left"]:
            identity = "ID checked" if corridor["identity_accepted"] is True else "ID REJECTED" if corridor["identity_accepted"] is False else "ID source flag"
            source = f"Native width {number(corridor['native_left_width'])}m | {identity}"
        text(draw, (x+10, MAIN_HEIGHT+199), source, "tiny", MUTED, 300)
        if corridor["inferred"]:
            width = (f"{'ID prior' if corridor['inferred_left'] else 'Width'} {number(corridor['width'])}m | age@capture "
                     f"{number(corridor['capture_age'])}/{number(corridor['limit'])}s")
        elif corridor["paired"]:
            width = f"Measured pair width: {number(corridor['width'])}m"
        else:
            width = "No current corridor; fallback is unsupported"
        text(draw, (x+10, MAIN_HEIGHT+213), width, "tiny", ORANGE if corridor["inferred"] else MUTED, 300)

    def valid(p):
        return p is not None and 0 <= p[0] <= 10 and -5 <= p[1] <= 5

    if age is None or age > .75:
        text(draw, (x+73, MAIN_HEIGHT+116), "No fresh recorded geometry", "small", ORANGE, 232)
        return
    # Raw publisher clouds are context, not the selected corridor: show only dim
    # dots so they cannot be mistaken for measured selected center geometry.
    for cloud in candidate.get("raw_clouds") or []:
        for raw in cloud:
            p = point(raw)
            if valid(p):
                u, v = project(-p[1], p[0])
                draw.point((u, v), fill=(82, 95, 109))
    for cloud, color in zip(candidate.get("clouds") or [], (CYAN, ORANGE, GREEN)):
        last = None
        for raw in cloud:
            p = point(raw)
            if not valid(p):
                last = None
                continue
            pixel = project(-p[1], p[0])
            if last is not None:
                draw.line((*last, *pixel), fill=color, width=2)
            draw.ellipse((pixel[0]-1, pixel[1]-1, pixel[0]+1, pixel[1]+1), fill=color)
            last = pixel
    for index, raw in enumerate((candidate.get("selected_points") or [])[:3]):
        p = point(raw)
        if valid(p):
            u, v = project(-p[1], p[0])
            color = MUTED if unsupported else ORANGE if corridor["inferred"] else WHITE
            draw.ellipse((u-3, v-3, u+3, v+3), fill=color)
            text(draw, (u+5, v-8), "ABC"[index], "tiny", color)
    semantic_color = MUTED if runtime["frozen"] else PURPLE
    for raw in (semantic.get("selected_points") or [])[:3]:
        p = point(raw)
        if valid(p):
            u, v = project(-p[1], p[0])
            draw.rectangle((u-3, v-3, u+3, v+3), outline=semantic_color, width=2)
    p = point(semantic.get("target") if semantic else candidate.get("target"))
    if valid(p):
        u, v = project(-p[1], p[0])
        draw.ellipse((u-5, v-5, u+5, v+5), outline=MUTED if runtime["frozen"] else PINK, width=2)
    mode = candidate.get("mode", "No published target")
    legend = "gray: frozen semantic snapshot" if runtime["frozen"] else "purple/pink: semantic LYA"
    if unsupported:
        count = len(candidate.get("selected_points") or [])
        role = "fallback" if candidate.get("selection_source") == "default_straight" else "unsupported"
        footer = f"ABC={count} {role} | {candidate.get('selection_reason') or mode}"
    else:
        footer = f"{mode} | {legend}"
    if corridor["present"] and not unsupported:
        footer = "raw: dim dots | current: color | " + ("LYA: frozen" if runtime["frozen"] else "LYA: purple/pink")
    text(draw, (x+10, MAIN_HEIGHT+(227 if corridor["present"] else 222)), footer, "tiny", ORANGE if unsupported else MUTED, 304)


def semantic_value(semantic, *names):
    for name in names:
        value = semantic.get(name)
        if value is not None:
            return value
    return None


def draw_semantic(draw, x, row, age):
    semantic = row.get("semantic") or {}
    runtime = semantic_runtime(row, age)
    title = "Semantic | " + runtime["label"] if runtime["runtimeRecorded"] else "Semantic planning"
    text(draw, (x+10, MAIN_HEIGHT+8), title, "title", ORANGE if runtime["frozen"] else WHITE, 300)
    state = semantic_value(semantic, "status", "state", "behavior", "mode")
    status = state if isinstance(state, dict) else {}
    if status:
        state = " | ".join(str(status[key]) for key in ("phase", "traffic_mode") if status.get(key))
        if runtime["frozen"]:
            state = "Last: " + state
        text(draw, (x+10, MAIN_HEIGHT+32), state or "Semantic output", "small", PURPLE, 300)
        # Independent recorded fields: a velocity gate must not hide a new
        # traffic reason such as red_approach_reacquired or pending recovery.
        text(draw, (x+10, MAIN_HEIGHT+50),
             "Traffic: " + str(status.get("traffic_reason") or "--"), "tiny", MUTED, 300)
        gate = str(status.get("velocity_gate_reason") or status.get("reason") or "--")
        if runtime["frozen"]:
            gate += f" | age {number(runtime['age'])}s"
        text(draw, (x+10, MAIN_HEIGHT+65), "Gate: " + gate, "tiny", MUTED, 300)
    else:
        lines(draw, (x+10, MAIN_HEIGHT+34), state or "No semantic output recorded", 300, 2, fill=PURPLE)
    truth = row.get("signal_truth", "unknown")
    detected = row.get("detected_light", "unknown")
    text(draw, (x+10, MAIN_HEIGHT+83), f"Scenario light: {truth}", "small", light_color(truth), 300)
    text(draw, (x+10, MAIN_HEIGHT+105), f"Detected light: {detected}", "small", light_color(detected), 300)
    text(draw, (x+10, MAIN_HEIGHT+127),
         f"{'Last' if runtime['frozen'] else 'Semantic'} v/w: {number(semantic.get('v'))} / {number(semantic.get('w'))}",
         "small", MUTED if runtime["frozen"] else WHITE, 300)
    target = point(semantic_value(semantic, "target", "effective_target", "selected_target"))
    target_text = f"{'Last' if runtime['frozen'] else 'LYA'} target x/y: {target[0]:.2f} / {target[1]:.2f} m" if target else "No semantic target published"
    text(draw, (x+10, MAIN_HEIGHT+149), target_text, "small", PURPLE, 300)
    text(draw, (x+10, MAIN_HEIGHT+171),
         f"Applied v/w: {number(row.get('applied_v'))} / {number(row.get('applied_w'))}", "small", WHITE, 300)
    text(draw, (x+10, MAIN_HEIGHT+193),
         f"Actual v/w: {number(row.get('actual_v'))} / {number(row.get('actual_w'))}", "small", CYAN, 300)
    guard = semantic.get("corridor_guard")
    if isinstance(guard, dict):
        # Report the worker's recorded decision and age. Do not infer a traffic
        # state or recompute freshness while rendering a held telemetry sample.
        guard_state = "VALID" if guard.get("valid") is True else "EXPIRED" if guard.get("valid") is False else "unrecorded"
        prefix = "Last width guard" if runtime["frozen"] else "Width guard"
        guard_text = (f"{prefix}: {guard_state} | age "
                      f"{number(guard.get('calibration_age_sec'))}/{number(guard.get('max_age_sec'))}s")
        text(draw, (x+10, MAIN_HEIGHT+209), guard_text, "tiny",
             MUTED if runtime["frozen"] else ORANGE if guard.get("valid") is False else WHITE, 300)
    reason = semantic_value(semantic, "reason", "message", "source") or status.get("reason")
    footer = str(reason) if reason is not None else "v: m/s | w: rad/s (+ left)"
    stopped = row.get("phase") == "stopped_tail"
    if stopped:
        footer = "STOP: " + str(row.get("follower_status") or "recorded terminal hold")
    elif age is not None and age > .75:
        footer = f"TELEMETRY GAP {age:.2f}s | " + footer
    text(draw, (x+10, MAIN_HEIGHT+222), footer, "tiny", ORANGE if stopped or age is not None and age > .75 else MUTED, 300)


def draw_trajectory(draw, x, trial, ti, row):
    stopping = stopping_diagnostics(row)
    text(draw, (x+10, MAIN_HEIGHT+8), "Trajectory + stopping distance" if stopping["present"] else "Measured Unity trajectory", "title", width=300)
    area = (x+62, MAIN_HEIGHT+54, x+265, MAIN_HEIGHT+(133 if stopping["present"] else 203))
    # Plot actual coordinates with equal metres per pixel, keeping the requested
    # fixed map extent. No route or interpolation is supplied to a controller.
    xb, zb = trial.bounds
    xspan, zspan = xb[1]-xb[0], zb[1]-zb[0]
    center = sum(xb)/2
    xspan = zspan*(area[2]-area[0])/(area[3]-area[1])
    xb = (center-xspan/2, center+xspan/2)
    project = chart(draw, area, xb, zb, "world X [m]", "Z [m]")
    prior = None
    for sample in trial.motion[:max(0, ti+1)]:
        xx, zz = finite(sample.get("x")), finite(sample.get("z"))
        if xx is None or zz is None:
            prior = None
            continue
        pixel = project(xx, zz)
        if prior is not None:
            draw.line((*prior, *pixel), fill=light_color(sample.get("signal_truth")), width=3)
        prior = pixel
    if prior is not None:
        draw.ellipse((prior[0]-4, prior[1]-4, prior[0]+4, prior[1]+4), fill=WHITE)
        yaw = finite(row.get("yaw"))
        if yaw is not None:
            angle = math.radians(yaw)
            draw.line((*prior, prior[0]+10*math.sin(angle), prior[1]-10*math.cos(angle)), fill=WHITE, width=2)
    if stopping["present"]:
        prefix = "Last RGB body-forward depth" if stopping["frozen"] else "RGB body-forward depth"
        source = str(stopping["source"])
        if stopping["age"] is not None:
            source += f" | age {stopping['age']:.2f}s"
        text(draw, (x+10, MAIN_HEIGHT+152), source, "tiny", MUTED, 300)
        estimate = f"{prefix}: {number(stopping['distance'])} m"
        if stopping["uncertainty"] is not None:
            estimate += f" +/- {stopping['uncertainty']:.2f}"
        estimate_style = "small" if draw.textlength(estimate, font=FONTS["small"]) <= 300 else "tiny"
        text(draw, (x+10, MAIN_HEIGHT+168), estimate, estimate_style, MUTED if stopping["frozen"] else PURPLE, 300)
        text(draw, (x+10, MAIN_HEIGHT+187),
             f"8-9 m requested (plane) | {'last cap' if stopping['frozen'] else 'cap'} {number(stopping['speed_limit'])} m/s",
             "tiny", MUTED if stopping["frozen"] else WHITE, 300)
        text(draw, (x+10, MAIN_HEIGHT+206),
             f"Measured plane front gap: {number(stopping['evaluation_gap'])} m [eval]", "small", CYAN, 300)
        footer = str(stopping["reason"] or "Red/green: scenario | white: car")
        text(draw, (x+10, MAIN_HEIGHT+225), footer, "tiny", ORANGE if stopping["valid"] is False else MUTED, 300)
    else:
        text(draw, (x+10, MAIN_HEIGHT+222), "Trace color: scenario red / green; white: car", "tiny", MUTED, 300)


def wrap_complete(draw, value, face, width):
    """Wrap every character, including long tokens; never ellipsize a result."""
    wrapped = []
    for paragraph in str(value).split("\n"):
        row = ""
        for char in paragraph:
            if row and draw.textlength(row + char, font=face) > width:
                wrapped.append(row)
                row = char
            else:
                row += char
        wrapped.append(row)
    return wrapped


def terminal_overlay(trial, row=None):
    """A retrospective terminal label inside the diagnostic bar, never the scene."""
    result = trial.result or {}
    accepted = result.get("accepted")
    accepted_label = str(accepted).lower() if isinstance(accepted, bool) else "not recorded"
    current_gap = finite((row or {}).get("actual_front_gap_m"))
    headline = (f"Recorded accepted: {accepted_label} | {trial.name}"
                f" | Current fixed-plane FRONT gap: {number(current_gap)} m [eval]"
                " | Requested red-stop 8-9 m")
    if trial._terminal_overlay is not None and getattr(trial, "_terminal_overlay_key", None) == headline:
        return trial._terminal_overlay
    messages = [headline,
                "Terminal reason: " + str(result.get("reason") or "not recorded")]
    gap = finite(result.get("stoppedFrontGap"))
    if gap is not None:
        target_check = result.get("redStopInTarget")
        check_label = str(target_check).lower() if isinstance(target_check, bool) else "not recorded"
        messages.append(f"Red-stop plane front gap: {gap:.6f} m [eval] | Recorded target-band check: {check_label}")
    scratch = ImageDraw.Draw(Image.new("RGB", (1, 1)))
    for size in (15, 13, 11):
        face = font(size)
        wrapped = [line for message in messages for line in wrap_complete(scratch, message, face, WIDTH-24)]
        ascent, descent = face.getmetrics()
        line_height = ascent + descent + 2
        height = 16 + len(wrapped)*line_height
        if height <= HEIGHT-MAIN_HEIGHT:
            overlay = Image.new("RGB", (WIDTH, height), (25, 31, 39))
            draw = ImageDraw.Draw(overlay)
            color = ORANGE if accepted is False else MUTED
            draw.line((0, 0, WIDTH, 0), fill=color, width=2)
            for index, line in enumerate(wrapped):
                draw.text((12, 8+index*line_height), line, font=face, fill=color if index == 0 else WHITE)
            trial._terminal_overlay = overlay
            trial._terminal_overlay_key = headline
            return overlay
    raise ValueError(f"Terminal result is too long for the diagnostic bar without truncation: {trial.directory}")


def compose(trial, wall):
    frame, row, fi, ti = trial.at(wall)
    canvas = Image.new("RGB", (WIDTH, HEIGHT), BACKGROUND)
    paste_contained(canvas, trial.image(frame["path"]), (0, 0, WIDTH, MAIN_HEIGHT))
    draw = ImageDraw.Draw(canvas)
    for column in range(1, 4):
        draw.line((column*PANEL_WIDTH, MAIN_HEIGHT, column*PANEL_WIDTH, HEIGHT), fill=GRID, width=2)
    text(draw, (10, MAIN_HEIGHT+8), "YOLOP combined overlay", "title")
    if frame.get("overlay"):
        paste_contained(canvas, trial.image(frame["overlay"]), (8, MAIN_HEIGHT+32, 312, MAIN_HEIGHT+203))
    else:
        text(draw, (14, MAIN_HEIGHT+105), "No overlay captured for this frame", "small", ORANGE, 290)
    label = ("SYNTHETIC QA | " if trial.synthetic else "") + trial.name
    text(draw, (10, MAIN_HEIGHT+205), label, "tiny", ORANGE if trial.synthetic else WHITE, 300)
    frame_age = max(0, wall-float(frame["wall"]))
    text(draw, (10, MAIN_HEIGHT+222),
         f"Wall {wall-trial.start:.2f}s | sim {number(frame.get('t'))}s | frame age {frame_age:.2f}s",
         "tiny", ORANGE if frame_age > .5 else MUTED, 300)
    age = max(0, wall-float(row["wall"])) if row else None
    perception_age = finite(row.get("age"))
    if perception_age is not None and age is not None:
        perception_age += age
    draw_lane(draw, 320, row, perception_age if perception_age is not None else age)
    draw_semantic(draw, 640, row, age)
    draw_trajectory(draw, 960, trial, ti, row)
    # Preserve the full wall-clock sequence. Only the last diagnostic rows are
    # replaced by recorded outcome text; the 1280x720 observer is untouched.
    if trial.result is not None and (row.get("phase") == "stopped_tail" or wall >= trial.end-2.0):
        overlay = terminal_overlay(trial, row)
        canvas.paste(overlay, (0, HEIGHT-overlay.height))
    return canvas, fi


def find_ffmpeg(explicit=None):
    if explicit:
        path = Path(explicit)
        if not path.is_file():
            raise FileNotFoundError(path)
        return str(path)
    # Script lives below workspace/diagnostics/<task>/staging/video.
    for ancestor in Path(__file__).resolve().parents:
        candidate = ancestor / "tools/video_deps/imageio_ffmpeg/binaries/ffmpeg-win-x86_64-v7.1.exe"
        if candidate.is_file():
            return str(candidate)
    from shutil import which
    path = which("ffmpeg")
    if not path:
        raise FileNotFoundError("Provide --ffmpeg /path/to/ffmpeg")
    return path


def encode(trials, destination, ffmpeg, fps=FPS):
    destination = destination.resolve()
    if destination.suffix.lower() != ".mp4":
        raise ValueError("Output must end in .mp4")
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise FileExistsError(f"Refusing to replace existing video: {destination}")
    command = [ffmpeg, "-hide_banner", "-loglevel", "warning", "-n", "-f", "rawvideo", "-pix_fmt", "rgb24",
               "-s", f"{WIDTH}x{HEIGHT}", "-r", str(fps), "-i", "pipe:0", "-an", "-c:v", "libx264",
               "-preset", "fast", "-crf", "19", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(destination)]
    log = destination.with_suffix(".encode.log")
    contacts, segments, frame_total = [], [], 0
    with log.open("wb") as stderr:
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=stderr)
        try:
            for trial in trials:
                duration = trial.end-trial.start
                # Include the true final captured frame; preserve every stationary interval.
                count = math.ceil(duration*fps)+1
                preview_indices = {round((count-1)*f) for f in (0, .25, .5, .75, 1)}
                source_indices = set()
                for index in range(count):
                    wall = min(trial.end, trial.start+index/fps)
                    canvas, source_index = compose(trial, wall)
                    source_indices.add(source_index)
                    process.stdin.write(canvas.tobytes())
                    if index in preview_indices:
                        preview = canvas.resize((640, 480), Image.Resampling.LANCZOS)
                        contacts.append(preview)
                segments.append(dict(**trial.summary(), outputStartFrame=frame_total,
                                     outputFrames=count, outputDuration=count/fps,
                                     uniqueSourceFramesShown=len(source_indices),
                                     timingRoundingSeconds=count/fps-duration))
                frame_total += count
        finally:
            process.stdin.close()
            returncode = process.wait()
    if returncode:
        raise RuntimeError(f"FFmpeg returned {returncode}; see {log}")
    contact = Image.new("RGB", (1280, math.ceil(len(contacts)/2)*480), BACKGROUND)
    for index, preview in enumerate(contacts):
        contact.paste(preview, ((index%2)*640, (index//2)*480))
    sheet = destination.with_suffix(".contact.jpg")
    contact.save(sheet, quality=94)
    verification_path = destination.with_suffix(".decode.json")
    verify_command = [ffmpeg, "-v", "error", "-i", str(destination), "-progress", "pipe:1", "-f", "null", "-"]
    verified = subprocess.run(verify_command, capture_output=True, text=True, check=True)
    progress = {}
    for line in verified.stdout.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            progress[key] = value
    if int(progress.get("frame", -1)) != frame_total or progress.get("progress") != "end":
        raise RuntimeError(f"Decode frame count mismatch: {progress}")
    verification_path.write_text(json.dumps(dict(progress=progress, stderr=verified.stderr), indent=2), encoding="utf-8")
    report = dict(video=str(destination), videoSha256=digest(destination), contactSheet=str(sheet),
                  encoderSha256=digest(Path(__file__)),
                  stoppingDistanceDisplay=dict(requestedFrontGapBandM=[8, 9],
                      perceptionField="signal_range.distance_m: RGB body-forward depth at capture, projected along the current vehicle forward axis; not a fixed-track-plane gap",
                      controllerPolicyField="semantic.status.signal_approach.target_stop_distance_m: lower-bound policy, not nominal requested gap",
                      evaluationField="actual_front_gap_m: independent evaluator-only leading bumper corner clearance to the fixed track signal plane"),
                  corridorDisplay=dict(inferred="candidate.corridor.source=current_right_inferred: ESTIMATED center from the current measured right boundary and a recorded scalar width prior; orange ABC",
                      inferredLeft="candidate.corridor.source=current_left_identity_checked_native: ESTIMATED center from CURRENT left; native_left_width_m is the native asymmetric ABC compensation parameter, distinct from the paired width/identity prior. left_identity_match.accepted is displayed only as recorded. No measured-center claim or fabricated center cloud.",
                      paired="candidate.corridor_debug.target_type=observed_center with left+right clouds: MEASURED L+R boundaries with a derived center",
                      rawClouds="candidate.raw_clouds: dim context dots; candidate.clouds: colored current selected geometry",
                      widthAge="candidate.corridor.calibration_age_at_capture/max_calibration_age_sec: age at capture and recorded limit, not freshness extended by video rendering",
                      semanticGuard="semantic.corridor_guard: worker-recorded VALID/EXPIRED and calibration_age_sec/max_age_sec; frozen values explicitly marked Last",
                      gate="semantic.status.velocity_gate_reason remains separate from semantic.status.traffic_reason; width expiry is a command gate, not a light state"),
                  width=WIDTH, height=HEIGHT, fps=fps, frames=frame_total, duration=frame_total/fps,
                  codec="H.264", pixelFormat="yuv420p", observerRegion=[0, 0, WIDTH, MAIN_HEIGHT],
                  observerRegionFraction=MAIN_HEIGHT/HEIGHT, playbackSpeed=1,
                  stationaryIntervalsPreserved=True, resampling="latest recorded frame at output wall timestamp",
                  finalFrameIncluded=True, segments=segments,
                  terminalResultDisplay="Original result.json accepted and full reason plus current recorded evaluator fixed-plane FRONT gap and requested red-stop 8-9 m, shown in the lower diagnostic bar during stopped_tail or final 2 captured seconds; no timeline trimming or added motion.",
                  evidence="Recorded observer and original overlay only; no new inference or simulated trajectory.",
                  truthRole="Scenario labels and trajectory colors only; independent of detected_light.",
                  resultInterpretation="No success/failure claim inferred by encoder.")
    destination.with_suffix(".verification.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path, nargs="+", help="One trial or an ordered list of trials to concatenate")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ffmpeg")
    parser.add_argument("--fps", type=int, default=FPS, choices=[10, 15, 30])
    parser.add_argument("--synthetic", action="store_true", help="Label test fixture output SYNTHETIC QA")
    args = parser.parse_args()
    trials = [Trial(path, args.synthetic) for path in args.trial]
    print(json.dumps(encode(trials, args.output, find_ffmpeg(args.ffmpeg), args.fps), indent=2))


if __name__ == "__main__":
    main()
