"""Replay actual simulated traces, score ablations, and export a portable viewer."""
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from course import PREVIEW
from driving import Conditions, simulate
from dynamics import DT


NAMES = {"outer": "原始双车道外环", "gates": "外环 · 交错窄门", "tight": "外环 · 收紧第二组窄门"}
SHIFTS = [(g, b) for g in (.85, 1.10) for b in (-.055, .055)]
COLORS = ("#b45c32", "#007771")


def measured(course, plant, weights, condition=Conditions(), adapt=False):
    _, finish, clearance, tr = simulate(course, plant, weights, condition, trace=True, adapt=adapt)
    elapsed = float(finish[0]) if np.isfinite(finish[0]) else None
    if elapsed is not None:
        t0, t1 = tr[-2:, 0]
        tr[-1] = tr[-2] + (tr[-1]-tr[-2]) * ((elapsed-t0)/(t1-t0))
    return {"finished": elapsed is not None, "time_s": elapsed, "progress_m": float(tr[-1, 1]),
            "min_clearance_m": float(clearance[0])}, tr


def playback(course, tr):
    xy = course.positions(tr[:, 1], tr[:, 2])
    direction = course.dir[course.indices(tr[:, 1])]
    heading = np.arctan2(direction[:, 1], direction[:, 0])+tr[:, 3]
    # seconds, x, y, progress, observed speed, right RPM, left RPM, heading.
    return np.column_stack((tr[:, 0], xy, tr[:, 1], tr[:, 4], tr[:, 6:8], heading))


def font(size, bold=False):
    name = "msyhbd.ttc" if bold else "msyh.ttc"
    for face in (Path("C:/Windows/Fonts")/name, "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(str(face), size)
        except OSError:
            continue
    return ImageFont.load_default()


def corner_animation(out, case, scene):
    # The two runs enter the same section at t=0; the HTML uses a full-lap clock.
    runs = [np.asarray(scene[k]) for k in ("first", "best")]
    s0, s1 = case.length*.56-8, case.length*.56+24
    starts = [np.interp(s0, row[:, 3], row[:, 0]) for row in runs]
    ends = [np.interp(s1, row[:, 3], row[:, 0])-start for row, start in zip(runs, starts)]
    ids = np.where((np.arange(len(case.xy))*case.ds>=s0-3) &
                   (np.arange(len(case.xy))*case.ds<=s1+3))[0]
    left = case.xy[ids]+case.normal[ids]*case.left[ids, None]
    right = case.xy[ids]+case.normal[ids]*case.right[ids, None]
    # Rotate the selected section so motion reads left to right.
    axis = case.xy[ids[-1]] - case.xy[ids[0]]
    axis /= np.linalg.norm(axis)
    rotate = np.array([[axis[0], -axis[1]], [axis[1], axis[0]]])
    verts = np.vstack((left, right)) @ rotate
    mid = (verts.max(axis=0)+verts.min(axis=0))/2
    scale = min(840/np.ptp(verts[:, 0]), 205/np.ptp(verts[:, 1]))
    def pixels(points):
        return ((np.asarray(points)@rotate-mid)*[scale,-scale] + [470,225]).round().astype(int)
    def path(pts):
        return [tuple(x) for x in pixels(pts).tolist()]
    f_title, f_main, f_small = font(26, True), font(18, True), font(15)
    base = Image.new("RGB", (940, 412), "#f6f6f1")
    pen = ImageDraw.Draw(base)
    pen.text((32, 16), "同一组窄门，两次训练结果", font=f_title, fill="#152e38")
    pen.text((33, 54), "离线仿真  /  同一入口开始计时  /  2 倍播放", font=f_small, fill="#526269")
    pen.polygon(path(np.vstack((left, right[::-1]))), fill="#e1e6df")
    for edge in (left,right):
        pen.line(path(edge),fill="#8e9d97",width=2)
    for run,color in zip(runs,COLORS):
        points=run[(run[:,3]>=s0)&(run[:,3]<=s1),1:3]
        pen.line(path(points),fill=tuple(int(color[i:i+2],16)//3+165 for i in (1,3,5)),width=2)
    for s, label in ((s0,"入口"),(s1,"出口")):
        idx=case.indices(np.array([s]))[0]
        a,b=case.xy[idx]+case.normal[idx]*case.left[idx],case.xy[idx]+case.normal[idx]*case.right[idx]
        pen.line(path([a,b]), fill="#89989a",width=2)
        x,y=pixels([a])[0]; pen.text((int(x)-12,int(y)-24),label, font=f_small,fill="#3e535a")
    frames=[]
    for tick in np.arange(0, max(ends)+1.0, .20):
        frame=base.copy(); draw=ImageDraw.Draw(frame)
        draw.text((742,28),f"区段  {tick:04.1f} s",font=f_main,fill="#152e38")
        for j,(run,start,end,color,label) in enumerate(zip(runs,starts,ends,COLORS,("首批达标","训练后"))):
            t=start+min(tick,end)
            row=np.array([np.interp(t,run[:,0],run[:,i]) for i in range(run.shape[1])])
            tail=run[(run[:,0]>=start)&(run[:,0]<t),1:3]
            if len(tail):
                draw.line(path(np.vstack((tail,row[1:3]))),fill=color,width=3)
            x,y=pixels([row[1:3]])[0].tolist()
            draw.ellipse((x-6,y-6,x+6,y+6),fill=color,outline="white",width=2)
            xbase=32+j*455
            draw.rounded_rectangle((xbase,342,xbase+419,391),radius=8,fill="#ffffff")
            draw.text((xbase+12,354),label,font=f_main,fill=color)
            msg=(f"{end:.2f} s 已通过" if tick>=end else
                 f"{row[4]:.2f} m/s    轮速指令 {row[5]:.0f} / {row[6]:.0f} RPM")
            draw.text((xbase+105,356),msg,font=f_small,fill="#293e49")
        frames.append(frame)
    frames[len(frames)//2].save(out/"corner.png")
    frames[0].save(out/"corner.gif",save_all=True,append_images=frames[1:],duration=100,loop=0,optimize=True)
    return list(map(float,ends))


def report(out, cases, plant, policy):
    scenes, scores = {}, {}
    summary = {"track_length_m": float(cases["outer"].length), "layouts": scores,
               "heldout_1s_speed_yaw_mae": plant["heldout_1s_speed_yaw_mae"],
               "changed_conditions": [], "same_driver_across_layouts": {}, "fixed_rpm_grid": [80, 300, 2]}
    for name, course in cases.items():
        weights = policy["courses"][name]
        traces = {}
        score = {}
        for label in ("first", "best"):
            score[label], tr = measured(course, plant, weights[label])
            traces[label] = playback(course, tr).round(4).tolist()
        grid = np.arange(80., 301., 2.)
        batch = np.tile(weights["best"], (len(grid), 1))
        _, times, _, _ = simulate(course, plant, batch, constant_rpm=grid)
        finished = np.isfinite(times)
        score["fixed_rpm_steering_frozen"] = {
            "finished": int(finished.sum()), "tried": len(grid),
            "best_rpm": float(grid[np.argmin(times)]) if finished.any() else None,
            "time_s": float(times.min()) if finished.any() else None}
        score["first_success_generation"] = next(row["generation"] for row in weights["history"] if row["finished"])
        score["candidate_evaluations"] = sum(row["trials"] for row in weights["history"])
        score["simulated_starts"] = score["candidate_evaluations"]*(3 if weights["robust_practice"] else 1)
        scores[name] = score
        summary["same_driver_across_layouts"][name], _ = measured(course, plant, policy["courses"]["tight"]["best"])
        left = course.xy + course.normal*course.left[:, None]
        right = course.xy + course.normal*course.right[:, None]
        scenes[name] = {"name": NAMES[name], "left": left[::2].round(3).tolist(),
                        "right": right[::2].round(3).tolist(), **traces, "score": score}
        for gain,bias in SHIFTS:
            condition = Conditions(speed_gain=gain, yaw_bias=bias)
            row = {"layout": name, "speed_gain_factor": gain, "added_yaw_per_100rpm": bias}
            for adapt,label in ((False,"frozen"),(True,"adaptive")):
                row[label], _ = measured(course, plant, weights["best"], condition, adapt)
            summary["changed_conditions"].append(row)
    scene = "tight"
    summary["clip_section_times_s_first_best"] = corner_animation(out, cases[scene], scenes[scene])
    driver = {"schema": 1, "weights": policy["courses"][scene]["best"], "edge_scale_m": cases[scene].width,
              "preview_m": PREVIEW.tolist(), "interval_s": DT}
    (out/"driver.json").write_text(json.dumps(driver, indent=2, allow_nan=False), encoding="utf8")
    template = Path(__file__).with_name("replay_template.html").read_text(encoding="utf8")
    payload = json.dumps(scenes, ensure_ascii=False, separators=(",",":"), allow_nan=False)
    (out/"replay.html").write_text(template.replace("/* TRACE_DATA */", payload),encoding="utf8")
    (out/"results.json").write_text(json.dumps(summary,indent=2,allow_nan=False),encoding="utf8")
    for name, score in scores.items():
        print(name, score["first"]["time_s"], score["best"]["time_s"], score["fixed_rpm_steering_frozen"])
    return summary
