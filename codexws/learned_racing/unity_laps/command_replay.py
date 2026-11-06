"""Film an actual moving-cone encounter and synchronize it with causal control records."""
import argparse
import bisect
import json
import math
from pathlib import Path

import av
import numpy as np
from PIL import Image, ImageDraw

from render import BLUE, BG, INK, LIME, MUTE, ORANGE, PANEL, font


FPS = 20
FOCAL = 254.391622
CX, CY, HEIGHT = 330.01302, 181.14964, .655
DEPTHS = (2., 3.5, 5.5, 8.)


def text(draw, xy, message, size=18, color=INK, bold=False, mono=False):
    draw.text(xy, str(message), font=font(size, bold, mono), fill=color, anchor="lt")


def phase(decision):
    reason = decision["reason"]
    if reason == "insufficient_lane":
        return "可用车线不足，限制车速"
    if reason == "yield":
        if decision["guardCap"] < decision["cruise"]:
            return "前方存在冲突，降速避让"
        return "已观察到前方冲突"
    if reason == "recover":
        return "回线阶段，检查恢复限速"
    if decision["ratio"] < .99:
        return "绕障候选选择较低车速"
    return "按车线曲率设定车速" if reason == "road" else str(reason)


def camera_marks(image, decision):
    """Project the actual last controller observation, at its own image timestamp."""
    draw = ImageDraw.Draw(image)
    points = []
    for z, x in zip(DEPTHS, decision["observed"]):
        if x is not None:
            points.append((CX + FOCAL * x / z, CY + HEIGHT * FOCAL / z))
    if len(points) > 1:
        draw.line(points, fill=BLUE, width=2)
    for u, v in points:
        draw.ellipse((u-3, v-3, u+3, v+3), fill=BLUE)
    for z, x, _, _ in decision["tracks"]:
        if z <= .2:
            continue
        u, v = CX + FOCAL*x/z, CY + HEIGHT*FOCAL/z
        if 0 < u < image.width and 0 < v < image.height:
            draw.ellipse((u-7, v-7, u+7, v+7), outline=ORANGE, width=3)


def crop_window(run):
    moving = [prop for prop in run["props"] if abs(prop[4]) > 1.e-5 and abs(prop[6]) > 1.e-5]
    if not moving:
        raise ValueError("Run has no moving obstacle")
    rows = np.asarray(run["rows"])
    station = moving[0][-1]
    if rows[-1, 7] < station-7:
        raise ValueError("Run terminated before approaching the moving obstacle")
    arrival = float(np.interp(station, rows[:, 7], rows[:, 0]))
    end = min(arrival + 3.0, run["frames"][-1][0])
    return max(0., end - 8.0), end


def plot(draw, box, times, series, start, end, now, bounds, key):
    left, top, right, bottom = box
    minimum, maximum = bounds
    def xp(t):
        return left + (t-start)/(end-start)*(right-left)
    def yp(y):
        return bottom - (y-minimum)/(maximum-minimum)*(bottom-top)
    for value in (minimum, (minimum+maximum)/2, maximum):
        y = yp(value)
        draw.line((left, y, right, y), fill="#405158")
        draw.text((left-6, y), f"{value:g}", font=font(13, mono=True), fill=MUTE, anchor="rm")
    for i in range(5):
        t = start + (end-start)*i/4
        draw.text((xp(t), bottom+5), f"{t:.1f}", font=font(13, mono=True), fill=MUTE, anchor="mt")
    upto = bisect.bisect_right(times, now + 1.e-6)
    first = max(0, bisect.bisect_right(times, start)-1)
    for index, color in enumerate((LIME, BLUE)):
        points = []
        for i in range(first, upto):
            x, y = xp(max(times[i], start)), yp(series[i, index])
            if points:
                points.append((x, points[-1][1]))
            points.append((x, y))
        if points:
            points.append((xp(now), points[-1][1]))
            draw.line(points, fill=color, width=3)
    draw.line((xp(now), top, xp(now), bottom), fill=MUTE)
    text(draw, (left, top-21), key[0], 14, LIME)
    text(draw, (left+95, top-21), key[1], 14, BLUE)


def encode(root, run_id, out):
    run = json.loads((root/"runs"/(run_id+".json")).read_bytes())
    decisions = run.get("decisions", [])
    frames = run["frames"]
    if not decisions or not frames:
        raise ValueError("The selected run needs both Unity camera frames and control decisions")
    start, end = crop_window(run)
    dtimes = [d["t"] for d in decisions]
    ftimes = [f[0] for f in frames]
    speed = np.array([[d["speedCmd"], d["speed"]] for d in decisions])
    rpm = np.array([[d["rightCan"], d["leftCan"]] for d in decisions])
    window = slice(max(0, bisect.bisect_right(dtimes, start)-1), bisect.bisect_right(dtimes, end))
    v_max = max(2, math.ceil(speed[window].max()/2)*2)
    rpm_min = min(0, math.floor(rpm[window].min()/100)*100)
    rpm_max = max(rpm_min+200, math.ceil(rpm[window].max()/200)*200)
    images = root/".work"/"frames"/run_id
    camera_dir = root/".work"/"camera"/run_id/"a4_samples"
    a4_capture = run["request"]["vision"] and camera_dir.exists()
    frame_count = math.ceil((end-start)*FPS)+1
    out.parent.mkdir(parents=True, exist_ok=True)
    # The timestamp on a raw optical frame can differ from the last decision by
    # one capture period. Only add projected marks to a time-matched camera frame.
    tmp = out.with_name(out.stem+".part.mp4")
    with av.open(str(tmp), "w", options={"movflags":"+faststart"}) as box:
        stream = box.add_stream("libx264", rate=FPS)
        stream.width, stream.height = 1280, 816
        stream.pix_fmt = "yuv420p"
        stream.options = {"crf":"20", "preset":"veryfast"}
        for k in range(frame_count):
            now = min(end, start+k/FPS)
            df = max(0, bisect.bisect_right(dtimes, now+1.e-6)-1)
            fi = max(0, bisect.bisect_right(ftimes, now+1.e-6)-1)
            d = decisions[df]
            number = int(frames[fi][1])
            canvas = Image.new("RGB", (1280, 816), BG)
            draw = ImageDraw.Draw(canvas)
            text(draw, (24, 15), "AI FORMULA   /   UNITY", 15, LIME, True, True)
            text(draw, (22, 41), "横移路障前后的控制指令", 30, INK, True)
            text(draw, (921, 26), f"种子 {run['request']['seed']}   ·   实速 1×", 18, MUTE)
            text(draw, (1119, 45), f"{now:6.2f} s", 23, INK, True, True)
            text(draw, (24, 98), "Unity 追车机位", 17, INK, True)
            text(draw, (710, 98), "控制器最近一次前向观测", 17, INK, True)
            with Image.open(images/f"{number:04d}c.jpg") as im:
                canvas.paste(im.resize((660, 371), Image.Resampling.BILINEAR), (24, 127))
            detector = camera_dir/f"{df:05d}.jpg"
            if a4_capture and detector.is_file():
                with Image.open(detector) as im:
                    front = im.convert("RGB")
                camera_marks(front, d)
                stamp = dtimes[df]
            else:
                # Use the nearest genuine RGB frame to the measurement and
                # disclose the timestamps if a perfectly matched frame is absent.
                near = min((max(0, min(len(frames)-1, i)) for i in
                            (bisect.bisect_left(ftimes, dtimes[df])-1, bisect.bisect_left(ftimes, dtimes[df]))),
                           key=lambda i: abs(ftimes[i]-dtimes[df]))
                with Image.open(images/f"{int(frames[near][1]):04d}f.jpg") as im:
                    front = im.convert("RGB")
                stamp = ftimes[near]
                if abs(stamp-dtimes[df]) < .006:
                    camera_marks(front, d)
            canvas.paste(front.resize((546,307),Image.Resampling.BILINEAR),(710,127))
            text(draw,(712,447),f"画面 {stamp:.2f} s    决策 {d['t']:.2f} s    跟踪 {len(d['tracks'])} 个目标",16,MUTE)
            conflict = d["conflictRange"]
            obstacle_range = f"{conflict:.2f} m" if conflict < 12 else "无"
            text(draw,(712,477),f"前方冲突距离 {obstacle_range}     横移目标 {d['offset']:+.2f} m",17,INK)
            draw.rectangle((24,521,1256,600), fill=PANEL)
            text(draw,(39,534),phase(d),21,ORANGE if d["reason"]=="yield" else INK,True)
            text(draw,(39,571),f"弯形目标 {d['cruise']:.2f}   风险上限 {d['guardCap']:.2f} m/s   候选比例 {d['ratio']:.2f}",15,MUTE)
            text(draw,(693,533),"偏航指令 / 实测（右转为正）",15,MUTE)
            text(draw,(693,564),f"{d['yawCmd']:+.2f} / {d['yaw']:+.2f} rad/s",21,INK,True,True)
            text(draw,(24,622),"目标与实测车速",19,INK,True)
            text(draw,(219,622),f"{d['speedCmd']:.2f} / {d['speed']:.2f} m/s",20,LIME,True,True)
            text(draw,(662,622),"左右轮 CAN 目标",19,INK,True)
            text(draw,(879,622),f"{d['rightCan']} / {d['leftCan']} RPM",20,BLUE,True,True)
            plot(draw,(58,681,608,762),dtimes,speed,start,end,now,(0,v_max),("目标 m/s","实测 m/s"))
            plot(draw,(701,681,1237,762),dtimes,rpm,start,end,now,(rpm_min,rpm_max),("右轮 RPM","左轮 RPM"))
            text(draw,(25,796),"时间轴为 Unity 仿真秒。车身速度与偏航来自运动反馈，右图 RPM 为发送给仿真电机的目标。",13,MUTE)
            frame = av.VideoFrame.from_ndarray(np.asarray(canvas), format="rgb24")
            for packet in stream.encode(frame):
                box.mux(packet)
        for packet in stream.encode():
            box.mux(packet)
    tmp.replace(out)
    print(f"{run_id}: {start:.2f}–{end:.2f} s, {frame_count} frames, {out}")


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--root",type=Path,required=True)
    parser.add_argument("--run",required=True)
    parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args()
    encode(args.root,args.run,args.out)
