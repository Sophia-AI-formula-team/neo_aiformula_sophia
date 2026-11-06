"""Render a synchronized, real-time Unity chase, optical A4 and local decision film."""
import argparse
import bisect
import hashlib
import json
import math
from functools import lru_cache
from pathlib import Path

import av
from PIL import Image, ImageDraw, ImageFont

from referee_inset import CHASE_INSET, RefereeInset, RefereeTimeline
from learned_film_panel import HEIGHT as LEARNED_HEIGHT, LearnedPanel


SIZE = (2048, 1280)
FPS = 20
INK = "#122c36"
MUTED = "#48656e"
PAPER = "#eef4f3"
WHITE = "#ffffff"
GRID = "#d2deda"
TEAL = "#087d80"
LIME = "#e1fa79"
PURPLE = "#6447aa"
ORANGE = "#aa4115"
AHEAD = (2, 3.5, 5.5, 8)
PHASE = {
    "road": ("沿道路选线", "按道路弯曲程度调整目标速度", TEAL),
    "yield": ("减速避让", "预测锥桶位置，调整速度和横向目标", ORANGE),
    "recover": ("通过后恢复", "继续控制横移，逐步放开速度限制", PURPLE),
    "hold_pass": ("保持通过侧", "短时保持已选一侧", ORANGE),
    "yield_corridor": ("选择通过空间", "按障碍与道路之间的余量调整目标", ORANGE),
    "insufficient_lane": ("道路观测不足", "控制器限制目标速度", ORANGE),
    "learned_encounter": ("执行学习动作", "按所选动作限制车速并跟踪横移目标", PURPLE),
    "go_first": ("选择加速先行", "根据车身清空断面的预测时间调整车速", TEAL),
}


@lru_cache(maxsize=60)
def font(size, bold=False, mono=False):
    windows = Path("C:/Windows/Fonts")
    sources = ([windows / ("consolab.ttf" if bold else "consola.ttf"),
                Path("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf")] if mono else
               [windows / ("msyhbd.ttc" if bold else "msyh.ttc"),
                Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc" if bold
                     else "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")])
    for source in sources:
        if source.exists():
            return ImageFont.truetype(str(source), size)
    raise RuntimeError(f"Missing {'monospace' if mono else 'Chinese'} font")


def label(d, xy, message, size, color=INK, bold=False, mono=False, anchor="lm"):
    d.text(xy, message, font=font(size, bold, mono), fill=color, anchor=anchor)


def arrow(d, start, end, color, width=4, head=13):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 4:
        return
    ux, uy = dx / length, dy / length
    head = min(head, length * .52)
    base = (end[0] - head * ux, end[1] - head * uy)
    d.line((start, base), fill=color, width=width)
    d.polygon([end, (base[0] + uy * head * .5, base[1] - ux * head * .5),
               (base[0] - uy * head * .5, base[1] + ux * head * .5)], fill=color)


@lru_cache(maxsize=12)
def camera(path):
    if not path.is_file():
        return None
    with Image.open(path) as source:
        if source.size != (640, 360):
            raise ValueError(f"Unexpected source size {source.size}: {path}")
        return source.convert("RGB").resize((960, 540), Image.Resampling.BICUBIC)


def layout(title, note):
    canvas = Image.new("RGB", SIZE, PAPER)
    d = ImageDraw.Draw(canvas)
    d.rectangle((0, 0, 2048, 100), fill=INK)
    label(d, (44, 49), title, 47, WHITE, True)
    label(d, (2000, 73), "仿真时钟   /   原速 1×", 21, "#b9d9d6", anchor="rm")
    for x, heading, sub in [(42, "Unity 追车", "观察车辆与锥桶的实际位置"),
                            (1046, "车载相机  /  YOLOP A4", "原始检测框、车线掩膜与采样点")]:
        label(d, (x, 141), heading, 31, INK, True)
        label(d, (x + 960, 145), sub, 23, MUTED, anchor="rm")
        d.rectangle((x, 173, x + 959, 712), fill="#dce6e4")
    d.rounded_rectangle((42, 737, 1002, 1234), radius=18, fill=WHITE)
    d.rounded_rectangle((1046, 737, 2005, 1234), radius=18, fill=WHITE)
    label(d, (68, 774), "车身坐标下的观测与选点", 30, INK, True)
    label(d, (979, 776), "向上为车头方向 · 单位 m", 21, MUTED, anchor="rm")
    label(d, (1072, 774), "此刻的控制指令", 30, INK, True)
    label(d, (44, 1257), note or "画面与数据来自同次 Unity 闭环运行。原始框会有重叠或误检，左下只画控制器收到的道路观测与锥桶跟踪。", 21, MUTED)
    return canvas


# Draw the map in a separate image so off-screen observations are naturally clipped.
MAP = (660, 410)
SCALE = 31
ORIGIN = (330, 361)


def xy(right, ahead):
    return (round(ORIGIN[0] + right * SCALE), round(ORIGIN[1] - ahead * SCALE))


@lru_cache(maxsize=1)
def local_grid():
    m = Image.new("RGB", MAP, "#f1f6f4")
    d = ImageDraw.Draw(m)
    for x in range(-10, 11):
        a, b = xy(x, 12), xy(x, -2)
        d.line((a, b), fill="#a8bfba" if x == 0 else GRID, width=2 if x == 0 else 1)
    for z in range(-1, 13):
        a, b = xy(-12, z), xy(12, z)
        d.line((a, b), fill="#a8bfba" if z == 0 else GRID, width=2 if z == 0 else 1)
        if z in (0, 2, 4, 6, 8, 10):
            label(d, (6, xy(0, z)[1] - 2), f"{z} m", 16, MUTED, mono=True)
    label(d, (MAP[0] - 10, MAP[1] - 14), "右 →", 17, MUTED, anchor="rm")
    return m


def map_view(canvas, decision):
    m = local_grid().copy()
    d = ImageDraw.Draw(m)
    line = [(xy(value, z) if value is not None else None)
            for z, value in zip(AHEAD, decision.get("observed", []))]
    for a, b in zip(line, line[1:]):
        if a and b:
            d.line((a, b), fill=TEAL, width=4)
    for pt in filter(None, line):
        x, y = pt
        d.ellipse((x-6, y-6, x+6, y+6), fill=WHITE, outline=TEAL, width=4)
    if decision.get("look", 0) > 0 and decision.get("reason") != "insufficient_lane":
        x, y = xy(decision["target"], decision["look"])
        # Only a sightline from ego to the one selected point; no trajectory reconstruction.
        d.line((ORIGIN, (x, y)), fill="#ac9bc9", width=2)
        d.polygon([(x, y-12), (x+12, y), (x, y+12), (x-12, y)], fill=PURPLE, outline=WHITE, width=2)
    tracks = decision.get("tracks", [])
    for z, x, vx, vz in tracks:
        a, b = xy(x, z), xy(x+vx, z+vz)
        arrow(d, a, b, ORANGE, 5)
        px, py = a
        d.ellipse((px-8, py-8, px+8, py+8), fill="#ed7332", outline="#66320f", width=2)
    # Direction only. This is a half-second constant-yaw-command illustration,
    # not the planner's trajectory, a wheel angle, or measured future motion.
    angle = decision["yawCmd"] * .5
    arrow(d, ORIGIN, (ORIGIN[0]+39*math.sin(angle), ORIGIN[1]-39*math.cos(angle)), "#2e668c", 5)
    px, py = ORIGIN
    d.rounded_rectangle((px-12, py-9, px+12, py+27), radius=4, fill=INK)
    d.polygon([(px, py-7), (px-6, py+2), (px+6, py+2)], fill=LIME)
    canvas.paste(m, (66, 803))
    d = ImageDraw.Draw(canvas)
    for index, (color, string, symbol) in enumerate([(TEAL, "道路观测点", "circle"),
                                                   (PURPLE, "选定目标点", "diamond"),
                                                   (ORANGE, "跟踪中的锥桶", "dot"),
                                                   (ORANGE, "估计运动 1 秒", "arrow"),
                                                   (INK, "车辆与车头", "car"),
                                                   ("#2e668c", "0.5 s 偏航示意", "arrow")]):
        y = 835 + index*49
        x = 767
        if symbol == "circle":
            d.ellipse((x-8,y-8,x+8,y+8), outline=color, width=4)
        elif symbol == "diamond":
            d.polygon([(x,y-10),(x+10,y),(x,y+10),(x-10,y)], fill=color)
        elif symbol == "dot":
            d.ellipse((x-8,y-8,x+8,y+8), fill="#ed7332", outline="#66320f", width=2)
        elif symbol == "arrow":
            arrow(d, (x-12,y), (x+13,y), color, 4)
        else:
            d.rounded_rectangle((x-9,y-11,x+9,y+12), radius=3, fill=color)
            d.polygon([(x,y-9),(x-4,y-2),(x+4,y-2)],fill=LIME)
        label(d, (791,y), string, 22)
    label(d, (751, 1112), "当前跟踪", 22, MUTED)
    label(d, (751, 1160), f"{len(tracks)} 个目标", 29, ORANGE if tracks else MUTED, True)


def speed_chart(d, decisions, times, index):
    now = times[index]
    x0, x1, y0, y1 = 1084, 1978, 1058, 1147
    d.rectangle((x0, y0, x1, y1), fill="#f3f7f5")
    start = max(0, bisect.bisect_right(times, now - 5) - 1)
    history = decisions[start:index+1]
    maximum = max(5, math.ceil(max(max(r["speed"], r["speedCmd"]) for r in history)))
    for v in (0, maximum):
        yy = y1 - v/maximum * (y1 - y0)
        d.line((x0, yy, x1, yy), fill=GRID, width=1)
        label(d, (x0+5, yy), str(v), 15, MUTED, mono=True, anchor="lb" if v==0 else "lt")
    for key, color in [("speedCmd", PURPLE), ("speed", TEAL)]:
        pts = [(max(x0, x1 + (r["t"]-now)/5*(x1-x0)), min(y1, y1 - r[key]/maximum*(y1-y0))) for r in history]
        if len(pts)>1:
            d.line(pts, fill=color, width=4)
    label(d, (x0, y1+17), "过去 5 秒", 17, MUTED)
    label(d, (x1, y1+17), "当前", 17, MUTED, anchor="rm")


def controls(canvas, decisions, times, i):
    rec = decisions[i]
    d = ImageDraw.Draw(canvas)
    phase, desc, accent = PHASE.get(rec["reason"], (rec["reason"], "", MUTED))
    label(d, (1075, 836), phase, 43, accent, True)
    label(d, (1977, 839), desc, 23, MUTED, anchor="rm")
    d.line((1073, 884, 1978, 884), fill=GRID, width=2)
    offset = rec.get("offset", 0)
    yaw = rec["yawCmd"]
    side = "居中" if abs(offset)<0.02 else ("右" if offset>0 else "左")
    turn = "直行" if abs(yaw)<0.02 else ("右转" if yaw>0 else "左转")
    measures = [(1076, "实际车速", f'{rec["speed"]:.2f}', "m/s", TEAL),
                (1300, "目标车速", f'{rec["speedCmd"]:.2f}', "m/s", PURPLE),
                (1533, f"横移目标 · {side}", f"{abs(offset):.2f}", "m", INK),
                (1757, f"偏航指令 · {turn}", f"{abs(yaw):.2f}", "rad/s", INK)]
    for x, title, value, unit, color in measures:
        label(d, (x, 917), title, 22, MUTED)
        label(d, (x, 970), value, 49, color, True, True)
        width=d.textlength(value,font=font(49,True,True))
        label(d, (x+width+6, 978), unit, 20, color, mono=True)
    label(d, (1080,1033), "速度变化", 22, INK, True)
    label(d, (1228,1033), "实测", 20, TEAL, True)
    label(d, (1303,1033), "目标", 20, PURPLE, True)
    speed_chart(d, decisions, times, i)
    label(d, (1080,1204), "本周期轮速指令", 23, MUTED)
    label(d, (1293,1204), f'左  {rec["leftCan"]:+d} rpm     右  {rec["rightCan"]:+d} rpm', 26, INK, True)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def at(times, t):
    # Unity serializes float32 seconds, up to 15 us from the intended fixed tick.
    return bisect.bisect_right(times, t + .00002) - 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("trajectory", "frames", "a4", "out"):
        parser.add_argument("--"+name, type=Path, required=True)
    for name in ("start", "end"):
        parser.add_argument("--"+name, type=float, required=True)
    parser.add_argument("--title", required=True)
    parser.add_argument("--note", help="Optional factual footer for this recording")
    parser.add_argument("--referee-inset", action="store_true", help="Show a fixed local ground-truth referee inset on the chase camera")
    parser.add_argument("--learned-panel", action="store_true", help="Add actual fitted-action choices, scores and terminal outcome")
    parser.add_argument("--whole-lap", action="store_true", help="Show recorded progress and switch the fixed referee view only between encounters")
    args = parser.parse_args()
    run = json.loads(args.trajectory.read_text(encoding="utf-8-sig"))
    frames, decisions = run["frames"], run["decisions"]
    ready = args.a4.parent / "ready.json"
    meta = json.loads(ready.read_text(encoding="utf-8-sig")) if ready.is_file() else {}
    if meta.get("sampleStride", 1) != 1:
        raise ValueError("This renderer requires A4 sampleStride=1 for frame-to-decision association.")
    if meta.get("release", "a4") != "a4":
        raise ValueError("Overlay archive is not A4")
    if not (0 <= args.start < args.end <= run["seconds"]+.00002):
        raise ValueError(f"Window must be within the run (0 .. {run['seconds']} s)")
    if not frames or not decisions:
        raise ValueError("This run has no synchronized chase frames / decisions")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    ft = [a[0] for a in frames]
    dt = [r["t"] for r in decisions]
    motion_frames = math.ceil((args.end-args.start)*FPS-1e-7)
    if motion_frames < 1:
        raise ValueError("Empty output range")
    terminal = abs(args.end-run["seconds"])<.00002 and (run.get("status") in ("obstacle", "boundary")
                or (args.whole_lap and run.get("status")=="lap"))
    hold_frames = round(.8*FPS) if terminal else 0
    count = motion_frames+hold_frames
    background = layout(args.title, args.note)
    panel = LearnedPanel(run) if args.learned_panel else None
    size = (SIZE[0],SIZE[1]+LEARNED_HEIGHT) if panel else SIZE
    if panel:
        base=Image.new("RGB",size,PAPER)
        base.paste(background,(0,0))
        background=base
    referee = RefereeInset(run,args.start,args.end) if args.referee_inset else None
    timeline = RefereeTimeline(run) if args.whole_lap else None
    row_times=[r[0] for r in run["rows"]]
    indices = {"chase": set(), "a4": set()}
    gaps = {"chase": 0, "a4": 0}
    max_age = {"chase": 0.0, "a4": 0.0}
    checks = []
    check_frames={0,motion_frames//2,motion_frames-1,count-1}
    if panel:
        for i in panel.selected:
            if args.start-.00002<=dt[i]<args.end:
                check_frames.add(max(0,math.ceil((dt[i]-args.start-.00002)*FPS)))
    with av.open(str(args.out), "w", options={"movflags": "+faststart"}) as out:
        video = out.add_stream("libx264", rate=FPS, options={"preset":"veryfast","crf":"19","threads":"4"})
        video.width, video.height = size
        video.pix_fmt = "yuv420p"
        for n in range(count):
            stopped = terminal and n >= motion_frames
            t = run["seconds"] if stopped else args.start+n/FPS
            fi, di = at(ft, t), at(dt, t)
            if min(fi, di) < 0:
                raise ValueError("Requested interval precedes the first synchronized observation")
            cf, af = int(frames[fi][1]), di
            sources = {"chase": args.frames/f"{cf:04d}c.jpg", "a4": args.a4/f"{af:05d}.jpg"}
            canvas = background.copy()
            d = ImageDraw.Draw(canvas)
            label(d, (2000, 35), f"t = {t:06.2f} s", 36, WHITE, True, True, "rm")
            if args.whole_lap:
                progress=run["rows"][at(row_times,t)][7]
                label(d,(1510,48),f"沿程 {progress:.1f} / {run['length']:.1f} m",29,"#c4e4e0",True,anchor="rm")
            for key, x, ix, ts in (("chase",42,cf,ft[fi]),("a4",1046,af,dt[di])):
                source = camera(sources[key])
                if source is None:
                    gaps[key] += 1
                    label(d,(x+480,445),"该时刻没有已归档画面",32,MUTED,anchor="mm")
                else:
                    canvas.paste(source, (x,173))
                    indices[key].add(ix)
                    max_age[key] = max(max_age[key], t-ts)
            map_view(canvas,decisions[di])
            controls(canvas,decisions,dt,di)
            if panel:
                canvas.paste(panel.render(di,label),(0,SIZE[1]))
            active_ref=timeline.current(ft[fi]) if timeline else referee
            if active_ref and active_ref.ids:
                canvas.paste(active_ref.render(ft[fi],label),(42+CHASE_INSET[0],173+CHASE_INSET[1]))
            if stopped:
                finish=run["status"]=="lap"
                message = f"通过终点  ·  {run['gate']} / 32 道计时门" if finish else (
                    "发生接触  ·  本次试验终止" if run["status"]=="obstacle" else "车辆触及边界  ·  本次试验终止")
                d.rectangle((42, 636, 1001, 712),fill="#195c48" if finish else "#7b2b13")
                label(d,(60,673),message,33,WHITE,True)
                badge=f"停帧 · 追车 {ft[fi]:.2f} s" if args.whole_lap else "终止停帧"
                label(d,(981,674),badge,22,WHITE,anchor="rm")
                d.rectangle((1560,57,2004,90),fill=INK)
                label(d,(2000,73),f"末次感知与控制  {dt[di]:.2f} s",21,"#b9d9d6",anchor="rm")
                # Map and control sections continue to show the last issued action.
            if n in check_frames:
                checks.append({"film_frame":n, "simulation_s":t, "refereeActive":bool(active_ref and active_ref.ids),
                               "chase":{"id":cf,"time_s":ft[fi],"sha256":digest(sources["chase"]) if sources["chase"].is_file() else None},
                               "a4":{"id":di,"time_s":dt[di],"sha256":digest(sources["a4"]) if sources["a4"].is_file() else None},
                               "track_count":len(decisions[di]["tracks"]), "phase":decisions[di]["reason"]})
            for packet in video.encode(av.VideoFrame.from_image(canvas)):
                out.mux(packet)
        for packet in video.encode():
            out.mux(packet)
    with av.open(str(args.out)) as playback:
        stream = playback.streams.video[0]
        encoded = {"codec":stream.codec_context.name,"width":stream.width,"height":stream.height,
                   "fps":float(stream.average_rate),"frames":stream.frames,"duration_s":float(stream.duration*stream.time_base)}
        if stream.frames != count or (stream.width,stream.height) != size or abs(encoded["duration_s"]-count/FPS)>1e-5:
            raise ValueError(f"Encoded stream did not match the requested timing: {encoded}")
    receipt = {"title":args.title,"run":run.get("id"),"seed":run.get("request",{}).get("seed"),
               "trajectory_sha256":digest(args.trajectory),"model":run.get("request",{}).get("model"),"a4_worker":meta,
               "start_s":args.start,"end_s":args.end,"end_included":terminal,
               "labelled_terminal_hold_s":hold_frames/FPS,"status":run.get("status"),"encoded":encoded,
               "referee_inset":timeline.receipt() if timeline else referee.receipt() if referee and referee.ids else None,
               "learned_panel":None if panel is None else {"modelId":panel.model, "actionTable":panel.actions,
                   "selections":[{"t":decisions[i]["t"],**decisions[i]["encounter"]} for i in panel.selected]},
               "method":"Latest available capture and decision at each simulation tick. A4 JPEG uses the same optical inference index as the decision. Orange arrow is one second of tracked velocity. The blue arrow illustrates commanded yaw held for half a second; arrow length has no distance meaning. Terminal hold uses the actual final chase capture and last available decision.",
               "unique_sources":{key:len(value) for key,value in indices.items()},"max_display_age_s":max_age,
               "missing_camera_output_frames":gaps,"samples":checks}
    args.out.with_suffix(".json").write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+"\n",encoding="utf8")
    print(json.dumps({"film":str(args.out),"encoded":encoded,"missing":gaps},ensure_ascii=False),flush=True)


if __name__=="__main__":
    main()
