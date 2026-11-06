"""Make the two-view replay from Unity's real orthographic camera frames."""
import argparse
import base64
from io import BytesIO
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
CROP = (18, 77, 704, 326)
SCALE = 416 / 7.8  # 2 * camera.orthographicSize, converted to source pixels


def project(station, right):
    return [round(360+(station-5.6)*SCALE-CROP[0], 2), round(208+right*SCALE-CROP[1], 2)]


def make_clips(experiment, repeat, frames):
    output = {}
    for layout, modes in experiment["evaluations"].items():
        output[layout] = {}
        for mode, case in modes.items():
            run = case["run"]
            decision = case["decision"]
            trace = [[round(t, 3), *project(s, x)] for t,s,x,*rest in run["rows"]]
            images = []
            for time, index in repeat[layout][mode]["frames"]:
                p = frames/layout/mode/f"{int(index):04d}.png"
                with Image.open(p) as source:
                    if source.size != (720,416):
                        raise ValueError(f"Unexpected Unity camera size in {p}")
                    frame = source.convert("RGB").crop(CROP)
                stream = BytesIO()
                frame.save(stream, format="WEBP", quality=62, method=4)
                images.append([round(time, 3), "data:image/webp;base64,"+base64.b64encode(stream.getvalue()).decode("ascii")])
            candidates = []
            for choice in decision["sides"]:
                pts = [project(s, row[0]) for s,row in zip(experiment["station_m"], choice["prediction"])]
                candidates.append(pts[::4])
            output[layout][mode] = {"images":images, "trace":trace, "candidates":candidates,
                "end":round(run["rows"][-1][0],3), "side":"向右" if decision["chosen"]["first_side"]>0 else "向左",
                "result":"通过 11 m" if run["status"]=="finished" else "在第二只桶处触及判定外廓"}
    return output


def rendered_gif(experiment, repeat, frames, target, font_path):
    font = ImageFont.truetype(str(font_path), 19)
    small = ImageFont.truetype(str(font_path), 16)
    layouts = list(experiment["evaluations"])
    end = max(experiment["evaluations"][k]["both"]["run"]["rows"][-1][0] for k in layouts)
    timeline = np.unique(np.round(np.r_[np.arange(0,end,.2),end], 3))
    output = []
    for t in timeline:
        page = Image.new("RGB", (710,640), "#102126")
        ink = ImageDraw.Draw(page)
        ink.text((13,8),"Unity 赛道仿真  /  连续两桶",font=font,fill="#f6f4eb")
        ink.text((613,12),f"{t:4.2f} s",font=small,fill="#ccded8")
        for i, layout in enumerate(layouts):
            result = experiment["evaluations"][layout]["both"]
            run, chosen = result["run"], result["decision"]["chosen"]
            pick = max((item for item in repeat[layout]["both"]["frames"] if item[0]<=t+.0001), key=lambda a:a[0])
            p = frames/layout/"both"/f"{int(pick[1]):04d}.png"
            with Image.open(p) as src:
                im = src.convert("RGB").crop(CROP)
            layer = ImageDraw.Draw(im)
            # Leave the photo car visible. Its origin is near the front axle.
            rows = [row for row in run["rows"] if row[0]<=pick[0]+1e-5]
            tail = [tuple(project(row[1],row[2])) for row in rows if rows[-1][1]-row[1]>.85]
            if len(tail)>1:
                layer.line(tail, fill="#133939", width=5)
                layer.line(tail, fill="#7ff5d3", width=3)
            top=41+286*i
            move="右绕" if chosen["first_side"]>0 else "左绕"
            second="左" if layout=="second_left" else "右"
            ink.text((13,top),f"第二只桶在{second}   ·   车辆{move}",font=font,fill="#ffffff")
            ink.text((475,top+2),f"最小避桶余量  {run['min_gap']*100:.1f} cm",font=small,fill="#b4d3c9")
            page.paste(im,(12,top+29))
        ink.text((13,613),"同一起点、同一时钟  ·  绿线为实际车身轨迹  ·  向右行驶",font=small,fill="#e0eae4")
        output.append(page)
    durations=[max(20,round((b-a)*1000)) for a,b in zip(timeline,timeline[1:])]+[1800]
    output[0].save(target, save_all=True, append_images=output[1:], duration=durations,loop=0,
                   optimize=True,disposal=1)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--frames",type=Path,required=True)
    p.add_argument("--fragment",type=Path,required=True)
    p.add_argument("--results",type=Path,default=HERE/"results")
    p.add_argument("--font",type=Path,default=Path("C:/Windows/Fonts/msyh.ttc"))
    args=p.parse_args()
    data=json.loads((args.results/"experiment.json").read_bytes())
    repeat=json.loads((args.results/"repeat.json").read_bytes())
    clips=make_clips(data,repeat,args.frames)
    template=(HERE/"replay.template.html").read_text(encoding="utf8")
    fragment=template.replace("__CLIPS__",json.dumps(clips,ensure_ascii=False,separators=(",",":")))
    if len(fragment.encode("utf8"))>=1_000_000:
        raise ValueError("Camera recording exceeds inline limit")
    args.fragment.parent.mkdir(parents=True,exist_ok=True)
    args.fragment.write_text(fragment,encoding="utf8")
    rendered_gif(data,repeat,args.frames,args.results/"unity-cones.gif",args.font)


if __name__=="__main__":
    main()
