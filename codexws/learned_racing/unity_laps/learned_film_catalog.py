"""Audit published labels against the simulated physical props and rebuild preview/asset manifests."""
import argparse
import hashlib
import json
from pathlib import Path

import av
from PIL import Image,ImageDraw

from three_view import font


def catalogue(package,runs,prefix):
    evidence=package/"evidence";videos=package/"videos"
    collection=json.loads((evidence/"collection.json").read_bytes())
    assets=[];physical=[]
    for path in sorted(videos.glob("*.mp4")):
        meta=evidence/(path.stem+".json")
        info=json.loads(meta.read_bytes()) if meta.exists() else {}
        with av.open(str(path)) as media:
            s=media.streams.video[0]
            row={"file":path.name,"sizeBytes":path.stat().st_size,"sha256":hashlib.sha256(path.read_bytes()).hexdigest(),
                 "frameCount":s.frames,"durationS":float(s.duration*s.time_base),"fps":float(s.average_rate),"dimensions":[s.width,s.height]}
        if info.get("videoSHA256") and row["sha256"]!=info["videoSHA256"]:raise ValueError("Video checksum changed "+path.name)
        if path.name==collection["video"] and row["sha256"]!=collection["sha256"]:raise ValueError("Collection checksum changed")
        row.update({k:info[k] for k in ("sourceRun","rangeS","runs","status","modelId") if k in info})
        assets.append(row)
        ids=[info["sourceRun"]] if info.get("sourceRun") else info.get("runs",[])
        for trial in ids:
            source=runs/(trial+".json");record=json.loads(source.read_bytes())
            props=[{"index":i,"amplitudeM":v[4],"omegaRadS":v[6],"stationFromStartM":v[7],
                    "moving":bool(v[4] and v[6])} for i,v in enumerate(record["props"])]
            moving=sum(p["moving"] for p in props)
            physical.append({"file":path.name,"title":info.get("title"),"run":trial,"sourceSHA256":hashlib.sha256(source.read_bytes()).hexdigest(),
                             "static":len(props)-moving,"moving":moving,"physicalProps":props})
    cases=collection["chapters"];height=360*((len(cases)+1)//2)
    sheet=Image.new("RGB",(1200,height),"#ecf1ef");d=ImageDraw.Draw(sheet)
    for i,c in enumerate(cases):
        block=next(r for r in physical if r["file"]==c["name"]+".mp4")
        entry=next(r for r in assets if r["file"]==block["file"])
        if entry["sha256"]!=c["videoSHA256"]:raise ValueError("Chapter checksum changed: "+entry["file"])
        if block["title"]!=c["title"]:raise ValueError("Chapter title mismatch: "+entry["file"])
        with Image.open(evidence/(c["name"]+".jpg")) as im:
            camera=im.crop((42,173,1002,713)).resize((576,324))
        x=(i%2)*600+12;y=(i//2)*360+6
        sheet.paste(camera,(x,y+33));d.text((x,y),c["title"],font=font(21,True),fill="#163a43")
    sheet.save(evidence/(prefix+"-contact-sheet.jpg"),quality=92)
    def save(suffix,data):
        (evidence/(prefix+suffix)).write_text(json.dumps(data,ensure_ascii=False,indent=2)+"\n",encoding="utf8")
    save("-physical-props-audit.json",{"source":"Unity run.props; physical count derived from prop amplitude and angular frequency","films":physical})
    save("-asset-manifest.json",{"assets":assets})
    print(f"{len(assets)} clips/reels, {len(physical)} source runs, checked {len(cases)} chapters")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for k in ("package","runs"):p.add_argument("--"+k,type=Path,required=True)
    p.add_argument("--prefix",required=True)
    a=p.parse_args();catalogue(a.package,a.runs,a.prefix)
