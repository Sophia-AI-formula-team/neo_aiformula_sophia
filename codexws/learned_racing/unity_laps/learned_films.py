"""Render only current, recorded learned-encounter runs and verify the video before export."""
import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

from assemble_obstacle_films import inspect, join
from verify_three_view import verify


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path,record):
    path.write_text(json.dumps(record,ensure_ascii=False,indent=2)+"\n",encoding="utf8")


def slug(name):
    if not name or not all(c in "abcdefghijklmnopqrstuvwxyz0123456789-_" for c in name):
        raise ValueError("Use a lowercase ASCII asset name")
    return name


def paths(package):
    videos=package/"videos"
    evidence=package/"evidence"
    videos.mkdir(parents=True,exist_ok=True)
    evidence.mkdir(parents=True,exist_ok=True)
    return videos,evidence


def export(args):
    name=slug(args.name)
    if Path(args.trial).name!=args.trial:raise ValueError("Expected a run ID")
    run_path=args.capture_root/"runs"/(args.trial+".json")
    run=json.loads(run_path.read_bytes())
    if run.get("id")!=args.trial or not any(isinstance(d.get("encounter"),dict) for d in run["decisions"]):
        raise ValueError("This export requires decision evidence from the current learned-encounter experiment")
    raw=args.capture_root/".work"
    chase=raw/"frames"/args.trial
    a4=raw/"camera"/args.trial/"a4_samples"
    ready=json.loads((a4.parent/"ready.json").read_bytes())
    if ready.get("release")!="a4" or ready.get("sampleStride")!=1 or run["request"]["model"].find(ready["sha256"])<0:
        raise ValueError("A4 sources do not match the driving request")
    begins=[z for z in run.get("zoneStart",[]) if z>=0]
    ends=[z for z in run.get("zoneEnd",[]) if z>=0]
    start=args.start if args.start is not None else max(0,round((min(begins) if begins else run["seconds"]-7)-2.0,2))
    end=args.end if args.end is not None else min(run["seconds"],round((max(ends) if ends else run["seconds"])+1.5,2))
    if args.end is None and run["status"] in ("obstacle","boundary"):
        end=run["seconds"]
    videos,evidence=paths(args.package)
    pending=args.package/".work"/name
    pending.mkdir(parents=True,exist_ok=True)
    temp=pending/(name+".mp4")
    script=Path(__file__).with_name("three_view.py")
    command=[sys.executable,"-B",str(script),"--trajectory",str(run_path),"--frames",str(chase),"--a4",str(a4),
             "--start",str(start),"--end",str(end),"--title",args.title,"--referee-inset","--learned-panel","--out",str(temp)]
    subprocess.run(command,check=True)
    receipt_path=temp.with_suffix(".json")
    receipt=json.loads(receipt_path.read_bytes())
    if any(receipt["missing_camera_output_frames"].values()) or not receipt["referee_inset"]:
        raise ValueError("This clip requires both complete optical feeds and a local referee view")
    audit=verify(receipt_path,chase,a4)
    poster=pending/(name+".jpg")
    if args.poster_at is not None:
        poster_t=args.poster_at
    else:
        # Take a decoded frame while the judge and both optical cameras see the encounter.
        first=min(begins) if begins else start
        last=min(ends) if ends else end
        poster_t=max(start,min((first+last)*.5,end-.1))
    poster_offset=max(0,min(poster_t-start,receipt["encoded"]["duration_s"]-1/receipt["encoded"]["fps"]))
    fps,size,_,frames=inspect(temp,poster,poster_offset)
    if len(frames)!=receipt["encoded"]["frames"]:raise ValueError("Decoded timeline length differs")
    record={"name":name,"title":args.title,"sourceRun":args.trial,"status":run["status"],
            "rangeS":[start,end],"sourceSHA256":sha(run_path),"weightsSHA256":ready["sha256"],
            "solver":run["request"].get("solver"),"videoSHA256":sha(temp),"bytes":temp.stat().st_size,
            "posterSourceTimeS":min(end,start+round(poster_offset*fps)/float(fps)),"minRoadM":run["minRoad"],"minConeM":run["minGap"],
            "fps":int(fps),"frameSize":list(size),"frames":len(frames),"durationS":float(len(frames)/fps)}
    write(evidence/(name+".verified.json"),audit)
    shutil.copyfile(receipt_path,evidence/(name+".source.json"))
    shutil.copyfile(poster,evidence/(name+".jpg"))
    target=videos/(name+".mp4")
    temp.replace(target)
    write(evidence/(name+".json"),record)
    print(json.dumps(record,ensure_ascii=False))


def combine(args):
    videos,evidence=paths(args.package)
    records=[json.loads((evidence/(slug(n)+".json")).read_bytes()) for n in args.names]
    movies=[videos/(r["name"]+".mp4") for r in records]
    expected=[];counts=[];signature=None
    for movie,r in zip(movies,records):
        if sha(movie)!=r["videoSHA256"]:raise ValueError(f"Video hash changed: {movie}")
        fps,size,codec,marks=inspect(movie)
        if signature and signature!=(fps,size,codec):raise ValueError("Incompatible movie settings")
        signature=(fps,size,codec);counts.append(len(marks));expected+=marks
    reel=videos/(slug(args.name)+".mp4")
    if reel in movies:raise ValueError("Reel cannot replace a chapter")
    join(movies,signature[0],counts,[r["title"] for r in records],reel)
    _,_,_,actual=inspect(reel)
    if expected!=actual:raise ValueError("Joined movie changed decoded content or order")
    running=0;chapters=[]
    for r,count in zip(records,counts):
        chapters.append({"name":r["name"],"title":r["title"],"videoSHA256":r["videoSHA256"],
                         "startS":float(running/signature[0]),"endS":float((running+count)/signature[0])})
        running+=count
    report={"video":reel.name,"sha256":sha(reel),"frames":running,"fps":int(signature[0]),
            "durationS":float(running/signature[0]),"decodedFrameFingerprintsVerified":len(actual),"chapters":chapters}
    write(evidence/"collection.json",report)
    print(json.dumps(report,ensure_ascii=False))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest="task",required=True)
    clip=sub.add_parser("clip")
    clip.add_argument("--capture-root",type=Path,required=True)
    clip.add_argument("--package",type=Path,required=True)
    for key in ("trial","name","title"):clip.add_argument("--"+key,required=True)
    for key in ("start","end","poster-at"):clip.add_argument("--"+key,type=float)
    reel=sub.add_parser("join")
    reel.add_argument("--package",type=Path,required=True)
    reel.add_argument("--name",default="00-all-scenes")
    reel.add_argument("names",nargs="+")
    args=p.parse_args()
    (export if args.task=="clip" else combine)(args)


if __name__=="__main__":
    main()
