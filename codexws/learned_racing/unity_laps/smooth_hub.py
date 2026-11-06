"""Export the verified training, lap, and gate films into a single offline Chinese player."""
import argparse
from hashlib import sha256
from html import escape
import json
import os
from pathlib import Path
import shutil
import zipfile

from three_view import digest

HERE=Path(__file__).parent


def read(path):return json.loads(path.read_bytes())


def save(path,data):
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+"\n",encoding="utf8")


def deliver(src,dst):
    dst.parent.mkdir(exist_ok=True,parents=True)
    if dst.exists():
        if digest(src)!=digest(dst):raise ValueError(f"Existing delivery differs: {dst}")
        return
    try:os.link(src,dst)
    except OSError:shutil.copy2(src,dst)


def main(args):
    target=args.out;target.mkdir(parents=True,exist_ok=True)
    groups=[];allitems=[];s=read(args.smooth/"evidence/collection.json");g=read(args.gates/"evidence/collection.json")
    sscore=read(args.smooth_score);gscore=read(args.gate_score)
    def item(folder,key,name,title,detail,sha,poster,receipt,stage):
        movie=folder/"videos"/(name+".mp4")
        if digest(movie)!=sha:raise ValueError(f"Video differs from certified bytes: {movie}")
        stem=key+"/"+name
        for source,dest in ((movie,"media/"+stem+".mp4"),(poster,"posters/"+stem+".jpg"),(receipt,"evidence/"+stem+".json")):
            deliver(source,target/dest)
        meta=dict(title=title,note=detail,group=stage,path="media/"+stem+".mp4",poster="posters/"+stem+".jpg",receipt="evidence/"+stem+".json",sha256=sha)
        allitems.append(meta);return meta
    for c in s["chapters"]:
        n=c["name"];ev=args.smooth/"evidence"
        receipt=ev/(n+".json") if (ev/(n+".json")).exists() else args.smooth/"videos"/(n+".json")
        phase="训练与开发复测" if n.startswith("01") else "冻结策略独立考核"
        item(args.smooth,"smooth",n,c["title"],c["selection"],c["sha256"],ev/"posters"/(n+".jpg"),receipt,phase)
    ev=args.gates/"evidence"
    for c in g["chapters"]:
        n=Path(c["file"]).stem;receipt=ev/(n+".json");d=read(receipt)
        row=next(p for p in gscore["pairs"] if p["id"]==c["id"])
        branch=("冻结、仅提速和寻缝的中穿排数分别为 "
                + " / ".join(str(row[a]["middleGates"]) for a in ("base","pace","learned"))
                +f"（共 {len(row['learned']['gates'])} 排）。")
        item(args.gates,"gates",n,d["title"],branch+c["reason"],c["sha256"],ev/(n+".jpg"),receipt,"三臂中缝对照")
    n="04-comparable-time";d=read(ev/(n+".json"))
    w=d["comparedWindows"];desc="同一距离窗口，冻结、仅提速和寻缝分别用 "+" / ".join(f"{w[a]['seconds']:.2f}" for a in ("base","pace","learned"))+" 秒，三者都完整中穿。"
    item(args.gates,"gates",n,d["title"],desc,d["sha256"],ev/(n+".jpg"),ev/(n+".json"),"三臂中缝对照")
    combo=[]
    for key,src,meta,n,title in (("smooth",args.smooth,s,"00-all-evidence","长训与闭环考核合辑"),("gates",args.gates,g,"00-gate-scenes","中缝成功与失败三章合辑")):
        source=src/"videos"/(n+".mp4");rel=f"media/{key}/{n}.mp4"
        if digest(source)!=meta["sha256"]:raise ValueError("Chapter reel differs: "+rel)
        deliver(source,target/rel);deliver(src/"evidence/collection.json",target/f"evidence/{key}/collection.json")
        combo.append(dict(title=title,path=rel,sha256=meta["sha256"]))
    for src,name in ((args.smooth_score,"smooth-score.json"),(args.gate_score,"gate-score.json"),
                     (args.smooth/"evidence/selection.json","film-selection.json")):
        deliver(src,target/"evidence"/name)
    for label in ("训练与开发复测","冻结策略独立考核","三臂中缝对照"):
        groups.append(dict(name=label,items=[i for i,q in enumerate(allitems) if q["group"]==label]))
    data=dict(items=allitems,groups=groups,reels=combo,rawScoreHashes={"smooth":digest(args.smooth_score),"gates":digest(args.gate_score)})
    save(target/"films.json",data)
    cards=[]
    for group in groups:
        rows=[]
        for k in group["items"]:
            it=allitems[k]
            rows.append(f'<article><button type="button" data-id="{k}"><img loading="lazy" src="{it["poster"]}" alt=""><b>{escape(it["title"])}</b></button><p>{escape(it["note"])}</p><footer><a href="{it["path"]}">独立 MP4</a><a href="{it["receipt"]}">来源与校验</a></footer></article>')
        cards.append(f'<section><h2>{escape(group["name"])}</h2><div class="grid">{"".join(rows)}</div></section>')
    markup=(HERE/"smooth_hub.html").read_text(encoding="utf-8-sig")
    markup=markup.replace("{{GROUPS}}","".join(cards)).replace("{{DATA}}",json.dumps(data,ensure_ascii=False).replace("<","\\u003c"))
    markup=markup.replace("{{POSTER}}",allitems[0]["poster"]).replace("{{SOURCE}}",allitems[0]["path"])
    markup=markup.replace("{{REELS}}","".join(f'<a href="{c["path"]}">{escape(c["title"])}</a>' for c in combo))
    local=sscore["cohorts"]["segment"];laps=sscore["cohorts"]["lap"]
    caption=f"盲测局部完段 {local['success'][0]}/{local['pairs']} → {local['success'][1]}/{local['pairs']}，整圈 {laps['success'][0]}/{laps['pairs']} → {laps['success'][1]}/{laps['pairs']}。中缝另用 {len(gscore['pairs'])} 个独立布局比较三种控制。"
    (target/"index.html").write_text(markup.replace("{{CAPTION}}",caption),encoding="utf8")
    files=sorted(p for p in target.rglob("*") if p.is_file() and p.name!="SHA256SUMS.txt")
    sums=target/"SHA256SUMS.txt";sums.write_text("".join(f"{digest(p)}  {p.relative_to(target).as_posix()}\n" for p in files),encoding="utf8")
    if args.zip:
        with zipfile.ZipFile(args.zip,"w",compression=zipfile.ZIP_STORED) as archive:
            for path in [*files,sums]:archive.write(path,path.relative_to(target).as_posix())
        with zipfile.ZipFile(args.zip) as archive:
            if archive.testzip() is not None:raise ValueError("Offline ZIP failed its CRC check")
        print("ZIP",args.zip,digest(args.zip))
    print("SUMS",sums,digest(sums),"videos",len(allitems)+len(combo))


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for k in ("smooth","gates","out","smooth-score","gate-score"):p.add_argument("--"+k,type=Path,required=True)
    p.add_argument("--zip",type=Path);main(p.parse_args())
