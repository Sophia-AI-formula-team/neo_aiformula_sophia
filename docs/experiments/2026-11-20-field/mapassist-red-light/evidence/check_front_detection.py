"""Check actual model proposals and color classification on three front-lit board frames."""
from pathlib import Path
import cv2
import json
import sys
import torch

root=Path(__file__).resolve().parent
profile=sys.argv[1];bundle=root.parent/'replay'/profile
sys.path.insert(0,str(bundle))
from yolop_a3_runtime import PerceptionModel
from yolop_a3_runtime.color_semantics import classify_traffic_light

source=root/'originals'/'rosbag2_2026_11_20-09_09_31'
regions={1795133423669267000:(521,166,539,182),1795133424069407000:(555,166,575,185),1795133424402655000:(584,162,607,184)}
torch.set_num_threads(4)
m=PerceptionModel(bundle/'weights/shared_best.pth',device='cpu',score_threshold=.01)
underlying=getattr(m.model,'base',m.model)

def overlap(a,b):
    if min(a[2],b[2])<=max(a[0],b[0]) or min(a[3],b[3])<=max(a[1],b[1]): return 0
    return (min(a[2],b[2])-max(a[0],b[0]))*(min(a[3],b[3])-max(a[1],b[1]))/((a[2]-a[0])*(a[3]-a[1]))

records=[]
for entry in json.loads((source/'index.json').read_text()):
    ns=entry['header_ns']
    if ns not in regions:continue
    roi=regions[ns];img=cv2.imread(str(source/entry['file']));source_bgr=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    input=torch.from_numpy(source_bgr.transpose(2,0,1).copy()).float()/255
    with torch.inference_mode():
        _,raw=m.model([input])[0:2]
    raw=raw[0];boxes=[]
    for b,c,s in zip(raw['boxes'].tolist(),raw['labels'].tolist(),raw['scores'].tolist()):
      if c==10:
        boxes.append({'box':b,'score':s,'board_coverage':overlap(roi,b)})
    reading=classify_traffic_light(img,roi)
    records.append(dict(header_ns=ns,roi=roi,manual_roi_light=reading.label,manual_roi_red_pixels=reading.pixel_count,traffic_light_candidates=boxes,model_score_threshold=getattr(underlying,'score_thresh',None)))
(root/(profile.lower()+'-front-light-diagnosis.json')).write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n')
print(profile,[(r['header_ns'],r['manual_roi_light'],r['manual_roi_red_pixels'],[(round(x['score'],2),round(x['board_coverage'],2)) for x in r['traffic_light_candidates']]) for r in records])
