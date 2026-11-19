from pathlib import Path
import sqlite3,json,struct,collections
import numpy as np
from concept_visualization.node import draw_path,draw_candidates
from concept_visualization.geometry import pixel
import cv2
p=next(Path.home().glob('rosbags/concept/*134918*/*.db3'))
d=sqlite3.connect(p.as_uri()+'?mode=ro&immutable=1',uri=True)
rows=d.execute("select data from messages join topics on topics.id=topic_id where name='/concept/decision' order by timestamp")
counts=collections.Counter(); lengths=[]; region=[]; endpoints=[]; prev=None; changing=collections.Counter()
for (raw,) in rows:
  m=json.loads(raw[8:7+struct.unpack('<I',raw[4:8])[0]])
  can=np.zeros((360,640,3),np.uint8); selected=np.zeros_like(can)
  draw_candidates(can,m); draw_path(selected,m['selected_path'],(170,72,250),4)
  yy,xx=np.nonzero(np.any(selected,axis=2))
  pink=bool(len(yy)); gray=bool(np.any(can))
  counts[str((bool(m['selected_path']),m['selected_index'] is not None,pink,gray))]+=1
  if pink:
    lengths.append(len(yy)); region.append([int(xx.min()),int(yy.min()),int(xx.max()),int(yy.max())])
  if m['selected_index'] is not None and m['selected_path']:
    endpoints.append((m['selected_path'][-1][0],m['selected_path'][-1][1]))
  if prev is not None and prev['selected_index'] is not None and m['selected_index'] is not None:
    changing['consecutive_selected']+=1
    if prev['selected_index']!=m['selected_index']: changing['index_changed']+=1
    if prev['command']['angular_z_radps']*m['command']['angular_z_radps']<0: changing['yaw_sign_changed']+=1
  prev=m
print(json.dumps(dict(counts=counts,region_min=np.min(region,axis=0).tolist(),region_max=np.max(region,axis=0).tolist(),region_median=np.median(region,axis=0).tolist(),pink_pixel_percentiles=np.percentile(lengths,[0,50,95,100]).tolist(),changes=changing,endpoint_min=np.min(endpoints,axis=0).tolist(),endpoint_max=np.max(endpoints,axis=0).tolist()),indent=2))
