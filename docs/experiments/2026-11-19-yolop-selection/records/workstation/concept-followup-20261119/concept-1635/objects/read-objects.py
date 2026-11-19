from pathlib import Path
import json,sqlite3,sys,cv2,numpy as np,tarfile,io,bisect
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
b=list((Path.home()/'rosbags/concept').glob('concept_20261119_163533*/*.db3'))[0]
db=sqlite3.connect('file:'+str(b)+'?mode=ro',uri=True)
def rows(topic):
    return db.execute('select timestamp,data from messages join topics on topics.id=messages.topic_id where topics.name=? order by timestamp',(topic,))
def save(out,name,content):
    item=tarfile.TarInfo(name);item.size=len(content);out.addfile(item,io.BytesIO(content))
out=tarfile.open(fileobj=sys.stdout.buffer,mode='w|gz')
times={}
ds=[]
for ns,raw in rows('/concept/decision'):
    d=json.loads(deserialize_message(raw,String).data)
    keys=['stamp','status','session','reason','planner_reason','rejected','objects','tracks','capture_age_s','road_rows','geometry_profile','speed_estimate_mps','command','feasible_count']
    r={k:d.get(k) for k in keys}
    r['motor']=d.get('command_native')
    r['objects']=[v[:6] for v in (r['objects'] or [])]
    ds.append(r)
    if r['stamp']: times[r['stamp']['sec']*1000000000+r['stamp']['nanosec']]=len(ds)-1
save(out,'decisions.json',json.dumps(ds,separators=(',',':')).encode())
selected=set()
for j in range(0,len(ds),20):selected.add(j)
best={}
for j,d in enumerate(ds):
    ts=d['tracks'] or []
    for t in ts:
        key=(d['session'],t['id'],t['source'])
        if key not in best or abs(t['position'][0])<abs(best[key][1]):best[key]=(j,t['position'][0])
for i,z in sorted(best.values(),key=lambda r:abs(r[1]))[:18]:selected.add(i)
for ns,raw in rows('/concept/perception/overlay/compressed'):
    m=deserialize_message(raw,CompressedImage)
    t=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
    if t in times and times[t] in selected:
        save(out,'frame-%04d.jpg'%times[t],bytes(m.data))
out.close()
