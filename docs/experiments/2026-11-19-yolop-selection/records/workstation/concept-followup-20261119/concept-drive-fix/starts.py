"""Extract sustained, near-straight from-rest commands with real wheel feedback."""
import bisect
import json
import sqlite3
import struct
from pathlib import Path
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

def audit(path):
    db = sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True)
    db.execute('begin')
    def read(topic,typ):
        rows = db.execute('select timestamp,data from messages join topics on topics.id=topic_id where topics.name=? order by timestamp',(topic,))
        for t,msg in rows:
            yield t/1e9,deserialize_message(msg,typ)
    wheel = [(t,struct.unpack('<ii',bytes(m.data))) for t,m in read('/aiformula_sensing/vehicle_info',Frame) if m.id==0x711 and m.dlc==8 and not(m.is_extended or m.is_rtr or m.is_error)]
    can = [(t,struct.unpack('<ii', bytes(m.data))) for t,m in read('/concept/internal/motor_can',Frame) if m.id==0x210 and m.dlc==8]
    gates = [(t,json.loads(m.data)) for t,m in read('/concept/gate/status',String)]
    starts=[]
    run=[]
    def finish():
        if len(run)<2: return
        t,v = run[0]
        end = run[-1][0]
        if end-t < .22: return
        before=[(u,w) for u,w in wheel if t-.25<=u<t]
        after=[(u,w) for u,w in wheel if t+.12<=u<=end]
        out=[w for u,w in can if t+.08<=u<=end]
        modes={g.get('mode') for u,g in gates if t<=u<=end}
        if len(before)<3 or len(after)<3 or len(out)<4 or not modes or not modes.issubset({'manual','auto'}): return
        sign=1 if v>0 else -1
        if not all(min(x*sign for x in w)>0 for w in out): return
        if any(max(abs(x) for x in w)>2 for _,w in before): return
        moving=[(u,w) for u,w in after if min(x*sign for x in w)>=5]
        starts.append(dict(t=t, speed=round(v,3), duration=round(end-t,3), max_demand_variation=round(max(abs(v2-v) for u,v2 in run),4),mode=sorted(modes),wheel_samples=len(after),peak_wheel_signed=[max(w[i]*sign for _,w in after) for i in range(2)],moving_samples=len(moving),first_moving_ms=round((moving[0][0]-t)*1000) if moving else None,can_range=[min(min(x*sign for x in w) for w in out),max(max(x*sign for x in w) for w in out)]))
    for t,c in read('/concept/internal/motor_cmd',Twist):
        v,w=c.linear.x,c.angular.z
        if run and (t-run[-1][0]>.075 or abs(v-run[0][1])>.04 or abs(w)>.05 or v*run[0][1]<=0):
            finish();run=[]
        if v and abs(w)<=.05:run.append((t,v))
    finish()
    return dict(bag=path.parent.name,starts=starts,wheel_range=[min(min(w) for t,w in wheel),max(max(w) for t,w in wheel)] if wheel else None,wheels=len(wheel))
print(json.dumps([audit(p) for p in sorted((Path.home()/'rosbags/concept').glob('*/*.db3'))],indent=2))
