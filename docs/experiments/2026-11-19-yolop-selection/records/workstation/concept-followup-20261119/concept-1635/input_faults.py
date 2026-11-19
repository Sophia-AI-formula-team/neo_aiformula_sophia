from bisect import bisect, bisect_left
from collections import Counter
from pathlib import Path
import struct, json, sqlite3
from can_msgs.msg import Frame
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from rclpy.serialization import deserialize_message
p,=(Path.home()/'rosbags/concept').glob('concept_20261119_163533*/*.db3')
db=sqlite3.connect(p.as_uri()+'?mode=ro&immutable=1',uri=True)
def read(name,typ,fn):
    return [(t,fn(m)) for t,data in db.execute('select timestamp,data from messages join topics on topics.id=topic_id where topics.name=? order by timestamp',(name,)) for m in [deserialize_message(data,typ)]]
def stamp(h):return h.stamp.sec*10**9+h.stamp.nanosec
w=[(t,(h,r,l)) for t,(h,k,r,l) in read('/aiformula_sensing/vehicle_info',Frame,lambda m:(stamp(m.header),m.id,*struct.unpack('<ii',bytes(m.data)))) if k==0x711]
g=read('/aiformula_sensing/vectornav/imu',Imu,lambda m:(stamp(m.header),m.angular_velocity.z,m.angular_velocity_covariance[0]))
s=read('/concept/vehicle_state',String,lambda m:json.loads(m.data));a=read('/concept/gate/status',String,lambda m:json.loads(m.data)); rr=read('/concept/decision',String,lambda m:json.loads(m.data))
def near(a,t,lo,hi):return [(round((u-t)/1e6,1),x) for u,x in a if -lo*1e6<=u-t<=hi*1e6]
out=[]
for t,x in s:
    if x['valid']:continue
    out.append(dict(t=t,reason=x['reason'],wheel=near(w,t,130,60),gyro=near(g,t,240,60),gate=[(dt,(q['mode'],q['reason'])) for dt,q in near(a,t,180,230)]))
seq=[]
for i,(t,d) in enumerate(rr):
    status=d['reason'];old=rr[i-1][1]['reason'] if i else ''
    if status.startswith('image gap') and not old.startswith('image gap'):
        seq.append([(round((v-t)/1e6,1),q['status'],q['reason'],q['stamp'],q['capture_age_s'],q.get('sensor_age_s')) for v,q in rr[max(0,i-4):i+2]])
print(json.dumps(dict(states=out,rpm_min=min(min(v[1:]) for _,v in w),rpm_max=max(max(v[1:]) for _,v in w),gaps=seq),indent=2))
