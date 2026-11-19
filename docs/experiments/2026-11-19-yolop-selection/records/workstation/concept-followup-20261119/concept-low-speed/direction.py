import bisect,collections,json,sqlite3,struct,sys
from pathlib import Path
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist
from rclpy.serialization import deserialize_message
p=Path(sys.argv[1]);db=sqlite3.connect(p.as_uri()+'?mode=ro',uri=True)
def read(t,typ):
 for tm,b in db.execute('select timestamp,data from messages join topics on topics.id=topic_id where name=? order by timestamp',(t,)):
  yield tm,deserialize_message(b,typ)
def frames(t,id):
 return [(tm,struct.unpack('<ii',bytes(m.data))) for tm,m in read(t,Frame) if m.id==id and m.dlc==8]
wheel=frames('/aiformula_sensing/vehicle_info',0x711)
motor=frames('/concept/internal/motor_can',0x210)
cmd=list(read('/concept/internal/motor_cmd',Twist))
mt=[t for t,_ in motor]; ct=[t for t,_ in cmd]
def find(ts,data,t):
 i=bisect.bisect_right(ts,t)-1
 if i<0 or t-ts[i]>40_000_000:return None
 return data[i][1]
matches=[];groups=collections.Counter()
for t,w in wheel:
 m=find(mt,motor,t); c=find(ct,cmd,t)
 if not m or not c:continue
 if min(w)<0:
  matches.append(dict(wheel=w,motor=m,command=[c.linear.x,c.angular.z]))
  groups[('both reverse motor' if max(m)<0 else 'other motor', 'reverse twist' if c.linear.x<0 else 'other twist')]+=1
print(json.dumps(dict(total_wheel=len(wheel), matched_negative=len(matches),groups=[[list(k),v] for k,v in groups.items()],wheel_range=[min(min(w) for _,w in wheel),max(max(w) for _,w in wheel)],examples=matches[:12]),indent=2))
