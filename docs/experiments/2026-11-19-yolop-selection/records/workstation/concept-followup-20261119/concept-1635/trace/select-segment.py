import collections,json,sqlite3,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from can_msgs.msg import Frame
from sensor_msgs.msg import Joy,Imu
p,=Path.home().glob('rosbags/concept/*163533*/*.db3')
db=sqlite3.connect(p.as_uri()+'?mode=ro&immutable=1',uri=True); zero=db.execute('select min(timestamp) from messages').fetchone()[0]
streams={}
for topic,kind in [('/aiformula_sensing/vehicle_info',Frame),('/aiformula_sensing/vectornav/imu',Imu),('/aiformula_control/joy_node/joy_l2_throttle',Joy)]:
 vals=[]
 for t,b in db.execute('select timestamp,data from messages join topics on topic_id=topics.id where topics.name=? order by timestamp',(topic,)):
  m=deserialize_message(b,kind)
  if kind is Frame and m.id!=0x711:continue
  accepted=not any(m.data) if kind is Frame else abs(m.angular_velocity.z)<.03 if kind is Imu else not any(m.buttons)
  old=(t-(m.header.stamp.sec*10**9+m.header.stamp.nanosec))/1e6
  vals.append(((t-zero)/1e9,accepted,old))
 streams[topic]=vals
scores=[]
for quarter in range(0,411):
 offset=quarter*.25
 best=[]
 for vals in streams.values():
  vals=[v for v in vals if offset<=v[0]<offset+2]
  if len(vals)<12 or not all(x[1] and 0<=x[2]<40 for x in vals):break
  intervals=[(b[0]-a[0])*1000+b[2] for a,b in zip(vals,vals[1:]+[(vals[0][0]+2,True,vals[0][2])])]
  best.append(round(max(intervals),2))
 else:scores.append((max(best),offset,best))
print(sorted(scores)[:6])
