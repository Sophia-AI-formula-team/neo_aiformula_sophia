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
for offset,length in [(2.,1.),(62.25,2.)]:
 stats={}
 for topic,series in streams.items():
  seq=[v for v in series if offset<=v[0]<offset+length]
  wrapped=[(seq[0][0]+length,True,seq[0][2])]
  gaps=[(b[0]-a[0])*1000 for a,b in zip(seq,seq[1:]+wrapped)]
  stats[topic]=dict(samples=len(seq),max_arrival_gap_ms=round(max(gaps),2),wrap_arrival_gap_ms=round(gaps[-1],2),max_message_age_ms=round(max(x[2] for x in seq),2))
 print(json.dumps(dict(offset_s=offset,duration_s=length,channels=stats)))
