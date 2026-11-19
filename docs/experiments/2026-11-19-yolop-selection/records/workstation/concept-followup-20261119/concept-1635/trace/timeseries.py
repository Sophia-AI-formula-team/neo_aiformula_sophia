import json,math,sqlite3,struct,statistics
from collections import defaultdict
from pathlib import Path
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from rclpy.serialization import deserialize_message
p,=Path.home().glob('rosbags/concept/*163533*/*.db3')
db=sqlite3.connect(p.as_uri()+'?mode=ro&immutable=1',uri=True)
t0=db.execute("select min(timestamp) from messages").fetchone()[0]
rows=defaultdict(lambda:defaultdict(list))
for topic,typ,convert in [
 ('/concept/internal/motor_cmd',Twist,lambda m:dict(vcmd=m.linear.x,wcmd=m.angular.z)),
 ('/aiformula_sensing/vectornav/imu',Imu,lambda m:dict(gyro=m.angular_velocity.z)),
 ('/aiformula_sensing/vehicle_info',Frame,lambda m:dict(wheel=sum(struct.unpack('<ii',bytes(m.data)))*math.pi*.127/60) if m.id==0x711 and m.dlc==8 else {}),
 ('/concept/gate/status',String,lambda m:dict(mode=json.loads(m.data)['mode'])),
 ('/concept/decision',String,lambda m:dict(reason=json.loads(m.data)['reason']))]:
 for t,b in db.execute('select timestamp,data from messages join topics on topics.id=topic_id where topics.name=? order by timestamp',(topic,)):
  vals=convert(deserialize_message(b,typ)); bucket=int((t-t0)//100_000_000)
  for key,val in vals.items():rows[bucket][key].append(val)
print(json.dumps([dict(t=k/10,**{key:(v[-1] if isinstance(v[0],str) else round(statistics.median(v),4)) for key,v in vals.items()}) for k,vals in sorted(rows.items())],separators=(',',':')))
