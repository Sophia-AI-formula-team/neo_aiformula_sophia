import collections,json,sqlite3,struct
from pathlib import Path
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from can_msgs.msg import Frame
from sensor_msgs.msg import Imu
bag=next((Path.home()/'rosbags/concept').glob('concept_20261119_163533*/*.db3')).resolve()
db=sqlite3.connect(bag.as_uri()+'?mode=ro&immutable=1',uri=True)
for t,topic,payload in db.execute('select timestamp,name,data from messages join topics on topic_id=topics.id where name in (?,?,?,?,?) order by timestamp',('/concept/gate/status','/concept/internal/motor_cmd','/concept/vehicle_state','/concept/decision','/concept/internal/motor_can')):
 if topic.endswith('decision'):
  d=json.loads(deserialize_message(payload,String).data)
  print(json.dumps([t,'d', {k:d.get(k) for k in ('target_speed_mps','effective_speed_mps','status','reason','command','speed_estimate_mps','recovery','processing_ms','capture_age_s','feasible_count','geometry_profile')}],separators=(',',':')))
 elif topic.endswith('vehicle_state'):
  d=json.loads(deserialize_message(payload,String).data)
  print(json.dumps([t,'w',d['leftRpm'],d['rightRpm'],d['gyro'],d['valid']]))
 elif topic.endswith('motor_cmd'):
  m=deserialize_message(payload,Twist)
  print(json.dumps([t,'m',m.linear.x,m.angular.z]))
 elif topic.endswith('motor_can'):
  m=deserialize_message(payload,Frame)
  if m.id==0x210:print(json.dumps([t,'c',*struct.unpack('<ii',bytes(m.data))]))
 else:
  g=json.loads(deserialize_message(payload,String).data)
  print(json.dumps([t,'g',g['mode'],g['reason'],g['linear_mps'],g['yaw_rps'],g['max_auto_speed_mps']]))
