import json
import math
import sqlite3
import struct
from pathlib import Path
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
p, = (Path.home()/'rosbags/concept').glob('concept_20261119_155042*/*.db3')
db = sqlite3.connect(p.as_uri()+'?mode=ro&immutable=1',uri=True)
start = db.execute('select min(timestamp) from messages').fetchone()[0]
sets = {
    'target': ('/concept/decision',String,lambda m: json.loads(m.data).get('target_speed_mps')),
    'gate_limit': ('/concept/gate/status',String,lambda m: json.loads(m.data).get('max_auto_speed_mps')),
    'motor_input': ('/concept/internal/motor_cmd',Twist,lambda m: m.linear.x),
    'auto_cmd': ('/concept/auto_cmd',TwistStamped,lambda m:m.twist.linear.x),
    'wheel_speed': ('/aiformula_sensing/vehicle_info',Frame,lambda m: sum(struct.unpack('<ii',bytes(m.data)))*math.pi*.127/60 if m.id==0x711 and m.dlc==8 else None),
}
out={}
for key,(topic,kind,extract) in sets.items():
    rows=[]
    for t,data in db.execute('select timestamp,data from messages join topics on topics.id=topic_id where topics.name=? order by timestamp',(topic,)):
        val=extract(deserialize_message(data,kind))
        if val is not None: rows.append([round((t-start)/1e9,4),round(val,4)])
    out[key]=rows
print(json.dumps(out,separators=(',',':')))
