"""Extract the three stationary ROS sensor topics' serialized payloads, without imagery."""
import base64
import gzip
import json
import sqlite3
import sys
from pathlib import Path
from can_msgs.msg import Frame
from rclpy.serialization import deserialize_message

bag,=Path.home().glob('rosbags/concept/*163533*/*.db3')
db=sqlite3.connect(bag.as_uri()+'?mode=ro&immutable=1',uri=True)
start=db.execute('select min(timestamp) from messages').fetchone()[0]+62_250_000_000
topics=('/aiformula_sensing/vehicle_info','/aiformula_sensing/vectornav/imu','/aiformula_control/joy_node/joy_l2_throttle')
rows=db.execute('select timestamp,name,type,data from messages join topics on topic_id=topics.id where timestamp>=? and timestamp<? and name in (?,?,?) order by timestamp',(start,start+2_000_000_000,*topics))
items=[]
for stamp,topic,kind,raw in rows:
    if kind=='can_msgs/msg/Frame' and deserialize_message(raw,Frame).id!=0x711:
        continue
    items.append(dict(offset_ns=stamp-start,topic=topic,ros_type=kind,cdr_b64=base64.b64encode(raw).decode('ascii')))
sys.stdout.buffer.write(gzip.compress(json.dumps(dict(bag=bag.parent.name,start_ns=start,period_ns=2_000_000_000,messages=items),separators=(',',':')).encode(),mtime=0))
