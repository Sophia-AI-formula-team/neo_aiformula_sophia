from pathlib import Path
import sqlite3, json, numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation
dbpath=next((Path.home()/'rosbags/concept').glob('concept_20261119_163533*/*.db3'))
db=sqlite3.connect('file:'+str(dbpath)+'?mode=ro',uri=True)
res=[]
for t,raw in db.execute("select timestamp,data from messages join topics on topics.id=messages.topic_id where name='/aiformula_sensing/vectornav/imu' order by timestamp"):
 m=deserialize_message(raw,Imu);q=m.orientation
 if m.orientation_covariance[0]<0 or abs(q.w)+abs(q.x)+abs(q.y)+abs(q.z)<.2:continue
 r=Rotation.from_quat([q.x,q.y,q.z,q.w]);p=r.as_euler('xyz',degrees=True)
 res.append([t/1.e9,*p,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
print(json.dumps(dict(frame_id=m.header.frame_id,n=len(res),quantiles=np.percentile(res,[1,50,99],axis=0).tolist() if res else [])))
