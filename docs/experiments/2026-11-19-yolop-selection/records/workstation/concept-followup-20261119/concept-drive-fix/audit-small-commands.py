"""Find recorded continuous straight low-speed motor requests across today's bags."""
from pathlib import Path
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Twist
import sqlite3
import json

def summary(path, lo, hi, sign):
    db=sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True)
    sql="select timestamp,data from messages join topics on topic_id=topics.id where name='/concept/internal/motor_cmd' order by timestamp"
    episodes=[]
    first=previous=None
    for t,raw in db.execute(sql):
        m=deserialize_message(raw,Twist)
        if previous is not None and t-previous>60_000_000:
            episodes.append((previous-first)/1e9);first=previous=None
        if lo<=sign*m.linear.x<=hi and abs(m.angular.z)<=.05:
            if first is None:first=t
            previous=t
        elif first is not None:
            episodes.append((previous-first)/1e9);first=previous=None
    if first is not None:episodes.append((previous-first)/1e9)
    db.close()
    return dict(episodes=len(episodes),longest_s=round(max(episodes,default=0.),3))
result={}
for p in sorted(Path.home().glob('rosbags/concept/concept_20261119*/*.db3')):
    result[p.parent.name]={('forward' if sign>0 else 'reverse')+'_0.15_to_0.25_mps':summary(p,.15,.25,sign) for sign in (1,-1)}
print(json.dumps(result, indent=2))
