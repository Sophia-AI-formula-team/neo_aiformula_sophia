import json,sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
p=next((Path.home()/'rosbags/concept').glob('concept_20261119_163533*/*.db3')).resolve()
db=sqlite3.connect(p.as_uri()+'?mode=ro&immutable=1',uri=True)
start=db.execute('select min(timestamp) from messages').fetchone()[0]
sets=[]
for t,raw in db.execute('select timestamp,data from messages join topics on topics.id=topic_id where name=? order by timestamp',('/concept/decision',)):
 d=json.loads(deserialize_message(raw,String).data)
 if (4<(t-start)/1e9<7) and d.get('status')=='ok':
  sets.append(dict(t=t,**{k:d.get(k) for k in ('stamp','objects','road_rows','heading','speed_estimate_mps','command','reason')}))
print(json.dumps(sets,separators=(',',':')))
