"""Render the unmodified camera callback on actual recorded decisions in memory."""
from collections import Counter
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sqlite3
import struct
from types import SimpleNamespace

from cv_bridge import CvBridge
import numpy as np
from concept_visualization import node as visualization

package_path = Path(visualization.__file__).resolve()
assert package_path.parent.parent == Path.cwd()
bag = next(Path.home().glob('rosbags/concept/*134918*/*.db3'))
db = sqlite3.connect(bag.as_uri() + '?mode=ro&immutable=1', uri=True)
rows = db.execute("SELECT data FROM messages JOIN topics ON topic_id=topics.id WHERE name='/concept/decision' ORDER BY timestamp")
clock = SimpleNamespace(last_annotated=float('-inf'), last_annotated_key=None, bridge=CvBridge(), view=None)
last_image = []
clock.view = SimpleNamespace(publish=last_image.append)
image = clock.bridge.cv2_to_imgmsg(np.zeros((360,640,3),np.uint8), 'bgr8')
stats = Counter()
source = []
for raw, in rows:
    decision = json.loads(raw[8:7+struct.unpack_from('<I',raw,4)[0]])
    source.append(decision['status'])
    stamp = decision['stamp']
    image.header.stamp.sec, image.header.stamp.nanosec = stamp['sec'], stamp['nanosec']
    visualization.Visualization.camera(clock,image,decision)
    assert len(last_image)==1
    pix = clock.bridge.imgmsg_to_cv2(last_image.pop(), 'bgr8')
    near = pix[6:136,6:136]
    outside = pix[136:,:]
    color = (170,72,250)
    have = bool(decision['selected_path'])
    near_ink = np.any(np.all(near == color,axis=2))
    perspective_ink = np.any(np.all(outside == color,axis=2))
    stats['nonempty_path' if have else 'empty_path']+=1
    stats['inset_tp' if near_ink and have else 'inset_fn' if have else 'inset_fp' if near_ink else 'inset_tn']+=1
    if have and not perspective_ink:
        stats['inset_only_visible']+=int(near_ink)
    if have and (not np.ptp(np.array(decision['selected_path'])[:,:2], axis=0).any()):
        stats['degenerate_path_dot_only']+=int(near_ink)
result=dict(jst=datetime.now().astimezone().isoformat(timespec='seconds'), source=bag.parent.name,
            visualization_node_sha256=hashlib.sha256(package_path.read_bytes()).hexdigest(),
            statuses=Counter(source), counts={key:stats[key] for key in
                    ('nonempty_path','empty_path','inset_tp','inset_fp','inset_tn','inset_fn','inset_only_visible','degenerate_path_dot_only')})
print(json.dumps(result,indent=2))
assert stats['inset_fp']==0 and stats['inset_fn']==0 and stats['inset_tp'] and stats['inset_tn']
