from pathlib import Path
import time,json
import rclpy
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
import numpy as np, cv2
from concept_yolop.frame_codec import parse_frame_header,decode_frame_data
base=Path.cwd();rclpy.init(); n=rclpy.create_node('concept_parked_perception_sample'); rows=[]
def sample(msg):
    packet,ts,_,profile=parse_frame_header(msg.data)
    data=decode_frame_data(packet)
    number=len(rows)
    if number>=5:return
    cv2.imwrite(str(base/('sample-%d.png'%number)),data['bgr'])
    np.savez_compressed(str(base/('sample-%d.npz'%number)), **{k:data[k] for k in ('road','lane','scores','labels','boxes')})
    rows.append([ts,profile,sum(int(label)==91 and score>=.5 for label,score in zip(data['labels'],data['scores']))])
n.create_subscription(String,'/concept/perception/frame',sample,qos_profile_sensor_data)
end=time.monotonic()+8
while len(rows)<5 and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.12)
print(json.dumps(rows,default=int))
n.destroy_node();rclpy.shutdown()
