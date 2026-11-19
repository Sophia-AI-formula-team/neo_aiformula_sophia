import time,json,statistics,sys
import rclpy
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=rclpy.create_node('field_camera_idle_timing')
rows=[]
def read(m):
    d=json.loads(m.data)
    rows.append(d.get('camera_timing_ms',{}))
n.create_subscription(String,'/concept/decision',read,qos_profile_sensor_data)
end=time.monotonic()+12.
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
out={k:[round(statistics.median(vals),2),round(sorted(vals)[int(.95*(len(vals)-1))],2)] for k in {k for r in rows for k in r} for vals in [[r[k] for r in rows if k in r]]}
print(json.dumps(dict(count=len(rows),med_p95_ms=out)))
n.destroy_node();rclpy.shutdown()
