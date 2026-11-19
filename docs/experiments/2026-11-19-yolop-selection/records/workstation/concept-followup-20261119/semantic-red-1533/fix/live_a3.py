import collections, json, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Bool
from pixel_stop_flag import observation, INPUT, OUTPUT
rclpy.init()
n=rclpy.create_node('pixel_input_audit')
stats=collections.Counter()
ages=[]
pixels=[]
def on_frame(msg):
    d=json.loads(msg.data)
    h=d.get('header',{})
    s=h.get('stamp',{})
    now=n.get_clock().now().nanoseconds
    stats['detections']+=1
    stats['frame='+str(h.get('frame_id'))]+=1
    if s:
        ages.append((now-s['sec']*10**9-s['nanosec'])/1e6)
    if 'red_pixels_count' in d:
        stats['has_pixel_count']+=1
        pixels.append(d['red_pixels_count'])
    if observation(msg.data,now):
        stats['adapter_valid']+=1
def on_flag(msg):
    stats['flag='+str(msg.data)]+=1
n.create_subscription(String,INPUT,on_frame,qos_profile_sensor_data)
n.create_subscription(Bool,OUTPUT,on_flag,qos_profile_sensor_data)
end=time.monotonic()+22
while time.monotonic()<end:
    rclpy.spin_once(n,timeout_sec=.09)
print(json.dumps(dict(stats=stats,age_ms=[min(ages), sorted(ages)[len(ages)//2], max(ages)] if ages else [],
    pixel_range=[min(pixels),max(pixels)] if pixels else [], flag_publishers=[i.node_name for i in n.get_publishers_info_by_topic(OUTPUT)])))
n.destroy_node()
rclpy.shutdown()
