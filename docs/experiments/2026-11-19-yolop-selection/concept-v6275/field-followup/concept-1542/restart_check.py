import rclpy,time,json,collections
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n=rclpy.create_node('concept_restart_check')
gate={}; wheels={}; stats=collections.Counter(); history=[]
def update(m, target):
 target.clear(); target.update(json.loads(m.data))
def decision(m):
 d=json.loads(m.data); k=(d['session'],d['status'],d['reason']);stats[str(k)]+=1
 if not history or history[-1][1:]!=list(k):
  history.append([time.time(),*k])
n.create_subscription(String,'/concept/gate/status',lambda m:update(m,gate),qos_profile_sensor_data)
n.create_subscription(String,'/concept/vehicle_state',lambda m:update(m,wheels),qos_profile_sensor_data)
n.create_subscription(String,'/concept/decision',decision,qos_profile_sensor_data)
def run(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.08)
run(3)
now=n.get_clock().now().nanoseconds/1e9
assert gate and gate['mode']=='paused' and gate['linear_mps']==0 and gate['yaw_rps']==0 and gate['manual_held'] is False and abs(now-gate['t'])<.3,gate
assert wheels.get('valid') and wheels.get('leftRpm')==0 and wheels.get('rightRpm')==0 and abs(now-wheels['t'])<.3,wheels
cli=n.create_client(Trigger,'/concept/drive/restart')
assert cli.wait_for_service(timeout_sec=.7)
future=cli.call_async(Trigger.Request())
rclpy.spin_until_future_complete(n,future,timeout_sec=3)
print('restart',future.result(),flush=True)
run(14)
print(json.dumps(dict(stats=stats, transitions=history[:12],final_gate=gate),indent=2),flush=True)
n.destroy_node();rclpy.shutdown()
