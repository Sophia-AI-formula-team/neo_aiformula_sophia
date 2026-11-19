from collections import deque
import json,struct,time,os,signal
from pathlib import Path
import rclpy
from can_msgs.msg import Frame
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
rclpy.init()
n=rclpy.create_node('verify_bench_handoff_standstill')
w=deque(maxlen=400)
last={}
def wheel(m):
    now=n.get_clock().now().nanoseconds/1e9
    stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
    if m.id==0x711 and m.dlc==8 and not(m.is_error or m.is_rtr or m.is_extended) and 0<=now-stamp<.2:
        w.append((time.monotonic(),struct.unpack('<ii',bytes(m.data))))
def motor(m):
    last.clear()
    last.update(json.loads(m.data), receipt=time.monotonic())
n.create_subscription(Frame,'/aiformula_sensing/vehicle_info',wheel,qos_profile_sensor_data)
n.create_subscription(String,'/aiformula_control/motor_controller/gate_status',motor,qos_profile_sensor_data)
end=time.monotonic()+70
rest=[]
while time.monotonic()<end:
    rclpy.spin_once(n,timeout_sec=.04)
    now=time.monotonic()
    rest=[(t,v) for t,v in w if now-t<1.3]
    if (len(rest)>=12 and rest[-1][0]-rest[0][0]>=1. and now-rest[-1][0]<.1 and
        all(v==(0,0) for t,v in rest) and now-last.get('receipt',0)<.3 and last.get('command')==[0.,0.] and not last.get('manual_held')):
        print(json.dumps(dict(raw_frames_at_rest=len(rest),span_s=rest[-1][0]-rest[0][0],motor=last)),flush=True)
        for proc in Path('/proc').glob('[0-9]*'):
            try:
                args=(proc/'cmdline').read_bytes().split(b'\0')
                ix=args.index(b'/opt/ros/foxy/bin/ros2')
            except (ValueError,FileNotFoundError,ProcessLookupError,PermissionError):
                continue
            cmd=args[ix+1:ix+4]
            pid=int(proc.name)
            if cmd in ([b'run',b'obsticle_avoidence',b'oa_2'], [b'run',b'trajectory_follower',b'lya_follower_connected_omegat_global']):
                if os.getpgid(pid)==pid:
                    os.killpg(pid,signal.SIGINT)
                    print('Stopped',pid,cmd,flush=True)
            if cmd==[b'launch',b'launchers',b'allnodes.launch.py']:
                os.kill(pid,signal.SIGINT)
                print('Stopped',pid,cmd,flush=True)
        break
else:
    raise RuntimeError('Stationary handoff not confirmed: '+str((list(w)[-3:],last)))
n.destroy_node();rclpy.shutdown()
