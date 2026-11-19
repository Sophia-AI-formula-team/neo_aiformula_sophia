import json, os, signal, time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage, Joy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool,Trigger
assert os.environ['ROS_DOMAIN_ID']=='198' and os.environ['ROS_LOCALHOST_ONLY']=='1'
rclpy.init()
node=rclpy.create_node('concept_timing_observer')
def ns(s):return s.sec*10**9+s.nanosec
raw,dec,cmd,gate,states,joy={},{},{},[],[],[]
def decision(msg):
    d=json.loads(msg.data)
    if not d['stamp']:return
    k=d['stamp']['sec']*10**9+d['stamp']['nanosec']
    dec[k]={i:d.get(i) for i in ('status','reason','capture_age_s','processing_ms','geometry_profile','sensor_age_s','target_speed_mps','effective_speed_mps','camera_timing_ms','command','geometry_profile')}
def gate_cb(msg):
    d=json.loads(msg.data)
    gate.append([time.time(),d['mode'],d['reason'],d['linear_mps'],d['max_auto_speed_mps']])
def state_cb(msg):
    d=json.loads(msg.data)
    states.append([time.time(),d['valid'],d['source']])
for typ,topic,fn in [
    (Image,'/aiformula_sensing/zed_node/left_image/undistorted',lambda m:raw.update({ns(m.header.stamp):time.time_ns()})),
    (Image,'/concept/perception/overlay',lambda m:None),(CompressedImage,'/concept/perception/overlay/compressed',lambda m:None),
    (TwistStamped,'/concept/auto_cmd',lambda m:cmd.update({ns(m.header.stamp):[time.time_ns(),m.twist.linear.x]})),
    (String,'/concept/decision',decision),(String,'/concept/gate/status',gate_cb),(String,'/concept/vehicle_state',state_cb),
    (Joy,'/aiformula_control/joy_node/joy_l2_throttle',lambda m:joy.append(time.time()))]:
    node.create_subscription(typ,topic,fn,qos_profile_sensor_data)
def pump():rclpy.spin_once(node,timeout_sec=.025)
def service(typ, name, request):
    cli=node.create_client(typ,'/concept/gate/'+name)
    assert cli.wait_for_service(timeout_sec=3.)
    f=cli.call_async(request)
    deadline=time.monotonic()+4
    while not f.done() and time.monotonic()<deadline:pump()
    assert f.done() and f.result().success,(name,f.result())
events=[]
fault={}
motors=[]
try:
    deadline=time.monotonic()+32
    while not (len(cmd)>12 and gate and len(joy)>3 and states and states[-1][1]) and time.monotonic()<deadline:pump()
    assert len(cmd)>12 and gate and joy and states[-1][1], (len(cmd),len(gate),len(joy),len(states))
    assert not node.count_subscribers('/aiformula_control/motor_controller/reference_signal')
    assert not node.count_publishers('/aiformula_control/motor_controller/reference_signal')
    assert not any(name=='socket_can_sender' for name,ns in node.get_node_names_and_namespaces())
    assert [s.node_name for s in node.get_subscriptions_info_by_topic('/concept/replay/reference_signal')]==['virtual_receiver']
    node.create_subscription(Twist,'/concept/internal/motor_cmd',
            lambda m:motors.append([time.time(),m.linear.x,m.angular.z]),qos_profile_sensor_data)
    service(SetBool,'enable_output',SetBool.Request(data=True))
    for i in range(int(os.environ.get('ATTEMPTS','1'))):
        service(Trigger,'resume',Trigger.Request())
        events.append(time.time())
        deadline=time.monotonic()+float(os.environ['RUN_SECS'])
        while time.monotonic()<deadline:pump()
        if os.environ.get('LOSS_TEST')=='1' and i+1==int(os.environ['ATTEMPTS']):
            pids=[]
            for proc in Path('/proc').glob('[0-9]*'):
                try:
                    args=(proc/'cmdline').read_bytes().split(b'\0')
                    env=(proc/'environ').read_bytes().split(b'\0')
                    if any(a.endswith(b'/lib/concept_drive/drive') for a in args) and b'ROS_DOMAIN_ID=198' in env:
                        pids.append(int(proc.name))
                except (FileNotFoundError,ProcessLookupError,PermissionError):pass
            assert len(pids)==1,pids
            fault['before']=gate[-1]
            fault['suspend_at']=time.time()
            os.kill(pids[0],signal.SIGSTOP)
            try:
                end=time.monotonic()+1.35
                while time.monotonic()<end:pump()
            finally:
                os.kill(pids[0],signal.SIGCONT)
            fault['resume_process_at']=time.time()
            end=time.monotonic()+.65
            while time.monotonic()<end:pump()
            fault['after']=gate[-1]
            fault['nonzero_after_500ms']=sum(bool(v or w) for t,v,w in motors if t>fault['suspend_at']+.52)
            fault['motor_samples_after_500ms']=sum(t>fault['suspend_at']+.52 for t,v,w in motors)
        service(Trigger,'pause',Trigger.Request())
    service(SetBool,'enable_output',SetBool.Request(data=False))
finally:
    Path(os.environ['RUN_DIR'],'probe.json').write_text(json.dumps(dict(raw=raw,dec=dec,cmd=cmd,gate=gate,states=states,joy=joy,events=events,fault=fault,motors=motors)))
    node.destroy_node()
    rclpy.shutdown()
