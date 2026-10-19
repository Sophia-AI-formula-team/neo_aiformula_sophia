"""Real installed ROS graph in an isolated localhost domain. Never opens can0."""
import os
assert os.environ.get('ROS_DOMAIN_ID') == '93'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
import json, time, subprocess, signal, struct, importlib.util
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from can_msgs.msg import Frame
from motor_controller.motor_controller import MotorController

out=Path('/home/control/aiformula_runtime/motor_traffic_gate_20261012')
args=['--ros-args','-r','__ns:=/gate_acceptance',
 '-r','sub_speed_command:=/aiformula_control/game_pad/cmd_vel',
 '-r','sub_manual_command:=/gate_acceptance/manual',
 '-r','sub_manual_joy:=/gate_acceptance/joy',
 '-r','pub_can:=/gate_acceptance/can',
 '-p','wheel.tread:=0.6','-p','wheel.diameter:=0.254','-p','wheel.gear_ratio:=1.1',
 '-p','publish_timer_loop_duration:=0.01','-p','motion_enabled:=false']
rclpy.init(args=args)
motor=MotorController()
assert motor.can_pub.topic_name=='/gate_acceptance/can'
node=Node('probe',namespace='/gate_acceptance',use_global_arguments=False)
executor=SingleThreadedExecutor();executor.add_node(motor);executor.add_node(node)
auto=node.create_publisher(Twist,'/aiformula_control/game_pad/cmd_vel',1)
manual=node.create_publisher(Twist,'/gate_acceptance/manual',1)
joy=node.create_publisher(Joy,'/gate_acceptance/joy',1)
detections=node.create_publisher(String,'/aiformula_perception/yolop_a3/detections',1)
odom=node.create_publisher(Odometry,'/aiformula_sensing/gyro_odometry_publisher/odom',1)
points=[node.create_publisher(Pose2D,'/processed_point_'+k,1) for k in 'abc']
frames=[];statuses=[];map_status=[]
node.create_subscription(Frame,'/gate_acceptance/can',lambda m:frames.append((time.monotonic(),bytes(m.data))),100)
node.create_subscription(String,'/gate_acceptance/motor_controller/gate_status',lambda m:statuses.append(json.loads(m.data)),20)
node.create_subscription(String,'/semantic_drive/map_status',lambda m:map_status.append(json.loads(m.data)),20)
log=open(out/'semantic_isolated.log','w')
launch=subprocess.Popen(['ros2','launch','launchers','semantic_vehicle.launch.py'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
results=[]

def run(name, seconds=1.0, state='unknown', auto_v=(1.0,0.3), held=False,
        manual_v=None, send_signal=True, send_joy=True, expect='zero'):
    begin=time.monotonic();next_send=begin
    while time.monotonic()-begin<seconds:
        now=time.monotonic()
        if now>=next_send:
            next_send=now+0.04
            stamp=node.get_clock().now().to_msg()
            if send_signal:
                detections.publish(String(data=json.dumps(dict(header=dict(frame_id='zed_left_camera_optical_frame',
                    stamp=dict(sec=stamp.sec,nanosec=stamp.nanosec)),traffic_light_state=state))))
            m=Odometry();m.header.stamp=stamp;m.header.frame_id='odom';m.child_frame_id='base_footprint';m.pose.pose.orientation.w=1.0
            odom.publish(m)
            for i,pub in enumerate(points): pub.publish(Pose2D(x=float(i+1),y=0.,theta=0.))
            if send_joy:
                j=Joy();j.header.stamp=stamp;j.buttons=[0]*13;j.buttons[6]=int(held);joy.publish(j)
            if auto_v is not None:
                t=Twist();t.linear.x=float(auto_v[0]);t.angular.z=float(auto_v[1]);auto.publish(t)
            if manual_v is not None:
                t=Twist();t.linear.x=float(manual_v[0]);t.angular.z=float(manual_v[1]);manual.publish(t)
        executor.spin_once(timeout_sec=0.005)
    tail=[data for t,data in frames if t>time.monotonic()-0.2]
    assert len(tail)>=5,(name,'no CAN messages',len(tail),statuses[-2:])
    if expect=='zero': assert all(data==bytes(8) for data in tail),(name,'not zero',tail[-2:],statuses[-2:])
    elif expect=='nonzero': assert all(data!=bytes(8) for data in tail),(name,'not moving command',statuses[-2:])
    elif isinstance(expect,tuple): assert all(struct.unpack('<ii',data)==expect for data in tail),(name,tail[-2:],expect,statuses[-2:])
    results.append(dict(case=name,pass_=True,tail_frames=len(tail),rpm=struct.unpack('<ii',tail[-1]),status=statuses[-1] if statuses else None))
    print(name,'PASS',results[-1]['rpm'],flush=True)

try:
    run('deployment_inhibit',seconds=4,expect='zero')
    names=node.get_node_names_and_namespaces()
    assert all(name not in ('socket_can_sender','socket_can_receiver','teleop_node','trajectory_follower') for name,ns in names),names
    assert len(node.get_subscriptions_info_by_topic('/gate_acceptance/can'))==1
    assert map_status and map_status[-1]['traffic_policy']=='stop_wait'
    motor.motion_enabled=True  # Isolated test instance only, never installed YAML.
    run('automatic_go',expect='nonzero')
    run('stable_red_stop',state='red')
    run('unknown_after_red_holds',state='unknown')
    run('green_go',state='green',expect='nonzero')
    run('zero_command_zero_CAN',state='green',auto_v=(0.,0.))
    run('negative_steering_CAN',state='green',auto_v=(1.,-.3),expect=tuple(round(v) for v in motor.toRefRPM(1.,-.3)))
    run('manual_priority_red',state='red',held=True,manual_v=(-.2,.3),expect=tuple(round(v) for v in motor.toRefRPM(-.2,.3)))
    run('manual_zero_owns_control',state='red',held=True,manual_v=(0.,0.))
    run('manual_joy_loss_holds',state='red',send_joy=False,manual_v=(.2,.3))
    run('release_at_red_still_stops',state='red')
    run('green_restores_auto',state='green',expect='nonzero')
    run('LYA_timeout_zero',state='green',auto_v=None)
    run('LYA_fresh_resumes',state='green',expect='nonzero')
    run('camera_loss_zero',send_signal=False)
    run('fresh_camera_resumes',state='green',expect='nonzero')
    duplicate=node.create_publisher(Twist,'/aiformula_control/game_pad/cmd_vel',1)
    run('multiple_auto_publishers_hold',state='green')
    node.destroy_publisher(duplicate)
    run('sole_publisher_resumes',state='green',expect='nonzero')
    # Exact steering regression against previously deployed USB implementation.
    oldpath=Path('/home/control/aiformula_backups/20261012_motor_traffic_gate/source/control/motor_controller/motor_controller/motor_controller.py')
    spec=importlib.util.spec_from_file_location('old_usb_motor',oldpath);oldmod=importlib.util.module_from_spec(spec);spec.loader.exec_module(oldmod)
    old=object.__new__(oldmod.MotorController)
    for key in ('tread','diameter','gear_ratio','Lp','kp','x0p','Cp','Ln','kn','x0n','Cn'): setattr(old,key,getattr(motor,key))
    for v,w in [(1.,.3),(1.,-.3),(.2,.7),(-.2,-.7),(0.,.3),(3.5,.7)]:
        assert motor.toRefRPM(v,w)==old.toRefRPM(v,w),(v,w)
    assert motor.toRefRPM(0.,0.)==[0.,0.]
    results.append(dict(case='nonzero_sigmoid_regression_and_zero_fix',pass_=True))
    os.killpg(launch.pid,signal.SIGINT);launch.wait(timeout=15)
    run('semantic_process_loss_stops',seconds=1.2,state='green')
    assert motor.motion_enabled is True
    (out/'dds_results.json').write_text(json.dumps(dict(domain=93,localhost_only=True,can_output='/gate_acceptance/can',
        no_socketcan_process=True,vehicle_moved=False,cases=results,total_CAN_frames=len(frames),
        topo_status=map_status[-1] if map_status else None),indent=2))
    print('ALL',len(results),'CASES PASSED; no CAN socket was opened',flush=True)
finally:
    if launch.poll() is None:
        os.killpg(launch.pid,signal.SIGINT)
        try: launch.wait(timeout=10)
        except subprocess.TimeoutExpired: os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=5)
    executor.remove_node(motor);executor.remove_node(node);motor.destroy_node();node.destroy_node();executor.shutdown();rclpy.shutdown();log.close()
