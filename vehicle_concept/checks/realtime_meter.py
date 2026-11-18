"""Measure the deployed graph against wall time in an isolated replay domain."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import cv2
from realtime_analysis import rescore, summary
import rclpy
from builtin_interfaces.msg import Time
from can_msgs.msg import Frame
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String

BILLION = 10**9
TOPIC = '/aiformula_sensing/zed_node/left_image/undistorted'
STREAMS = {'perception': '/concept/perception/frame', 'decision': '/concept/decision',
           'twist': '/concept/auto_cmd'}


def ns(stamp):
    return stamp['sec'] * BILLION + stamp['nanosec']


def stamp(t):
    return Time(sec=t//BILLION, nanosec=t%BILLION)


def collect(kind, out, node):
    records = []
    def receive(msg):
        at = time.monotonic_ns()
        if kind == 'twist':
            source = msg.header.stamp.sec * BILLION + msg.header.stamp.nanosec
            record = {'cmd': [msg.twist.linear.x, msg.twist.angular.z]}
        elif kind == 'decision':
            value = json.loads(msg.data)
            source = ns(value['stamp']) if value.get('stamp') else None
            record = {k:value.get(k) for k in ('status','reason','processing_ms','capture_age_s','selected_index',
                                               'feasible_count','geometry_profile','session','planning_hz',
                                               'rate_skipped_total')}
        else:
            value = json.loads(msg.data)
            source = ns(value['stamp'])
            record = {'geometry_profile': value['geometry_profile']}
        records.append(dict(received_ns=at,stamp_ns=source,**record))
    node.create_subscription(TwistStamped if kind=='twist' else String, STREAMS[kind], receive, 32)
    (out/(kind+'.ready')).touch()
    while not (out/'done').exists():
        rclpy.spin_once(node,timeout_sec=.05)
    (out/(kind+'.json')).write_text(json.dumps(records))


def measure(args,node):
    out=args.out
    out.mkdir(parents=True,exist_ok=False)
    root=args.data
    archive=json.loads((root/'fixture.json').read_bytes())
    frames=archive['frames'][:args.seconds*5]
    if len(frames)<args.seconds*5:
        raise ValueError('Need enough unique chronological archives for the entire run')
    images=[]
    for row in frames:
        path=root/row['file']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:
            raise ValueError('Archive hash mismatch')
        bgr=cv2.imread(str(path))
        images.append(Image(height=360,width=640,encoding='bgr8',step=1920,data=bgr.tobytes()))
    profile=json.loads((root/'virtual-camera.json').read_bytes())
    camera=node.create_publisher(Image,TOPIC,qos_profile_sensor_data)
    info_pub=node.create_publisher(CameraInfo,'/aiformula_sensing/zed_node/left/camera_info',qos_profile_sensor_data)
    wheel=node.create_publisher(Frame,'/aiformula_sensing/vehicle_info',1)
    gyro=node.create_publisher(Imu,'/aiformula_sensing/vectornav/imu',1)
    state=[]
    node.create_subscription(String,'/concept/vehicle_state',lambda m:state.append(json.loads(m.data)),1)
    info=CameraInfo(height=360,width=640,p=list(map(float,profile['projection'])))
    info.k=[float(profile['projection'][i]) for i in (0,1,2,4,5,6,8,9,10)]
    info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
    info.distortion_model='plumb_bob'
    info.d=[0.]*5
    info.header.frame_id=profile['frame_id']
    def feedback(i):
        t=node.get_clock().now().nanoseconds
        s=frames[i]['sensor']
        if args.feedback=='archive':
            r,l=round(s['rightRpm']),round(s['leftRpm'])
            if not 0<=min(r,l)<=max(r,l)<=255:raise ValueError('0x711 cannot encode this recorded RPM')
            g=-s['gyro']
        else:
            r=l=65
            g=0.
        imu=Imu()
        imu.header.frame_id='bench_imu_link'
        imu.header.stamp=stamp(t)
        imu.angular_velocity.z=float(g)
        gyro.publish(imu)
        msg=Frame(id=0x711,dlc=8,data=[r,0,0,0,l,0,0,0])
        msg.header.stamp=stamp(t+1)
        wheel.publish(msg)
        info.header.stamp=stamp(t)
        info_pub.publish(info)
    def isolated():
        names=node.get_node_names_and_namespaces()
        if any('socket_can_sender' in name for name,_ in names):
            raise RuntimeError('Physical sender present')
        physical='/aiformula_control/motor_controller/reference_signal'
        if node.count_publishers(physical) or node.count_subscribers(physical):raise RuntimeError('Physical CAN graph present')
        return ('virtual_receiver','/concept/replay') in names
    processes=[subprocess.Popen([sys.executable,__file__,'--watch',k,'--out',str(out)]) for k in STREAMS]
    try:
        until=time.monotonic()+50
        while time.monotonic()<until:
            feedback(0)
            rclpy.spin_once(node,timeout_sec=.025)
            if (isolated() and camera.get_subscription_count() and info_pub.get_subscription_count()
                    and all((out/(k+'.ready')).exists() and node.count_publishers(v) for k,v in STREAMS.items())
                    and state and state[-1]['valid']):break
        else:raise RuntimeError('Replay graph, feedback, or calibration not ready')
        sent=[]
        started=time.monotonic_ns()+300_000_000
        seq=0
        while seq < args.seconds*args.camera_hz:
            planned=started+seq*BILLION//args.camera_hz
            remaining=planned-time.monotonic_ns()
            if remaining>0: time.sleep(remaining/BILLION)
            actual=time.monotonic_ns()
            if actual>=planned+BILLION//args.camera_hz:
                seq+=1
                continue
            i=seq*5//args.camera_hz
            feedback(i)
            actual=time.monotonic_ns()
            t=node.get_clock().now().nanoseconds
            image=images[i]
            image.header.frame_id=profile['frame_id']
            image.header.stamp=stamp(t)
            camera.publish(image)
            sent.append(dict(seq=seq,archive_frame=frames[i]['archive_frame'],planned_ns=planned,
                             capture_mono_ns=actual,stamp_ns=t,publish_done_ns=time.monotonic_ns()))
            seq+=1
        (out/'sent.json').write_text(json.dumps(sent))
        for _ in range(35):
            feedback(len(frames)-1)
            time.sleep(.05)
        assert isolated()
    finally:
        (out/'done').touch()
        for proc in processes:
            if proc.wait(timeout=15):raise RuntimeError('Collector failed')
    summary(args,sent,started,out)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--watch',choices=list(STREAMS))
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--rescore-from',type=Path,help='Score recorded receptions against another fixed slot grid')
    parser.add_argument('--data',type=Path)
    parser.add_argument('--seconds',type=int,default=60)
    parser.add_argument('--decision-hz',type=int,choices=(5,15),default=5)
    parser.add_argument('--camera-hz',type=int,choices=(15,),default=15)
    parser.add_argument('--feedback',choices=('bench','archive'),default='bench')
    args=parser.parse_args()
    if args.rescore_from:
        rescore(args,args.rescore_from,args.out)
        return
    if os.environ.get('ROS_DOMAIN_ID') not in ('77','78') or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Requires isolated, localhost-only domain 77 or 78')
    rclpy.init()
    node=Node('meter_'+(args.watch or 'camera'))
    try:
        if node.get_parameter('use_sim_time').value:raise RuntimeError('Real clock required')
        if args.watch:collect(args.watch,args.out,node)
        else:measure(args,node)
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
