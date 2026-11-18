"""Measure the deployed graph against wall time in an isolated replay domain."""
import argparse
from bisect import bisect_right
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np
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


def pct(values):
    if not values:
        return None
    return dict(zip(('p50','p95','p99','max'),np.percentile(values,[50,95,99,100]).round(3).tolist()))


def summary(args, sent, started, out, record_dir=None):
    data={k:json.loads(((record_dir or out)/(k+'.json')).read_text()) for k in STREAMS}
    by_stamp={r['stamp_ns']:r for r in sent}
    def slot(record):
        return by_stamp[record['stamp_ns']]['seq']*args.decision_hz//args.camera_hz
    end_ns = started + args.seconds*BILLION
    valid = [r for r in data['twist'] if r['stamp_ns'] in by_stamp]
    earliest = {}
    for r in valid:
        key = slot(r)
        if key not in earliest or r['received_ns']<earliest[key]['received_ns']:
            earliest[key]=r
    expected = args.seconds*args.decision_hz
    spacings = np.diff([r['received_ns'] for r in valid]).tolist()
    stop_gaps = np.diff([started]+[r['received_ns'] for r in valid if started<=r['received_ns']<=end_ns]+[end_ns])
    deadline_count=sum(r['received_ns']<=started+(k+1)*BILLION//args.decision_hz for k,r in earliest.items())
    latency=[(r['received_ns']-by_stamp[r['stamp_ns']]['capture_mono_ns'])/1e6 for r in valid]
    within200=sum(r['received_ns']-by_stamp[r['stamp_ns']]['capture_mono_ns']<=200_000_000 for r in earliest.values())
    diagnostics=[r for r in data['decision'] if r['stamp_ns'] in by_stamp]
    stationary=[r for r in valid if r['cmd'][0]==0]
    nonshadow={r['stamp_ns'] for r in diagnostics if r['status']=='ok'}
    result=dict(camera_hz=args.camera_hz,decision_hz=args.decision_hz,seconds=args.seconds,
        planned_frames=args.seconds*args.camera_hz,sent_frames=len(sent),
        unique_archived_images=len({r['archive_frame'] for r in sent}),archive_hz=5,feedback=args.feedback,
        publish_lag_ms=pct([(r['capture_mono_ns']-r['planned_ns'])/1e6 for r in sent]),
        perception_frames=len({r['stamp_ns'] for r in data['perception'] if r['stamp_ns'] in by_stamp}),
        decision_frames=len(diagnostics),expected_slots=expected,distinct_output_slots=len(earliest),
        raw_matching_twists=len(valid),twists_with_matching_ok_decision=sum(r['stamp_ns'] in nonshadow for r in valid),
        zero_linear_outputs=len(stationary),
        nonzero_linear_outputs=sum(r['cmd'][0]!=0 for r in valid),
        on_slot_deadline=deadline_count,late_slots=len(earliest)-deadline_count,
        within_200ms_capture=within200,missing_slots=expected-len(earliest),
        latency_ms=pct(latency),output_interval_ms=pct([x/1e6 for x in spacings]),
        longest_gap_including_run_edges_ms=round(max(stop_gaps)/1e6,3),
        pauses_over_200ms=int(sum(stop_gaps>200_000_000)),status_reason=dict(Counter(
            str(r['status'])+' / '+str(r['reason']) for r in diagnostics)),
        first_output_ms=round((valid[0]['received_ns']-started)/1e6,3) if valid else None,
        policy_processing_ms=pct([r['processing_ms'] for r in diagnostics if r['processing_ms'] is not None]))
    rows=[]
    seen={r['stamp_ns'] for r in data['perception']}
    for k in range(expected):
        inputs=[r for r in sent if r['seq']*args.decision_hz//args.camera_hz==k]
        captures={r['stamp_ns'] for r in inputs}
        reasoning=[r for r in diagnostics if r['stamp_ns'] in captures]
        cmd=earliest.get(k)
        due=started+(k+1)*BILLION//args.decision_hz
        rows.append(dict(slot=k,input_ids=[r['seq'] for r in inputs],perceived=len(captures&seen),
            decision_count=len(reasoning),reasons=[r['reason'] for r in reasoning],has_output=cmd is not None,
            on_deadline=bool(cmd and cmd['received_ns']<=due),
            latency_ms=None if cmd is None else (cmd['received_ns']-by_stamp[cmd['stamp_ns']]['capture_mono_ns'])/1e6))
    def portion(items):
        delays=[r['latency_ms'] for r in items if r['has_output']]
        return dict(expected=len(items),received=len(delays),on_deadline=sum(r['on_deadline'] for r in items),
                    within_200ms=sum(t<=200 for t in delays),latency_ms=pct(delays))
    first=min(expected,5*args.decision_hz)
    result['cold_first_5s']=portion(rows[:first])
    result['following_slots']=portion(rows[first:])
    # The production driver anchors its period to the first accepted camera stamp.
    # Keep the exogenous source grid and report its phase separately.
    camera_slots_per_decision=args.camera_hz//args.decision_hz
    result['output_source_phase_counts']=dict(Counter(
        by_stamp[r['stamp_ns']]['seq'] % camera_slots_per_decision for r in earliest.values()))
    result['distinct_receive_slots']=len({
        (r['received_ns']-started)*args.decision_hz//BILLION for r in valid
        if started<=r['received_ns']<end_ns})
    result['receive_slots_with_capture_age_at_most_200ms']=len({
        (r['received_ns']-started)*args.decision_hz//BILLION for r in valid
        if started<=r['received_ns']<end_ns
        and r['received_ns']-by_stamp[r['stamp_ns']]['capture_mono_ns']<=200_000_000})
    result['duplicate_output_stamps']=len(valid)-len({r['stamp_ns'] for r in valid})
    # Exact minimum over all one-second (left, right] windows fully inside the run.
    # The count can decrease only when a reception crosses the left edge.
    receipts=sorted(r['received_ns'] for r in valid if started<=r['received_ns']<=end_ns)
    edges=[started,end_ns-BILLION]+[t for t in receipts if t<=end_ns-BILLION]
    result['min_twists_in_sliding_1s']=min(
        bisect_right(receipts,t+BILLION)-bisect_right(receipts,t) for t in edges)
    result['missing_slot_evidence']=dict(Counter(
        ('capture_not_sent' if not r['input_ids'] else 'no_perception_observed' if not r['perceived']
         else 'no_decision_observed' if not r['decision_count'] else 'no_twist_despite_decision')
        for r in rows if not r['has_output']))
    result['capture_plan_to_receive_ms']=pct([(r['received_ns']-by_stamp[r['stamp_ns']]['planned_ns'])/1e6 for r in valid])
    (out/'slots.json').write_text(json.dumps(rows))
    (out/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2),flush=True)


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
        recorded=json.loads((args.rescore_from/'summary.json').read_text())
        args.seconds,args.camera_hz,args.feedback=(recorded[k] for k in ('seconds','camera_hz','feedback'))
        sent=json.loads((args.rescore_from/'sent.json').read_text())
        if not sent:raise ValueError('Cannot reconstruct the acquisition grid without any sent images')
        started=sent[0]['planned_ns']-sent[0]['seq']*BILLION//args.camera_hz
        args.out.mkdir(parents=True,exist_ok=False)
        summary(args,sent,started,args.out,args.rescore_from)
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
