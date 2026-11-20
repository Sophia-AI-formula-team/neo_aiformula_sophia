"""Foxy smoke test of the normal OA 1B perception CLI, its stock topics and its private branch."""
import argparse
import gzip
import json
import signal
import subprocess
import sys
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String


PER='/aiformula_perception'
RAW='/aiformula_sensing/zed_node/left_image/undistorted'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('weights',type=Path)
    args=parser.parse_args()
    with gzip.open(args.run/'trial/topic-events.jsonl.gz','rt') as stream:
        frames=[v for _,k,v in map(json.loads,stream) if k=='detector_frame']
    sample=next(v for v in frames if v['red_pixels_count']==50 and len(v['detections'])>=2)
    h=sample['header']['stamp']
    image=cv2.imread(str(args.run/'raw'/('%d.png'%(h['sec']*1000+h['nanosec']//10**6))))
    rclpy.init()
    probe=Node('oa1b_perception_probe')
    raw=probe.create_publisher(Image,RAW,3)
    odom=probe.create_publisher(Odometry,'/aiformula_sensing/gyro_odometry_publisher/odom',5)
    status=probe.create_publisher(String,'/semantic_drive/map_assist_045/status',5)
    observed={}
    topics={'stock':(String,PER+'/yolop_a3/detections'),'new':(String,'/oa_1b/perception/detections'),
            'lane':(Image,PER+'/road_detector/mask_image'),'cone':(Image,'/cone_mask'),
            'light':(String,PER+'/yolop_a3/traffic_light/state'),'private_light':(String,'/oa_1b/perception/traffic_light/state'),
            'pixels':(Int32,PER+'/yolop_a3/red_pixels_count')}
    for key,(cls,name) in topics.items():
        probe.create_subscription(cls,name,lambda m,k=key:observed.__setitem__(k,m),5)
    cmd=[sys.executable,'-m','oa_1b.perception','--ros-args','-r','__ns:='+PER,
         '-p','weights:='+str(args.weights),'-p','device:=cuda',
         '-r','pub_mask_image:='+PER+'/road_detector/mask_image',
         '-r','/traffic_light/state:='+PER+'/yolop_a3/traffic_light/state',
         '-r','/red_pixels_count:='+PER+'/yolop_a3/red_pixels_count']
    process=subprocess.Popen(cmd)
    try:
        deadline=time.monotonic()+35
        bridge=CvBridge()
        while time.monotonic()<deadline:
            if process.poll() is not None:
                raise RuntimeError('perception process exited: '+str(process.returncode))
            if probe.count_subscribers(RAW)==1 and probe.count_publishers('/oa_1b/perception/detections')==1:
                now=probe.get_clock().now().to_msg()
                packet=bridge.cv2_to_imgmsg(image,'bgr8')
                packet.header.frame_id='zed_left_camera_optical_frame';packet.header.stamp=now
                sample_odom=Odometry();sample_odom.header.stamp=now;odom.publish(sample_odom)
                status.publish(String(data=json.dumps(dict(traffic_mode='normal'))))
                raw.publish(packet)
            for _ in range(5):
                rclpy.spin_once(probe,timeout_sec=.04)
            if set(observed)==set(topics):
                stock=json.loads(observed['stock'].data)
                fixed=json.loads(observed['new'].data)
                if stock['header']!=fixed['header']:
                    continue
                assert stock['traffic_light_state']==sample['traffic_light_state']=='red'
                assert fixed['traffic_light_state']=='red'
                assert observed['light'].data==observed['private_light'].data=='red'
                assert stock['red_pixels_count']==observed['pixels'].data==50 and fixed['red_pixels_count']==25
                assert (observed['lane'].width,observed['lane'].height)==(640,360)
                assert observed['lane'].encoding==observed['cone'].encoding=='mono8'
                print(json.dumps({'camera_subscribers':1, 'stock_red_pixels':50,'private_unique_pixels':25,
                                  'received':sorted(observed),'stamp':stock['header']['stamp']}),flush=True)
                break
        else:
            raise TimeoutError('Missing DDS outputs: '+str(set(topics)-set(observed)))
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill();process.wait();raise
        probe.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
