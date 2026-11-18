"""Exercise an already running replay launch in dedicated localhost-only domain 76."""
import argparse
import json
import os
import time

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from can_msgs.msg import Frame
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('camera_png')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '76' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Use isolated localhost-only domain 76')
    bridge = CvBridge()
    bgr = cv2.imread(args.camera_png)
    assert bgr is not None and bgr.shape == (360,640,3)
    rclpy.init()
    node = Node('concept_camera_replay_check')
    clock = node.create_publisher(Clock, '/clock', 1)
    can = node.create_publisher(Frame, '/aiformula_sensing/vehicle_info', 1)
    gyro = node.create_publisher(Imu, '/aiformula_sensing/vectornav/imu', 1)
    camera = node.create_publisher(Image, '/aiformula_sensing/zed_node/left_image/undistorted', qos_profile_sensor_data)
    decisions, selected, memory, images, auto = [], [], [], [], []
    states = []
    node.create_subscription(String, '/concept/decision', lambda m: decisions.append(json.loads(m.data)), 1)
    node.create_subscription(String, '/concept/vehicle_state', lambda m: states.append(json.loads(m.data)), 1)
    node.create_subscription(TwistStamped, '/concept/auto_cmd', auto.append, 1)
    node.create_subscription(MarkerArray, '/concept/view/local', selected.append, 2)
    node.create_subscription(MarkerArray, '/concept/view/map', memory.append, 2)
    node.create_subscription(Image, '/concept/view/camera', images.append, qos_profile_sensor_data)
    name = '/aiformula_control/motor_controller/reference_signal'
    def isolated():
        names = {ns.rstrip('/')+'/'+n for n,ns in node.get_node_names_and_namespaces()}
        if any('socket_can_sender' in n for n in names) or node.count_publishers(name) or node.count_subscribers(name):
            raise RuntimeError('Physical sender or command topic detected')
        return '/concept/replay/virtual_receiver' in names
    sent = None
    chosen_pixels = 0
    ns = int(time.time())*10**9
    # Keep simulated sensor time advancing through slow CPU inference without
    # weakening the driver's production capture-age checks.
    start = time.monotonic()
    try:
        while time.monotonic()-start < 75:
            ok = isolated()
            ns += 1_000_000
            stamp = Time(sec=ns//10**9,nanosec=ns%10**9)
            clock.publish(Clock(clock=stamp))
            time.sleep(.01)
            imu = Imu()
            imu.header.frame_id = 'imu_link'
            imu.header.stamp = stamp
            gyro.publish(imu)
            frame = Frame(id=0x711,dlc=8,data=[0]*8)
            frame.header.stamp = stamp
            can.publish(frame)
            for _ in range(8):
                rclpy.spin_once(node,timeout_sec=.002)
            if (sent is None and ok and states and states[-1].get('valid') and
                    camera.get_subscription_count() and node.count_subscribers('/concept/perception/frame')
                    and node.count_publishers('/concept/view/local') and time.monotonic()-start > 3):
                image = bridge.cv2_to_imgmsg(bgr,encoding='bgr8')
                image.header.frame_id = 'replay_uncalibrated_camera'
                image.header.stamp = stamp
                camera.publish(image)
                sent = ns
            if decisions and decisions[-1].get('status') == 'invalid':
                raise AssertionError(decisions[-1].get('reason'))
            if decisions and selected and memory and images:
                pixels = bridge.imgmsg_to_cv2(images[-1],desired_encoding='bgr8')
                chosen_pixels = int(np.all(pixels==[170,72,250],axis=2).sum())
                if chosen_pixels:
                    break
        assert sent and decisions and selected and memory and images, 'Replay output missing'
        d = decisions[-1]
        assert d['status']=='shadow' and d['selected_path'] and d['feasible_count'] > 0, (d['status'],d['reason'])
        assert d['stamp']==dict(sec=sent//10**9,nanosec=sent%10**9)
        choices = [m for m in selected[-1].markers if m.ns=='selected']
        assert len(choices)==1 and len(choices[0].points)==len(d['selected_path'])
        map_points=sum(len(m.points) for m in memory[-1].markers if m.ns.startswith('memory_'))
        assert map_points and chosen_pixels and not auto and isolated()
        print(json.dumps(dict(status=d['status'],reason=d['reason'],candidates=len(d['candidates']),
             feasible=d['feasible_count'],selected=d['selected_index'],selected_points=len(choices[0].points),
             map_points=map_points,chosen_pixels=chosen_pixels,auto_messages=len(auto),
             source_stamp=d['stamp'],processing_ms=d['processing_ms'])),flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
