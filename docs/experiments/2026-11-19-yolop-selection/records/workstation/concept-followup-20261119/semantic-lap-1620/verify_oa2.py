"""Verify the installed OA and MapAssist topic connection using synthetic images."""
from collections import Counter
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String

assert os.environ.get('ROS_LOCALHOST_ONLY') == '1' and os.environ.get('ROS_DOMAIN_ID') == '196'
prefix = '/semantic_drive/map_assist_045'
rclpy.init()
node = rclpy.create_node('mapassist_oa2_isolated_probe')
bridge = CvBridge()
odom_pub = node.create_publisher(Odometry, '/aiformula_sensing/gyro_odometry_publisher/odom', 10)
image_pub = node.create_publisher(Image, '/cone_bbox_image', 10)
mask_pub = node.create_publisher(Image, '/cone_mask', 10)
lane_pubs = [node.create_publisher(Pose2D, '/processed_point_' + key, 10) for key in 'abc']
image = np.zeros((360, 640, 3), np.uint8)
cone = np.zeros((360, 640), np.uint8)
cv2.rectangle(cone, (282, 269), (329, 319), 255, -1)
streams = {}
for name, cls in [
        ('/point_flag', Bool), ('/lane_stop_flag', Bool), ('/cone_pixels_count', Int32),
        *(('/oa_point_' + k, Pose2D) for k in 'abc'),
        (prefix + '/filtered_lane_pose', Pose2D), (prefix + '/status', String)]:
    streams[name] = []
    node.create_subscription(cls, name, lambda msg, topic=name: streams[topic].append(msg), 10)


def pump(seconds, obstacle):
    end = time.monotonic() + seconds
    next_at = 0.
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= next_at:
            odom = Odometry()
            odom.header.stamp = node.get_clock().now().to_msg()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
            odom.pose.pose.orientation.w = 1.
            odom_pub.publish(odom)
            image_pub.publish(bridge.cv2_to_imgmsg(image, 'bgr8'))
            mask_pub.publish(bridge.cv2_to_imgmsg(cone if obstacle else np.zeros_like(cone), 'mono8'))
            for i, pub in enumerate(lane_pubs):
                pub.publish(Pose2D(x=1.+i, y=.15))
            next_at = now + .10
        rclpy.spin_once(node, timeout_sec=.02)


def names(topic):
    return sorted(i.node_name for i in node.get_publishers_info_by_topic(topic))


def flags(topic):
    return dict(Counter(str(m.data) for m in streams[topic]))


def states(flag):
    return sum(json.loads(m.data).get('avoidance_active') is flag for m in streams[prefix + '/status'])


def control_chain_absent():
    assert not node.count_publishers('/aiformula_control/motor_controller/reference_signal')
    assert not node.count_publishers('/aiformula_control/game_pad/cmd_vel')
    assert not node.count_publishers('/semantic_drive/motor_session')


def run_oa(remapped):
    for frames in streams.values():
        frames.clear()
    args = ['ros2', 'run', 'obsticle_avoidence', 'oa_2']
    if remapped:
        args += ['--ros-args', '-r', '/lane_stop_flag:=/point_flag']
        for k in 'abc':
            args += ['-r', '/processed_point_' + k + ':=/oa_point_' + k]
    log = Path('oa-isolated-' + ('mapped' if remapped else 'raw') + '.log')
    with log.open('w') as fh:
        proc = subprocess.Popen(args, stdout=fh, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            pump(2., False)
            assert proc.poll() is None, log.read_text()
            control_chain_absent()
            pump(3., True)
            result = dict(roi_pixels=max((m.data for m in streams['/cone_pixels_count']), default=0),
                          lane_flag=flags('/lane_stop_flag'), point_flag=flags('/point_flag'),
                          oa_triplets=[len(streams['/oa_point_' + k]) for k in 'abc'],
                          lane_a_publishers=names('/processed_point_a'),
                          mapassist_avoidance_samples=states(True),
                          selected_oa_pose_samples=sum(.25 < m.x < .35 for m in streams[prefix + '/filtered_lane_pose']))
            assert result['roi_pixels'] > 1000, result
            if remapped:
                assert result['point_flag'].get('True', 0) and all(result['oa_triplets']), result
                assert result['mapassist_avoidance_samples'] and result['selected_oa_pose_samples'], result
                for frames in streams.values():
                    frames.clear()
                pump(3., False)
                result['returned_to_lane'] = (flags('/point_flag').get('False', 0)>0 and states(False)>0
                                               and any(abs(m.x - 1.)<1e-6 for m in streams[prefix + '/filtered_lane_pose']))
                assert result['returned_to_lane'], result
            else:
                assert result['lane_flag'].get('True', 0) and not result['point_flag'], result
                assert not result['mapassist_avoidance_samples'] and not any(result['oa_triplets']), result
                assert 'cone_mask_avoider' in result['lane_a_publishers'], result
            return result
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=3)


try:
    pump(3., False)
    assert states(False)>0, 'No initial MapAssist status'
    control_chain_absent()
    print(json.dumps(dict(unmodified_shortcut=run_oa(False), explicit_remaps=run_oa(True),
                          physical_can_publishers=0, domain=196), indent=2))
finally:
    node.destroy_node()
    rclpy.shutdown()
