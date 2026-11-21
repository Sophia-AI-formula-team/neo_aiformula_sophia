"""Drive OA and the installed lane/OA selector with isolated ROS messages."""

from datetime import datetime
import json
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D, PoseArray
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from fresh_oa_switch import FreshOaSwitch, GROUP_TOPIC, OA_READY_TOPIC
from oa_2_3 import ConeMaskAvoider

MANUAL_TOPIC = '/aiformula_control/motor_controller/gate_status'
MAPASSIST = '/semantic_drive/map_assist_045'


def report(message):
    print(f'{datetime.now().astimezone().isoformat()} {message}', flush=True)


def main(variant):
    rclpy.init(args=[
        '--ros-args', '-r', '/lane_stop_flag:=/point_flag',
        '-r', 'oa_2_3:/processed_point_a:=/oa_point_a',
        '-r', 'oa_2_3:/processed_point_b:=/oa_point_b',
        '-r', 'oa_2_3:/processed_point_c:=/oa_point_c',
        '-r', f'data_processing_node:/processed_point_a:={MAPASSIST}/point_a',
        '-r', f'data_processing_node:/processed_point_b:={MAPASSIST}/point_b',
        '-r', f'data_processing_node:/processed_point_c:={MAPASSIST}/point_c',
        '-r', f'data_processing_node:/filtered_lane_pose:={MAPASSIST}/filtered_lane_pose',
        '-p', f'manual_state_topic:={MANUAL_TOPIC}',
        '-p', 'triplet_period:=10.0',
    ])
    oa = ConeMaskAvoider()
    selector = FreshOaSwitch()
    io = rclpy.create_node('oa_clear_release_test')
    executor = SingleThreadedExecutor()
    for node in (oa, selector, io):
        executor.add_node(node)

    flags, readiness, groups, poses = [], [], [], []
    io.create_subscription(Bool, '/point_flag', lambda msg: flags.append(msg.data), 10)
    io.create_subscription(Bool, OA_READY_TOPIC, lambda msg: readiness.append(msg.data), 10)
    io.create_subscription(PoseArray, GROUP_TOPIC, groups.append, 10)
    io.create_subscription(Pose2D, MAPASSIST + '/filtered_lane_pose', poses.append, 10)
    manual_pub = io.create_publisher(String, MANUAL_TOPIC, 1)
    mask_pub = io.create_publisher(Image, '/cone_mask', qos_profile_sensor_data)
    image_pub = io.create_publisher(Image, '/cone_bbox_image', 10)
    odom_pub = io.create_publisher(Odometry, oa.odom_topic, 10)
    lanes = [io.create_publisher(Pose2D, f'{MAPASSIST}/point_{key}', 10) for key in 'abc']
    empty = np.zeros((160, 200), dtype=np.uint8)
    tiny_obstacle = empty.copy()
    tiny_obstacle[130, 100] = 255

    def wait_for(predicate, detail, timeout=4.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), detail

    def drain(seconds=0.04):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)

    def manual(held, transition=None):
        now = io.get_clock().now().nanoseconds
        transition = now if transition is None else transition
        manual_pub.publish(String(data=json.dumps({
            'manual_held': held, 'manual_transition_ns': transition, 'stamp_ns': now,
        })))
        wait_for(lambda: oa.manual_transition_ns == selector.manual_transition_ns == transition,
                 'manual transition did not reach both nodes')
        return transition

    def inputs(y=0.0):
        now = io.get_clock().now()
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'vehicle'
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0
        image = oa.bridge.cv2_to_imgmsg(np.zeros((*empty.shape, 3), np.uint8), 'bgr8')
        image.header.stamp = now.to_msg()
        odom_pub.publish(msg)
        image_pub.publish(image)
        wait_for(lambda: oa.odom is not None and oa.odom[1] == y
                 and oa.latest_vis_hdr is not None
                 and oa.latest_vis_hdr.stamp == image.header.stamp,
                 'fresh odometry and image did not reach OA')

    def mask(array=empty, stamp=None, accepted=True):
        saved = oa.mask_sequence
        msg = oa.bridge.cv2_to_imgmsg(array, 'mono8')
        msg.header.stamp = stamp or io.get_clock().now().to_msg()
        mask_pub.publish(msg)
        if accepted:
            wait_for(lambda: oa.mask_sequence == saved + 1, 'fresh mask was not consumed')
        else:
            drain()
            assert oa.mask_sequence == saved, 'old or duplicate mask was consumed'
        return msg.header.stamp

    def activate():
        oa.counted_mask_sequence = oa.mask_sequence
        oa.lane_forward = False
        oa.execution_state = 'tracking_avoidance'
        oa.world_path = np.array([[0.2, 0.0], [0.4, 0.0], [0.6, 0.0]])
        oa.current_path = oa.world_path.copy()
        oa.path_cumulative = np.array([0.0, 0.2, 0.4])
        oa.path_progress_m = 0.4
        oa.remaining_m = 0.0
        oa.path_frame = oa.odom[3]
        oa.path_peak_index = 1
        oa.last_window_world = oa.world_path.copy()
        oa.last_plan_sequence = oa.mask_sequence
        oa.last_plan_ns = oa.get_clock().now().nanoseconds
        oa.planned_mask = empty.copy()
        oa._selected_plan = object()
        oa._preview_targets = oa.world_path.copy()
        oa._candidate_side = oa.avoid_direction = oa.vote_side = oa.pending_side = 1
        oa.vote_count = oa.pending_frames = 2
        oa.force_direction_review = True
        oa.timer_handle = oa.create_timer(oa.triplet_period, oa.avoidance_step)
        oa._publish_avoidance_flag()
        wait_for(lambda: selector.current_source == 'oa' and flags and flags[-1] is True,
                 'selector did not take OA points')

    def assert_reset(reason):
        assert oa.execution_state == reason, oa.execution_state
        assert oa.lane_forward and oa.timer_handle is None
        assert not oa.obstacle_cleared and oa.clear_frames == 0
        assert oa.remaining_m is None and oa.path_progress_m == 0.0 and len(oa.current_path) == 0
        for name in ('world_path', 'path_cumulative', 'path_frame', 'path_peak_index',
                     'last_window_world', 'planned_mask', '_selected_plan', '_preview_targets',
                     '_candidate_side', 'avoid_direction', 'vote_side', 'pending_side'):
            assert getattr(oa, name) is None, name
        assert oa.vote_count == oa.pending_frames == 0 and not oa.force_direction_review
        wait_for(lambda: selector.current_source == 'processed' and flags[-1] is False
                 and readiness and readiness[-1] is False, 'selector did not return to lane points')
        assert selector.A is None and selector.B is None and selector.C is None

    def lane_output():
        before = len(poses)
        wait_for(lambda: all(pub.get_subscription_count() for pub in lanes),
                 'lane DDS subscriptions did not return')
        lanes[0].publish(Pose2D(x=2.0, y=0.1))
        lanes[1].publish(Pose2D(x=3.0, y=0.1))
        wait_for(lambda: selector.A is not None and selector.B is not None,
                 'selector did not accept new lane A and B')
        lanes[2].publish(Pose2D(x=4.0, y=0.1))
        wait_for(lambda: len(poses) > before and poses[-1].x == 2.0,
                 'fresh lane ABC did not produce a filtered lane pose')

    try:
        wait_for(lambda: manual_pub.get_subscription_count() == 2
                 and all(pub.get_subscription_count() for pub in (mask_pub, image_pub, odom_pub)),
                 'fixture DDS subscriptions did not discover all inputs')
        manual(False, 0)
        inputs(y=4.0)
        mask()
        activate()
        oa._publish_window_unchecked(oa.world_path)
        wait_for(lambda: readiness and readiness[-1] is True and groups,
                 'selector did not accept active OA ABC')
        previous_groups = len(groups)
        first = mask()
        mask(stamp=first, accepted=False)
        mask()
        oa.avoidance_step()
        assert not oa.lane_forward and oa.clear_frames == 2, (
            oa.lane_forward, oa.clear_frames, oa.execution_state,
        )
        assert oa.execution_state == 'path_tracking_error', oa.execution_state
        mask()
        assert oa.obstacle_cleared
        oa.avoidance_step()

        if variant == 'before':
            assert oa.obstacle_cleared and oa.remaining_m == 0.0
            assert oa.execution_state == 'path_tracking_error'
            assert not oa.lane_forward and flags[-1] is True and selector.current_source == 'oa'
            report('baseline reproduced: 3 clear masks, remaining 0 m, tracking error, flag still true')
            return

        assert_reset('obstacle_clear')
        drain()
        assert len(groups) == previous_groups, 'old OA ABC was republished after clear'
        lane_output()
        report('three distinct clear masks released OA despite 4 m old-path error; fresh lane ABC produced LYA input')

        inputs()
        mask()
        activate()
        for _ in range(3):
            mask(tiny_obstacle)
        assert oa.obstacle_cleared
        assert oa._path_blocked(tiny_obstacle)
        oa.avoidance_step()
        assert not oa.lane_forward and oa.clear_frames == 0 and not oa.obstacle_cleared
        report('one remaining pixel on the path vetoed release while total pixels were below threshold')

        inputs()
        for _ in range(3):
            mask()
        assert oa.obstacle_cleared
        oa.odom_received_ns -= 1_000_000_000
        oa.avoidance_step()
        assert not oa.lane_forward and oa.clear_frames == 0 and oa.execution_state == 'odom_stale'
        report('stale odometry invalidated clearance votes')

        press = manual(True)
        assert_reset('manual_takeover')
        assert oa.latest_mask is None and not oa.manual_vision_ready
        mask(accepted=False)
        release = manual(False)
        assert release > press
        assert_reset('manual_waiting_fresh_mask')
        old_stamp = rclpy.time.Time(nanoseconds=release - 1).to_msg()
        mask(stamp=old_stamp, accepted=False)
        assert oa.latest_mask is None and not oa.manual_vision_ready
        inputs()
        mask()
        wait_for(lambda: oa.manual_vision_ready and oa.execution_state == 'idle',
                 'manual release did not start a fresh lane session')
        lane_output()
        report('manual press reset OA, rejected masks while held and from before release, then restored fresh lane ABC')

        activate()
        executor.remove_node(oa)
        command = [
            sys.executable, '-c',
            'import sys; from run_obstacle_avoidance import run_child; '
            'sys.exit(run_child([sys.executable, "-c", "pass"]))',
        ]
        with subprocess.Popen(command) as runner:
            wait_for(lambda: flags[-1] is False and selector.current_source == 'processed',
                     'OA exit wrapper did not restore lane selection')
            assert runner.wait(timeout=4.0) == 0
        lane_output()
        report('OA runner restored lane selection after process exit')
    finally:
        for node in (oa, selector, io):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1])
