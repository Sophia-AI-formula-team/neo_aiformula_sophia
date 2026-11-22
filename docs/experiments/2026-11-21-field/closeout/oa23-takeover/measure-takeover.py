"""Exercise the frozen OA and selector in an isolated ROS graph."""

from collections import Counter
from datetime import datetime
import argparse
import json
import multiprocessing
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String

from fresh_oa_switch import FreshOaSwitch
from oa_2_3 import ConeMaskAvoider

PERIOD = .10
SHAPE = (360, 640)
FRAME_ID = 'zed_left_camera_optical_frame'
MOTOR = '/aiformula_control/motor_controller/gate_status'
ROI = '/aiformula_perception/lane_line_publisher/dynamic_roi_mask'
STEPS = (600, 800, 950, 1000, 1100, 1300, 1450, 1600, 1800)

def mask_of_area(area):
    mask = np.zeros(SHAPE, dtype=np.uint8)
    row, col = np.divmod(np.arange(area), 53)
    mask[row + 245, col + 361] = 255
    return mask


def place_near_roi_top(row):
    mask = np.zeros(SHAPE, dtype=np.uint8)
    mask[row:row + 24, 361:414] = 255
    return mask


class TimedOA(ConeMaskAvoider):
    def __init__(self):
        self.ownership_times = []
        self.window_times = []
        super().__init__()

    def _publish_avoidance_flag(self):
        super()._publish_avoidance_flag()
        if not self.lane_forward:
            self.ownership_times.append(time.monotonic())

    def _publish_window_unchecked(self, points):
        super()._publish_window_unchecked(points)
        self.window_times.append(time.monotonic())


class TimedSelector(FreshOaSwitch):
    def __init__(self):
        self.ready_times = []
        self.discards = Counter()
        super().__init__()

    def group_callback(self, msg):
        previous = self.last_stamp
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        source, switched_at = self.current_source, self.source_changed_at
        super().group_callback(msg)
        if self.last_stamp != previous:
            self.ready_times.append(time.monotonic())
        elif source != 'oa':
            self.discards['selector_still_on_lane'] += 1
        elif stamp <= max(previous, switched_at):
            self.discards['group_predates_source_switch_or_last_group'] += 1
        else:
            self.discards['other_rejection'] += 1


def selector_worker(args, tx, stop):
    rclpy.init(args=args)
    selector = TimedSelector()
    executor = SingleThreadedExecutor()
    executor.add_node(selector)
    try:
        while not stop.is_set():
            executor.spin_once(timeout_sec=.02)
        tx.send((selector.ready_times, dict(selector.discards)))
    finally:
        tx.close()
        executor.remove_node(selector)
        selector.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def run_case(label, threshold, masks, *, roi=None, old_first=0, separate=False):
    args = [
        '--ros-args', '-r', '/lane_stop_flag:=/point_flag',
        '-r', 'oa_2_3:/processed_point_a:=/oa_point_a',
        '-r', 'oa_2_3:/processed_point_b:=/oa_point_b',
        '-r', 'oa_2_3:/processed_point_c:=/oa_point_c',
        '-p', f'cone_thresh_px:={threshold}',
        '-p', f'manual_state_topic:={MOTOR}', '-p', 'oa_health_topic:=/oa/status',
    ]
    selector_child = None
    if separate:
        context = multiprocessing.get_context('spawn')
        rx, tx = context.Pipe(duplex=False)
        stop = context.Event()
        selector_child = context.Process(target=selector_worker, args=(args, tx, stop))
        selector_child.start()
        tx.close()
    rclpy.init(args=args)
    oa = TimedOA()
    switch = None if separate else TimedSelector()
    io = rclpy.create_node('oa_timing_test')
    ex = SingleThreadedExecutor()
    nodes = (oa, io) if separate else (oa, switch, io)
    for node in nodes:
        ex.add_node(node)
    joy = io.create_publisher(String, MOTOR, 10)
    odom_pub = io.create_publisher(Odometry, oa.odom_topic, 10)
    images = {topic: io.create_publisher(Image, topic, qos_profile_sensor_data if topic != '/cone_bbox_image' else 10)
              for topic in ('/cone_bbox_image', '/cone_mask', ROI)}
    synthetic_image = np.zeros((*SHAPE, 3), np.uint8)

    def spin_until(predicate, limit):
        while not predicate() and time.monotonic() < limit:
            ex.spin_once(timeout_sec=.007)
        return predicate()

    def publish_frame(pixels, src_age, expect_accepted):
        t = time.monotonic()
        now = io.get_clock().now()
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'vehicle'
        msg.pose.pose.orientation.w = 1.0
        odom_pub.publish(msg)
        joy.publish(String(data=json.dumps(dict(manual_held=False, manual_transition_ns=0, stamp_ns=now.nanoseconds))))
        for name, image, encoding in (('/cone_bbox_image', synthetic_image, 'bgr8'), (ROI, roi, 'mono8')):
            if image is not None:
                packet = oa.bridge.cv2_to_imgmsg(image, encoding)
                packet.header.stamp = now.to_msg()
                packet.header.frame_id = FRAME_ID
                images[name].publish(packet)
        if roi is not None:
            assert spin_until(lambda: oa.roi_cache and oa.roi_cache[-1][0] == now.nanoseconds,
                              t + .08), label
        packet = oa.bridge.cv2_to_imgmsg(pixels, 'mono8')
        packet.header.stamp = Time(nanoseconds=now.nanoseconds - int(src_age * 1e9)).to_msg()
        packet.header.frame_id = FRAME_ID
        count = oa.mask_sequence
        images['/cone_mask'].publish(packet)
        received = spin_until(lambda: oa.mask_sequence > count, t + .080)
        assert received == expect_accepted, (label, received)
        area = None
        if received:
            oa.h, oa.w, oa.cx = SHAPE[0], SHAPE[1], SHAPE[1] // 2
            area = oa._compute_roi_counts(synthetic_image, pixels)[1]
        spin_until(lambda: False, t + PERIOD)
        return dict(t=t, mask_pixels=int(np.count_nonzero(pixels)), roi_pixels=area,
                    roi_mode=oa.roi_mode, source_age_ms=int(src_age * 1000),
                    state=oa.execution_state, active=not oa.lane_forward,
                    first_window=oa.window_times[0] if oa.window_times else None)

    try:
        assert spin_until(lambda: joy.get_subscription_count() == 2
                          and oa.group_pub.get_subscription_count() >= 1
                          and all(pub.get_subscription_count() for pub in images.values())
                          and odom_pub.get_subscription_count(), time.monotonic() + 5)
        for _ in range(3):
            publish_frame(np.zeros(SHAPE, np.uint8), 0, True)
        rows = []
        for i, pixels in enumerate(masks):
            rows.append(publish_frame(pixels, .65 if i < old_first else 0, i >= old_first))
        if separate or not switch.ready_times:
            for _ in range(3):
                rows.append(publish_frame(masks[-1], 0, True))
        if separate:
            stop.set()
            assert rx.poll(4), 'No data from the selector process'
            ready_times, discards = rx.recv()
        else:
            ready_times, discards = switch.ready_times, dict(switch.discards)
        crossing = next((row['t'] for row in rows if (row['roi_pixels'] or 0) > threshold), None)
        start = rows[0]['t']
        events = {name: times[0] if times else None for name, times in
                  (('flag', oa.ownership_times), ('group', oa.window_times), ('ready', ready_times))}

        def delta(value, origin):
            return round(1000 * (value - origin), 2) if value is not None and origin is not None else None

        return dict(
            case=label, threshold=threshold,
            crossing_ms=delta(crossing, start),
            flag_ms=delta(events['flag'], start), group_ms=delta(events['group'], start),
            ready_ms=delta(events['ready'], start),
            crossing_to_flag_ms=delta(events['flag'], crossing),
            flag_to_group_ms=delta(events['group'], events['flag']),
            group_to_ready_ms=delta(events['ready'], events['group']),
            discarded_groups=discards,
            frames=[dict(ms=delta(row['t'], start), **{k: v for k, v in row.items()
                    if k not in ('t', 'first_window')}) for row in rows],
        )
    finally:
        if selector_child is not None:
            stop.set()
            selector_child.join(timeout=4)
            if selector_child.is_alive():
                selector_child.terminate()
                selector_child.join()
            rx.close()
        for node in nodes:
            ex.remove_node(node)
            node.destroy_node()
        ex.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    ramp = [mask_of_area(n) for n in STEPS]
    clip = np.full(SHAPE, 255, np.uint8)
    clip[:, 398:] = 0
    cases = (
        ('static_1000', 1000, ramp, None, 0),
        ('clipped_1000', 1000, ramp, clip, 0),
        ('clipped_800_test_only', 800, ramp, clip, 0),
        ('vertical_roi_1000', 1000, [place_near_roi_top(r) for r in (182, 191, 199, 204, 210, 215)], None, 0),
        ('planner_blocked_then_clear_route', 1000, [np.full(SHAPE, 255, np.uint8)] * 5 + ramp[-2:], None, 0),
        ('two_expired_masks', 1000, [mask_of_area(1272)] * 5, None, 2),
    )
    parser = argparse.ArgumentParser()
    parser.add_argument('repetitions', nargs='?', default=1, type=int)
    parser.add_argument('--separate', action='store_true')
    options = parser.parse_args()
    for run in range(1, options.repetitions + 1):
        for name, threshold, masks, roi, old in cases:
            result = run_case(name, threshold, masks, roi=roi, old_first=old, separate=options.separate)
            print(json.dumps(dict(timestamp=datetime.now().astimezone().isoformat(),
                                  repetition=run, separate_processes=options.separate, **result)), flush=True)
