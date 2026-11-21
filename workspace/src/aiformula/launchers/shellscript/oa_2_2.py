#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
import json
import math
from collections import deque
from rclpy.qos import qos_profile_sensor_data

from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Pose2D, PoseArray
from std_msgs.msg import Int32, Bool, String
from cv_bridge import CvBridge


class ConeMaskAvoider(Node):
    """
    Keep OA ownership throughout one avoidance. Review the chosen side
    periodically, cancel clearing when the ROI rises again, and follow the
    final path using odometry. The original NURBS shape and image-to-vehicle
    calibration assumptions are preserved.

    OA 2.2 limits routine replanning, reacts to changed obstacles, and intersects
    the detector ROI with the dynamic lane corridor. Pixel clearance uses the
    original approximate image projection; it is not a physical safety proof. /oa/status reports tracking state; /lane_stop_flag keeps the
    0.2 second heartbeat used by the existing map-assistance node.
    """

    VERSION = 'oa_2_2'
    REVISION = 'stability_1'
    CHECK_SAME_PATH_WINDOW_JUMP = True
    DEFAULT_CONE_THRESH_PX = 1200
    RELEASE_ON_CONFIRMED_CLEAR = False

    def __init__(self):
        super().__init__(self.VERSION)

        # ========= 参数 =========
        self.m_per_px        = float(self.declare_parameter('meters_per_pixel', 0.01).value)
        self.min_offset_m    = float(self.declare_parameter('min_offset_amplitude_m', 0.5).value)
        self.single_side_amp = float(self.declare_parameter('single_side_amp', 0.30).value)
        self.extra_safe_m    = float(self.declare_parameter('extra_safe_m', 0.40).value)

        self.cone_thresh_px  = int(self.declare_parameter(
            'cone_thresh_px', self.DEFAULT_CONE_THRESH_PX).value)
        self.cone_resume_px  = int(self.declare_parameter('cone_resume_px', 500).value)

        self.n_sample        = int(self.declare_parameter('n_sample', 45).value)
        self.n_ctrl          = int(self.declare_parameter('n_ctrl', 7).value)
        self.spline_deg      = int(self.declare_parameter('spline_deg', 3).value)

        self.triplet_period  = float(self.declare_parameter('triplet_period', 0.1).value)
        direction_recheck_period = float(self.declare_parameter('direction_recheck_period', 3.0).value)
        if not math.isfinite(direction_recheck_period) or direction_recheck_period <= 0.0:
            raise ValueError('direction_recheck_period must be finite and positive')
        self.direction_recheck_ns = max(1, round(direction_recheck_period * 1_000_000_000))
        self.desired_x_m     = float(self.declare_parameter('desired_x_m', 0.3).value)
        self.point_spacing_m = float(self.declare_parameter('point_spacing_m', 0.3).value)
        self.min_point_spacing_m = float(self.declare_parameter('min_point_spacing_m', 0.05).value)
        self.clear_confirm_frames = int(self.declare_parameter('clear_confirm_frames', 3).value)
        self.max_offset_step_m = float(self.declare_parameter('max_offset_step_m', 0.10).value)
        self.max_tracking_error_m = float(self.declare_parameter('max_tracking_error_m', 1.5).value)
        self.finish_tolerance_m = float(self.declare_parameter('finish_tolerance_m', 0.35).value)
        self.finish_heading_deg = float(self.declare_parameter('finish_heading_deg', 45.0).value)
        self.odom_topic = str(self.declare_parameter(
            'odom_topic', '/aiformula_sensing/gyro_odometry_publisher/odom').value)

        if (not all(math.isfinite(v) and v > 0.0 for v in
                    (self.m_per_px, self.triplet_period, self.desired_x_m, self.point_spacing_m,
                     self.min_point_spacing_m,
                     self.max_offset_step_m, self.max_tracking_error_m, self.finish_tolerance_m))
                or not math.isfinite(self.finish_heading_deg) or not 0.0 < self.finish_heading_deg < 90.0
                or self.min_point_spacing_m > self.point_spacing_m
                or self.clear_confirm_frames < 1 or not 0 <= self.cone_resume_px < self.cone_thresh_px):
            raise ValueError('Invalid OA tracking parameters or cone thresholds')

        self.trap_top_ratio  = float(self.declare_parameter('trap_top_ratio', 0.5).value)
        self.trap_left_ratio, self.trap_right_ratio = 0.373, 0.625
        self.trap_bottom_left_ratio, self.trap_bottom_right_ratio = 0.15, 0.85

        self.overlay_mask    = bool(self.declare_parameter('overlay_mask', True).value)
        self.nurbs_peak_weight = float(self.declare_parameter('nurbs_peak_weight', 1.0).value)

        self.replan_period = float(self.declare_parameter('replan_period', 0.8).value)
        self.motion_threshold_px = float(self.declare_parameter('motion_threshold_px', 12.0).value)
        self.path_margin_m = float(self.declare_parameter('path_margin_m', 0.15).value)
        self.sensor_timeout = float(self.declare_parameter('sensor_timeout', 0.5).value)
        self.dynamic_roi_enabled = bool(self.declare_parameter('dynamic_roi_enabled', True).value)
        self.dynamic_roi_max_skew = float(self.declare_parameter('dynamic_roi_max_skew', 0.15).value)
        self.dynamic_roi_topic = str(self.declare_parameter('dynamic_roi_topic',
            '/aiformula_perception/lane_line_publisher/dynamic_roi_mask').value)
        if not all(math.isfinite(v) and v > 0 for v in (
                self.replan_period, self.motion_threshold_px, self.path_margin_m,
                self.sensor_timeout, self.dynamic_roi_max_skew)):
            raise ValueError('Invalid OA 2.2 timing/clearance parameters')
        self.roi_cache = deque(maxlen=12)
        self.active_roi = None
        self.roi_mode = 'static_missing'
        self.mask_received_ns = None
        self.odom_received_ns = None
        self.last_mask_stamp = None
        self.last_plan_ns = None
        self.last_plan_sequence = -1
        self.planned_mask = None
        self.replan_reason = 'initial'
        self.replan_count = 0

        self.switch_advantage = float(self.declare_parameter('switch_advantage', 0.25).value)
        self.switch_confirm_frames = int(self.declare_parameter('switch_confirm_frames', 3).value)
        self.switch_confirm_seconds = float(self.declare_parameter('switch_confirm_seconds', 0.5).value)
        self.switch_hold_seconds = float(self.declare_parameter('switch_hold_seconds', 1.5).value)
        self.max_window_jump_m = float(self.declare_parameter('max_window_jump_m', 0.12).value)
        self.rebase_stop_speed = float(self.declare_parameter('rebase_stop_speed', 0.05).value)
        if (not 0 < self.switch_advantage <= 1 or self.switch_confirm_frames < 2
                or not all(math.isfinite(v) and v > 0 for v in (
                    self.switch_confirm_seconds, self.switch_hold_seconds,
                    self.max_window_jump_m, self.rebase_stop_speed))):
            raise ValueError('Invalid stability parameters')
        self.measured_speed = None
        self.measured_yaw_rate = None
        self.stopped_since = None
        self._preview_only = False
        self.direction_switch_count = 0
        # ========= ROS IO =========
        self.bridge = CvBridge()
        self.create_subscription(Image, '/cone_bbox_image', self.cone_image_cb, 10)
        self.create_subscription(Image, '/cone_mask', self.mask_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.dynamic_roi_topic, self.dynamic_roi_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

        for n, cb in [('center', self.pc_center_cb), ('left', self.pc_left_cb), ('right', self.pc_right_cb)]:
            self.create_subscription(
                PointCloud2,
                f'/aiformula_perception/lane_line_publisher/lane_lines/{n}',
                cb,
                10
            )

        self.pub_pc_center = self.create_publisher(PointCloud2, 'lane_line_center', 10)
        self.pub_pc_left   = self.create_publisher(PointCloud2, 'lane_line_left', 10)
        self.pub_pc_right  = self.create_publisher(PointCloud2, 'lane_line_right', 10)

        self.pub_pt_a = self.create_publisher(Pose2D, '/processed_point_a', 10)
        self.pub_pt_b = self.create_publisher(Pose2D, '/processed_point_b', 10)
        self.pub_pt_c = self.create_publisher(Pose2D, '/processed_point_c', 10)
        self.triplet_pubs = [self.pub_pt_a, self.pub_pt_b, self.pub_pt_c]
        self.group_pub = self.create_publisher(
            PoseArray, '/semantic_drive/map_assist_045/oa_triplet', 10)

        self.pub_dbg_img   = self.create_publisher(Image, '/processed_image', 10)
        self.pub_cnt       = self.create_publisher(Int32, '/cone_pixels_count', 10)
        self.pub_stop_flag = self.create_publisher(Bool, '/lane_stop_flag', 10)
        self.pub_status    = self.create_publisher(String, '/oa/status', 10)

        # ========= 状态 =========
        self.latest_vis_img = None
        self.latest_vis_hdr = None
        self.latest_mask    = None
        self.mask_sequence = 0
        self.counted_mask_sequence = -1
        self.odom = None

        self.h = self.w = self.cx = 0

        self.lane_forward = True
        self.obstacle_cleared = False
        self.clear_frames = 0
        self._reset_direction()
        self.execution_state = 'idle'

        self.current_path = []
        self.timer_handle = None
        self.world_path = None
        self.path_cumulative = None
        self.path_frame = None
        self.path_peak_index = None
        self.path_progress_m = 0.0
        self.remaining_m = None

        self.last_path_px = None

        self.manual_state_topic = str(self.declare_parameter('manual_state_topic', '').value)
        self.manual_held = False
        self.manual_transition_ns = None
        self.manual_vision_ready = False
        if self.manual_state_topic:
            from semantic_vehicle_wiring.manual_handoff import read_state

            self._read_handoff_state = read_state
            self.create_subscription(String, self.manual_state_topic, self._on_manual_state, 1)

        self._publish_avoidance_flag()
        self.flag_timer = self.create_timer(0.2, self._publish_avoidance_flag)
        self.create_timer(0.03, self.tick)

    def _publish_avoidance_flag(self):
        """Publish whether OA currently owns the path-point outputs."""
        self.pub_stop_flag.publish(Bool(data=not self.lane_forward))
        self.pub_status.publish(String(data=json.dumps({
            'version': self.VERSION, 'revision': self.REVISION,
            'direction_switch_count': self.direction_switch_count, 'roi_mode': self.roi_mode,
            'replan_reason': self.replan_reason, 'replan_count': self.replan_count,
            'state': self.execution_state, 'avoidance_active': not self.lane_forward,
            'obstacle_cleared': self.obstacle_cleared, 'clear_frames': self.clear_frames,
            'direction': ('left' if self.avoid_direction == 1 else
                          'right' if self.avoid_direction == -1 else None),
            'odom_available': self.odom is not None, 'remaining_m': self.remaining_m,
            'manual_held': self.manual_held,
            'manual_transition_ns': self.manual_transition_ns if self.manual_vision_ready else None,
            'stamp_ns': self.get_clock().now().nanoseconds,
        }, allow_nan=False)))

    def _on_manual_state(self, msg):
        state = self._read_handoff_state(msg.data, self.get_clock().now().nanoseconds, 'manual_held')
        if state is None:
            return
        held, transition = state
        if self.manual_transition_ns is not None and transition <= self.manual_transition_ns:
            return
        self.manual_held, self.manual_transition_ns = held, transition
        if transition == 0:
            return
        self.manual_vision_ready = False
        self.latest_mask = None
        self.latest_vis_img = self.latest_vis_hdr = None
        self.mask_received_ns = None
        self.active_roi = None
        self.roi_cache.clear()
        self.roi_mode = 'static_missing'
        self._finish_avoidance('manual_takeover' if held else 'manual_waiting_fresh_mask')

    def _accept_handoff_image(self, msg):
        return not self.manual_held and (
            not self.manual_transition_ns or self._stamp(msg) > self.manual_transition_ns
        )

    # ---------------- 回调：底图/Mask ----------------
    def cone_image_cb(self, msg: Image):
        if not self._accept_handoff_image(msg):
            return
        try:
            self.latest_vis_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_vis_hdr = msg.header
        except Exception:
            self.latest_vis_img = None
            self.latest_vis_hdr = None

    @staticmethod
    def _stamp(msg):
        header = getattr(msg, 'header', None)
        stamp = getattr(header, 'stamp', None)
        return (stamp.sec * 1000000000 + stamp.nanosec) if stamp else 0

    def dynamic_roi_cb(self, msg):
        if not self._accept_handoff_image(msg):
            return
        try:
            if msg.encoding != 'mono8':
                return  # Never interpret the colored debug overlay as a mask.
            mask = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            stamp = self._stamp(msg)
            if stamp <= 0 or mask.ndim != 2:
                return
            self.roi_cache.append((stamp, self.get_clock().now().nanoseconds,
                                   msg.header.frame_id, mask.copy()))
        except Exception:
            return

    def _select_roi(self, msg):
        self.active_roi = None
        self.roi_mode = 'static_disabled' if not self.dynamic_roi_enabled else 'static_missing'
        if not self.dynamic_roi_enabled:
            return
        stamp = self._stamp(msg)
        now = self.get_clock().now().nanoseconds
        frame = getattr(getattr(msg, 'header', None), 'frame_id', '')
        candidates = [item for item in self.roi_cache
                      if stamp > 0 and abs(item[0] - stamp) <= self.dynamic_roi_max_skew * 1e9
                      and 0 <= now - item[1] <= self.sensor_timeout * 1e9
                      and (not frame or not item[2] or frame == item[2])
                      and item[3].shape == self.latest_mask.shape]
        if candidates:
            selected = min(candidates, key=lambda item: abs(item[0] - stamp))[3]
            if np.any(selected):
                self.active_roi = selected
                self.roi_mode = 'dynamic_intersection'
            else:
                self.roi_mode = 'static_invalid_lane'

    def _inputs_fresh(self):
        now = self.get_clock().now().nanoseconds
        for name, received in (('mask', self.mask_received_ns), ('odom', self.odom_received_ns)):
            if received is None or not 0 <= now - received <= self.sensor_timeout * 1e9:
                self.vote_side, self.vote_since, self.vote_count = None, None, 0
                self.pending_side, self.pending_since = None, None
                self.execution_state = name + '_stale'
                self.clear_frames = 0
                self.obstacle_cleared = False
                return False
        return True

    def mask_cb(self, msg: Image):
        if not self._accept_handoff_image(msg):
            return
        try:
            stamp = self._stamp(msg)
            now = self.get_clock().now().nanoseconds
            if stamp > 0 and (now - stamp > self.sensor_timeout * 1e9 or stamp > now + 1e8):
                return
            if stamp > 0 and self.last_mask_stamp is not None and stamp <= self.last_mask_stamp:
                return
            self.last_mask_stamp = stamp if stamp > 0 else self.last_mask_stamp
            self.latest_mask = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            self.mask_received_ns = now
            self._select_roi(msg)
            self.mask_sequence += 1
            # Count each received mask while active, rather than counting only
            # the last one seen by a slower timer. An intervening high frame
            # must cancel an in-progress clear vote immediately.
            if not self.lane_forward and self.latest_vis_img is not None:
                self.h, self.w = self.latest_vis_img.shape[:2]
                self.cx = self.w // 2
                _, total, _, _, observed_mask = self._compute_roi_counts(self.latest_vis_img, self.latest_mask)
                self._observe_direction(observed_mask)
                self._update_clear_observation(total)
                self.counted_mask_sequence = self.mask_sequence
        except Exception:
            self.latest_mask = None
            if not self.lane_forward:
                # A failed conversion is not a clear detector observation.
                self.clear_frames = 0
                self.obstacle_cleared = False
                self.execution_state = 'mask_conversion_failed'

    def _update_clear_observation(self, roi_total):
        if roi_total >= self.cone_resume_px:
            self.clear_frames = 0
            self.obstacle_cleared = False
        else:
            self.clear_frames = min(self.clear_frames + 1, self.clear_confirm_frames)
            self.obstacle_cleared = self.clear_frames >= self.clear_confirm_frames

    def odom_cb(self, msg: Odometry):
        """Store odom pose in the same frame used to anchor the planned path."""
        self.measured_speed = self.measured_yaw_rate = None
        stamp = self._stamp(msg)
        now = self.get_clock().now().nanoseconds
        if stamp > 0 and not 0 <= now - stamp <= self.sensor_timeout * 1e9:
            self.odom = None
            self.stopped_since = None
            self.odom_received_ns = None
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            self.odom = None
            self.stopped_since = None
            return
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-8:
            self.odom = None
            self.stopped_since = None
            return
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
        frame = (msg.header.frame_id or 'odom', msg.child_frame_id or '')
        self.odom = (float(p.x), float(p.y), yaw, frame)
        previous_received = self.odom_received_ns
        self.odom_received_ns = self.get_clock().now().nanoseconds
        try:
            t = msg.twist.twist
            speed = math.hypot(t.linear.x, t.linear.y)
            rate = float(t.angular.z)
            if math.isfinite(speed) and math.isfinite(rate):
                self.measured_speed, self.measured_yaw_rate = speed, rate
        except (AttributeError, TypeError, ValueError):
            pass
        if (self.measured_speed is None or self.measured_yaw_rate is None
                or self.measured_speed > self.rebase_stop_speed or abs(self.measured_yaw_rate) > 0.05):
            self.stopped_since = None
        elif (self.stopped_since is None or previous_received is None
              or not 0 <= self.odom_received_ns - previous_received <= self.sensor_timeout * 1e9):
            self.stopped_since = self.odom_received_ns

    # ---------------- 点云透传 ----------------
    def pc_center_cb(self, m):
        if self.lane_forward:
            self.pub_pc_center.publish(m)

    def pc_left_cb(self, m):
        if self.lane_forward:
            self.pub_pc_left.publish(m)

    def pc_right_cb(self, m):
        if self.lane_forward:
            self.pub_pc_right.publish(m)

    # ---------------- 坐标 ----------------
    def px2robot(self, p):
        # x 前向，y 左正
        return np.array([
            (self.h - p[1]) * self.m_per_px,
            (self.cx - p[0]) * self.m_per_px
        ], dtype=float)

    def robot2px(self, p):
        return (
            int(self.cx - p[1] / self.m_per_px),
            int(self.h - p[0] / self.m_per_px)
        )

    # ---------------- 主循环（只负责状态切换与可视化） ----------------
    def tick(self):
        if self.manual_held or (self.manual_state_topic and self.manual_transition_ns is None):
            return
        if self.latest_vis_img is None or self.latest_mask is None:
            return

        if not self._inputs_fresh():
            return
        vis = self.latest_vis_img.copy()
        mask = self.latest_mask

        self.h, self.w = vis.shape[:2]
        self.cx = self.w // 2

        roi_cnt, roi_total, L_cnt, R_cnt, col_mask = self._compute_roi_counts(vis, mask)
        self.pub_cnt.publish(Int32(data=int(roi_total)))

        if self.overlay_mask and col_mask is not None:
            overlay = vis.copy()
            overlay[col_mask > 0] = (0, 0, 255)
            vis = cv2.addWeighted(overlay, 0.25, vis, 0.75, 0.0)

        # Count detector messages, not repeated timer reads of the same mask.
        if self.mask_sequence != self.counted_mask_sequence:
            self.counted_mask_sequence = self.mask_sequence
            if self.lane_forward and roi_total > self.cone_thresh_px:
                self.lane_forward = False
                self.obstacle_cleared = False
                self.clear_frames = 0
                self._reset_direction()
                self.execution_state = 'planning'
                self._publish_avoidance_flag()
                self.compute_and_store_path(col_mask, vis, publish_window=True)
                if self.timer_handle is None:
                    self.timer_handle = self.create_timer(self.triplet_period, self.avoidance_step)
            elif not self.lane_forward:
                self._update_clear_observation(roi_total)
            if self.manual_transition_ns and not self.manual_vision_ready:
                # Acknowledge only after this fresh mask has chosen the point source.
                self.manual_vision_ready = True
                if self.lane_forward:
                    self.execution_state = 'idle'
                self._publish_avoidance_flag()

        self.draw_path(vis)
        self.draw_hud(vis, roi_cnt, roi_total, L_cnt, R_cnt)
        self.publish_dbg(vis)

    # ---------------- rolling/sliding window 的核心 ----------------
    def avoidance_step(self):
        if self.lane_forward:
            if self.timer_handle is not None:
                self.timer_handle.cancel()
                self.timer_handle = None
            return

        if self.latest_vis_img is None or self.latest_mask is None:
            return

        if not self._inputs_fresh():
            return
        vis = self.latest_vis_img.copy()
        self.h, self.w = vis.shape[:2]
        self.cx = self.w // 2

        _, roi_total, _, _, col_mask = self._compute_roi_counts(vis, self.latest_mask)

        # Check every current path even for small newly intruding obstacles.
        if roi_total >= self.cone_resume_px or self._path_blocked(col_mask):
            self.clear_frames = 0
            self.obstacle_cleared = False
            self._maybe_replan(col_mask, vis)
            return

        if self.RELEASE_ON_CONFIRMED_CLEAR and self.obstacle_cleared:
            self._finish_avoidance('obstacle_clear')
            return

        if not self.obstacle_cleared and self.execution_state in (
            'waiting_for_direction_recheck', 'switching_avoid_direction',
        ):
            return
        # Follow the last recorded path until odometry and clear observations
        # confirm it is time to return control.
        self.broadcast_triplet()

    def _path_in_vehicle(self):
        x, y, yaw, _ = self.odom
        delta = self.world_path - np.array([x, y])
        c, s = math.cos(yaw), math.sin(yaw)
        return np.column_stack((c * delta[:, 0] + s * delta[:, 1],
                                -s * delta[:, 0] + c * delta[:, 1]))

    def _path_pixels(self, points):
        return np.array([self.robot2px(p) for p in points], np.int32).reshape(-1, 1, 2)

    def _path_blocked(self, col_mask, points=None):
        if points is None:
            if self.world_path is None or self.odom is None:
                return False
            if self.odom[3] != self.path_frame:
                return True
            points = self._path_in_vehicle()
        points = np.asarray(points)
        points = points[points[:, 0] >= 0]
        if len(points) < 2:
            return False
        corridor = np.zeros(col_mask.shape, np.uint8)
        pixels = self._path_pixels(points)
        cv2.polylines(corridor, [pixels], False, 255,
                      max(1, int(math.ceil(2 * self.path_margin_m / self.m_per_px))))
        return bool(np.any((corridor > 0) & (col_mask > 0)))

    def _mask_moved(self, mask):
        if self.planned_mask is None or mask.shape != self.planned_mask.shape:
            return True
        # Compare with the last accepted plan, so slow cumulative movement is caught.
        radius = max(1, int(self.motion_threshold_px))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
        envelope = cv2.dilate((self.planned_mask > 0).astype(np.uint8), kernel)
        return np.count_nonzero((mask > 0) & (envelope == 0)) >= 20

    def _maybe_replan(self, mask, vis):
        new_frame = self.mask_sequence != self.last_plan_sequence
        blocked = self._path_blocked(mask)
        moved = new_frame and self._mask_moved(mask)
        now = self.get_clock().now().nanoseconds
        due = self.last_plan_ns is None or now - self.last_plan_ns >= self.replan_period * 1e9
        missing = self.world_path is None
        if new_frame and (missing or blocked or moved or due or self._switch_confirmed()):
            self.replan_reason = ('path_blocked' if blocked else 'obstacle_changed' if moved
                                  else 'no_path' if missing else 'periodic')
            self.force_direction_review = blocked or moved
            self.last_plan_sequence = self.mask_sequence
            try:
                self.compute_and_store_path(mask, vis, publish_window=True)
            finally:
                self.force_direction_review = False
        elif blocked:
            self.execution_state = 'path_blocked_waiting_new_mask'
        else:
            self.broadcast_triplet()

    def _roi_geometry(self):
        tl = (int(self.w * self.trap_left_ratio), int(self.h * self.trap_top_ratio))
        tr = (int(self.w * self.trap_right_ratio), int(self.h * self.trap_top_ratio))
        bl = (int(self.w * self.trap_bottom_left_ratio), self.h)
        br = (int(self.w * self.trap_bottom_right_ratio), self.h)
        roi_cnt = np.array([tl, tr, br, bl], np.int32).reshape((-1, 1, 2))

        roi_mask = np.zeros((self.h, self.w), np.uint8)
        cv2.fillPoly(roi_mask, [roi_cnt.reshape(-1, 2)], 255)

        # Match the lane publisher's 0.58 global ROI cutoff. Preserve the
        # trapezoid's original horizontal bounds in every retained row.
        roi_top = min(self.h - 1, int(self.h * 0.58 + 0.5))
        if roi_top > tl[1]:
            roi_mask[:roi_top] = 0
            top_columns = np.flatnonzero(roi_mask[roi_top])
            if top_columns.size:
                roi_cnt[0, 0] = (top_columns[0], roi_top)
                roi_cnt[1, 0] = (top_columns[-1], roi_top)

        if self.active_roi is not None and self.active_roi.shape == roi_mask.shape:
            roi_mask = cv2.bitwise_and(roi_mask, self.active_roi)
        elif self.active_roi is not None:
            self.roi_mode = 'static_shape_mismatch'
        return roi_cnt, roi_mask

    def _compute_roi_counts(self, vis, mask):
        if mask.shape[:2] != vis.shape[:2]:
            mask = cv2.resize(mask, (vis.shape[1], vis.shape[0]), cv2.INTER_NEAREST)

        roi_cnt, roi_mask = self._roi_geometry()
        col_mask = cv2.bitwise_and(mask, roi_mask)
        roi_total = int(cv2.countNonZero(col_mask))

        cm = (col_mask > 0)
        L_cnt = int(np.count_nonzero(cm[:, :self.cx]))
        R_cnt = int(np.count_nonzero(cm[:, self.cx:]))

        return roi_cnt, roi_total, L_cnt, R_cnt, col_mask

    # ---------------- 规划 ----------------
    def _discard_saved_path(self):
        self.current_path = []
        self.world_path = None
        self.path_cumulative = None
        self.path_frame = None
        self.path_peak_index = None
        self.path_progress_m = 0.0
        self.remaining_m = None
        self.last_path_px = None

    def _install_plan(self, col_mask, vis=None, publish_window=True):
        if self.odom is None:
            self._discard_saved_path()
            self.execution_state = 'waiting_for_odom'
            return False
        self.execution_state = 'planning'
        plan = self._selected_plan
        if plan is None:
            if self.execution_state == 'planning':
                self._discard_saved_path()
                self.execution_state = 'planning_unavailable'
            return False

        path, _, peak_index, _, _ = plan
        points = np.asarray(path, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
            self._discard_saved_path()
            self.execution_state = 'planning_invalid'
            return False
        # A planned path is expressed in the vehicle frame at this snapshot.
        # Anchor it once in odom so later poses can reproject it correctly.
        x, y, yaw, frame = self.odom
        c, s = math.cos(yaw), math.sin(yaw)
        world = np.empty_like(points)
        world[:, 0] = x + c * points[:, 0] - s * points[:, 1]
        world[:, 1] = y + s * points[:, 0] + c * points[:, 1]
        segment_lengths = np.linalg.norm(np.diff(world, axis=0), axis=1)
        if len(segment_lengths) < 2 or np.any(segment_lengths <= 1e-8):
            self._discard_saved_path()
            self.execution_state = 'planning_degenerate'
            return False
        if self._path_blocked(col_mask, points):
            self._discard_saved_path()
            self.execution_state = 'candidate_path_blocked'
            return False
        self.last_plan_ns = self.get_clock().now().nanoseconds
        self.last_plan_sequence = self.mask_sequence
        self.planned_mask = col_mask.copy()
        self.replan_count += 1
        self.current_path = path
        self.world_path = world
        self.path_cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        self.path_frame = frame
        self.path_peak_index = peak_index
        self.path_progress_m = 0.0
        self.remaining_m = float(self.path_cumulative[-1])

        if publish_window:
            self.broadcast_triplet()

        if vis is not None:
            self.last_path_px = self._path_pixels(path)

        return True

    def _reset_direction(self):
        self.vote_side, self.vote_since, self.vote_count = None, None, 0
        self.vote_sequence = -1
        self.last_switch_ns = None
        self.pending_side, self.pending_since, self.pending_targets = None, None, None
        self.pending_sequence, self.pending_frames = -1, 0
        self.last_window_world, self.last_window_frame = None, None
        self.last_window_path = None
        self.last_plan_ns = None
        self.last_plan_sequence = -1
        self.planned_mask = None
        self._selected_plan = None
        self._candidate_side = None
        self._preview_targets = None
        self.force_direction_review = False
        self.avoid_direction = None
        self.offset_amplitude = None
        self.direction_checked_at_ns = None
        self.direction_mask_sequence = -1
        self.switching_direction = False

    def plan_path(self, col_mask):
        if col_mask is None:
            return None

        ys, xs = np.where(col_mask > 0)
        if xs.size == 0:
            return None

        left_cnt = int(np.sum(xs < self.cx))
        right_cnt = int(np.sum(xs >= self.cx))

        d = self._candidate_side
        if d is None:
            return None
        if left_cnt == 0 or right_cnt == 0:
            target_amp = self.single_side_amp
        else:
            dist_px = (self.cx - int(xs[xs < self.cx].max()) if d == +1
                       else int(xs[xs >= self.cx].min()) - self.cx)
            target_amp = max(dist_px * self.m_per_px + self.extra_safe_m, self.min_offset_m)
        target_offset = d * target_amp
        if self.offset_amplitude is None:
            self.offset_amplitude = target_offset
        else:
            self.offset_amplitude = max(
                self.offset_amplitude - self.max_offset_step_m,
                min(self.offset_amplitude + self.max_offset_step_m, target_offset),
            )
        if self.switching_direction:
            if not math.isclose(self.offset_amplitude, target_offset, abs_tol=1e-9):
                self.execution_state = 'switching_avoid_direction'
                return None
            self.switching_direction = False
        amp_m = abs(self.offset_amplitude)

        p0_m = self.px2robot((self.cx, self.h))
        p1_m = self.px2robot((self.cx, self._path_end_row()))

        path = self.build_path(
            p0_m, p1_m, d, amp_m,
            n_ctrl=self.n_ctrl,
            n_sample=self.n_sample,
            p=self.spline_deg,
            peak_weight=self.nurbs_peak_weight
        )
        if len(path) < 3:
            return None

        p_start = self._point_at_lookahead_x(path)
        ys_list = [p[1] for p in path]
        idx_mid = int(np.argmax(ys_list)) if d == +1 else int(np.argmin(ys_list))
        p_mid = path[idx_mid].copy()
        p_end = path[-1].copy()

        return path, p_start, idx_mid, p_mid, p_end

    def _path_end_row(self):
        return int(self.h * self.trap_top_ratio)

    def _point_at_lookahead_x(self, path):
        desired_x = float(self.desired_x_m)
        for i in range(1, len(path)):
            x0, y0 = float(path[i - 1][0]), float(path[i - 1][1])
            x1, y1 = float(path[i][0]), float(path[i][1])
            if (x0 <= desired_x <= x1) or (x1 <= desired_x <= x0):
                t = (desired_x - x0) / (x1 - x0 + 1e-12)
                return np.array([desired_x, y0 + t * (y1 - y0)], dtype=float)
        return path[1].copy()

    # ---------------- 发送剩余路径 ----------------
    def broadcast_triplet(self):
        if not self._inputs_fresh():
            return False
        if self.latest_mask is not None and self.h and self.w:
            _, _, _, _, mask = self._compute_roi_counts(self.latest_vis_img, self.latest_mask)
            if self._path_blocked(mask):
                self.execution_state = 'path_blocked_waiting_replan'
                return False
        if self.odom is None:
            self.execution_state = 'waiting_for_odom'
            return False
        if self.world_path is None or self.path_cumulative is None:
            self.execution_state = 'waiting_for_path'
            return False
        x, y, yaw, frame = self.odom
        if frame != self.path_frame:
            self.execution_state = 'odom_frame_changed'
            return False

        # Project the actual odometry position onto the stored open path. A
        # timer tick alone cannot advance this projection or release the flag.
        position = np.asarray([x, y], dtype=float)
        starts, ends = self.world_path[:-1], self.world_path[1:]
        vectors = ends - starts
        lengths_sq = np.einsum('ij,ij->i', vectors, vectors)
        fractions = np.clip(np.einsum('ij,ij->i', position - starts, vectors) / lengths_sq, 0.0, 1.0)
        projections = starts + fractions[:, None] * vectors
        distances = np.linalg.norm(projections - position, axis=1)
        index = int(np.argmin(distances))
        if float(distances[index]) > self.max_tracking_error_m:
            self.execution_state = 'path_tracking_error'
            return False
        segment_length = float(self.path_cumulative[index + 1] - self.path_cumulative[index])
        measured_progress = float(self.path_cumulative[index] + fractions[index] * segment_length)
        self.path_progress_m = max(self.path_progress_m, measured_progress)
        total = float(self.path_cumulative[-1])
        self.remaining_m = max(0.0, total - self.path_progress_m)

        last_segment = self.world_path[-1] - self.world_path[-2]
        tangent = last_segment / np.linalg.norm(last_segment)
        end_offset = position - self.world_path[-1]
        along_end = float(np.dot(end_offset, tangent))
        lateral_end = abs(float(tangent[0] * end_offset[1] - tangent[1] * end_offset[0]))
        end_distance = float(np.linalg.norm(end_offset))
        heading = math.atan2(float(tangent[1]), float(tangent[0]))
        yaw_error = abs(math.atan2(math.sin(yaw - heading), math.cos(yaw - heading)))
        near_end = end_distance <= self.finish_tolerance_m
        passed_end = (0.0 <= along_end <= self.max_tracking_error_m
                      and lateral_end <= self.finish_tolerance_m)
        heading_ok = yaw_error <= math.radians(self.finish_heading_deg)
        if (self.obstacle_cleared and heading_ok and (near_end or passed_end)
                and self.path_progress_m >= total - self.finish_tolerance_m):
            self._finish_avoidance()
            return True

        if self.remaining_m <= 1e-5:
            self.execution_state = ('waiting_clear_confirmation' if not self.obstacle_cleared
                                    else 'path_end_pose_unusable')
            return False

        targets = self._select_window()
        if targets is None:
            self.execution_state = 'target_not_forward'
            return False
        if not self.publish_window(targets):
            return False
        self.execution_state = ('tracking_after_clear' if self.obstacle_cleared
                                else 'tracking_avoidance')
        return True

    def _select_window(self):
        # Find three distinct forward points on the real path. If a close
        # target is behind the current vehicle, try a farther lookahead; never
        # substitute repeated end points or a guessed vehicle position.
        x, y, yaw, _ = self.odom
        minimum = min(self.desired_x_m, self.remaining_m / 4.0)
        min_forward = min(self.desired_x_m, self.remaining_m / 2.0)
        step = min(self.point_spacing_m / 2.0, max(0.02, self.remaining_m / 16.0))
        c, s = math.cos(yaw), math.sin(yaw)
        for attempt in range(17):
            first = minimum + attempt * step
            if first >= self.remaining_m - 1e-5:
                break
            spacing = min(self.point_spacing_m, (self.remaining_m - first) / 3.0)
            if spacing < self.min_point_spacing_m:
                break
            trial = []
            for offset in (first, first + spacing, first + 2.0 * spacing):
                point = self._point_at_progress(self.path_progress_m + offset)
                dx, dy = float(point[0] - x), float(point[1] - y)
                trial.append((c * dx + s * dy, -s * dx + c * dy))
            if (trial[0][0] >= min_forward
                    and all(math.isfinite(a) and math.isfinite(b) and a > 1e-6 for a, b in trial)
                    and all(math.dist(a, b) >= self.min_point_spacing_m - 1e-9
                            for a, b in zip(trial, trial[1:]))):
                return trial
        return None

    def _publish_window_unchecked(self, points):
        group = PoseArray()
        group.header.stamp = self.get_clock().now().to_msg()
        group.header.frame_id = 'oa_window'
        for pub, (px, py) in zip(self.triplet_pubs, points):
            x, y = float(px), float(py)
            pose = Pose()
            pose.position.x, pose.position.y = x, y
            pose.orientation.w = 1.0
            group.poses.append(pose)
            pub.publish(Pose2D(x=x, y=y, theta=0.0))
        self.group_pub.publish(group)

    def _point_at_progress(self, distance, points=None):
        if points is None:
            points = self.world_path
        distance = max(0.0, min(float(distance), float(self.path_cumulative[-1])))
        index = min(int(np.searchsorted(self.path_cumulative, distance, side='right') - 1),
                    len(self.world_path) - 2)
        start, end = self.path_cumulative[index:index + 2]
        fraction = (distance - start) / (end - start)
        return points[index] + fraction * (points[index + 1] - points[index])

    def _finish_avoidance(self, state='idle'):
        if self.timer_handle is not None:
            self.timer_handle.cancel()
            self.timer_handle = None
        self._discard_saved_path()
        self.obstacle_cleared = False
        self.clear_frames = 0
        self.stopped_since = None
        self._reset_direction()
        self.lane_forward = True
        self.execution_state = state
        self._publish_avoidance_flag()

    def _observe_direction(self, mask):
        if self.vote_sequence == self.mask_sequence:
            return
        self.vote_sequence = self.mask_sequence
        left = int(np.count_nonzero(mask[:, :self.cx]))
        right = int(np.count_nonzero(mask[:, self.cx:]))
        preferred = 1 if left < right else -1
        now = self.get_clock().now().nanoseconds
        advantage = abs(left - right) / max(1, left + right)
        if (self.avoid_direction is None or preferred == self.avoid_direction
                or advantage < self.switch_advantage):
            self.vote_side, self.vote_count, self.vote_since = None, 0, None
            return
        if self.vote_side != preferred or self.vote_since is None or now < self.vote_since:
            self.vote_side, self.vote_count, self.vote_since = preferred, 1, now
        else:
            self.vote_count += 1

    def _switch_confirmed(self):
        now = self.get_clock().now().nanoseconds
        return (self.vote_side is not None and self.vote_count >= self.switch_confirm_frames
                and self.vote_since is not None
                and now - self.vote_since >= self.switch_confirm_seconds * 1e9
                and (self.last_switch_ns is None
                     or now - self.last_switch_ns >= self.switch_hold_seconds * 1e9))

    def _candidate(self, mask, side):
        previous = self.offset_amplitude
        self._candidate_side = side
        # Candidate computation must not change the committed direction/amplitude.
        if side != self.avoid_direction:
            self.offset_amplitude = None
        try:
            plan = self.plan_path(mask)
            amplitude = self.offset_amplitude
        finally:
            self.offset_amplitude = previous
        if plan is None or self._path_blocked(mask, plan[0]):
            return None
        return plan, amplitude

    def _hold_or_follow(self, mask, reason):
        self.execution_state = reason
        if self.world_path is not None and not self._path_blocked(mask):
            self.broadcast_triplet()
        return False

    def _stopped_rebase_ready(self, side, targets):
        now = self.get_clock().now().nanoseconds
        stable = (self.pending_side == side and self.pending_targets is not None
                  and np.max(np.abs(np.asarray(targets) - self.pending_targets)) <= 0.05)
        if not stable or self.pending_since is None or now < self.pending_since:
            self.pending_side = side
            self.pending_targets = np.asarray(targets).copy()
            self.pending_since = now
            self.pending_frames = 0
            self.pending_sequence = -1
        if self.pending_sequence != self.mask_sequence:
            self.pending_sequence = self.mask_sequence
            self.pending_frames += 1
        # Never infer standstill merely from repeated path points or a timer.
        return (self.measured_speed is not None and self.measured_yaw_rate is not None
                and self.measured_speed <= self.rebase_stop_speed
                and abs(self.measured_yaw_rate) <= 0.05
                and self.pending_frames >= self.switch_confirm_frames
                and now - self.pending_since >= self.switch_confirm_seconds * 1e9
                and self.stopped_since is not None
                and now - self.stopped_since >= self.switch_confirm_seconds * 1e9)

    def compute_and_store_path(self, col_mask, vis=None, publish_window=True):
        if self.odom is None or not self._inputs_fresh():
            return False
        self._observe_direction(col_mask)
        left = int(np.count_nonzero(col_mask[:, :self.cx]))
        right = int(np.count_nonzero(col_mask[:, self.cx:]))
        side = self.avoid_direction or (1 if left < right else -1)
        candidate = self._candidate(col_mask, side)
        old_blocked = self.world_path is not None and self._path_blocked(col_mask)
        emergency = candidate is None and (old_blocked or self.world_path is None)
        # Sensor changes alone never unlock the side. Emergency means the saved
        # route AND this side's candidate cannot be used under the current mask.
        if emergency or (self._switch_confirmed() and self.vote_side == -side):
            opposite = self._candidate(col_mask, -side)
            if opposite is not None:
                side, candidate = -side, opposite
        if candidate is None:
            self.pending_side, self.pending_since = None, None
            return self._hold_or_follow(col_mask, 'no_clear_candidate')

        # Install provisionally and use the actual ABC sampler for the preview.
        fields = ('current_path', 'world_path', 'path_cumulative', 'path_frame', 'path_peak_index',
                  'path_progress_m', 'remaining_m', 'last_path_px', 'last_plan_ns',
                  'last_plan_sequence', 'planned_mask', 'replan_count', 'execution_state')
        saved = {key: getattr(self, key) for key in fields}
        self._selected_plan = candidate[0]
        self._preview_targets = None
        installed = self._install_plan(col_mask, None, publish_window=False)
        self._preview_only = True
        try:
            if installed:
                self.broadcast_triplet()
        finally:
            self._preview_only = False
        targets = self._preview_targets
        for key, value in saved.items():
            setattr(self, key, value)
        if targets is None:
            return self._hold_or_follow(col_mask, 'candidate_window_unusable')
        continuous = self._window_continuous(targets)
        if not continuous:
            if not old_blocked and self.world_path is not None:
                self.pending_side, self.pending_since = None, None
                return self._hold_or_follow(col_mask, 'retained_path_for_continuity')
            if not self._stopped_rebase_ready(side, targets):
                self.execution_state = 'holding_for_stable_stopped_rebase'
                return False
            # Explicit stopped handoff after stable fresh evidence, not smoothing
            # an unsafe old path or pretending an unobserved stop has happened.
            self.last_window_world = None
        changed = self.avoid_direction is not None and side != self.avoid_direction
        self.avoid_direction = side
        self.offset_amplitude = candidate[1]
        if changed:
            self.direction_switch_count += 1
            self.last_switch_ns = self.get_clock().now().nanoseconds
        self.pending_side, self.pending_since = None, None
        self._selected_plan = candidate[0]
        return self._install_plan(col_mask, vis, publish_window)

    def _window_continuous(self, points):
        if self.last_window_world is None:
            return True
        if self.odom is None or self.last_window_frame != self.odom[3]:
            return False
        x, y, yaw, _ = self.odom
        delta = self.last_window_world - np.asarray([x, y])
        previous_y = -math.sin(yaw) * delta[:, 0] + math.cos(yaw) * delta[:, 1]
        return bool(np.max(np.abs(np.asarray(points)[:, 1] - previous_y)) <= self.max_window_jump_m)

    def publish_window(self, points):
        if self._preview_only:
            self._preview_targets = np.asarray(points).copy()
            return True
        same_path = self.last_window_path is self.world_path
        if (self.CHECK_SAME_PATH_WINDOW_JUMP or not same_path) and not self._window_continuous(points):
            self.execution_state = 'window_jump_rejected'
            return False
        self._publish_window_unchecked(points)
        x, y, yaw, frame = self.odom
        c, s = math.cos(yaw), math.sin(yaw)
        points = np.asarray(points)
        self.last_window_world = np.column_stack((x + c*points[:, 0] - s*points[:, 1],
                                                 y + s*points[:, 0] + c*points[:, 1]))
        self.last_window_frame = frame
        self.last_window_path = self.world_path
        return True

    # ---------------- 样条通用工具 ----------------
    @staticmethod
    def _build_ctrl_points(p0, p1, d, amp, n_ctrl):
        n_ctrl = max(2, int(n_ctrl))

        p0 = np.asarray(p0, dtype=float)
        p1 = np.asarray(p1, dtype=float)

        v = p1 - p0
        norm = np.linalg.norm(v)
        if norm < 1e-9:
            return np.vstack([p0.copy(), p1.copy()])

        v = v / norm
        perp = np.array([-v[1], v[0]], dtype=float)

        ctrl = [p0.copy()]
        for i in range(1, n_ctrl - 1):
            t = i / (n_ctrl - 1)
            offset = amp * (1.0 - t**2)
            pt = (1.0 - t) * p0 + t * p1 + d * offset * perp
            ctrl.append(pt)
        ctrl.append(p1.copy())

        return np.asarray(ctrl, dtype=float)

    @staticmethod
    def _open_nonuniform_knot(n_ctrl, p):
        prefix = [0.0] * (p + 1)
        suffix = [1.0] * (p + 1)
        total_knots = n_ctrl + p + 1
        n_internal = total_knots - len(prefix) - len(suffix)

        if n_internal > 0:
            inner = [(i / (n_internal + 1)) ** 2 for i in range(1, n_internal + 1)]
        else:
            inner = []

        return np.array(prefix + inner + suffix, dtype=float)

    @staticmethod
    def _basis(i, p, u, knots):
        eps = 1e-12

        if p == 0:
            left = knots[i]
            right = knots[i + 1]
            if (left <= u < right) or (abs(u - knots[-1]) < eps and abs(right - knots[-1]) < eps):
                return 1.0
            return 0.0

        denom1 = knots[i + p] - knots[i]
        denom2 = knots[i + p + 1] - knots[i + 1]

        term1 = 0.0
        if abs(denom1) > eps:
            term1 = ((u - knots[i]) / denom1) * ConeMaskAvoider._basis(i, p - 1, u, knots)

        term2 = 0.0
        if abs(denom2) > eps:
            term2 = ((knots[i + p + 1] - u) / denom2) * ConeMaskAvoider._basis(i + 1, p - 1, u, knots)

        return term1 + term2

    # ---------------- NURBS ----------------
    @staticmethod
    def _make_nurbs_weights(n_ctrl, peak_weight):
        peak_weight = max(1e-6, float(peak_weight))
        if abs(peak_weight - 1.0) < 1e-12:
            return np.ones(int(n_ctrl), dtype=float)

        ts = np.linspace(0.0, 1.0, int(n_ctrl))
        shape = 1.0 - np.abs(2.0 * ts - 1.0)
        return 1.0 + (peak_weight - 1.0) * shape

    @staticmethod
    def build_path(p0, p1, d, amp, n_ctrl=7, n_sample=45, p=3,
                   peak_weight=1.0):
        n_ctrl = max(2, int(n_ctrl))
        p = max(1, min(int(p), n_ctrl - 1))
        n_sample = max(2, int(n_sample))

        ctrl = ConeMaskAvoider._build_ctrl_points(p0, p1, d, amp, n_ctrl)
        knots = ConeMaskAvoider._open_nonuniform_knot(len(ctrl), p)
        weights = ConeMaskAvoider._make_nurbs_weights(len(ctrl), peak_weight)

        t_vals = np.linspace(0.0, 1.0, n_sample)
        path = []

        for u in t_vals:
            num = np.zeros(2, dtype=float)
            den = 0.0
            for i in range(len(ctrl)):
                b = ConeMaskAvoider._basis(i, p, float(u), knots)
                wb = weights[i] * b
                num += wb * ctrl[i]
                den += wb

            if den < 1e-12:
                pt = ctrl[-1].copy()
            else:
                pt = num / den
            path.append(pt)

        path[-1] = np.asarray(p1, dtype=float)
        return path

    # ---------------- HUD/DBG ----------------
    def draw_path(self, vis):
        if self.last_path_px is not None and len(self.last_path_px) >= 2:
            cv2.polylines(vis, [self.last_path_px], False, (255, 255, 0), 2)

    def draw_hud(self, vis, roi_cnt, roi_total, L_cnt, R_cnt):
        if roi_cnt is not None:
            cv2.polylines(vis, [roi_cnt], True, (0, 255, 255), 2)
        cv2.line(vis, (self.cx, 0), (self.cx, self.h), (0, 255, 0), 2)

        playing = (not self.lane_forward)
        mode = "Drain" if self.obstacle_cleared and playing else ("Sliding" if playing else "Forward")
        cv2.putText(
            vis, mode, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
            (0, 0, 255) if playing else (0, 255, 0), 2
        )

        cv2.putText(
            vis,
            f"ROI_total={int(roi_total)}  L={int(L_cnt)}  R={int(R_cnt)}",
            (10, self.h - 40),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2
        )
        cv2.putText(
            vis,
            f"th_enter={self.cone_thresh_px}  th_exit={self.cone_resume_px}  triplet={self.triplet_period:.2f}s",
            (10, self.h - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2
        )

    def publish_dbg(self, vis):
        try:
            msg = self.bridge.cv2_to_imgmsg(vis, 'bgr8')
            if self.latest_vis_hdr is not None:
                msg.header = self.latest_vis_hdr
            self.pub_dbg_img.publish(msg)
        except Exception as e:
            self.get_logger().error(str(e))


def main(args=None, *, avoider_cls=ConeMaskAvoider):
    rclpy.init(args=args)
    node = avoider_cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.timer_handle is not None:
            node.timer_handle.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
