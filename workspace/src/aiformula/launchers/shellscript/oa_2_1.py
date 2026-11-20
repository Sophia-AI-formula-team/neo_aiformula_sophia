#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
import json
import math

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

    This node does not validate sensor freshness, lane boundaries or obstacle
    clearance. /oa/status reports tracking state; /lane_stop_flag keeps the
    0.2 second heartbeat used by the existing map-assistance node.
    """

    def __init__(self):
        super().__init__('cone_mask_avoider')

        # ========= 参数 =========
        self.m_per_px        = float(self.declare_parameter('meters_per_pixel', 0.01).value)
        self.min_offset_m    = float(self.declare_parameter('min_offset_amplitude_m', 0.5).value)
        self.single_side_amp = float(self.declare_parameter('single_side_amp', 0.30).value)
        self.extra_safe_m    = float(self.declare_parameter('extra_safe_m', 0.40).value)

        self.cone_thresh_px  = int(self.declare_parameter('cone_thresh_px', 1000).value)
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

        # ========= ROS IO =========
        self.bridge = CvBridge()
        self.create_subscription(Image, '/cone_bbox_image', self.cone_image_cb, 10)
        self.create_subscription(Image, '/cone_mask', self.mask_cb, 10)
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
        self.path_progress_m = 0.0
        self.remaining_m = None

        self.last_path_px = None

        self._publish_avoidance_flag()
        self.flag_timer = self.create_timer(0.2, self._publish_avoidance_flag)
        self.create_timer(0.03, self.tick)

    def _publish_avoidance_flag(self):
        """Publish whether OA currently owns the path-point outputs."""
        self.pub_stop_flag.publish(Bool(data=not self.lane_forward))
        self.pub_status.publish(String(data=json.dumps({
            'state': self.execution_state, 'avoidance_active': not self.lane_forward,
            'obstacle_cleared': self.obstacle_cleared, 'clear_frames': self.clear_frames,
            'direction': ('left' if self.avoid_direction == 1 else
                          'right' if self.avoid_direction == -1 else None),
            'odom_available': self.odom is not None, 'remaining_m': self.remaining_m,
        }, allow_nan=False)))

    # ---------------- 回调：底图/Mask ----------------
    def cone_image_cb(self, msg: Image):
        try:
            self.latest_vis_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_vis_hdr = msg.header
        except Exception:
            self.latest_vis_img = None
            self.latest_vis_hdr = None

    def mask_cb(self, msg: Image):
        try:
            self.latest_mask = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            self.mask_sequence += 1
            # Count each received mask while active, rather than counting only
            # the last one seen by a slower timer. An intervening high frame
            # must cancel an in-progress clear vote immediately.
            if not self.lane_forward and self.latest_vis_img is not None:
                self.h, self.w = self.latest_vis_img.shape[:2]
                self.cx = self.w // 2
                _, total, _, _, _ = self._compute_roi_counts(self.latest_vis_img, self.latest_mask)
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
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            self.odom = None
            return
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-8:
            self.odom = None
            return
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
        frame = (msg.header.frame_id or 'odom', msg.child_frame_id or '')
        self.odom = (float(p.x), float(p.y), yaw, frame)

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
        if self.latest_vis_img is None or self.latest_mask is None:
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

        if self.last_path_px is not None and len(self.last_path_px) >= 2:
            cv2.polylines(vis, [self.last_path_px], False, (255, 255, 0), 2)

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

        vis = self.latest_vis_img.copy()
        self.h, self.w = vis.shape[:2]
        self.cx = self.w // 2

        _, roi_total, _, _, col_mask = self._compute_roi_counts(vis, self.latest_mask)

        # Check again here: a new mask can arrive before the next tick.
        if roi_total >= self.cone_resume_px:
            self.clear_frames = 0
            self.obstacle_cleared = False
            self.compute_and_store_path(col_mask, vis, publish_window=True)
            return

        if not self.obstacle_cleared and self.execution_state in (
            'waiting_for_direction_recheck', 'switching_avoid_direction',
        ):
            return
        # Follow the last recorded path until odometry and clear observations
        # confirm it is time to return control.
        self.broadcast_triplet()

    def _compute_roi_counts(self, vis, mask):
        if mask.shape[:2] != vis.shape[:2]:
            mask = cv2.resize(mask, (vis.shape[1], vis.shape[0]), cv2.INTER_NEAREST)

        tl = (int(self.w * self.trap_left_ratio), int(self.h * self.trap_top_ratio))
        tr = (int(self.w * self.trap_right_ratio), int(self.h * self.trap_top_ratio))
        bl = (int(self.w * self.trap_bottom_left_ratio), self.h)
        br = (int(self.w * self.trap_bottom_right_ratio), self.h)
        roi_cnt = np.array([tl, tr, br, bl], np.int32).reshape((-1, 1, 2))

        roi_mask = np.zeros((self.h, self.w), np.uint8)
        cv2.fillPoly(roi_mask, [roi_cnt.reshape(-1, 2)], 255)

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
        self.path_progress_m = 0.0
        self.remaining_m = None
        self.last_path_px = None

    def compute_and_store_path(self, col_mask, vis=None, publish_window=True):
        if self.odom is None:
            self._discard_saved_path()
            self.execution_state = 'waiting_for_odom'
            return False
        self.execution_state = 'planning'
        plan = self.plan_path(col_mask)
        if plan is None:
            if self.execution_state == 'planning':
                self._discard_saved_path()
                self.execution_state = 'planning_unavailable'
            return False

        path, _, _, _, _ = plan
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
        self.current_path = path
        self.world_path = world
        self.path_cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        self.path_frame = frame
        self.path_progress_m = 0.0
        self.remaining_m = float(self.path_cumulative[-1])

        if publish_window:
            self.broadcast_triplet()

        if vis is not None:
            dbg_px = [self.robot2px(p) for p in path]
            self.last_path_px = np.array(dbg_px, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(vis, [self.last_path_px], False, (255, 255, 0), 2)

        return True

    def _reset_direction(self):
        self.avoid_direction = None
        self.offset_amplitude = None
        self.direction_checked_at_ns = None
        self.direction_mask_sequence = -1
        self.switching_direction = False

    def _select_direction(self, left_count, right_count):
        preferred = (
            self.avoid_direction if left_count == right_count and self.avoid_direction is not None
            else 1 if left_count < right_count else -1
        )
        now = self.get_clock().now().nanoseconds
        new_mask = self.mask_sequence != self.direction_mask_sequence
        self.direction_mask_sequence = self.mask_sequence

        if self.direction_checked_at_ns is None:
            self.avoid_direction = preferred
            self.direction_checked_at_ns = now
        elif new_mask:
            elapsed = now - self.direction_checked_at_ns
            if elapsed < 0 or elapsed >= self.direction_recheck_ns:
                if self.avoid_direction != preferred:
                    self.switching_direction = True
                self.avoid_direction = preferred
                self.direction_checked_at_ns = now

        if self.switching_direction and not new_mask:
            self.execution_state = 'switching_avoid_direction'
            return None

        occupied = left_count if self.avoid_direction == 1 else right_count
        opposite = right_count if self.avoid_direction == 1 else left_count
        if occupied and not opposite:
            # Let the current triplet expire while waiting for direction review.
            self.execution_state = 'waiting_for_direction_recheck'
            return None
        return self.avoid_direction

    def plan_path(self, col_mask):
        if col_mask is None:
            return None

        ys, xs = np.where(col_mask > 0)
        if xs.size == 0:
            return None

        left_cnt = int(np.sum(xs < self.cx))
        right_cnt = int(np.sum(xs >= self.cx))

        d = self._select_direction(left_cnt, right_cnt)
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
        p1_m = self.px2robot((self.cx, int(self.h * self.trap_top_ratio)))

        path = self.build_path(
            p0_m, p1_m, d, amp_m,
            n_ctrl=self.n_ctrl,
            n_sample=self.n_sample,
            p=self.spline_deg,
            peak_weight=self.nurbs_peak_weight
        )
        if len(path) < 3:
            return None

        desired_x = float(self.desired_x_m)

        p_start = None
        for i in range(1, len(path)):
            x0, y0 = float(path[i - 1][0]), float(path[i - 1][1])
            x1, y1 = float(path[i][0]), float(path[i][1])
            if (x0 <= desired_x <= x1) or (x1 <= desired_x <= x0):
                t = (desired_x - x0) / (x1 - x0 + 1e-12)
                p_start = np.array([desired_x, y0 + t * (y1 - y0)], dtype=float)
                break

        if p_start is None:
            p_start = path[1].copy()

        ys_list = [p[1] for p in path]
        idx_mid = int(np.argmax(ys_list)) if d == +1 else int(np.argmin(ys_list))
        p_mid = path[idx_mid].copy()
        p_end = path[-1].copy()

        return path, p_start, idx_mid, p_mid, p_end

    # ---------------- 发送剩余路径 ----------------
    def broadcast_triplet(self):
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

        # Find three distinct forward points on the real path. If a close
        # target is behind the current vehicle, try a farther lookahead; never
        # substitute repeated end points or a guessed vehicle position.
        minimum = min(self.desired_x_m, self.remaining_m / 4.0)
        min_forward = min(self.desired_x_m, self.remaining_m / 2.0)
        step = min(self.point_spacing_m / 2.0, max(0.02, self.remaining_m / 16.0))
        c, s = math.cos(yaw), math.sin(yaw)
        targets = None
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
                targets = trial
                break
        if targets is None:
            self.execution_state = 'target_not_forward'
            return False
        self.publish_window(targets)
        self.execution_state = ('tracking_after_clear' if self.obstacle_cleared
                                else 'tracking_avoidance')
        return True

    def publish_window(self, points):
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

    def _point_at_progress(self, distance):
        distance = max(0.0, min(float(distance), float(self.path_cumulative[-1])))
        index = min(int(np.searchsorted(self.path_cumulative, distance, side='right') - 1),
                    len(self.world_path) - 2)
        start, end = self.path_cumulative[index:index + 2]
        fraction = (distance - start) / (end - start)
        return self.world_path[index] + fraction * (self.world_path[index + 1] - self.world_path[index])

    def _finish_avoidance(self):
        if self.timer_handle is not None:
            self.timer_handle.cancel()
            self.timer_handle = None
        self._discard_saved_path()
        self.obstacle_cleared = False
        self.clear_frames = 0
        self._reset_direction()
        self.lane_forward = True
        self.execution_state = 'idle'
        self._publish_avoidance_flag()

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


def main(args=None):
    rclpy.init(args=args)
    node = ConeMaskAvoider()
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
