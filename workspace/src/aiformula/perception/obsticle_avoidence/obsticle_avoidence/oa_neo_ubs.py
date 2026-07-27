#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32, Bool
from cv_bridge import CvBridge


class ConeMaskAvoider(Node):
    """
    主体状态机保持不变：

    1) roi_total > cone_thresh_px:
       - lane_forward=False
       - stop_flag=True
       - 取消 timer
       - 重新规划路径并立即发布 start/mid/end 三点

    2) roi_total < cone_resume_px 且 current_path 存在:
       - 启动 timer，按 triplet_period 分批发送剩余点

    3) 剩余点发完:
       - lane_forward=True
       - stop_flag=False
       - 清空避障状态
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
        self.desired_x_m     = float(self.declare_parameter('desired_x_m', 0.3).value)

        self.trap_top_ratio  = float(self.declare_parameter('trap_top_ratio', 0.5).value)
        self.trap_left_ratio, self.trap_right_ratio = 0.373, 0.625
        self.trap_bottom_left_ratio, self.trap_bottom_right_ratio = 0.15, 0.85

        self.overlay_mask    = bool(self.declare_parameter('overlay_mask', True).value)

        # ========= ROS IO =========
        self.bridge = CvBridge()
        self.create_subscription(Image, '/cone_bbox_image', self.cone_image_cb, 10)
        self.create_subscription(Image, '/cone_mask', self.mask_cb, 10)

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

        self.pub_dbg_img   = self.create_publisher(Image, '/processed_image', 10)
        self.pub_cnt       = self.create_publisher(Int32, '/cone_pixels_count', 10)
        self.pub_stop_flag = self.create_publisher(Bool, '/lane_stop_flag', 10)

        # ========= 状态 =========
        self.latest_vis_img = None
        self.latest_vis_hdr = None
        self.latest_mask    = None

        self.h = self.w = self.cx = 0

        self.lane_forward = True

        self.current_path = []
        self.path_idx = 0
        self.reference_point = None
        self.timer_handle = None

        self.last_path_px = None

        self.create_timer(0.03, self.tick)

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
        except Exception:
            self.latest_mask = None

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

    # ---------------- 主循环（检测端） ----------------
    def tick(self):
        if self.latest_vis_img is None or self.latest_mask is None:
            return

        # timer 在跑时，不重新规划，只做可视化
        if self.timer_handle is not None:
            vis = self.latest_vis_img.copy()
            self.h, self.w = vis.shape[:2]
            self.cx = self.w // 2

            roi_cnt, roi_total, L_cnt, R_cnt, col_mask = self._compute_roi_counts(vis, self.latest_mask)

            if self.overlay_mask and col_mask is not None:
                overlay = vis.copy()
                overlay[col_mask > 0] = (0, 0, 255)
                vis = cv2.addWeighted(overlay, 0.25, vis, 0.75, 0.0)

            if self.last_path_px is not None and len(self.last_path_px) >= 2:
                cv2.polylines(vis, [self.last_path_px], False, (255, 255, 0), 2)

            self.draw_hud(vis, roi_cnt, roi_total, L_cnt, R_cnt)
            self.publish_dbg(vis)
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

        # (1) 进入避障：每次检测到障碍都允许更新 path
        if roi_total > self.cone_thresh_px:
            if self.lane_forward:
                self.lane_forward = False
                self.pub_stop_flag.publish(Bool(data=True))

            if self.timer_handle is not None:
                self.timer_handle.cancel()
                self.timer_handle = None

            self.compute_and_store_path(col_mask, vis)
            self.draw_hud(vis, roi_cnt, roi_total, L_cnt, R_cnt)
            self.publish_dbg(vis)
            return

        # (2) 障碍消失，开始按 triplet 发送剩余点
        if (not self.lane_forward) and (roi_total < self.cone_resume_px) and self.current_path:
            if self.timer_handle is None:
                self.timer_handle = self.create_timer(self.triplet_period, self.broadcast_triplet)

        self.draw_hud(vis, roi_cnt, roi_total, L_cnt, R_cnt)
        self.publish_dbg(vis)

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
    def compute_and_store_path(self, col_mask, vis):
        if col_mask is None:
            return

        ys, xs = np.where(col_mask > 0)
        if xs.size == 0:
            return

        left_cnt = int(np.sum(xs < self.cx))
        right_cnt = int(np.sum(xs >= self.cx))

        # 方向仍然由左右占比决定
        if left_cnt == 0 and right_cnt > 0:
            d, amp_m = +1, self.single_side_amp
        elif right_cnt == 0 and left_cnt > 0:
            d, amp_m = -1, self.single_side_amp
        else:
            if left_cnt < right_cnt:
                d = +1
                # 取左侧里最靠近中心线的点
                dist_px = self.cx - int(xs[xs < self.cx].max())
            else:
                d = -1
                # 取右侧里最靠近中心线的点
                dist_px = int(xs[xs >= self.cx].min()) - self.cx
            amp_m = max(dist_px * self.m_per_px + self.extra_safe_m, self.min_offset_m)

        p0_m = self.px2robot((self.cx, self.h))
        p1_m = self.px2robot((self.cx, int(self.h * self.trap_top_ratio)))

        path = self.build_path(
            p0_m, p1_m, d, amp_m,
            n_ctrl=self.n_ctrl,
            n_sample=self.n_sample,
            p=self.spline_deg
        )
        if len(path) < 3:
            return

        self.current_path = path
        self.path_idx = 0

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

        for pub, p in zip(self.triplet_pubs, [p_start, p_mid, p_end]):
            pub.publish(Pose2D(x=float(p[0]), y=float(p[1]), theta=0.0))

        self.reference_point = p_mid.copy()

        # 保持你当前主体逻辑：后续从 mid 后面开始继续发
        self.path_idx = idx_mid + 1

        if vis is not None:
            dbg_px = [self.robot2px(p) for p in path]
            self.last_path_px = np.array(dbg_px, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(vis, [self.last_path_px], False, (255, 255, 0), 2)

    # ---------------- 分批发送 ----------------
    def broadcast_triplet(self):
        if self.path_idx >= len(self.current_path):
            if self.timer_handle is not None:
                self.timer_handle.cancel()
                self.timer_handle = None

            self.current_path = []
            self.path_idx = 0
            self.reference_point = None
            self.last_path_px = None

            self.lane_forward = True
            self.pub_stop_flag.publish(Bool(data=False))
            return

        chunk = [p.copy() for p in self.current_path[self.path_idx:self.path_idx + 3]]
        while len(chunk) < 3:
            chunk.append(chunk[-1].copy())

        rel_chunk = [p - self.reference_point for p in chunk] if self.reference_point is not None else chunk

        for pub, rel in zip(self.triplet_pubs, rel_chunk):
            pub.publish(Pose2D(x=float(rel[0]), y=float(rel[1]), theta=0.0))

        self.reference_point = chunk[1].copy()
        self.path_idx += 3

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
    def _open_uniform_knot(n_ctrl, p):
        prefix = [0.0] * (p + 1)
        suffix = [1.0] * (p + 1)
        total_knots = n_ctrl + p + 1
        n_internal = total_knots - len(prefix) - len(suffix)

        if n_internal > 0:
            inner = [i / (n_internal + 1) for i in range(1, n_internal + 1)]
        else:
            inner = []

        return np.array(prefix + inner + suffix, dtype=float)

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


    # ---------------- 均匀 B 样条 ----------------
    @staticmethod
    def build_path(p0, p1, d, amp, n_ctrl=7, n_sample=45, p=3):
        n_ctrl = max(2, int(n_ctrl))
        p = max(1, min(int(p), n_ctrl - 1))
        n_sample = max(2, int(n_sample))

        ctrl = ConeMaskAvoider._build_ctrl_points(p0, p1, d, amp, n_ctrl)
        knots = ConeMaskAvoider._open_uniform_knot(len(ctrl), p)

        t_vals = np.linspace(0.0, 1.0, n_sample)
        path = []

        for u in t_vals:
            pt = np.zeros(2, dtype=float)
            for i in range(len(ctrl)):
                b = ConeMaskAvoider._basis(i, p, float(u), knots)
                pt += b * ctrl[i]
            path.append(pt)

        path[-1] = np.asarray(p1, dtype=float)
        return path

    # ---------------- HUD/DBG ----------------
    def draw_hud(self, vis, roi_cnt, roi_total, L_cnt, R_cnt):
        if roi_cnt is not None:
            cv2.polylines(vis, [roi_cnt], True, (0, 255, 255), 2)
        cv2.line(vis, (self.cx, 0), (self.cx, self.h), (0, 255, 0), 2)

        playing = (not self.lane_forward)
        txt = "Avoiding..." if playing else "Forward"
        cv2.putText(
            vis, txt, (10, 30),
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
