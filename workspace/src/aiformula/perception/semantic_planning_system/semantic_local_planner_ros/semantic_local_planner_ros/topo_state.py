"""MapAssist diagnostics and checkpoint lifecycle, independent of traffic state."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import math
import time
from pathlib import Path

from semantic_planner_core.rough_route import RouteSample
from semantic_local_planner_ros.topo_checkpoint import CheckpointStore, digest

# Changes affecting recording, continuity or reconstruction invalidate old maps.
MAP_PARAMETERS = (
    'odom_topic', 'route_node_ids', 'sample_spacing_m', 'smooth_window_m',
    'max_detour_gap_m', 'min_lap_distance_m', 'lap_closure_radius_m',
    'lap_closure_heading_deg', 'max_recorded_samples', 'alignment_max_rmse_m',
    'alignment_ambiguity_margin_m', 'fusion_node_search_m', 'fusion_feature_window_m',
    'fusion_shape_weight', 'fusion_max_displacement_m', 'max_odom_speed_mps',
    'max_odom_yaw_rate_radps', 'max_odom_position_std_m', 'max_odom_yaw_std_deg',
    'max_odom_gap_sec', 'point_timeout_sec', 'max_point_skew_sec',
)


def heading_difference(a, b):
    return abs(math.degrees(math.atan2(math.sin(a - b), math.cos(a - b))))


class TopoState:
    def _init_checkpoint(self):
        self.checkpoint = None
        self.checkpoint_worker = None
        self.checkpoint_future = None
        self.restore_pending = None
        self.restore_blocked = False
        self.checkpoint_dirty = False
        self.last_checkpoint_time = -math.inf
        self.last_topo_log_time = -math.inf
        self.persistence_status = dict(state='disabled', path=self.p['checkpoint_path'], error=None,
                                       saved_samples=0, saved_distance_m=0.0)
        if not self.p['checkpoint_path']:
            return
        if self.get_parameter('use_sim_time').value:
            self.persistence_status.update(state='disabled_sim_time')
            return
        try:
            fingerprint = digest(dict(
                map_sha256=hashlib.sha256(Path(self.p['topomap_path']).read_bytes()).hexdigest(),
                parameters={key: self.p[key] for key in MAP_PARAMETERS},
                reconstruction_version=1))
            self.checkpoint = CheckpointStore(self.p['checkpoint_path'], fingerprint,
                                              self.p['max_recorded_samples'])
            self.checkpoint_worker = ThreadPoolExecutor(max_workers=1)
            self.persistence_status['path'] = str(self.checkpoint.path)
            state = self.checkpoint.load()
            if state and state['samples']:
                self.restore_pending = state
                self.persistence_status.update(state='waiting_for_odom', saved_samples=len(state['samples']),
                                               saved_distance_m=state['recorded_distance'])
            else:
                self.persistence_status.update(state='new_map')
        except (OSError, ValueError, TypeError, KeyError) as exc:
            # Never silently replace an unreadable or incompatible previous map.
            self.restore_blocked = True
            self.persistence_status.update(state='restore_rejected', error=str(exc))
            self.get_logger().warn('Topo checkpoint: ' + str(exc))

    def _checkpoint_state(self):
        return dict(samples=[[s.x, s.y, s.yaw, s.excluded] for s in self.samples],
                    recorded_distance=self.recorded_distance, left_start_area=self.left_start_area,
                    excluded_since_sample=self.excluded_since_sample,
                    closed_at=list(self.closed_at) if self.closed_at is not None else None,
                    progress=self.progress, last_pose=list(self.odom) if self.odom is not None else None,
                    odom_frame=self.odom_frame, child_frame=self.child_frame, odom_stamp=self.odom_stamp)

    def _finish_checkpoint(self, wait=False):
        future = self.checkpoint_future
        if future is None or (not wait and not future.done()):
            return
        try:
            future.result()
            self.persistence_status.update(state='saved', error=None, **self.checkpoint_receipt)
        except (OSError, ValueError, TypeError) as exc:
            self.checkpoint_dirty = True
            self.persistence_status.update(state='save_failed', error=str(exc))
            self.get_logger().warn('Topo checkpoint save failed: ' + str(exc))
        self.checkpoint_future = None

    def _save_checkpoint(self, force=False):
        if getattr(self, 'checkpoint', None) is None:
            return
        self._finish_checkpoint(wait=force)
        if (self.restore_pending is not None or self.restore_blocked or not self.checkpoint_dirty
                or self.checkpoint_future is not None):
            return
        now = time.monotonic()
        if not force and now - self.last_checkpoint_time < self.p['checkpoint_interval_sec']:
            return
        self.last_checkpoint_time = now
        # Copy mutable state on the ROS callback thread. JSON + fsync run on a
        # single disk worker; routine saves cannot stall traffic/status callbacks.
        state = self._checkpoint_state()
        self.checkpoint_receipt = dict(saved_samples=len(self.samples), saved_distance_m=self.recorded_distance)
        self.checkpoint_dirty = False
        self.checkpoint_future = self.checkpoint_worker.submit(self.checkpoint.save, state)
        self._finish_checkpoint(wait=force)

    def close_checkpoint(self):
        self._save_checkpoint(force=True)
        if getattr(self, 'checkpoint_worker', None) is not None:
            self.checkpoint_worker.shutdown(wait=True)
            self.checkpoint_worker = None
        if getattr(self, 'checkpoint', None) is not None:
            self.checkpoint.close()
            self.checkpoint = None

    def _resume_checkpoint(self, current, frame, child_frame, stamp):
        if self.restore_blocked:
            return False
        state = self.restore_pending
        if state is None:
            return True
        distance = math.hypot(current[0] - state['last_pose'][0], current[1] - state['last_pose'][1])
        heading = heading_difference(current[2], state['last_pose'][2])
        self.persistence_status.update(resume_distance_m=distance, resume_heading_deg=heading)
        error = None
        if frame != state['odom_frame'] or child_frame != state['child_frame']:
            error = 'odom_frame_mismatch'
        elif state['odom_stamp'] is not None and stamp < state['odom_stamp']:
            error = 'odom_time_rewind'
        elif distance > self.p['checkpoint_resume_distance_m']:
            error = 'odom_position_mismatch'
        elif heading > self.p['checkpoint_resume_heading_deg']:
            error = 'odom_heading_mismatch'
        if error:
            self.restore_blocked = True
            self.persistence_status.update(state='restore_rejected', error=error)
            self.get_logger().warn('Topo resume rejected: ' + error + '; saved map preserved')
            return False
        samples = [RouteSample(*row) for row in state['samples']]
        fusion = None
        if state['closed_at'] is not None:
            try:
                fusion = self._build_reference(samples + [RouteSample(*state['closed_at'], excluded=False)])
            except ValueError as exc:
                self.restore_blocked = True
                self.persistence_status.update(state='restore_rejected', error='reference_build_failed: ' + str(exc))
                return False
        self.odom_frame, self.child_frame = state['odom_frame'], state['child_frame']
        self.samples = samples
        self.recorded_distance = state['recorded_distance']
        self.left_start_area = state['left_start_area']
        # No observations exist for the downtime segment; do not mark it trusted.
        self.excluded_since_sample = True
        self.closed_at = tuple(state['closed_at']) if state['closed_at'] is not None else None
        if fusion is not None:
            self._accept_reference(fusion)
            self.progress = state['progress']
            self._publish_path()
        self.restore_pending = None
        self.persistence_status.update(state='restored', error=None)
        self.reason = 'checkpoint_restored'
        self.get_logger().info('Topo restored: %d samples, %.2f m' % (len(samples), self.recorded_distance))
        return True

    def _closure_diagnostics(self, now):
        first = self.samples[0] if self.samples else None
        current = self.odom
        distance = math.hypot(current[0] - first.x, current[1] - first.y) if first and current else None
        heading = heading_difference(current[2], first.yaw) if first and current else None
        fresh = current is not None and now - self.odom_time <= self.p['odom_timeout_sec']
        exclusion = self._bypass_reason(now)
        points_valid = self._recording_points_valid(now)
        point_issues = []
        for label in 'abc':
            point = self.points[label]
            if point is None:
                point_issues.append(label + '_missing')
            elif now - self.point_times[label] > self.p['point_timeout_sec']:
                point_issues.append(label + '_stale')
            elif not all(math.isfinite(v) for v in (point.x, point.y, point.theta)):
                point_issues.append(label + '_nonfinite')
            elif point.x <= 0:
                point_issues.append(label + '_not_forward')
        if all(self.points[k] is not None for k in 'abc') and (
                max(self.point_times.values()) - min(self.point_times.values()) > self.p['max_point_skew_sec']):
            point_issues.append('abc_time_skew')
        checks = dict(
            restore_ready=not getattr(self, 'restore_blocked', False) and getattr(self, 'restore_pending', None) is None,
            odom_fresh=fresh, start_recorded=first is not None,
            left_start_area=self.left_start_area,
            recording_points_valid=points_valid, not_excluded=exclusion is None,
            within_start_radius=distance is not None and distance <= self.p['lap_closure_radius_m'],
            heading_aligned=heading is not None and heading <= self.p['lap_closure_heading_deg'],
            enough_samples=len(self.samples) >= 30,
            enough_distance=self.recorded_distance >= self.p['min_lap_distance_m'])
        blocked = [key for key, passed in checks.items() if not passed]
        if self.reason == 'recording_limit_reached_reset_required':
            blocked.append('recording_limit_reached')
        if self.closure_build_error:
            blocked.append('reference_build_failed')
        complete = self.reference is not None
        return dict(state='complete' if complete else ('blocked' if blocked else 'ready'),
                    blocker=None if complete else (blocked[0] if blocked else None),
                    blockers=[] if complete else blocked, checks=checks,
                    distance_to_start_m=distance, heading_error_deg=heading,
                    start_pose=([first.x, first.y, first.yaw] if first else None),
                    current_pose=(list(current) if current else None),
                    closure_radius_m=self.p['lap_closure_radius_m'],
                    closure_heading_deg=self.p['lap_closure_heading_deg'],
                    leave_start_radius_m=max(3.0, 2.0 * self.p['lap_closure_radius_m']),
                    recorded_distance_m=self.recorded_distance, min_distance_m=self.p['min_lap_distance_m'],
                    recorded_samples=len(self.samples), min_samples=30,
                    exclusion_reason=exclusion, point_issues=point_issues, build_error=self.closure_build_error,
                    last_learning_reset_reason=self.last_learning_reset_reason)

    def _topo_status_tick(self, now):
        self._save_checkpoint()
        if now - self.last_topo_log_time >= self.p['topo_log_interval_sec']:
            self.last_topo_log_time = now
            d = self._closure_diagnostics(now)
            distance = '?' if d['distance_to_start_m'] is None else '%.2f' % d['distance_to_start_m']
            heading = '?' if d['heading_error_deg'] is None else '%.1f' % d['heading_error_deg']
            self.get_logger().info(
                'Topo %s: start=%s/%g m, heading=%s/%g deg, path=%.2f/%g m, samples=%d/30; '
                'blocked=%s; checkpoint=%s%s' % (
                    d['state'], distance, d['closure_radius_m'], heading, d['closure_heading_deg'],
                    self.recorded_distance, d['min_distance_m'], len(self.samples),
                    ','.join(d['blockers']) or 'none', self.persistence_status['state'],
                    (':' + self.persistence_status['error']) if self.persistence_status.get('error') else ''))
