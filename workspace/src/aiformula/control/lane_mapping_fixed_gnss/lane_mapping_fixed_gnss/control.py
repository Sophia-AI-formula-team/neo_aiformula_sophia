"""ROS-independent safety policy for locally mapped, GNSS-free driving.

Inputs are metric poses already expressed in the frozen map frame. This module
never reads GNSS or converts LLA. A caller validates anchor windows and bundle
hashes, and calls ``visual_accept`` ONLY after its matcher transaction commits.
``invalidate_callback`` must synchronously cancel the shared worker fence and
return its new generation. A diagnostic/event callback is not a commit fence.
"""
from collections import deque
import math

from lane_mapping_lya_reference.controller import (
    ClosedRouteController, Command, Pose, SafetyArbiter, fresh,
)


DEFAULT_CONFIG = {
    "command_output_topic": "/lane_learning_gnss/cmd_vel",
    "enable_vehicle_output": False,
    "hardware_stop_verified": False,
    "motor_zero_passthrough_verified": False,
    "maximum_speed_mps": 0.8,
    "maximum_yaw_rate_rps": 0.4,
    "maximum_acceleration_mps2": 0.5,
    "maximum_yaw_acceleration_rps2": 0.8,
    "accepted_teacher_max_speed_mps": 2.25,
    "accepted_teacher_max_yaw_rate_rps": 0.4,
    "preserve_teacher_command": False,
    "pose_timeout_s": 0.25,
    "teacher_timeout_s": 0.25,
    "teacher_state_timeout_s": 1.0,
    "visual_max_age_s": 0.5,
    "visual_min_consecutive_matches": 3,
    "maximum_match_compute_ms": 40.0,
    "stopped_speed_mps": 0.08,
    "stopped_settle_s": 1.0,
    "position_jump_tolerance_m": 0.35,
    "motion_maximum_speed_mps": 8.0,
    "heading_jump_tolerance_rad": 0.05,
    "motion_maximum_yaw_rate_rps": 3.0,
    "visual_max_total_correction_m": 1.0,
    "visual_max_total_yaw_rad": 0.25,
}


def _stamp(value):
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


class RuntimeSafety:
    """Local TEACH -> reviewed model; independent GNSS evidence gates REPEAT.

    All times are integer nanoseconds. Source time uses the ROS clock; receipt
    time uses one monotonic clock. No fault recovery automatically rearms.
    ``Pose.speed`` is a nonnegative planar measured speed, not commanded speed.
    """
    def __init__(self, config=None, event_sink=None, invalidate_callback=None):
        self.config = dict(DEFAULT_CONFIG)
        if config is not None:
            if not isinstance(config, dict) or set(config) - set(DEFAULT_CONFIG):
                raise ValueError("unknown runtime safety configuration")
            self.config.update(config)
        for key, default in DEFAULT_CONFIG.items():
            value = self.config[key]
            if isinstance(default, bool):
                if not isinstance(value, bool):
                    raise ValueError("boolean required: " + key)
            elif isinstance(default, str):
                if not isinstance(value, str) or not value.startswith("/"):
                    raise ValueError("absolute command topic required")
            else:
                unconfigured_teacher = (key == "accepted_teacher_max_speed_mps"
                    and self.config["preserve_teacher_command"] and value == 0)
                if (isinstance(value, bool) or not isinstance(value, (int, float))
                        or not math.isfinite(value) or (value <= 0 and not unconfigured_teacher)):
                    raise ValueError("finite positive limit required: " + key)
        count = self.config["visual_min_consecutive_matches"]
        if not isinstance(count, int) or count < 3:
            raise ValueError("at least three visual confirmations required")
        for key, ceiling in (("pose_timeout_s", .25), ("teacher_timeout_s", .25),
                ("teacher_state_timeout_s", 1.), ("visual_max_age_s", .5),
                ("maximum_match_compute_ms", 40.)):
            if self.config[key] > ceiling:
                raise ValueError("freshness/compute gate cannot be weakened: " + key)
        c = self.config
        self.arbiter = SafetyArbiter(
            maximum_speed=c["maximum_speed_mps"], maximum_yaw_rate=c["maximum_yaw_rate_rps"],
            maximum_acceleration=c["maximum_acceleration_mps2"],
            maximum_yaw_acceleration=c["maximum_yaw_acceleration_rps2"],
            accepted_teacher_max_speed=c["accepted_teacher_max_speed_mps"],
            accepted_teacher_max_yaw_rate=c["accepted_teacher_max_yaw_rate_rps"],
            preserve_teacher_command=c["preserve_teacher_command"],
            pose_timeout_s=c["pose_timeout_s"], teacher_timeout_s=c["teacher_timeout_s"],
            stopped_speed=c["stopped_speed_mps"], stopped_settle_s=c["stopped_settle_s"],
            safety_mode="fixed_only")
        self.phase = "START_ANCHOR_PENDING"
        self._gnss_window = "start"
        self.generation = 0
        self.start_anchor_id = self.end_anchor_id = self.bundle_id = None
        self.endpoint_proximity_validated = False
        self.ready_controller = None
        self.motion_pose = None
        self.last_accepted_motion = None
        self.last_motion_stamp_ns = None
        self.motion_stopped_since_ns = None
        self.visual_streak = 0
        self.visual_source_ns = self.visual_receipt_ns = None
        self.last_visual_seen_ns = None
        self.visual_correction = (0.0, 0.0, 0.0)
        self.prepare_source_ns = self.prepare_receipt_ns = None
        self.teacher_status = None
        self.teacher_status_source_ns = self.teacher_status_receipt_ns = None
        self.last_teacher_source_ns = None
        self.last_now_ns = self.last_steady_ns = None
        self.clock_fault = False
        self.last_command = Command()
        self.events = deque(maxlen=256)
        self.event_sink = event_sink
        self.invalidate_callback = invalidate_callback
        self.logging_error = None
        self._event("startup", config=dict(c), localization_source="metric_odometry",
                    runtime_gnss_enabled=False)

    @property
    def state(self):
        return self.arbiter.state

    @property
    def reason(self):
        return self.arbiter.reason

    @property
    def teacher_enabled(self):
        # The private LYA process may remain alive through freeze/review. The
        # arbiter forwards nothing in HOLD; managed shutdown begins at prepare.
        return not self.arbiter.repeat_selected

    @property
    def gnss_window(self):
        return self._gnss_window

    def _event(self, event, **fields):
        record = dict(event=event, state=self.state, phase=self.phase,
                      generation=self.generation, **fields)
        self.events.append(record)
        if self.event_sink is not None:
            try:
                self.event_sink(record)
            except Exception as error:
                self.logging_error = str(error)
                self.arbiter.estop("safety journal failed")
                self.last_command = Command()

    def _invalidate(self, reason):
        # The external fence is revoked synchronously, before confidence changes.
        if self.invalidate_callback is None:
            self.generation += 1
        else:
            try:
                generation = self.invalidate_callback()
                if not isinstance(generation, int) or isinstance(generation, bool) or generation <= self.generation:
                    raise ValueError("commit fence generation did not advance")
                self.generation = generation
            except Exception as error:
                self.arbiter.estop("worker fence invalidation failed: " + str(error))
                self.generation += 1
        self.visual_streak = 0
        self.visual_source_ns = self.visual_receipt_ns = None
        self.arbiter.stopped_since_ns = None
        if self.arbiter.repeat_selected:
            self.arbiter.pose = None
        self.last_command = Command()
        self._event("localization_invalidated", reason=str(reason))

    def _fail(self, reason):
        self.arbiter.hold(str(reason))
        self._invalidate(reason)
        return False, str(reason)

    def _times_ok(self, now_ns, steady_ns):
        if (self.clock_fault or not _stamp(now_ns) or not _stamp(steady_ns)
                or (self.last_now_ns is not None and now_ns < self.last_now_ns)
                or (self.last_steady_ns is not None and steady_ns < self.last_steady_ns)):
            if not self.clock_fault:
                self.clock_fault = True
                self._fail("clock invalid or regressed; restart required")
            return False
        self.last_now_ns, self.last_steady_ns = now_ns, steady_ns
        return True

    def _motion_fresh(self, now_ns, steady_ns):
        p = self.motion_pose
        return p is not None and fresh(p.stamp_ns, p.received_steady_ns,
            now_ns, steady_ns, self.config["pose_timeout_s"])

    def _stationary(self, now_ns, steady_ns):
        return (self._motion_fresh(now_ns, steady_ns)
            and self.motion_pose.speed <= self.config["stopped_speed_mps"]
            and self.motion_stopped_since_ns is not None
            and steady_ns - self.motion_stopped_since_ns >= int(self.config["stopped_settle_s"] * 1e9))

    def _deployment_issue(self):
        c = self.config
        if not c["hardware_stop_verified"]:
            return "physical emergency stop has not been verified"
        if (c["command_output_topic"] != "/lane_learning_gnss/cmd_vel"
                and c["preserve_teacher_command"] and c["accepted_teacher_max_speed_mps"] <= 0):
            return "vehicle output requires an explicitly approved positive teacher speed ceiling"
        if c["command_output_topic"] != "/lane_learning_gnss/cmd_vel" and not (
                c["enable_vehicle_output"] and c["motor_zero_passthrough_verified"]):
            return "vehicle command output and motor zero passthrough are not verified"
        return None

    def anchor_window(self, now_ns, steady_ns):
        if (not self._times_ok(now_ns, steady_ns) or self.state == "ESTOP"
                or not self._stationary(now_ns, steady_ns)):
            return None
        return self.gnss_window

    def accept_start_anchor(self, anchor_id, now_ns, steady_ns):
        if self.anchor_window(now_ns, steady_ns) != "start" or not isinstance(anchor_id, str) or not anchor_id:
            self._event("anchor_rejected", reason="start anchor requires a validated stopped window")
            return False, "start anchor requires a validated window while stopped"
        self.start_anchor_id = anchor_id
        self._gnss_window = None
        self._event("start_anchor_accepted", anchor_id=anchor_id, geometry_changed=False)
        return True, "optional start GNSS evidence recorded; local geometry unchanged"

    def close_anchor_window(self, reason="endpoint window closed"):
        previous = self._gnss_window
        self._gnss_window = None
        self._event("anchor_window_closed", window=previous, reason=str(reason), geometry_changed=False)

    def begin_teach_session(self, now_ns, steady_ns):
        if (not self._times_ok(now_ns, steady_ns) or self.phase != "START_ANCHOR_PENDING"
                or self.state not in ("DISARMED", "HOLD") or not self._stationary(now_ns, steady_ns)):
            return self._fail("begin teaching requires the initial stopped local-odometry boundary")
        self.phase = "TEACH_READY"
        self._gnss_window = None
        # The caller resets its LOCAL odometry origin at this boundary, whether
        # or not optional GNSS was observed. GNSS callbacks never reset it. Do not
        # compare the next zero-origin pose with the pre-anchor coordinates, but
        # retain the source-stamp frontier so delayed old messages stay rejected.
        self.motion_pose = self.arbiter.pose = None
        self.last_accepted_motion = None
        self.motion_stopped_since_ns = None
        self._invalidate("local teaching session started; metric origin reset boundary")
        self._event("teach_session_started", optional_start_anchor_id=self.start_anchor_id)
        return True, "local origin reset requested; fresh stopped interval then explicit teacher arm required"

    def arm_teach(self, now_ns, steady_ns):
        if not self._times_ok(now_ns, steady_ns):
            return False, self.reason
        issue = self._deployment_issue()
        if (issue or not self.teacher_enabled
                or self.phase not in ("TEACH_READY", "TEACH")):
            return self._fail(issue or "explicit local teach session required")
        if not self._motion_fresh(now_ns, steady_ns):
            return self._fail("fresh metric Odometry required")
        if not self._stationary(now_ns, steady_ns):
            return self._fail("vehicle must remain stopped through the teacher-arm settle interval")
        self.arbiter.update_pose(self.motion_pose)
        ok, reason = self.arbiter.arm(now_ns, steady_ns)
        if not ok:
            return self._fail(reason.replace("VectorNav", "metric Odometry"))
        self.phase = "TEACH"
        self._event("teacher_armed")
        return True, reason

    def stop(self, reason="operator stop; explicit resume required"):
        return self._fail(reason)

    def begin_end_anchor(self, now_ns, steady_ns):
        if (not self._times_ok(now_ns, steady_ns) or self.phase != "TEACH"
                or self.state != "HOLD" or not self._stationary(now_ns, steady_ns)):
            return self._fail("end anchor window requires operator-stopped teaching lap")
        self.phase = "END_ANCHOR_PENDING"
        self._gnss_window = "end"
        self._invalidate("teacher lap ended; end anchor window opened")
        self._event("end_anchor_window_opened")
        return True, "teaching stopped; freeze/build model independently of optional end GNSS"

    def accept_end_anchor(self, anchor_id, now_ns, steady_ns, proximity_validated=False):
        if (self.anchor_window(now_ns, steady_ns) != "end" or not isinstance(anchor_id, str)
                or not anchor_id or not isinstance(proximity_validated, bool)):
            self._event("anchor_rejected", reason="end anchor requires a validated stopped window")
            return False, "end anchor requires a validated window while stopped"
        self.end_anchor_id = anchor_id
        self.endpoint_proximity_validated = proximity_validated
        self._gnss_window = None
        self._event("end_anchor_accepted", anchor_id=anchor_id, proximity_validated=proximity_validated,
                    geometry_changed=False)
        return True, "optional end GNSS evidence recorded; local model state unchanged"

    def set_ready(self, controller, bundle_id, now_ns, steady_ns):
        if (not self._times_ok(now_ns, steady_ns) or self.phase not in ("END_ANCHOR_PENDING", "MODEL_PENDING")
                or self.state != "HOLD"
                or not isinstance(controller, ClosedRouteController)
                or not isinstance(bundle_id, str) or not bundle_id):
            return self._fail("completed local teaching and reviewed valid route required")
        self.ready_controller, self.bundle_id = controller, bundle_id
        self.phase = "MODEL_READY"
        self._event("bundle_ready", bundle_id=bundle_id)
        return True, "bundle ready; explicit repeat preparation required"

    def prepare_repeat(self, now_ns, steady_ns):
        if (not self._times_ok(now_ns, steady_ns) or self.phase != "MODEL_READY"
                or self.state == "ESTOP" or self.ready_controller is None):
            return self._fail("ready reviewed bundle required before handover")
        if self.start_anchor_id is None or self.end_anchor_id is None or not self.endpoint_proximity_validated:
            self.arbiter.hold("start/end GNSS with validated coarse proximity required for repeat only")
            self.last_command = Command()
            self._event("repeat_preparation_denied", reason=self.reason, model_preserved=True)
            return False, self.reason
        ok, reason = self.arbiter.prepare_repeat(self.ready_controller)
        if not ok:
            return self._fail(reason)
        self.phase = "REPEAT_PREPARED"
        self._gnss_window = None
        self.prepare_source_ns, self.prepare_receipt_ns = now_ns, steady_ns
        self.teacher_status = None
        self._invalidate("repeat prepared; waiting for visual acquisition and managed stop")
        return True, self.reason

    def _teacher_stopped_fresh(self, now_ns, steady_ns):
        return (self.teacher_status == "STOPPED" and self.prepare_source_ns is not None
            and self.teacher_status_source_ns >= self.prepare_source_ns
            and self.teacher_status_receipt_ns >= self.prepare_receipt_ns
            and fresh(self.teacher_status_source_ns, self.teacher_status_receipt_ns,
                      now_ns, steady_ns, self.config["teacher_state_timeout_s"]))

    def start_repeat(self, now_ns, steady_ns):
        if not self._times_ok(now_ns, steady_ns):
            return False, self.reason
        issue = self._deployment_issue()
        if issue or not self.arbiter.repeat_selected or self.state == "ESTOP":
            return self._fail(issue or "repeat not prepared or emergency stop latched")
        if not self._teacher_stopped_fresh(now_ns, steady_ns):
            self.arbiter.hold("fresh managed LYA STOPPED after handover required")
            self.last_command = Command()
            return False, "fresh managed LYA STOPPED after handover required"
        if not self.visual_ready(now_ns, steady_ns):
            self.arbiter.hold("three fresh trusted mask commits required")
            self.last_command = Command()
            return False, "three fresh trusted mask commits required"
        self._update_control_pose(now_ns, steady_ns)
        ok, reason = self.arbiter.start_repeat(now_ns, steady_ns)
        if not ok:
            self.arbiter.hold(reason.replace("VectorNav", "metric Odometry"))
            self.last_command = Command()
            return False, self.reason
        self.phase = "REPEAT"
        self._event("repeat_started")
        return True, self.reason

    def update_pose(self, pose, now_ns, steady_ns):
        if not self._times_ok(now_ns, steady_ns):
            return False
        valid = (isinstance(pose, Pose)
            and all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw, pose.speed))
            and 0 <= pose.speed <= self.config["motion_maximum_speed_mps"]
            and _stamp(pose.stamp_ns) and _stamp(pose.received_steady_ns)
            and fresh(pose.stamp_ns, pose.received_steady_ns, now_ns, steady_ns, self.config["pose_timeout_s"])
            and (self.last_motion_stamp_ns is None or pose.stamp_ns > self.last_motion_stamp_ns))
        if not valid:
            self.motion_pose = None
            self.motion_stopped_since_ns = None
            self._fail("invalid, stale, future or nonincreasing metric Odometry")
            return False
        previous = self.last_accepted_motion
        if previous is not None:
            dt = (pose.stamp_ns - previous.stamp_ns) * 1e-9
            distance = math.hypot(pose.x - previous.x, pose.y - previous.y)
            allowed = self.config["position_jump_tolerance_m"] + max(previous.speed, pose.speed) * dt
            yaw_allowed = self.config["heading_jump_tolerance_rad"] + self.config["motion_maximum_yaw_rate_rps"] * dt
            if distance > allowed or abs(_wrap(pose.yaw - previous.yaw)) > yaw_allowed:
                self.motion_pose = None
                self.motion_stopped_since_ns = None
                self._fail("metric Odometry position or heading jump")
                return False
        if previous is None or pose.received_steady_ns - previous.received_steady_ns > int(self.config["pose_timeout_s"] * 1e9):
            self.motion_stopped_since_ns = None
        self.motion_pose, self.last_motion_stamp_ns = pose, pose.stamp_ns
        self.last_accepted_motion = pose
        if pose.speed <= self.config["stopped_speed_mps"]:
            if self.motion_stopped_since_ns is None:
                self.motion_stopped_since_ns = pose.received_steady_ns
        else:
            self.motion_stopped_since_ns = None
            if self.arbiter.repeat_selected and self.visual_streak < self.config["visual_min_consecutive_matches"]:
                self._fail("vehicle moved during visual acquisition")
        self._update_control_pose(now_ns, steady_ns)
        return True

    def _update_control_pose(self, now_ns, steady_ns):
        if not self._motion_fresh(now_ns, steady_ns):
            self.arbiter.pose = None
            self.arbiter.stopped_since_ns = None
            return
        p = self.motion_pose
        if self.arbiter.repeat_selected:
            if not self.visual_ready(now_ns, steady_ns):
                self.arbiter.pose = None
                self.arbiter.stopped_since_ns = None
                return
            dx, dy, yaw = self.visual_correction
            cosine, sine = math.cos(yaw), math.sin(yaw)
            p = Pose(cosine * p.x - sine * p.y + dx, sine * p.x + cosine * p.y + dy,
                     _wrap(p.yaw + yaw), p.speed, p.stamp_ns, p.received_steady_ns)
        self.arbiter.update_pose(p)

    def teacher_command(self, values, source_ns, now_ns, steady_ns):
        if self.arbiter.repeat_selected:
            return True  # No parsing, freshness refresh, fallback or actuation effect.
        if not self._times_ok(now_ns, steady_ns):
            return False
        if (not _stamp(source_ns) or not fresh(source_ns, steady_ns, now_ns, steady_ns,
                self.config["teacher_timeout_s"])
                or (self.last_teacher_source_ns is not None and source_ns <= self.last_teacher_source_ns)):
            self.arbiter.teacher = None
            self._fail("invalid, stale, future or nonincreasing teacher command")
            return False
        try:
            accepted = self.arbiter.set_teacher(tuple(values), now_ns, steady_ns)
        except (TypeError, ValueError, OverflowError):
            accepted = False
        if not accepted:
            self.arbiter.teacher = None
            self._fail("teacher command outside finite bounded envelope")
            return False
        self.last_teacher_source_ns = source_ns
        self.arbiter.teacher_stamp_ns = source_ns
        if self.arbiter.teacher.speed <= 0:
            self.last_command = Command()
        return True

    def teacher_state(self, state, source_ns, now_ns, steady_ns):
        if not self._times_ok(now_ns, steady_ns):
            return False
        if (state not in ("RUNNING", "STOPPING", "STOPPED", "FAILED") or not _stamp(source_ns)
                or not fresh(source_ns, steady_ns, now_ns, steady_ns, self.config["teacher_state_timeout_s"])
                or (self.teacher_status_source_ns is not None and source_ns <= self.teacher_status_source_ns)
                or (self.prepare_source_ns is not None and source_ns < self.prepare_source_ns)):
            self.teacher_status = None
            self._fail("managed teacher heartbeat is invalid, stale, future or before handover")
            return False
        self.teacher_status = state
        self.teacher_status_source_ns, self.teacher_status_receipt_ns = source_ns, steady_ns
        if state == "FAILED":
            self._fail("managed teacher reported failure")
            return False
        return True

    def visual_ready(self, now_ns, steady_ns):
        return (self.visual_streak >= self.config["visual_min_consecutive_matches"]
            and self.visual_source_ns is not None
            and fresh(self.visual_source_ns, self.visual_receipt_ns,
                      now_ns, steady_ns, self.config["visual_max_age_s"]))

    def visual_deadline(self, source_ns, received_steady_ns, now_ns, steady_ns):
        if (not self._times_ok(now_ns, steady_ns) or not _stamp(source_ns)
                or not _stamp(received_steady_ns) or not self._motion_fresh(now_ns, steady_ns)
                or not fresh(source_ns, received_steady_ns, now_ns, steady_ns, self.config["visual_max_age_s"])):
            return None
        visual_limit = int(self.config["visual_max_age_s"] * 1e9)
        pose_limit = int(self.config["pose_timeout_s"] * 1e9)
        p = self.motion_pose
        return min(steady_ns + visual_limit - (now_ns - source_ns), received_steady_ns + visual_limit,
                   steady_ns + pose_limit - (now_ns - p.stamp_ns), p.received_steady_ns + pose_limit)

    def visual_accept(self, source_ns, received_steady_ns, generation, now_ns, steady_ns,
                      correction_se2, compute_ms):
        if generation != self.generation:
            self._event("visual_commit_ignored", reason="obsolete generation")
            return False
        deadline = self.visual_deadline(source_ns, received_steady_ns, now_ns, steady_ns)
        try:
            correction = tuple(float(value) for value in correction_se2)
            valid = len(correction) == 3 and all(math.isfinite(value) for value in correction)
            valid = valid and math.isfinite(compute_ms) and 0 <= compute_ms <= self.config["maximum_match_compute_ms"]
        except (TypeError, ValueError, OverflowError):
            valid = False
        if (not self.arbiter.repeat_selected or deadline is None or deadline < steady_ns or not valid
                or (self.last_visual_seen_ns is not None and source_ns <= self.last_visual_seen_ns)):
            self._fail("visual commit invalid, expired, replayed or over compute budget")
            return False
        self.last_visual_seen_ns = source_ns
        if self.visual_streak < self.config["visual_min_consecutive_matches"] and (
                self.motion_pose.speed > self.config["stopped_speed_mps"] or self.state not in ("HOLD", "DISARMED", "ESTOP")):
            self._fail("visual acquisition requires stopped vehicle")
            return False
        p = self.motion_pose
        dx, dy, angle = correction
        distance = math.hypot(math.cos(angle) * p.x - math.sin(angle) * p.y + dx - p.x,
                              math.sin(angle) * p.x + math.cos(angle) * p.y + dy - p.y)
        if distance > self.config["visual_max_total_correction_m"] or abs(_wrap(angle)) > self.config["visual_max_total_yaw_rad"]:
            self._fail("visual correction outside total pose envelope")
            return False
        if self.visual_source_ns is not None and source_ns - self.visual_source_ns > int(self.config["visual_max_age_s"] * 1e9):
            self.visual_streak = 0
            self.arbiter.stopped_since_ns = None
        self.visual_streak += 1
        self.visual_source_ns, self.visual_receipt_ns = source_ns, received_steady_ns
        self.visual_correction = correction
        self._update_control_pose(now_ns, steady_ns)
        self._event("visual_committed", source_ns=source_ns, compute_ms=float(compute_ms),
                    correction_se2=list(correction), trusted_streak=self.visual_streak)
        return True

    def visual_reject(self, reason):
        return self._fail("visual localization rejected: " + str(reason))

    def estop(self, reason="operator emergency stop", manual=False):
        if manual:
            self.arbiter.manual_estop = True
        self.arbiter.estop(str(reason))
        self._invalidate(reason)
        return True, self.reason

    def release_manual_estop(self):
        self.arbiter.manual_estop = False
        self._event("manual_estop_released")

    def reset_estop(self):
        ok, reason = self.arbiter.reset_estop()
        self.last_command = Command()
        self._event("estop_reset", accepted=ok, reason=reason)
        return ok, reason

    def tick(self, now_ns, steady_ns, dt):
        if not self._times_ok(now_ns, steady_ns):
            return Command()
        if self.logging_error:
            self.estop("safety journal failed")
            return Command()
        if not self._motion_fresh(now_ns, steady_ns):
            self.motion_stopped_since_ns = None
            if self.state in ("TEACH", "REPEAT") or self.visual_source_ns is not None:
                self._fail("metric Odometry stale or clock invalid")
        if self.arbiter.repeat_selected and self.visual_source_ns is not None and not self.visual_ready(now_ns, steady_ns):
            # A fresh one/two-frame bootstrap is not yet a timeout failure.
            if not fresh(self.visual_source_ns, self.visual_receipt_ns, now_ns, steady_ns, self.config["visual_max_age_s"]):
                self._fail("mask localization expired")
        self._update_control_pose(now_ns, steady_ns)
        old_state = self.state
        try:
            self.last_command = self.arbiter.command(now_ns, steady_ns, dt)
        except Exception as error:
            self.estop("unexpected controller error: " + str(error))
            return Command()
        if old_state in ("TEACH", "REPEAT") and self.state == "HOLD":
            self._invalidate(self.reason.replace("VectorNav", "metric Odometry"))
        self._event("command", speed_mps=self.last_command.speed, yaw_rate_rps=self.last_command.yaw_rate,
                    pose_source="metric_odometry", runtime_gnss_enabled=False)
        return self.last_command

    def snapshot(self):
        return dict(state=self.state, phase=self.phase, reason=self.reason, generation=self.generation,
                    teacher_enabled=self.teacher_enabled, gnss_window=self.gnss_window,
                    start_anchor_id=self.start_anchor_id, end_anchor_id=self.end_anchor_id,
                    endpoint_proximity_validated=self.endpoint_proximity_validated,
                    bundle_id=self.bundle_id, visual_streak=self.visual_streak,
                    visual_source_ns=self.visual_source_ns, runtime_gnss_enabled=False,
                    command_output_topic=self.config["command_output_topic"])
