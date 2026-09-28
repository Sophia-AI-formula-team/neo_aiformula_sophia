"""ROS-independent safety state machine and bounded Lyapunov-style tracker.

Coordinates are ENU metres/radians. This is a guarded body-frame tracking law,
not the legacy controller's singular 1/sin(alpha) expression. The operational
gates and saturation invalidate any claim of an unconditional stability proof.
"""
from dataclasses import dataclass
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
from typing import Optional


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lower, upper):
    return min(upper, max(lower, value))


def sinc(value):
    return math.sin(value) / value if abs(value) > 1e-6 else 1.0 - value * value / 6.0


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    speed: float
    stamp_ns: int
    received_steady_ns: int


@dataclass(frozen=True)
class Command:
    speed: float = 0.0
    yaw_rate: float = 0.0


class ControlFault(ValueError):
    pass


def fresh(stamp_ns, received_steady_ns, now_ns, steady_ns, timeout_s,
          future_tolerance_s=0.02):
    return (now_ns > 0 and stamp_ns > 0 and received_steady_ns > 0
            and -future_tolerance_s <= (now_ns - stamp_ns) * 1e-9 <= timeout_s
            and 0.0 <= (steady_ns - received_steady_ns) * 1e-9 <= timeout_s)


def _read_json(path, maximum_bytes=16 * 1024 * 1024):
    if path.stat().st_size > maximum_bytes:
        raise ValueError("bundle file exceeds size limit")
    raw = path.read_bytes()
    if len(raw) > maximum_bytes:
        raise ValueError("bundle file exceeds size limit")
    decoded = json.loads(raw.decode("utf-8"))
    if not isinstance(decoded, dict):
        raise ValueError("bundle JSON must be an object")
    return decoded, hashlib.sha256(raw).hexdigest()


def load_bundle(manifest_path):
    """Only ready, hash-checked bundles whose files remain in their directory."""
    path = Path(manifest_path).expanduser().resolve(strict=True)
    manifest, _ = _read_json(path)
    if manifest.get("schema_version") != 1 or manifest.get("ready") is not True:
        raise ValueError("bundle is not ready schema version 1")
    origin = manifest.get("origin_lla", [])
    if (not isinstance(origin, list) or len(origin) != 3
            or not all(math.isfinite(float(x)) for x in origin)
            or abs(float(origin[0])) > 90 or abs(float(origin[1])) > 180):
        raise ValueError("invalid saved LLA origin")
    if manifest.get("heading_source") != "vectornav_yaw_ned_to_enu":
        raise ValueError("unsupported heading convention")
    offset = float(manifest.get("yaw_offset_rad", 0.0))
    if not math.isfinite(offset):
        raise ValueError("invalid yaw offset")
    payloads = {}
    for kind in ("route", "metadata"):
        relative = Path(manifest[kind + "_path"])
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("bundle paths must be relative and cannot escape")
        target = (path.parent / relative).resolve(strict=True)
        try:
            target.relative_to(path.parent)
        except ValueError:
            raise ValueError("bundle target escapes directory")
        payload, digest = _read_json(target)
        if digest != manifest.get(kind + "_sha256"):
            raise ValueError(kind + " checksum mismatch")
        payloads[kind] = payload
    if payloads["metadata"].get("finished") is not True:
        raise ValueError("bundle metadata is not a completed lap")
    for key in ("origin_lla", "yaw_offset_rad", "frame_id", "heading_source"):
        if key not in manifest or any(payloads[kind].get(key) != manifest[key]
                                     for kind in ("route", "metadata")):
            raise ValueError("bundle coordinate metadata mismatch: " + key)
    if not isinstance(manifest["frame_id"], str) or not manifest["frame_id"]:
        raise ValueError("bundle frame_id is invalid")
    return manifest, payloads["route"], payloads["metadata"]


class ClosedRouteController:
    """Monotonic arc-length matching, restricted to a reachable forward window.

    Initial attachment is only at route start. Every later nearest-point query
    searches a bounded reachable arc interval, never the entire loop/other lane.
    A non-None reference speed requires matching fixed-reference route metadata;
    infeasible bends are rejected, not silently turned into a slower profile.
    """
    def __init__(self, route, maximum_speed=0.8, maximum_yaw_rate=0.4,
                 maximum_acceleration=0.5, maximum_yaw_acceleration=0.8,
                 maximum_lateral_acceleration=0.35, maximum_cross_track=0.8,
                 maximum_heading_error=1.0, start_distance=1.0,
                 start_heading_error=0.6, k_x=0.5, k_y=1.2, k_yaw=1.0,
                 reference_speed_mps=None):
        if route.get("schema_version") != 1 or route.get("closed") is not True:
            raise ValueError("only a validated closed schema-1 route is accepted")
        if route.get("diagnostics", {}).get("valid") is not True:
            raise ValueError("route diagnostics do not report a valid candidate")
        self.reference_speed_mps = reference_speed_mps
        policy = route.get("speed_policy", "profile")
        if reference_speed_mps is None:
            if policy != "profile":
                raise ValueError("route speed policy requires explicit fixed reference configuration")
        else:
            if (isinstance(reference_speed_mps, bool)
                    or not math.isfinite(float(reference_speed_mps))
                    or float(reference_speed_mps) <= 0):
                raise ValueError("reference speed must be finite and positive")
            self.reference_speed_mps = float(reference_speed_mps)
            saved_reference = route.get("reference_speed_mps")
            if (policy != "fixed_reference" or isinstance(saved_reference, bool)
                    or not isinstance(saved_reference, (int, float))
                    or not math.isfinite(saved_reference)
                    or abs(saved_reference - self.reference_speed_mps) > 1e-9):
                raise ValueError("route speed policy/reference mismatch; rebuild the fixed reference route")
        samples = route.get("route_samples", [])
        if not 8 <= len(samples) <= 5000:
            raise ValueError("route must contain 8..5000 samples")
        fields = ("s_m", "x_m", "y_m", "yaw_rad", "curvature_1pm", "speed_mps")
        self.samples = []
        for item in samples:
            row = {field: float(item[field]) for field in fields}
            if not all(math.isfinite(value) for value in row.values()):
                raise ValueError("route contains non-finite values")
            if row["speed_mps"] <= 0 or row["speed_mps"] > 8.0:
                raise ValueError("invalid route speed profile")
            if (self.reference_speed_mps is not None
                    and abs(row["speed_mps"] - self.reference_speed_mps) > 1e-9):
                raise ValueError("fixed reference route contains a different speed profile")
            if self.samples and row["s_m"] <= self.samples[-1]["s_m"]:
                raise ValueError("route arc lengths must strictly increase")
            self.samples.append(row)
        if abs(self.samples[0]["s_m"]) > 1e-6:
            raise ValueError("route must begin at s=0")
        tail, head = self.samples[-1], self.samples[0]
        closing = math.hypot(tail["x_m"] - head["x_m"], tail["y_m"] - head["y_m"])
        if closing < 1e-6:
            raise ValueError("route must not duplicate its seam endpoint")
        self.length = tail["s_m"] + closing
        if self.length < 2.0 or closing > 2.0:
            raise ValueError("route seam is open or route is too short")
        for a, b in zip(self.samples, self.samples[1:]):
            chord = math.hypot(b["x_m"] - a["x_m"], b["y_m"] - a["y_m"])
            if chord <= 1e-6 or abs((b["s_m"] - a["s_m"]) - chord) > max(0.05, chord * 0.15):
                raise ValueError("route arc length inconsistent with geometry")
        # Independently verify the bundle's tangent and curvature, including its
        # seam. A checksum proves file identity, not geometric correctness.
        for index, current in enumerate(self.samples):
            before, after = self.samples[index - 1], self.samples[(index + 1) % len(self.samples)]
            ax, ay = current["x_m"] - before["x_m"], current["y_m"] - before["y_m"]
            bx, by = after["x_m"] - current["x_m"], after["y_m"] - current["y_m"]
            chord = math.hypot(ax + bx, ay + by)
            denominator = math.hypot(ax, ay) * math.hypot(bx, by) * chord
            if denominator < 1e-9:
                raise ValueError("route tangent is degenerate")
            geometric_yaw = math.atan2(ay + by, ax + bx)
            geometric_curvature = 2.0 * (ax * by - ay * bx) / denominator
            if abs(wrap(current["yaw_rad"] - geometric_yaw)) > 1e-4:
                raise ValueError("route yaw is inconsistent with geometry")
            if abs(current["curvature_1pm"] - geometric_curvature) > max(1e-4, abs(geometric_curvature) * 1e-3):
                raise ValueError("route curvature is inconsistent with geometry")
        self.maximum_speed = float(maximum_speed)
        self.maximum_yaw_rate = float(maximum_yaw_rate)
        self.maximum_acceleration = float(maximum_acceleration)
        self.maximum_yaw_acceleration = float(maximum_yaw_acceleration)
        self.maximum_lateral_acceleration = float(maximum_lateral_acceleration)
        self.maximum_cross_track = float(maximum_cross_track)
        self.maximum_heading_error = float(maximum_heading_error)
        self.start_distance = float(start_distance)
        self.start_heading_error = float(start_heading_error)
        self.k_x, self.k_y, self.k_yaw = float(k_x), float(k_y), float(k_yaw)
        if any(not math.isfinite(v) or v <= 0 for v in (
                self.maximum_speed, self.maximum_yaw_rate, self.maximum_acceleration,
                self.maximum_yaw_acceleration, self.maximum_lateral_acceleration,
                self.maximum_cross_track, self.maximum_heading_error,
                self.start_distance, self.start_heading_error, self.k_x, self.k_y,
                self.k_yaw)):
            raise ValueError("controller limits and gains must be finite and positive")
        if self.reference_speed_mps is not None:
            if self.reference_speed_mps > self.maximum_speed:
                raise ValueError("fixed speed infeasible: reference={:.6g} m/s; allowed={:.6g} m/s".format(
                    self.reference_speed_mps, self.maximum_speed))
            self._check_fixed_speed(max(abs(row["curvature_1pm"]) for row in self.samples))
        self.progress = 0.0
        self.arc_lengths = [row["s_m"] for row in self.samples]
        self.last_command = Command()
        self.attached = False
        self.last_metrics = {}

    def _check_fixed_speed(self, curvature):
        reference = self.reference_speed_mps
        required_yaw = reference * abs(curvature)
        required_lateral = reference ** 2 * abs(curvature)
        if (required_yaw > self.maximum_yaw_rate + 1e-9
                or required_lateral > self.maximum_lateral_acceleration + 1e-9):
            raise ControlFault(
                "fixed speed infeasible: reference={:.6g} m/s; required yaw={:.6g} rad/s "
                "(allowed={:.6g}), required lateral acceleration={:.6g} m/s^2 "
                "(allowed={:.6g})".format(reference, required_yaw, self.maximum_yaw_rate,
                    required_lateral, self.maximum_lateral_acceleration))

    def attach(self, pose):
        head = self.samples[0]
        if math.hypot(pose.x - head["x_m"], pose.y - head["y_m"]) > self.start_distance:
            raise ControlFault("vehicle is not at the route start")
        if abs(wrap(pose.yaw - head["yaw_rad"])) > self.start_heading_error:
            raise ControlFault("vehicle heading does not match the route start")
        self.progress = 0.0
        self.last_command = Command()
        self.attached = True

    def _reference(self, pose, dt):
        minimum = self.progress
        maximum = minimum + self.maximum_speed * dt + 0.15
        best = None
        # O(N) with a bounded route-size gate; geometry candidates remain local.
        base_lap = int(minimum // self.length)
        for lap in (base_lap, base_lap + 1):
            first = max(0, bisect_right(self.arc_lengths, minimum - lap * self.length) - 1)
            for index in range(first, len(self.samples)):
                a = self.samples[index]
                b = self.samples[(index + 1) % len(self.samples)]
                s0 = lap * self.length + a["s_m"]
                s1 = lap * self.length + (b["s_m"] if index + 1 < len(self.samples) else self.length)
                if s0 > maximum:
                    break
                if s1 <= s0 or s1 < minimum or s0 > maximum:
                    continue
                dx, dy = b["x_m"] - a["x_m"], b["y_m"] - a["y_m"]
                squared = dx * dx + dy * dy
                if squared < 1e-12:
                    continue
                lo = max(0.0, (minimum - s0) / (s1 - s0))
                hi = min(1.0, (maximum - s0) / (s1 - s0))
                fraction = clamp(((pose.x - a["x_m"]) * dx + (pose.y - a["y_m"]) * dy) / squared, lo, hi)
                x, y = a["x_m"] + fraction * dx, a["y_m"] + fraction * dy
                distance = math.hypot(pose.x - x, pose.y - y)
                candidate_s = s0 + fraction * (s1 - s0)
                if best is None or distance < best[0]:
                    reference = dict(a)
                    reference.update(x_m=x, y_m=y, s_m=candidate_s,
                                     yaw_rad=a["yaw_rad"] + fraction * wrap(b["yaw_rad"] - a["yaw_rad"]),
                                     curvature_1pm=a["curvature_1pm"] + fraction * (b["curvature_1pm"] - a["curvature_1pm"]),
                                     speed_mps=min(a["speed_mps"], b["speed_mps"]))
                    best = distance, reference
        if best is None or best[0] > self.maximum_cross_track:
            raise ControlFault("route distance/progress gate exceeded")
        self.progress = best[1]["s_m"]
        return best[1], best[0]

    def update(self, pose, dt):
        if not self.attached:
            raise ControlFault("controller is not attached")
        if not 0.001 <= dt <= 0.25:
            raise ControlFault("control timer missed its deadline")
        if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw, pose.speed)):
            raise ControlFault("non-finite pose")
        reference, distance = self._reference(pose, dt)
        dx, dy = reference["x_m"] - pose.x, reference["y_m"] - pose.y
        e_x = math.cos(pose.yaw) * dx + math.sin(pose.yaw) * dy
        e_y = -math.sin(pose.yaw) * dx + math.cos(pose.yaw) * dy
        e_yaw = wrap(reference["yaw_rad"] - pose.yaw)
        if abs(e_yaw) > self.maximum_heading_error:
            raise ControlFault("route heading gate exceeded")
        curvature = reference["curvature_1pm"]
        if self.reference_speed_mps is not None:
            self._check_fixed_speed(curvature)
            v_ref = self.reference_speed_mps
        else:
            v_ref = min(reference["speed_mps"], self.maximum_speed)
            if abs(curvature) > 1e-6:
                v_ref = min(v_ref, math.sqrt(self.maximum_lateral_acceleration / abs(curvature)),
                            self.maximum_yaw_rate / abs(curvature))
        speed = clamp(v_ref * math.cos(e_yaw) + self.k_x * e_x, 0.0, self.maximum_speed)
        omega = v_ref * curvature + self.k_y * v_ref * sinc(e_yaw) * e_y + self.k_yaw * e_yaw
        omega = clamp(omega, -self.maximum_yaw_rate, self.maximum_yaw_rate)
        requested_speed, requested_omega = speed, omega
        speed = clamp(speed, self.last_command.speed - self.maximum_acceleration * dt,
                      self.last_command.speed + self.maximum_acceleration * dt)
        omega = clamp(omega, self.last_command.yaw_rate - self.maximum_yaw_acceleration * dt,
                      self.last_command.yaw_rate + self.maximum_yaw_acceleration * dt)
        self.last_command = Command(speed, omega)
        self.last_metrics = dict(progress_m=self.progress, lap=int(self.progress // self.length),
                                 speed_policy="fixed_reference" if self.reference_speed_mps is not None else "profile",
                                 reference_speed_mps=self.reference_speed_mps,
                                 cross_track_m=distance, heading_error_rad=e_yaw,
                                 v_ref_mps=v_ref, omega_ref_rps=v_ref * curvature,
                                 requested_speed_mps=requested_speed,
                                 requested_yaw_rate_rps=requested_omega)
        return self.last_command


class SafetyArbiter:
    """No automatic rearming; fixed_only forbids teacher use after handover.

    Preserve mode leaves the teacher's accepted command unchanged. A zero
    teacher speed ceiling means unconfigured private replay, not vehicle
    approval: the ROS deployment boundary must reject it for vehicle output.
    """
    def __init__(self, maximum_speed=0.8, maximum_yaw_rate=0.4,
                 pose_timeout_s=0.25, teacher_timeout_s=0.25,
                 stopped_speed=0.08, stopped_settle_s=1.0,
                 safety_mode="fixed_only", reference_speed_delta=0.35,
                 reference_yaw_delta=0.15, reference_recovery_s=0.5,
                 accepted_teacher_max_speed=2.25, maximum_acceleration=0.5,
                 maximum_yaw_acceleration=0.8, preserve_teacher_command=False,
                 accepted_teacher_max_yaw_rate=None):
        if safety_mode not in ("fixed_only", "lya_reference"):
            raise ValueError("unknown safety mode")
        self.safety_mode = safety_mode
        if not isinstance(preserve_teacher_command, bool):
            raise ValueError("preserve_teacher_command must be boolean")
        self.preserve_teacher_command = preserve_teacher_command
        self.accepted_teacher_max_speed = accepted_teacher_max_speed
        if (isinstance(accepted_teacher_max_speed, bool)
                or not math.isfinite(float(accepted_teacher_max_speed))
                or accepted_teacher_max_speed < 0
                or (accepted_teacher_max_speed == 0 and not preserve_teacher_command)):
            raise ValueError("teacher speed limit must be positive, or zero for private preserve-mode replay")
        self.accepted_teacher_max_yaw_rate = (maximum_yaw_rate if accepted_teacher_max_yaw_rate is None
                                               else accepted_teacher_max_yaw_rate)
        if any(not math.isfinite(float(v)) or float(v) <= 0 for v in (
                maximum_speed, maximum_yaw_rate, pose_timeout_s, teacher_timeout_s,
                stopped_speed, stopped_settle_s, reference_speed_delta,
                reference_yaw_delta, reference_recovery_s, self.accepted_teacher_max_yaw_rate,
                maximum_acceleration, maximum_yaw_acceleration)):
            raise ValueError("all safety limits must be finite and positive")
        self.reference_speed_delta = reference_speed_delta
        self.reference_yaw_delta = reference_yaw_delta
        self.reference_recovery_s = reference_recovery_s
        self.reference_agree_since_ns = None
        self.maximum_speed = maximum_speed
        self.maximum_yaw_rate = maximum_yaw_rate
        self.maximum_acceleration = maximum_acceleration
        self.maximum_yaw_acceleration = maximum_yaw_acceleration
        self.last_output = Command()
        self.pose_timeout_s = pose_timeout_s
        self.teacher_timeout_s = teacher_timeout_s
        self.stopped_speed = stopped_speed
        self.stopped_settle_s = stopped_settle_s
        self.state = "DISARMED"
        self.reason = "explicit arm required"
        self.pose = None
        self.teacher = None
        self.raw_teacher = None
        self.teacher_stamp_ns = 0
        self.teacher_received_ns = 0
        self.repeat_selected = False
        self.controller = None
        self.manual_estop = False
        self.stopped_since_ns = None

    def hold(self, reason):
        if self.state != "ESTOP":
            self.state = "HOLD"
        self.reason = reason
        self.last_output = Command()

    def estop(self, reason):
        self.state, self.reason = "ESTOP", reason
        self.last_output = Command()

    def reset_estop(self):
        if self.manual_estop:
            return False, "manual emergency stop is still asserted"
        if self.state != "ESTOP":
            return False, "emergency stop is not latched"
        self.state, self.reason = "DISARMED", "reset complete; explicit arm required"
        self.teacher = None
        self.last_output = Command()
        return True, self.reason

    def update_pose(self, pose):
        if (self.pose is None or (pose.received_steady_ns - self.pose.received_steady_ns)
                * 1e-9 > self.pose_timeout_s):
            self.stopped_since_ns = None
        self.pose = pose
        if abs(pose.speed) <= self.stopped_speed:
            if self.stopped_since_ns is None:
                self.stopped_since_ns = pose.received_steady_ns
        else:
            self.stopped_since_ns = None

    def set_teacher(self, values, now_ns, steady_ns):
        if self.repeat_selected and self.safety_mode == "fixed_only":
            return True  # No teacher influence after independent handover.
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            self.teacher = None
            self.hold("invalid teacher command")
            return False
        speed, ly, lz, ax, ay, omega = values
        if (any(abs(v) > 1e-6 for v in (ly, lz, ax, ay))
                or speed < 0.0
                or (self.accepted_teacher_max_speed > 0 and speed > self.accepted_teacher_max_speed)
                or abs(omega) > self.accepted_teacher_max_yaw_rate):
            self.teacher = None
            self.hold("teacher command outside configured envelope")
            return False
        self.raw_teacher = Command(speed, omega)
        ratio = (1.0 if self.preserve_teacher_command or speed <= 1e-6
                 else min(1.0, self.maximum_speed / speed))
        self.teacher = Command(speed * ratio, omega * ratio)
        self.teacher_stamp_ns = now_ns
        self.teacher_received_ns = steady_ns
        if speed <= 0.0:
            self.last_output = Command()
            if self.safety_mode == "lya_reference" and self.state in ("REPEAT", "FALLBACK"):
                self.reference_agree_since_ns = None
                self.state, self.reason = "FALLBACK", "reference teacher requested stop"
        return True

    def pose_fresh(self, now_ns, steady_ns):
        p = self.pose
        return p is not None and fresh(p.stamp_ns, p.received_steady_ns, now_ns,
                                       steady_ns, self.pose_timeout_s)

    def teacher_fresh(self, now_ns, steady_ns):
        return self.teacher is not None and fresh(self.teacher_stamp_ns,
            self.teacher_received_ns, now_ns, steady_ns, self.teacher_timeout_s)

    def arm(self, now_ns, steady_ns):
        if self.state == "ESTOP" or self.repeat_selected:
            return False, "estop latched or repeat selected; teacher cannot be armed"
        if not self.pose_fresh(now_ns, steady_ns) or not self.teacher_fresh(now_ns, steady_ns):
            return False, "fresh VectorNav pose and teacher command required"
        self.state, self.reason = "TEACH", "teacher command selected"
        self.last_output = Command()
        return True, self.reason

    def prepare_repeat(self, controller):
        if self.state == "ESTOP":
            return False, "reset emergency stop first"
        self.repeat_selected = True
        self.controller = controller
        self.stopped_since_ns = None
        self.hold("repeat loaded; stop, settle, then call start_repeat again")
        return True, self.reason

    def start_repeat(self, now_ns, steady_ns):
        if self.state == "ESTOP" or self.controller is None:
            return False, "no ready route or emergency stop latched"
        if not self.pose_fresh(now_ns, steady_ns):
            return False, "fresh VectorNav pose required"
        if self.safety_mode == "lya_reference" and not self.teacher_fresh(now_ns, steady_ns):
            return False, "reference mode requires a fresh teacher command"
        if (self.stopped_since_ns is None or abs(self.pose.speed) > self.stopped_speed
                or (steady_ns - self.stopped_since_ns) * 1e-9 < self.stopped_settle_s):
            return False, "vehicle must remain stopped through the settle interval"
        try:
            self.controller.attach(self.pose)
        except ControlFault as error:
            return False, str(error)
        self.state = "REPEAT"
        self.reason = ("fixed route selected; LYA safety reference active"
                       if self.safety_mode == "lya_reference"
                       else "fixed route selected; teacher disabled")
        self.last_output = Command()
        return True, self.reason

    def resume_reference(self, now_ns, steady_ns):
        if self.safety_mode != "lya_reference" or self.state != "FALLBACK":
            return False, "reference fallback is not active"
        if not self.pose_fresh(now_ns, steady_ns) or not self.teacher_fresh(now_ns, steady_ns):
            return False, "fresh pose and teacher required"
        if (self.reference_agree_since_ns is None or (steady_ns - self.reference_agree_since_ns)
                * 1e-9 < self.reference_recovery_s):
            return False, "fixed and teacher commands have not agreed through recovery interval"
        self.state, self.reason = "REPEAT", "operator resumed fixed route after stable agreement"
        return True, self.reason

    def _output(self, target, dt):
        """Repeat and legacy-mode limiter, including mode transitions.

        The controller's own limiter remains useful when used independently.
        Applying identical limits twice does not halve the ramp; the final
        limiter also protects transitions from a different command source.
        A requested zero bypasses slew limits and clears angular motion too.
        """
        if target.speed <= 0.0:
            self.last_output = Command()
            return self.last_output
        if not all(math.isfinite(v) for v in (target.speed, target.yaw_rate)):
            self.hold("non-finite output command")
            return Command()
        speed = clamp(target.speed, 0.0, self.maximum_speed)
        omega = clamp(target.yaw_rate, -self.maximum_yaw_rate, self.maximum_yaw_rate)
        self.last_output = Command(
            clamp(speed, self.last_output.speed - self.maximum_acceleration * dt,
                  self.last_output.speed + self.maximum_acceleration * dt),
            clamp(omega, self.last_output.yaw_rate - self.maximum_yaw_acceleration * dt,
                  self.last_output.yaw_rate + self.maximum_yaw_acceleration * dt))
        return self.last_output

    def _teacher_output(self, dt):
        if not self.preserve_teacher_command:
            return self._output(self.teacher, dt)
        # The real LYA owns TEACH/FALLBACK commands. Its configured acceptance
        # envelope and all mode/freshness/stop gates still apply; repeat's
        # speed cap and startup slew must not silently rewrite this command.
        target = self.teacher
        if (target is None or not all(math.isfinite(v) for v in (target.speed, target.yaw_rate))
                or target.speed < 0
                or (self.accepted_teacher_max_speed > 0 and target.speed > self.accepted_teacher_max_speed)
                or abs(target.yaw_rate) > self.accepted_teacher_max_yaw_rate):
            self.hold("teacher command outside configured envelope")
            return Command()
        self.last_output = target if target.speed > 0.0 else Command()
        return self.last_output

    def command(self, now_ns, steady_ns, dt):
        if self.state not in ("TEACH", "REPEAT", "FALLBACK"):
            self.last_output = Command()
            return Command()
        if not math.isfinite(dt) or not 0.001 <= dt <= 0.25:
            self.hold("control timer missed its deadline")
            return Command()
        if not self.pose_fresh(now_ns, steady_ns):
            self.stopped_since_ns = None
            self.hold("VectorNav pose stale or clock invalid")
            return Command()
        if self.state == "TEACH":
            if not self.teacher_fresh(now_ns, steady_ns):
                self.hold("teacher command stale or clock invalid")
                return Command()
            return self._teacher_output(dt)
        if self.safety_mode == "lya_reference" and not self.teacher_fresh(now_ns, steady_ns):
            self.hold("reference teacher command stale or clock invalid")
            return Command()
        if self.safety_mode == "lya_reference" and self.teacher.speed <= 0:
            self.reference_agree_since_ns = None
            self.state, self.reason = "FALLBACK", "reference teacher requested stop"
            return self._output(Command(), dt)
        try:
            candidate = self.controller.update(self.pose, dt)
        except ControlFault as error:
            self.reference_agree_since_ns = None
            if self.safety_mode == "lya_reference":
                self.state, self.reason = "FALLBACK", "route controller rejected: " + str(error)
                return self._teacher_output(dt)
            else:
                self.hold(str(error))
                return Command()
        if self.safety_mode == "lya_reference":
            # Compare desired bounded commands, not the startup ramp. Otherwise
            # a correct .8 m/s teacher disagrees with the first .025 m/s step.
            nominal = self.controller.last_metrics
            # LYA is an upper safety reference, not a minimum speed mandate.
            # Compare steering at the same (lower) speed so a normal slower bend
            # does not look like disagreement merely because omega=v*curvature.
            ratio = min(1.0, nominal["requested_speed_mps"] / self.teacher.speed)
            reference = Command(self.teacher.speed * ratio, self.teacher.yaw_rate * ratio)
            agrees = (nominal["requested_speed_mps"] <= self.teacher.speed + self.reference_speed_delta
                      and abs(nominal["requested_yaw_rate_rps"] - reference.yaw_rate) <= self.reference_yaw_delta)
            if agrees:
                if self.reference_agree_since_ns is None:
                    self.reference_agree_since_ns = steady_ns
            else:
                self.reference_agree_since_ns = None
                self.state, self.reason = "FALLBACK", "fixed/teacher command disagreement"
            if self.state == "FALLBACK":
                if self.preserve_teacher_command:
                    return self._teacher_output(dt)
                # While a valid candidate requests slowing, yaw disagreement
                # must not accelerate the car back to the faster teacher speed.
                return self._output(reference, dt)
        return self._output(candidate, dt)
