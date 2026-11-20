"""Pure gate state machine. All event handlers run on one ROS executor thread."""
import math

ZERO = (0.0, 0.0)
ZERO_FRAME = bytes(8)
COMMAND_TIMEOUT = .5
CAN_TIMEOUT = .1


class Gate:
    def __init__(self):
        self.enabled = False
        self.estop = False
        self.manual_held = False
        self.paused = True
        self.joy_at = -math.inf
        self.joy_stamp = -1
        self.auto_at = -math.inf
        self.auto_stamp = -1
        self.resume_stamp = math.inf
        self.manual = self.auto = ZERO
        self.reason = 'startup'
        self.zero_issued = False
        self.fence = math.inf
        self.last_can = -1
        self.ready = False
        self.last_receipt = -math.inf
        self.sent = ZERO

    def barrier(self, stamp, reason):
        self.ready = False
        self.fence = stamp
        self.zero_issued = False
        self.last_can = stamp
        self.last_receipt = -math.inf
        self.sent = ZERO
        self.reason = reason

    def pause(self, stamp, reason):
        self.paused = True
        self.auto_at = -math.inf
        self.resume_stamp = math.inf
        self.barrier(stamp, reason)

    def enable(self, enabled, stamp):
        if enabled and (self.estop or self.manual_held):
            return False
        self.enabled = enabled
        self.pause(stamp, 'output_enabled' if enabled else 'output_disabled')
        return True

    def emergency(self, active, stamp):
        if self.estop != active:
            self.estop = active
            self.enabled = False
            self.pause(stamp, 'estop' if active else 'estop_released')

    def joy(self, held, command, source, stamp, now):
        if not 0 <= stamp-source <= COMMAND_TIMEOUT or source <= self.joy_stamp:
            return
        self.joy_at, self.joy_stamp = now, source
        if self.manual_held != held:
            self.pause(stamp, 'manual_takeover' if held else 'manual_released')
        self.manual_held = held
        self.manual = command if held else ZERO

    def resume(self, stamp, now):
        if not self.enabled or self.estop or self.manual_held or now-self.joy_at > COMMAND_TIMEOUT:
            return False
        self.pause(stamp, 'waiting_new_auto')
        self.paused = False
        self.resume_stamp = stamp
        return True

    def auto_is_fresh(self, stamp, now):
        return (0 <= now-self.auto_at <= COMMAND_TIMEOUT
                and 0 <= stamp-self.auto_stamp <= COMMAND_TIMEOUT)

    def check_auto_time(self, stamp, now):
        if self.auto_at == -math.inf or self.auto_is_fresh(stamp, now):
            return
        age = stamp-self.auto_stamp
        if (0 <= now-self.auto_at <= COMMAND_TIMEOUT
                and age > COMMAND_TIMEOUT):
            if self.reason != 'waiting_fresh_auto':
                self.barrier(stamp, 'waiting_fresh_auto')
        else:
            self.pause(stamp, 'auto_timeout')

    def command(self, value, source, stamp, now):
        if self.paused or self.estop or not self.enabled or self.manual_held:
            return 'inactive'
        # Enforce deadlines even if a new callback runs before the watchdog tick.
        self.check_auto_time(stamp, now)
        if self.paused:
            return 'timed_out'
        if source <= max(self.resume_stamp, self.auto_stamp):
            return 'before_resume_or_duplicate'
        if not 0 <= stamp-source <= COMMAND_TIMEOUT:
            return 'expired_or_future'
        if not all(map(math.isfinite, value)):
            return 'nonfinite'
        self.auto, self.auto_stamp, self.auto_at = value, source, now
        self.reason = 'auto_command_received'
        return 'accepted'

    def demand(self, stamp, now):
        if not self.enabled or self.estop:
            return ZERO
        if now-self.joy_at > COMMAND_TIMEOUT:
            if self.reason != 'joy_timeout':
                self.manual = ZERO
                self.pause(stamp, 'joy_timeout')
            return ZERO
        if self.manual_held:
            return self.manual
        if self.paused or self.auto_at == -math.inf:
            return ZERO
        self.check_auto_time(stamp, now)
        if self.paused or self.reason == 'waiting_fresh_auto':
            return ZERO
        return self.auto

    def tick(self, stamp, now):
        demand = self.demand(stamp, now)
        if self.ready and now-self.last_receipt > CAN_TIMEOUT:
            self.enabled = False
            self.pause(stamp, 'motor_timeout')
            demand = ZERO
        # A new zero must reach the legacy converter and be acknowledged before
        # issuing nonzero commands after an authority transition.
        if demand == ZERO and self.sent != ZERO:
            self.barrier(stamp, 'zero_requested')
        if not self.ready and not self.zero_issued:
            self.fence = stamp
            self.zero_issued = True
        self.sent = demand if self.ready else ZERO
        return self.sent

    def motor(self, payload, source, stamp, now):
        demand = self.demand(stamp, now)
        if not self.enabled or self.estop or source <= self.last_can or not 0 <= stamp-source <= CAN_TIMEOUT:
            return ZERO_FRAME
        self.last_can = source
        if not self.ready:
            if self.zero_issued and payload == ZERO_FRAME and source > self.fence:
                self.ready = True
            self.last_receipt = now
            return ZERO_FRAME
        if now-self.last_receipt > CAN_TIMEOUT:
            self.enabled = False
            self.pause(stamp, 'motor_timeout')
            return ZERO_FRAME
        self.last_receipt = now
        return payload if demand != ZERO and self.sent != ZERO else ZERO_FRAME
