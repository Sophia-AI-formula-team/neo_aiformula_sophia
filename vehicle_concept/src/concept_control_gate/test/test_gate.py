from concept_control_gate.core import Gate, ZERO, ZERO_FRAME


def ready_auto():
    g = Gate()
    assert g.enable(True, 1)
    g.joy(False, ZERO, 1.01, 1.01, 1.01)
    assert g.resume(1.02, 1.02)
    g.tick(1.03, 1.03)
    g.motor(ZERO_FRAME, 1.04, 1.04, 1.04)
    return g


def test_new_command_required():
    g = ready_auto()
    g.command((1., 0.), 1.015, 1.05, 1.05)
    assert g.tick(1.05, 1.05) == ZERO
    g.command((1., 0.), 1.06, 1.06, 1.06)
    assert g.tick(1.07, 1.07) == (1., 0.)
    frame = bytes([1] * 8)
    assert g.motor(frame, 1.08, 1.08, 1.08) == frame


def test_takeover_release_and_estop():
    g = ready_auto()
    g.joy(True, (0., .5), 1.05, 1.05, 1.05)
    assert not g.resume(1.06, 1.06)
    g.tick(1.06, 1.06)
    assert g.motor(bytes([3]*8), 1.07, 1.07, 1.07) == ZERO_FRAME
    g.motor(ZERO_FRAME, 1.08, 1.08, 1.08)
    assert g.tick(1.09, 1.09) == (0., .5)
    g.joy(False, ZERO, 1.10, 1.10, 1.10)
    assert g.paused and g.tick(1.11, 1.11) == ZERO
    g.emergency(True, 1.12)
    assert not g.enabled and not g.enable(True, 1.13)
    g.emergency(False, 1.14)
    assert not g.enabled and g.paused


def test_motor_silence_stale_joy_and_disabled_manual():
    g = ready_auto()
    g.joy(True, (2., 0.), 1.05, 1.05, 1.05)
    g.tick(1.055, 1.055)
    g.motor(ZERO_FRAME, 1.06, 1.06, 1.06)
    assert g.tick(1.17, 1.17) == ZERO and not g.enabled
    assert not g.enable(True, 1.18)
    g.joy(False, ZERO, 1.01, 1.18, 1.18)
    assert g.manual_held
    g.joy(False, ZERO, 1.19, 1.19, 1.19)
    assert g.enable(True, 1.20)
    assert not g.resume(2, 2)


def test_frame_before_acknowledged_transition_is_never_forwarded():
    g = ready_auto()
    g.pause(1.08, 'pause')
    g.motor(bytes([1]*8), 1.079, 1.081, 1.081)
    assert not g.ready and g.paused
    g.tick(1.09, 1.09)
    g.motor(bytes([1]*8), 1.095, 1.096, 1.096)
    assert not g.ready


def test_capture_expiry_stops_until_zero_ack_and_fresh_frame():
    g = ready_auto()
    command = (1., 0.)
    moving = bytes([1] * 8)
    for t in (1.12, 1.20, 1.28, 1.36, 1.44):
        g.motor(ZERO_FRAME, t, t, t)
    g.joy(False, ZERO, 1.49, 1.49, 1.49)
    g.command(command, 1.06, 1.49, 1.49)
    g.motor(ZERO_FRAME, 1.50, 1.50, 1.50)
    assert g.tick(1.51, 1.51) == command
    assert g.motor(moving, 1.53, 1.53, 1.53) == moving
    assert g.motor(moving, 1.57, 1.57, 1.57) == ZERO_FRAME
    assert not g.paused and not g.auto_is_fresh(1.57, 1.57)
    assert g.tick(1.575, 1.575) == ZERO
    # Match the recorded gap: source age exceeds 600 ms while command delivery
    # is still within the 500 ms watchdog. Await a zero frame past the fence.
    g.command(command, 1.25, 1.73, 1.73)
    assert g.tick(1.735, 1.735) == ZERO
    assert g.motor(moving, 1.74, 1.74, 1.74) == ZERO_FRAME
    g.motor(ZERO_FRAME, 1.745, 1.745, 1.745)
    assert g.tick(1.75, 1.75) == command


def test_late_arrival_cannot_rearm_after_extended_gap():
    g = ready_auto()
    g.joy(False, ZERO, 1.49, 1.49, 1.49)
    g.command((1.,0.), 1.06, 1.49, 1.49)
    # A valid new command arrives before the next tick but beyond the deadline since the last valid command receipt.
    g.command((1.,0.), 1.60, 2.01, 2.01)
    assert g.paused and g.reason == 'auto_timeout'
    assert g.tick(2.01, 2.01) == ZERO


def test_silent_or_replayed_stream_still_latches_at_receipt_deadline():
    g = ready_auto()
    g.command((1.,0.), 1.06, 1.06, 1.06)
    g.joy(False, ZERO, 1.49, 1.49, 1.49)
    g.command((1.,0.), 1.06, 1.565, 1.565)
    assert g.paused and g.reason == 'auto_timeout'
