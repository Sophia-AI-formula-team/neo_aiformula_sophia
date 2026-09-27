"""Native metric-pose policy tests: no ROS, fake GNSS messages or vehicle IO."""
import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "lane_mapping_lya_reference"))
from lane_mapping_fixed_gnss.control import RuntimeSafety, ClosedRouteController, Command, Pose


def circle():
    count, radius = 360, 10.0
    chord = 2 * radius * math.sin(math.pi / count)
    return {"schema_version": 1, "closed": True, "diagnostics": {"valid": True},
        "route_samples": [dict(s_m=i * chord, x_m=radius * math.sin(i * 2 * math.pi / count),
            y_m=radius * (1 - math.cos(i * 2 * math.pi / count)), yaw_rad=i * 2 * math.pi / count,
            curvature_1pm=1 / radius, speed_mps=.4) for i in range(count)]}


class Scenario:
    def __init__(self, config=None, **kwargs):
        self.now = 1000000000
        self.safety = RuntimeSafety(dict(hardware_stop_verified=True, **(config or {})), **kwargs)

    def step(self, delta=.1, speed=0., x=0., y=0., yaw=0., teacher=True, visual=False):
        self.now += int(delta * 1e9)
        p = Pose(x, y, yaw, speed, self.now, self.now)
        assert self.safety.update_pose(p, self.now, self.now)
        if teacher:
            assert self.safety.teacher_command((.4, 0, 0, 0, 0, .04), self.now, self.now, self.now)
        if visual:
            assert self.safety.visual_accept(self.now, self.now, self.safety.generation,
                self.now, self.now, (0., 0., 0.), 2.0)
        return self.safety.tick(self.now, self.now, delta)

    def stationary(self, count=12, **kwargs):
        for _ in range(count):
            self.step(**kwargs)

    def teach(self, with_gnss=True):
        self.stationary()
        assert self.safety.anchor_window(self.now, self.now) == "start"
        if with_gnss:
            assert self.safety.accept_start_anchor("start-window", self.now, self.now)[0]
        assert self.safety.begin_teach_session(self.now, self.now)[0]
        self.stationary()  # Full new stopped interval after zero-origin reset.
        assert self.safety.arm_teach(self.now, self.now)[0]
        return self

    def prepared(self):
        self.teach()
        self.safety.stop()
        self.stationary()
        assert self.safety.begin_end_anchor(self.now, self.now)[0]
        assert self.safety.accept_end_anchor("end-window", self.now, self.now, proximity_validated=True)[0]
        assert self.safety.set_ready(ClosedRouteController(circle()), "bundle", self.now, self.now)[0]
        assert self.safety.prepare_repeat(self.now, self.now)[0]
        return self

    def repeat(self):
        self.prepared()
        self.stationary(count=14, visual=True)
        assert self.safety.teacher_state("STOPPED", self.now, self.now, self.now)
        assert self.safety.start_repeat(self.now, self.now)[0]
        return self


def test_default_disarmed_private_output_and_physical_stop_proof_required():
    safety = RuntimeSafety()
    assert safety.state == "DISARMED" and safety.phase == "START_ANCHOR_PENDING"
    assert safety.snapshot()["command_output_topic"] == "/lane_learning_gnss/cmd_vel"
    assert safety.teacher_enabled  # Process ownership is not permission to actuate.
    assert safety.tick(1000000000, 1000000000, .05) == Command()
    assert not safety.arm_teach(1000000000, 1000000000)[0]
    assert "physical" in safety.reason


@pytest.mark.parametrize("config", [dict(pose_timeout_s=float("nan")), dict(visual_min_consecutive_matches=2),
    dict(maximum_speed_mps=-1), dict(hardware_stop_verified=1), dict(command_output_topic="relative"),
    dict(unknown=1), dict(pose_timeout_s=.251), dict(visual_max_age_s=.501),
    dict(teacher_state_timeout_s=1.001), dict(maximum_match_compute_ms=40.01)])
def test_invalid_configuration_rejected(config):
    with pytest.raises(ValueError):
        RuntimeSafety(config)


def test_start_gnss_does_not_reset_origin_or_geometry_but_explicit_begin_does():
    case = Scenario()
    case.stationary(x=100., y=50.)
    safety = case.safety
    before_pose, generation, phase = safety.motion_pose, safety.generation, safety.phase
    assert safety.accept_start_anchor("start", case.now, case.now)[0]
    assert safety.motion_pose is before_pose and safety.generation == generation and safety.phase == phase
    assert safety.gnss_window is None
    assert not safety.arm_teach(case.now, case.now)[0]
    assert safety.begin_teach_session(case.now, case.now)[0]
    assert safety.motion_pose is None and safety.arbiter.pose is None
    case.step(x=0., y=0.)
    assert not safety.arm_teach(case.now, case.now)[0]
    assert "settle" in safety.reason
    case.stationary()
    assert safety.arm_teach(case.now, case.now)[0]
    assert safety.state == "TEACH"


def test_complete_phase_flow_never_reads_gnss_during_teach_or_repeat():
    case = Scenario().repeat()
    safety = case.safety
    assert safety.phase == "REPEAT" and safety.state == "REPEAT"
    assert safety.anchor_window(case.now, case.now) is None
    assert safety.snapshot()["runtime_gnss_enabled"] is False
    assert not safety.teacher_enabled
    assert case.step(visual=True, teacher=False).speed > 0


def test_no_gnss_can_teach_and_finish_ready_model_but_cannot_prepare_repeat():
    case = Scenario().teach(with_gnss=False)
    safety = case.safety
    assert safety.start_anchor_id is None and safety.state == "TEACH"
    assert case.step().speed > 0
    safety.stop()
    case.stationary()
    assert safety.begin_end_anchor(case.now, case.now)[0]
    controller = ClosedRouteController(circle())
    assert safety.set_ready(controller, "gnss-independent-map", case.now, case.now)[0]
    assert safety.phase == "MODEL_READY" and safety.end_anchor_id is None
    generation = safety.generation
    assert not safety.prepare_repeat(case.now, case.now)[0]
    assert safety.ready_controller is controller and safety.generation == generation
    assert safety.phase == "MODEL_READY" and not safety.arbiter.repeat_selected
    assert safety.gnss_window == "end"


def test_late_end_gnss_after_ready_model_changes_only_repeat_permission():
    case = Scenario().teach()
    safety = case.safety
    safety.stop()
    case.stationary()
    assert safety.begin_end_anchor(case.now, case.now)[0]
    controller = ClosedRouteController(circle())
    assert safety.set_ready(controller, "map-before-gnss", case.now, case.now)[0]
    assert safety.anchor_window(case.now, case.now) == "end"
    before_pose, generation = safety.motion_pose, safety.generation
    assert safety.accept_end_anchor("late-end", case.now, case.now, proximity_validated=True)[0]
    assert safety.phase == "MODEL_READY" and safety.ready_controller is controller
    assert safety.motion_pose is before_pose and safety.generation == generation
    assert safety.gnss_window is None
    assert safety.prepare_repeat(case.now, case.now)[0]


@pytest.mark.parametrize("validation", [False, True])
def test_missing_start_or_unvalidated_proximity_never_permits_repeat(validation):
    case = Scenario().teach(with_gnss=not validation)
    safety = case.safety
    safety.stop()
    case.stationary()
    assert safety.begin_end_anchor(case.now, case.now)[0]
    assert safety.accept_end_anchor("end", case.now, case.now, proximity_validated=validation)[0]
    assert safety.set_ready(ClosedRouteController(circle()), "valid-local-map", case.now, case.now)[0]
    assert not safety.prepare_repeat(case.now, case.now)[0]
    assert safety.phase == "MODEL_READY"


def test_gnss_rejection_and_window_closure_do_not_invalidate_local_geometry():
    case = Scenario().teach(with_gnss=False)
    safety = case.safety
    generation, pose, phase = safety.generation, safety.motion_pose, safety.phase
    assert not safety.accept_start_anchor("too-late", case.now, case.now)[0]
    safety.close_anchor_window("optional GNSS unavailable")
    assert safety.phase == phase and safety.generation == generation and safety.motion_pose is pose
    assert safety.state == "TEACH" and case.step().speed > 0


def test_end_anchor_requires_stop_and_cannot_rearm_teacher_after_end_window():
    case = Scenario().teach()
    assert not case.safety.begin_end_anchor(case.now, case.now)[0]
    case.stationary()
    assert case.safety.begin_end_anchor(case.now, case.now)[0]
    assert case.safety.teacher_enabled  # Process may run privately until prepare.
    assert case.safety.anchor_window(case.now, case.now) == "end"
    assert not case.safety.arm_teach(case.now, case.now)[0]
    assert case.safety.tick(case.now, case.now, .05) == Command()


def test_anchor_window_closes_on_motion_or_stale_pose():
    case = Scenario()
    case.stationary()
    assert case.safety.anchor_window(case.now, case.now) == "start"
    case.step(speed=.2)
    assert case.safety.anchor_window(case.now, case.now) is None
    case.stationary()
    assert case.safety.anchor_window(case.now + 300000000, case.now + 300000000) is None


def test_teacher_acceleration_limit_and_zero_immediate_stop():
    case = Scenario().teach()
    command = case.step(delta=.05)
    assert command.speed == pytest.approx(.025)
    case.now += 10000000
    assert case.safety.teacher_command((0, 0, 0, 0, 0, .3), case.now, case.now, case.now)
    assert case.safety.tick(case.now, case.now, .01) == Command()


@pytest.mark.parametrize("values", [(float("nan"), 0, 0, 0, 0, 0), (3, 0, 0, 0, 0, 0),
    (.4, 1, 0, 0, 0, 0), (.4, 0, 0, 0, 0, .5), (), None])
def test_bad_teacher_command_holds(values):
    case = Scenario().teach()
    case.now += 10000000
    assert not case.safety.teacher_command(values, case.now, case.now, case.now)
    assert case.safety.state == "HOLD" and case.safety.last_command == Command()


def test_stale_teacher_holds_and_fresh_recovery_does_not_rearm():
    case = Scenario().teach()
    case.stationary(count=4, teacher=False)
    assert case.safety.state == "HOLD"
    case.step()
    assert case.safety.state == "HOLD"
    assert case.safety.arm_teach(case.now, case.now)[0]


@pytest.mark.parametrize("fault", ["stale", "future", "duplicate", "nan", "jump", "yaw_jump"])
def test_bad_odometry_holds_and_invalidates_generation(fault):
    case = Scenario().teach()
    old_generation = case.safety.generation
    case.now += 10000000
    source, x, yaw = case.now, 0., 0.
    if fault == "stale":
        source -= 300000000
    elif fault == "future":
        source += 20000001
    elif fault == "duplicate":
        source = case.safety.motion_pose.stamp_ns
    elif fault == "nan":
        x = float("nan")
    elif fault == "jump":
        x = 5.
    elif fault == "yaw_jump":
        yaw = 2.
    assert not case.safety.update_pose(Pose(x, 0, yaw, 0, source, case.now), case.now, case.now)
    assert case.safety.state == "HOLD" and case.safety.generation > old_generation
    assert case.safety.tick(case.now, case.now, .05) == Command()


def test_repeated_position_jump_cannot_clear_motion_history_and_become_accepted():
    case = Scenario().teach()
    for _ in range(3):
        case.now += 10000000
        assert not case.safety.update_pose(Pose(5, 0, 0, 0, case.now, case.now), case.now, case.now)


def test_repeat_requires_three_commits_then_full_stopped_settle_and_fresh_stop():
    case = Scenario().prepared()
    safety = case.safety
    for _ in range(2):
        case.step(visual=True)
    assert not safety.start_repeat(case.now, case.now)[0]
    case.step(visual=True)
    assert safety.visual_ready(case.now, case.now)
    assert safety.teacher_state("STOPPED", case.now, case.now, case.now)
    assert not safety.start_repeat(case.now, case.now)[0]
    assert "settle" in safety.reason
    case.stationary(visual=True)
    assert not safety.start_repeat(case.now, case.now)[0]  # STOPPED receipt/source expired.
    assert safety.teacher_state("STOPPED", case.now, case.now, case.now)
    assert safety.start_repeat(case.now, case.now)[0]


@pytest.mark.parametrize("source_delta", [-1000000001, 20000001, None, True])
def test_invalid_stop_heartbeat_clears_confirmation_and_holds(source_delta):
    case = Scenario().prepared()
    source = source_delta if source_delta in (None, True) else case.now + source_delta
    assert not case.safety.teacher_state("STOPPED", source, case.now, case.now)
    assert case.safety.teacher_status is None and case.safety.state == "HOLD"


def test_preprepare_retained_stopped_is_not_posthandover_confirmation():
    case = Scenario().teach()
    earlier = case.now
    assert case.safety.teacher_state("STOPPED", earlier, earlier, earlier)
    case.safety.stop()
    case.stationary()
    assert case.safety.begin_end_anchor(case.now, case.now)[0]
    assert case.safety.accept_end_anchor("end", case.now, case.now, proximity_validated=True)[0]
    assert case.safety.set_ready(ClosedRouteController(circle()), "route", case.now, case.now)[0]
    assert case.safety.prepare_repeat(case.now, case.now)[0]
    assert not case.safety.teacher_state("STOPPED", earlier, case.now, case.now)


def test_teacher_has_no_effect_after_handover_even_if_invalid():
    case = Scenario().repeat()
    before = case.safety.arbiter.teacher_stamp_ns
    assert case.safety.teacher_command(None, None, None, None)
    assert case.safety.arbiter.teacher_stamp_ns == before
    assert case.step(teacher=False, visual=True).speed > 0


def test_mask_stale_stops_even_with_fresh_odometry_and_recovery_needs_manual_start():
    case = Scenario().repeat()
    case.stationary(count=6, teacher=False, visual=False)
    assert case.safety.state == "HOLD" and case.safety.last_command == Command()
    case.stationary(count=14, teacher=False, visual=True)
    assert case.safety.state == "HOLD"
    assert case.safety.teacher_state("STOPPED", case.now, case.now, case.now)
    assert case.safety.start_repeat(case.now, case.now)[0]


@pytest.mark.parametrize("fault", ["source_age", "receipt_age", "future", "compute", "replay", "correction"])
def test_visual_commit_gate_rejects_invalid_outputs(fault):
    case = Scenario().prepared()
    case.step(visual=True)
    case.now += 10000000
    assert case.safety.update_pose(Pose(0, 0, 0, 0, case.now, case.now), case.now, case.now)
    source, receipt, compute, correction = case.now, case.now, 2., (0., 0., 0.)
    if fault == "source_age": source -= 600000000
    if fault == "receipt_age": receipt -= 600000000
    if fault == "future": source += 20000001
    if fault == "compute": compute = 40.01
    if fault == "replay": source = case.safety.visual_source_ns
    if fault == "correction": correction = (2., 0., 0.)
    generation = case.safety.generation
    assert not case.safety.visual_accept(source, receipt, generation, case.now, case.now, correction, compute)
    assert case.safety.generation > generation and case.safety.visual_streak == 0


def test_obsolete_commit_never_restores_confidence_or_changes_current_generation():
    case = Scenario().prepared()
    old = case.safety.generation
    case.safety.visual_reject("late rejected mask")
    current = case.safety.generation
    assert not case.safety.visual_accept(case.now, case.now, old, case.now, case.now, (0, 0, 0), 1)
    assert case.safety.generation == current and case.safety.visual_streak == 0


def test_moving_during_visual_bootstrap_revokes_synchronous_worker_fence():
    cancelled = []
    def revoke():
        cancelled.append(len(cancelled) + 1)
        return cancelled[-1]
    case = Scenario(invalidate_callback=revoke).prepared()
    case.step(visual=True)
    generation = case.safety.generation
    case.step(speed=.2, visual=False)
    assert case.safety.generation == cancelled[-1] > generation
    assert case.safety.motion_pose.speed == .2 and case.safety.visual_streak == 0


def test_visual_correction_propagates_on_native_metric_pose():
    case = Scenario().prepared()
    for _ in range(3):
        case.step()
        assert case.safety.visual_accept(case.now, case.now, case.safety.generation,
            case.now, case.now, (.2, -.1, .03), 2)
    assert case.safety.arbiter.pose.x == pytest.approx(.2)
    case.step(x=.1)
    assert case.safety.arbiter.pose.x == pytest.approx(.2 + .1 * math.cos(.03))
    assert case.safety.arbiter.pose.yaw == pytest.approx(.03)


@pytest.mark.parametrize("limiting", ["mask_source", "mask_receipt", "pose_source", "pose_receipt"])
def test_worker_deadline_covers_four_independent_age_limits(limiting):
    case = Scenario().prepared()
    now, remaining = case.now, 30000000
    source = receipt = now
    psource = preceipt = now
    if limiting == "mask_source": source -= 500000000 - remaining
    if limiting == "mask_receipt": receipt -= 500000000 - remaining
    if limiting == "pose_source": psource -= 250000000 - remaining
    if limiting == "pose_receipt": preceipt -= 250000000 - remaining
    case.safety.motion_pose = Pose(0, 0, 0, 0, psource, preceipt)
    assert case.safety.visual_deadline(source, receipt, now, now) == now + remaining


def test_manual_estop_release_and_reset_never_rearm():
    case = Scenario().teach()
    case.safety.estop(manual=True)
    assert case.safety.state == "ESTOP" and not case.safety.reset_estop()[0]
    case.safety.release_manual_estop()
    assert case.safety.state == "ESTOP"
    assert case.safety.reset_estop()[0]
    assert case.safety.state == "DISARMED"
    assert case.step() == Command()


@pytest.mark.parametrize("clock", ["source", "steady"])
def test_clock_regression_latches_restart_requirement(clock):
    case = Scenario().teach()
    now, steady = case.now, case.now
    if clock == "source": now -= 1
    else: steady -= 1
    assert case.safety.tick(now, steady, .05) == Command()
    assert case.safety.clock_fault and case.safety.state == "HOLD"
    assert not case.safety.arm_teach(case.now + 1000000000, case.now + 1000000000)[0]


def test_control_deadline_and_log_failure_fail_closed():
    case = Scenario().teach()
    assert case.safety.tick(case.now, case.now, .3) == Command()
    assert case.safety.state == "HOLD"
    def broken(_):
        raise OSError("disk unavailable")
    safety = RuntimeSafety(event_sink=broken)
    assert safety.state == "ESTOP"
    assert safety.tick(1000000000, 1000000000, .05) == Command()


def test_vehicle_topic_cannot_arm_without_all_explicit_proofs():
    case = Scenario(dict(command_output_topic="/vehicle/cmd_vel"))
    case.stationary()
    assert case.safety.accept_start_anchor("start", case.now, case.now)[0]
    assert case.safety.begin_teach_session(case.now, case.now)[0]
    case.stationary()
    assert not case.safety.arm_teach(case.now, case.now)[0]
    assert case.safety.last_command == Command()
