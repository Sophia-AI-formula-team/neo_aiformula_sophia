from semantic_planner_core.constraints import SemanticConstraints
from semantic_planner_core.lane_recovery import LaneRecoveryGate


def test_lane_recovery_gate_rejects_horizontal_lane_lines():
    gate = LaneRecoveryGate(stable_required_frames=2, horizontal_angle_threshold_deg=15.0)
    result = gate.update(
        {"stable": True, "angle_deg": 3.0, "center_x": 0.5, "confidence": 0.9},
        SemanticConstraints(diversion_active=False),
    )

    assert result == "ignore_lane"


def test_lane_recovery_gate_only_recovers_after_enough_stable_frames():
    gate = LaneRecoveryGate(stable_required_frames=3, min_confidence=0.6)
    constraints = SemanticConstraints(diversion_active=False)
    lane = {"stable": True, "angle_deg": 72.0, "center_x": 0.5, "confidence": 0.9}

    assert gate.update(lane, constraints) == "candidate"
    assert gate.update(lane, constraints) == "candidate"
    assert gate.update(lane, constraints) == "recoverable"
