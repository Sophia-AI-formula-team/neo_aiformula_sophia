from semantic_planner_core.constraints import SemanticConstraints


def test_constraint_json_roundtrip():
    constraints = SemanticConstraints(
        red_light_active=True,
        red_forward_forbidden=True,
        allow_lane_follow=False,
        diversion_active=True,
        mode="RED_FORWARD_GATE",
    )

    restored = SemanticConstraints.from_json(constraints.to_json())

    assert restored == constraints
