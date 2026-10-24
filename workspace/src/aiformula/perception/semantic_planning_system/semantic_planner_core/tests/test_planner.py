import numpy as np

from semantic_planner_core.costmap import SemanticCostmapBuilder
from semantic_planner_core.constraints import SemanticConstraints
from semantic_planner_core.planner import LocalPrimitivePlanner
from semantic_planner_core.primitives import PrimitiveName, default_primitives


def _constraints():
    return SemanticConstraints(
        red_light_active=True,
        red_forward_forbidden=True,
        allow_lane_follow=False,
        diversion_active=True,
        mode="RED_FORWARD_GATE",
    )


def _scenario_costmap(name: str) -> np.ndarray:
    height, width = 100, 100
    road = np.zeros((height, width), dtype=np.uint8)
    offroad = np.zeros_like(road)
    road[35:, 25:75] = 255
    offroad[:, :16] = 255
    offroad[:, 82:] = 255
    if name == "red_gate_left_open":
        road[42:, 16:48] = 255
    elif name == "red_gate_left_blocked":
        offroad[42:, :58] = 255
        offroad[42:, 58:] = 255
    else:
        raise ValueError(name)
    return SemanticCostmapBuilder().build(road, offroad, _constraints())


def test_image_coordinate_conventions_for_primitives():
    # Image bottom is near the robot; image top is farther ahead.
    # x grows left-to-right, y grows top-to-bottom. This is front-camera
    # perspective rather than BEV, so moving primitives progress toward smaller
    # y values and should remain compressed into image-space trajectories.
    for primitive in default_primitives():
        if primitive.is_stop:
            continue
        ys = [point[1] for point in primitive.path_points]
        assert ys[0] > ys[-1]
        assert all(0.0 <= point[0] <= 1.0 and 0.0 <= point[1] <= 1.0 for point in primitive.path_points)


def test_primitives_fan_out_instead_of_converging_to_center():
    primitives = {primitive.name: primitive for primitive in default_primitives()}
    for name in (
        PrimitiveName.SMALL_LEFT.value,
        PrimitiveName.MEDIUM_LEFT.value,
        PrimitiveName.STRONG_LEFT.value,
        PrimitiveName.LEFT_THEN_ALIGN.value,
    ):
        points = primitives[name].path_points
        assert points[-1][0] < points[0][0]

    for name in (
        PrimitiveName.SMALL_RIGHT.value,
        PrimitiveName.MEDIUM_RIGHT.value,
        PrimitiveName.STRONG_RIGHT.value,
        PrimitiveName.RIGHT_THEN_ALIGN.value,
    ):
        points = primitives[name].path_points
        assert points[-1][0] > points[0][0]

    assert primitives[PrimitiveName.STRONG_LEFT.value].path_points[-1][0] < primitives[PrimitiveName.MEDIUM_LEFT.value].path_points[-1][0]
    assert primitives[PrimitiveName.MEDIUM_LEFT.value].path_points[-1][0] < primitives[PrimitiveName.SMALL_LEFT.value].path_points[-1][0]
    assert primitives[PrimitiveName.SMALL_RIGHT.value].path_points[-1][0] < primitives[PrimitiveName.MEDIUM_RIGHT.value].path_points[-1][0]
    assert primitives[PrimitiveName.MEDIUM_RIGHT.value].path_points[-1][0] < primitives[PrimitiveName.STRONG_RIGHT.value].path_points[-1][0]


def test_red_gate_left_blocked_selects_stop():
    costmap = _scenario_costmap("red_gate_left_blocked")

    selected, details = LocalPrimitivePlanner().plan(costmap, _constraints(), theta_ref=0.0)

    assert selected.name == PrimitiveName.STOP.value
    assert details[PrimitiveName.SLOW_FORWARD.value]["valid"] is False
    assert details[PrimitiveName.SLOW_FORWARD.value]["red_gate_hit"] is True


def test_red_gate_left_open_selects_left_primitive():
    costmap = _scenario_costmap("red_gate_left_open")

    selected, details = LocalPrimitivePlanner().plan(costmap, _constraints(), theta_ref=0.0)

    assert selected.name in {
        PrimitiveName.SMALL_LEFT.value,
        PrimitiveName.MEDIUM_LEFT.value,
        PrimitiveName.STRONG_LEFT.value,
        PrimitiveName.LEFT_THEN_ALIGN.value,
    }
    assert selected.name != PrimitiveName.STOP.value
    assert details[PrimitiveName.SLOW_FORWARD.value]["valid"] is False
    assert any(
        details[name]["valid"] is True
        for name in (
            PrimitiveName.SMALL_RIGHT.value,
            PrimitiveName.MEDIUM_RIGHT.value,
            PrimitiveName.STRONG_RIGHT.value,
            PrimitiveName.RIGHT_THEN_ALIGN.value,
        )
    )


def test_right_primitives_are_generated_but_left_is_preferred():
    costmap = _scenario_costmap("red_gate_left_open")

    selected, details = LocalPrimitivePlanner(left_preference_bonus=0.20).plan(
        costmap,
        _constraints(),
        theta_ref=0.0,
    )

    assert selected.name in {
        PrimitiveName.SMALL_LEFT.value,
        PrimitiveName.MEDIUM_LEFT.value,
        PrimitiveName.STRONG_LEFT.value,
        PrimitiveName.LEFT_THEN_ALIGN.value,
    }
    for name in (
        PrimitiveName.SMALL_RIGHT.value,
        PrimitiveName.MEDIUM_RIGHT.value,
        PrimitiveName.STRONG_RIGHT.value,
        PrimitiveName.RIGHT_THEN_ALIGN.value,
    ):
        assert name in details


def test_slow_forward_invalid_under_red_gate():
    costmap = _scenario_costmap("red_gate_left_open")
    _, details = LocalPrimitivePlanner().plan(costmap, _constraints(), theta_ref=0.0)

    slow = details[PrimitiveName.SLOW_FORWARD.value]
    assert slow["valid"] is False
    assert slow["red_gate_hit"] is True
    assert slow["reason"] == "red_forward_gate"


def test_forward_primitive_entering_red_gate_is_invalid():
    costmap = _scenario_costmap("red_gate_left_open")
    _, details = LocalPrimitivePlanner().plan(costmap, _constraints(), theta_ref=0.0)

    assert details[PrimitiveName.SLOW_FORWARD.value]["valid"] is False
    assert details[PrimitiveName.SLOW_FORWARD.value]["red_gate_hit"] is True
    assert details[PrimitiveName.SLOW_FORWARD.value]["reason"] == "red_forward_gate"


def test_offroad_lethal_invalidates_primitive():
    costmap = _scenario_costmap("red_gate_left_blocked")
    _, details = LocalPrimitivePlanner().plan(costmap, _constraints(), theta_ref=0.0)

    for name in (
        PrimitiveName.SMALL_LEFT.value,
        PrimitiveName.MEDIUM_LEFT.value,
        PrimitiveName.STRONG_LEFT.value,
        PrimitiveName.LEFT_THEN_ALIGN.value,
        PrimitiveName.SMALL_RIGHT.value,
        PrimitiveName.MEDIUM_RIGHT.value,
        PrimitiveName.STRONG_RIGHT.value,
        PrimitiveName.RIGHT_THEN_ALIGN.value,
    ):
        assert details[name]["valid"] is False
        assert details[name]["lethal_hit"] is True
