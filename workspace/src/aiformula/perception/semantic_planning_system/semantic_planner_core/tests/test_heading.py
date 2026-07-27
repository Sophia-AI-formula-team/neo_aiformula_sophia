import math

from semantic_planner_core.heading import angle_diff, angle_wrap


def test_angle_wrap():
    assert math.isclose(angle_wrap(3.0 * math.pi), math.pi)
    assert math.isclose(angle_wrap(0.5), 0.5)


def test_angle_diff_wraps_short_way():
    assert math.isclose(angle_diff(-math.pi + 0.1, math.pi - 0.1), 0.2, abs_tol=1e-6)
