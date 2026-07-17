import numpy as np

from semantic_planner_core.constraints import SemanticConstraints
from semantic_planner_core.costmap import SemanticCostmapBuilder


def test_costmap_generation_with_synthetic_masks():
    road = np.zeros((10, 10), dtype=np.uint8)
    road[4:, 2:8] = 255
    offroad = np.zeros_like(road)
    offroad[:, :2] = 255

    costmap = SemanticCostmapBuilder().build(road, offroad, SemanticConstraints())

    assert costmap[8, 5] == 20
    assert costmap[1, 5] == 180
    assert costmap[8, 0] == 255


def test_red_forward_forbidden_region_insertion():
    road = np.ones((20, 20), dtype=np.uint8) * 255
    offroad = np.zeros_like(road)
    constraints = SemanticConstraints(red_forward_forbidden=True)

    costmap = SemanticCostmapBuilder(
        center_x_ratio=0.5,
        width_ratio=0.2,
        top_y_ratio=0.25,
        bottom_y_ratio=0.62,
    ).build(road, offroad, constraints)

    assert costmap[8, 10] == 255
    assert costmap[18, 10] == 20
    assert costmap[8, 2] == 20
