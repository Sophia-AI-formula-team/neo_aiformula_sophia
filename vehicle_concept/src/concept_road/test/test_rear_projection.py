"""Verify the compiled camera-memory sampler across motion, occlusion and blackout."""
from unittest.mock import patch

import numpy as np

from concept_road import rear_memory


def test_native_rear_grid_matches_numpy():
    reference = rear_memory.RearSurfaceMemory()
    native = rear_memory.RearSurfaceMemory()
    rng = np.random.default_rng(7724)
    road = np.zeros((360, 640), dtype=bool)
    road[190:, 50:580] = True
    for i in range(9):
        measured = road.copy()
        measured[210:250, 210+i:280+i] = False
        visible = measured | (rng.random(road.shape) > .65)
        if i == 6:
            measured[:] = False
            visible[:] = False
        lane = None if i == 0 else measured & (rng.random(road.shape) < .02)
        args = measured, .9 if i < 5 else -.2, .18 - .04*i, .2
        with patch.object(rear_memory, 'project_mask', None):
            reference.current_lane = lane
            ref = reference.update(*args, visible=visible)
        assert rear_memory.project_mask is not None
        native.current_lane = lane
        result = native.update(*args, visible=visible)
        np.testing.assert_allclose(result, ref, rtol=0, atol=1e-14)
        np.testing.assert_array_equal(native.marking_points, reference.marking_points)
