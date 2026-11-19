import numpy as np

from concept_drive.core import clear_launch_view, intersect_confirmed_edges


def test_updated_camera_cannot_reopen_a_confirmed_outer_edge():
    z = np.arange(8, dtype=float)
    known = np.column_stack((z, -np.ones(8), np.ones(8), np.full(8, .20), np.full(8, .15)))
    camera = known.copy()
    camera[:, 1:3] *= 3.
    camera[:, 3:] = .05
    result = intersect_confirmed_edges(camera, known, {'left': [[2., -1.], [5., -1.]]})
    np.testing.assert_array_equal(result[2:6, 1], -1.)
    np.testing.assert_allclose(result[2:6, 3], .2)
    np.testing.assert_array_equal(result[:, 2], camera[:, 2])
    np.testing.assert_array_equal(result[:2], camera[:2])
    np.testing.assert_array_equal(result[6:], camera[6:])
    assert np.all(camera[:, 1] == -3.)
    camera[3, 2] = -1.2
    assert len(intersect_confirmed_edges(camera, known, {'left': [[2., -1.], [5., -1.]]})) == 3
    assert len(intersect_confirmed_edges(camera, known[:0], {'left': [[2., -1.], [5., -1.]]})) == 2


def test_launch_preference_defers_to_observed_or_unresolved_cones():
    def box(bottom):
        return {'labels': np.array([91]), 'scores': np.array([.6]),
                'boxes': np.array([[300., bottom-12., 320., bottom]])}
    assert not clear_launch_view(box(170), [], [])  # no ground intersection
    assert not clear_launch_view(box(250), [], [])  # close but not yet tracked
    assert not clear_launch_view(box(190), [object()], [])
    assert not clear_launch_view(box(190), [], [{}])
    # A range flag alone cannot associate a raw box to a measured foot.
    assert not clear_launch_view(box(190), [], [{'within_tracking_range': False}])
