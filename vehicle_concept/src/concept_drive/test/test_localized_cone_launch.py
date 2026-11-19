import numpy as np

from concept_candidates.launch_clearance import launch_clearance
from concept_candidates.rear_reference617 import RecoveryReference
from concept_drive.core import clear_launch_view
from concept_feasibility.response61515 import motor_profile
from concept_obstacles.launch_view import launch_objects
from concept_road.vision import CX, CY, FOCAL, HEIGHT


def detections(*boxes):
    return dict(boxes=np.asarray(boxes).reshape(-1, 4), labels=np.full(len(boxes), 91),
                scores=np.full(len(boxes), .3))


def measurement(z, x):
    u, v = CX+FOCAL*x/z, CY+HEIGHT*FOCAL/z
    return dict(foot_uv=[u, v], ground_forward_right_m=[z, x], within_tracking_range=z < 12.,
                source_boxes=[[u-3, v-8, u+3, v]], clipped=False)


def margins(*points):
    ms = [measurement(*point) for point in points]
    objects = launch_objects(detections(*(m['source_boxes'][0] for m in ms)), [], ms)
    assert objects is not None
    # Controls from rest: two nominal speeds, with a repeated restored slot.
    depth = np.arange(-2., 20., .2)
    road = np.column_stack([depth, np.full_like(depth, -5.), np.full_like(depth, 5.),
                            np.zeros_like(depth), np.zeros_like(depth)])
    obs = RecoveryReference(road, np.empty((0, 77)), 0., 0., .2, np.zeros_like(depth),
                            None, None, reference=None)
    controls = np.zeros((84, 2, 2))
    controls[:, :, 0] = [.05, .2]
    with motor_profile(True):
        return launch_clearance(objects, obs, dict(nominal_substeps=controls, nominal_indices=np.array([0, 1, 0])),
                                dict(horizon=4.2), np.arange(3))


def test_far_side_cone_clears_original_controls_and_lost_image_brake():
    for point in ((10.2, -6.1), (10.0, 6.0)):
        bounds = margins(point)
        assert (bounds >= .5).all()
        assert bounds[0] == bounds[2]


def test_close_crossing_and_multiple_cones_reject():
    assert (margins((.8, .15)) < .5).all()
    assert (margins((2., .7)) < .5).all()
    assert (margins((10.2, -6.1), (1.35, -.25)) < .5).all()


def test_missing_projection_unmatched_raw_box_and_live_track_still_block():
    far = measurement(10.2, -6.1)
    valid = far['source_boxes'][0]
    assert launch_objects(detections(valid), [], [{}]) is None
    assert launch_objects(detections(valid, [300, 240, 330, 260]), [], [far]) is None
    assert launch_objects(detections(valid), [object()], [far]) is None
    assert launch_objects(detections(valid), [], [{**far, 'ground_forward_right_m': [10., 6.]}]) is None
    # Logged low-confidence feet have no annotated raw boxes. They must
    # produce a risk input without inventing a detector box for the replay.
    del far['source_boxes']
    del far['clipped']
    assert launch_objects(detections(), [], [far]).shape == (2, 77)
    assert launch_objects(detections(valid), [], [far]) is None
    beyond = measurement(16., -6.)
    assert clear_launch_view(detections(beyond['source_boxes'][0]), [], [beyond])
