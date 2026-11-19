import numpy as np

from concept_obstacles.obstacles617 import instances, Occupancy617
from concept_road.vision import HEIGHT, FOCAL, CX, CY


def cone_at_foot(u, v):
    image = np.full((360, 640, 3), 140, np.uint8)
    image[v-8:v, u-3:u+4] = [10, 20, 180]
    pred = dict(boxes=np.array([[u-4, v-9, u+4, v]], float),
                labels=np.array([91]), scores=np.array([.91]))
    return image, pred


def test_field_observation_and_matching_share_ground_contact_pixels():
    image, pred = cone_at_foot(390, 215)
    measurements = []
    observation, = instances(pred, image.shape, image, field_ground=True, measurements=measurements)
    u, v = measurements[0]['foot_uv']
    z, x = measurements[0]['ground_forward_right_m']
    assert (u, v) == (390., 215.)
    np.testing.assert_allclose([z, x], [HEIGHT*FOCAL/(v-CY), (u-CX)*HEIGHT/(v-CY)])
    track = Occupancy617()
    obj = track.update([observation], 0., 0., .1, image=image)
    assert obj[0, 4] == .34
    np.testing.assert_allclose(track.image_box(track.tracks[0]), observation['box'], atol=1e-10)
    empty = np.zeros_like(image)
    for _ in range(3):
        obj = track.update([], 0., 0., .1, image=empty)
    assert len(obj) == 1 and track.last['tracks'][0]['source'] == 'remembered'


def test_distant_field_contact_stays_visible_without_an_in_range_track():
    image, pred = cone_at_foot(30, 194)
    measurements = []
    assert instances(pred, image.shape, image, field_ground=True, measurements=measurements) == []
    assert len(measurements) == 1
    assert measurements[0]['ground_forward_right_m'][0] > 12
    assert not measurements[0]['within_tracking_range']
    old, = instances(pred, image.shape, image)
    assert old['point'][0] < 12
