import numpy as np

from concept_visualization.geometry import pixel
from concept_visualization.node import draw_candidates, draw_path


def test_candidates_and_selection_have_distinct_camera_strokes():
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    paths = {index: [(z, x) for z in (2., 3., 4.)]
             for index, x in enumerate((-.8, .8, 0.))}
    decision = {'selected_index': 2, 'candidates': [
        {'index': index, 'allowed': index != 0, 'path': path}
        for index, path in paths.items()
    ]}
    draw_candidates(image, decision)
    for index in paths:
        x, y = pixel(paths[index][1])
        blue, green, red = map(int, image[y, x])
        if index == 0:
            assert blue > 0 and abs(green - red) < 10
        elif index == 1:
            assert green > blue > red > 0
        else:
            assert (blue, green, red) == (0, 0, 0)
    draw_path(image, paths[2], (170, 72, 250), 4)
    x, y = pixel(paths[2][1])
    assert tuple(image[y, x]) == (170, 72, 250)
