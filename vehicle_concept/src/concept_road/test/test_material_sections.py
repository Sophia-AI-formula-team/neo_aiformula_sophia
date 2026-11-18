from unittest.mock import patch

import numpy as np

from concept_road import material


def test_native_sections_preserve_thresholds_borders_and_disconnected_runs():
    depths = np.linspace(.95, 12., 45)
    rng = np.random.default_rng(62072)
    observed = np.zeros((360, 640), dtype=bool)
    observed[:, :80] = True
    observed[:, 100:108] = True
    observed[:, 145:154] = True
    observed[:, 350:485] = True
    observed[:, 560:] = True
    # Alternate exact two- and three-of-five column votes at the main run.
    observed[:, 310:350] = (np.arange(360)[:, None] % 5 < 2)
    observed[:, 485:520] = (np.arange(360)[:, None] % 5 < 3)
    assert material.native_sections is not None
    for mask in (observed, ~observed, observed & (rng.random(observed.shape) > .1),
                 np.ones_like(observed), np.zeros_like(observed)):
        actual = material.road_sections(mask, depths)
        with patch.object(material, 'native_sections', None):
            expected = material.road_sections(mask, depths)
        assert actual == expected
