import numpy as np
import pytest

from concept_candidates.curve_progress import project, reference_project


def test_projection_agrees_at_visible_ends_and_ambiguous_curve_points():
    pytest.importorskip('concept_candidates._progress')
    coef=np.array([.21,-.35,.096])
    stations=np.linspace(.2,9.3,115)
    points=np.c_[stations,np.polynomial.polynomial.polyval(stations,coef)]
    ref=dict(points=points,coef=coef,sigma=.28)
    rng=np.random.default_rng(626)
    z=np.r_[rng.uniform(-1.,13.,1200),points[:,0]]
    x=np.r_[rng.uniform(-4.,8.,1200),points[:,1]]
    value,valid=project(ref,z,x)
    reference,known=reference_project(ref,z,x)
    np.testing.assert_allclose(value,reference,rtol=0,atol=1e-12)
    np.testing.assert_array_equal(valid,known)
