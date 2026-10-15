"""Exact-rational oracles for additional UNKNOWN-distance bounds, not laps."""
from fractions import Fraction as F
import math
from unittest.mock import patch

import numpy as np
import pytest

from lane_mapping_lya_reference import curve_competitor_slab as module


def denominator(tangent):
    q = sum(F(x)*F(x) for x in tangent)
    value = float(q)
    return (float(np.nextafter(value, -np.inf)), float(np.nextafter(value, np.inf)))


def inverse(origin, tangent, u, v, d):
    n = (-tangent[1], tangent[0])
    return tuple(origin[i]+(tangent[i]*u+n[i]*v)/d for i in (0, 1))


def projection(point, origin, tangent):
    return sum(F(t)*(F(p)-F(o)) for p, o, t in zip(point, origin, tangent))


@pytest.mark.parametrize('origin', [(0., 0.), (9999999., -9999999.), (1.e-200, -1.e-200)])
@pytest.mark.parametrize('tangent', [(1., 0.), (0., -1.),
                                    (math.sqrt(.5), math.sqrt(.5)),
                                    (math.cos(.31), math.sin(.31)),
                                    (math.cos(2.1), math.sin(2.1))])
def test_all_rounded_inverse_points_inside_exact_certificate(origin, tangent):
    span, lateral = (-3., 4.), (-2., 5.)
    interval = denominator(tangent)
    slab = module.inverse_chart_slab(origin, tangent, span, lateral, interval)
    rng = np.random.default_rng(321)
    samples = list(zip(rng.uniform(*span, 100), rng.uniform(*lateral, 100)))
    samples += [(u, v) for u in span for v in lateral]
    for d in interval + (tangent[0]*tangent[0]+tangent[1]*tangent[1],):
        for u, v in samples:
            point = inverse(origin, tangent, float(u), float(v), d)
            actual = projection(point, origin, tangent)
            assert F(slab.bounds[0]) <= actual <= F(slab.bounds[1])
    assert F(slab.norm_squared_upper) >= sum(F(t)*F(t) for t in tangent)
    assert slab.bounds[0] <= span[0] <= span[1] <= slab.bounds[1]


def test_narrow_oblique_strip_is_not_its_large_axis_aligned_box():
    tangent = (math.sqrt(.5), math.sqrt(.5))
    slab = module.inverse_chart_slab((0., 0.), tangent, (0., 1.e-6), (-1., 1.), denominator(tangent))
    # Both points are inside the enclosing AABB, but only the second can be in
    # the original thin strip. An unchanged 1 cm candidate-distance upper bound
    # can therefore exclude the strip for the first point, not the second.
    points = np.array([[.3, .3], [.3, -.3]])
    lower = module.squared_distance_lower(points, slab)
    assert lower[0] > .0001
    assert lower[1] == 0.


def test_distance_bound_never_exceeds_exact_strip_distance():
    tangent = (math.cos(.81), math.sin(.81))
    origin = (1000000., -2000000.)
    slab = module.inverse_chart_slab(origin, tangent, (-.02, .03), (-3., 4.), denominator(tangent))
    rng = np.random.default_rng(55)
    points = np.asarray(origin) + rng.normal(size=(1000, 2))
    actual = module.squared_distance_lower(points, slab)
    q = sum(F(t)*F(t) for t in tangent)
    for point, lower in zip(points, actual):
        p = projection(point, origin, tangent)
        gap = max(F(0), F(slab.bounds[0])-p, p-F(slab.bounds[1]))
        assert F(float(lower)) <= gap*gap/q


def test_known_competitor_point_cannot_be_excluded_by_distance_bound():
    tangent = (math.cos(1.3), math.sin(1.3))
    origin = (10000., -20000.)
    interval = denominator(tangent)
    slab = module.inverse_chart_slab(origin, tangent, (-.5, .5), (-.2, .2), interval)
    rng = np.random.default_rng(63)
    for _ in range(100):
        support = inverse(origin, tangent, float(rng.uniform(-.5, .5)),
                          float(rng.uniform(-.2, .2)), interval[0])
        point = np.asarray(support)+rng.normal(size=2)*.05
        distance = sum((F(float(a))-F(b))**2 for a, b in zip(point, support))
        lower = module.squared_distance_lower(point[None, :], slab)[0]
        assert F(float(lower)) <= distance


def test_zero_subnormal_and_strict_numpy_policy_do_not_create_exclusion():
    tiny = float.fromhex('0x0.0000000000001p-1022')
    old = np.geterr()
    try:
        np.seterr(all='raise')
        slab = module.inverse_chart_slab((-0., 0.), (1., -0.), (-tiny, tiny), (-tiny, tiny), (1., 1.))
        values = module.squared_distance_lower(np.array([[0., 0.], [tiny, -tiny], [np.nan, 0.], [np.inf, 0.]]), slab)
        assert np.array_equal(values, np.zeros(4))
        assert np.geterr() == dict.fromkeys(old, 'raise')
    finally:
        np.seterr(**old)


@pytest.mark.parametrize('field,value', [('origin', [0., 0.]), ('tangent', (0., 0.)),
    ('span', (2., 1.)), ('lateral', (0., math.inf)), ('denominator', (0., 1.)),
    ('denominator', (.5, .8)), ('tangent', (1, 0))])
def test_invalid_certificate_is_rejected_not_used_to_remove_unknown(field, value):
    args = dict(origin=(0., 0.), tangent=(1., 0.), span=(-1., 1.), lateral=(-1., 1.), denominator=(1., 1.))
    args[field] = value
    with pytest.raises((ValueError, ArithmeticError)):
        module.inverse_chart_slab(**args)


def test_unsupported_environment_declines_and_factories_are_required():
    with patch.object(module, '_environment', return_value=False):
        with pytest.raises(ArithmeticError, match='environment'):
            module.inverse_chart_slab((0., 0.), (1., 0.), (-1., 1.), (-1., 1.), (1., 1.))
    with pytest.raises(ValueError, match='use inverse_chart_slab'):
        module.InverseChartSlab((0., 0.), (1., 0.), (-1., 1.), 1.)
    with pytest.raises(ValueError, match='certificate'):
        module.squared_distance_lower(np.zeros((1, 2)), object())


def test_no_production_hook_or_geometry_confirmation_is_installed():
    from lane_mapping_lya_reference import curve_geometry_builder as builder
    assert builder._remaining_boxes.__module__ != module.__name__
    assert not hasattr(module, 'confirm')


@pytest.mark.parametrize('angle', [0., .43, 2.3])
def test_complement_bound_never_excludes_any_original_unknown_point(angle):
    tangent = (math.cos(angle), math.sin(angle))
    origin = (9999999., -9999999.)
    interval = denominator(tangent)
    covered = ((-.8, -.2), (.2, .8))
    certificate = module.inverse_chart_complement(origin, tangent, (-1., 1.), (-2., 2.), interval, covered)
    rng = np.random.default_rng(91)
    for u in np.r_[np.linspace(-1., -.8, 20), np.linspace(-.2, .2, 20), np.linspace(.8, 1., 20)]:
        support = inverse(origin, tangent, float(u), float(rng.uniform(-2., 2.)), interval[0])
        p = projection(support, origin, tangent)
        assert all(not F(a) < p < F(b) for a, b in certificate.interiors)
        query = np.asarray(support)+rng.normal(size=2)*.05
        distance = sum((F(float(a))-F(b))**2 for a, b in zip(query, support))
        lower = module.complement_squared_distance_lower(query[None, :], certificate)[0]
        assert F(float(lower)) <= distance


def test_complement_preserves_gap_endpoints_and_does_not_merge_signatures():
    cert = module.inverse_chart_complement((0., 0.), (1., 0.), (-2., 2.), (-1., 1.), (1., 1.),
                                          ((-1., -.01), (.01, 1.)))
    points = np.array([[0., 0.], [-1., 0.], [-.01, 0.], [.01, 0.], [1., 0.], [.5, 100.]])
    lower = module.complement_squared_distance_lower(points, cert)
    assert np.array_equal(lower[:5], np.zeros(5))
    assert lower[-1] > .2
    assert cert.interiors[0][0] > -1. and cert.interiors[0][1] < -.01
    assert cert.interiors[1][0] > .01 and cert.interiors[1][1] < 1.
    unknown = module.inverse_chart_complement((0., 0.), (1., 0.), (-2., 2.), (-1., 1.), (1., 1.), ())
    assert np.array_equal(module.complement_squared_distance_lower(points, unknown), np.zeros(6))
    with patch.object(module, '_environment', return_value=False):
        assert np.array_equal(module.complement_squared_distance_lower(points, cert), np.zeros(6))


@pytest.mark.parametrize('covered', [([(-1., 1.)]), ((-2., 0.),), ((1., 0.),),
    ((-.5, .5), (0., .8)), ((0., 0.),)])
def test_incomplete_or_invalid_complement_contract_rejects(covered):
    with pytest.raises(ValueError):
        module.inverse_chart_complement((0., 0.), (1., 0.), (-1., 1.), (-1., 1.), (1., 1.), covered)
