"""Numerical oracle and fallback tests for the optional endpoint C kernel."""
import ctypes
import ctypes.util
import math
import os
import platform
import struct
import sys
from unittest import mock

import numpy as np
import pytest

from lane_mapping_lya_reference import curve_consensus as scalar
from lane_mapping_lya_reference import curve_consensus_compact as compact
from lane_mapping_lya_reference import curve_retained_state as retained
from lane_mapping_lya_reference import curve_endpoint_native as adapter


def state(origin=(0., 0.), tangent=(1., 0.)):
    return compact.create_curve_state(1, 1, (-1, -1), origin, tangent,
                                      normal_cosine=.9, max_support_records=200000)


def bits(value):
    return struct.pack('!4d', *(value[0] + value[1]))


@pytest.fixture
def kernel():
    value = adapter._NATIVE_ENDPOINT
    if value is None:
        if os.environ.get('LANE_REQUIRE_CURVE_ENDPOINT_NATIVE') == '1':
            pytest.fail('curve endpoint API must be built for this job')
        pytest.skip('optional curve endpoint extension not built in this environment')
    return value


def test_native_all_numeric_outputs_match_original_scalar(kernel):
    tangents = ((1., 0.), (0., 1.), (-1., 0.), (0., -1.), (.6, .8), (-.6, .8), (.8, -.6))
    origins = ((0., -0.), (17., -40.), (1.e6, -1.e6))
    domains = ((-1., 1.), (1., 2.), (0., 1.), (-1.e-12, 1.e-12), (1.e-12, 2.e-12),
               (-1.e-320, 1.e-320))
    coefficients = ((0., 0.), (.01, .03), (-.01, -.03), (1.e-20, -1.e-20))
    successes = 0
    for tangent in tangents:
        for origin in origins:
            header = state(origin, tangent)
            for lower, upper in domains:
                for intercept, slope in coefficients:
                    for first in (True, False):
                        args = origin, tangent, lower, upper, intercept, slope, first
                        try:
                            expected = scalar._endpoint_scalar(header, lower, upper, intercept, slope, first=first)
                        except ArithmeticError:
                            assert kernel(*args) is None, args
                        else:
                            actual = kernel(*args)
                            assert actual is not None, args
                            assert bits(actual) == bits(expected), args
                            assert lower <= actual[1][0] <= actual[1][1] <= upper
                            successes += 1
    assert successes > 300


def test_native_dispatch_used_for_all_factory_headers(kernel):
    header = state()
    raw = retained.from_materialized(header)
    original = scalar.create_curve_state(1, 1, (-1, -1), (0., 0.), (1., 0.),
                                         normal_cosine=.9, max_storage_units=200000)
    for value in (header, raw, original):
        # A profile hook does not replace the scalar binding and disable dispatch.
        calls = []
        def trace(frame, event, arg):
            if event == 'call' and frame.f_code is scalar._endpoint_scalar.__code__:
                calls.append(None)
        previous = sys.getprofile()
        try:
            sys.setprofile(trace)
            actual = scalar._endpoint(value, 1., 2., .01, .03, first=True)
        finally:
            sys.setprofile(previous)
        assert not calls
        assert bits(actual) == bits(scalar._endpoint_scalar(value, 1., 2., .01, .03, first=True))


def test_native_direct_api_rejects_coercions(kernel):
    valid = [(0., 0.), (1., 0.), 1., 2., .01, 0., True]
    for index, replacement in ((0, [0., 0.]), (1, (1.,)), (2, True), (3, 2),
                                (4, np.float64(.01)), (6, np.bool_(True))):
        args = valid.copy()
        args[index] = replacement
        with pytest.raises(TypeError):
            kernel(*args)


def test_native_decline_retains_original_failure(kernel):
    header = state()
    for lower, upper, intercept, slope in ((1., 1., 0., 0.), (1., 2., 1.e308, 1.e308),
                                           (-1.e-320, 1.e-320, 1.e7+1., 0.)):
        assert kernel(header.origin, header.tangent, lower, upper, intercept, slope, True) is None
        with pytest.raises(ArithmeticError) as expected:
            scalar._endpoint_scalar(header, lower, upper, intercept, slope, first=True)
        with pytest.raises(type(expected.value)) as actual:
            scalar._endpoint(header, lower, upper, intercept, slope, first=True)
        assert str(actual.value) == str(expected.value)


def test_non_nearest_hardware_mode_declines(kernel):
    if sys.platform != 'linux' or platform.machine().lower() not in ('x86_64', 'amd64'):
        pytest.skip('glibc x86 rounding-mode probe')
    library = ctypes.CDLL(ctypes.util.find_library('m'))
    library.fegetround.restype = ctypes.c_int
    library.fesetround.argtypes = (ctypes.c_int,)
    library.fesetround.restype = ctypes.c_int
    saved = library.fegetround()
    try:
        assert library.fesetround(0x400) == 0  # glibc x86 FE_DOWNWARD.
        assert kernel((0., 0.), (1., 0.), 1., 2., .01, 0., True) is None
    finally:
        assert library.fesetround(saved) == 0


def test_changed_projection_reader_runs_scalar_once():
    header = state()
    with mock.patch.object(scalar, '_projection_bounds', side_effect=ArithmeticError('reader changed')) as reader:
        with pytest.raises(ArithmeticError, match='reader changed'):
            scalar._endpoint(header, 1., 2., .01, 0., first=True)
    assert reader.call_count == 1


def test_changed_scalar_binding_is_not_hidden():
    header = state()
    expected = scalar._endpoint_scalar(header, 1., 2., .01, 0., first=True)
    with mock.patch.object(scalar, '_endpoint_scalar', wraps=scalar._endpoint_scalar) as helper:
        result = scalar._endpoint(header, 1., 2., .01, 0., first=True)
    assert helper.call_count == 1 and bits(result) == bits(expected)


def test_unregistered_header_has_original_property_reads():
    calls = []
    class Header:
        tangent = (1., 0.)
        @property
        def origin(self):
            calls.append('origin')
            return (0., 0.)
    expected = scalar._endpoint_scalar(Header(), 1., 2., .01, 0., first=True)
    expected_calls = calls[:]
    calls.clear()
    actual = scalar._endpoint(Header(), 1., 2., .01, 0., first=True)
    assert calls == expected_calls and bits(actual) == bits(expected)


def test_old_extension_or_missing_optional_api_keeps_scalar_available():
    header = state()
    expected = scalar._endpoint_scalar(header, 1., 2., .01, 0., first=True)
    with mock.patch.object(adapter, '_NATIVE_ENDPOINT', None):
        assert bits(scalar._endpoint(header, 1., 2., .01, 0., first=True)) == bits(expected)
