"""Optional scalar-equivalent curve endpoint kernel; no geometry/gate changes.

Unsupported types, readers, FP modes or uncertified endpoints run the original
Python helper once. Registered types are factory classes, never map instances.
The kernel guarantees numerical results, not errno/sticky-flag timing.
"""
from dataclasses import dataclass
import math
from types import BuiltinFunctionType

import numpy as np

from . import curve_rounding as rounding

try:
    import _lane_mapping_lya_native as _native
except ImportError:
    _native = None

_NATIVE_ENDPOINT = (getattr(_native, "curve_endpoint", None)
                    if getattr(_native, "CURVE_ENDPOINT_API_VERSION", None) == 1 else None)
_STATE_TYPES = []
_DESCRIPTORS = ("origin", "tangent", "_seal", "__getattribute__")


def _register_state_type(kind, seal):
    if not any(kind is entry[0] for entry in _STATE_TYPES):
        _STATE_TYPES.append((kind, seal, tuple(kind.__dict__.get(name) for name in _DESCRIPTORS)))


@dataclass(frozen=True)
class EndpointBindings:
    module: object
    entries: tuple


def _entry(owner, name):
    value = getattr(owner, name)
    return owner, name, value, getattr(value, "__code__", None), getattr(value, "__defaults__", None)


def bind_helpers(module):
    entries = [_entry(module, name) for name in
               ("_endpoint_scalar", "_projection_bounds", "_dot", "_step", "MAX_COORDINATE", "np", "math")]
    entries += [_entry(np, name) for name in ("errstate", "nextafter", "float64", "inf")]
    entries += [_entry(np.errstate, name) for name in ("__enter__", "__exit__")]
    entries += [_entry(math, "isfinite")]
    entries += [_entry(rounding, name) for name in
                ("_NUMPY_NEXTAFTER", "_NUMPY_FLOAT64", "_MATH_NEXTAFTER", "_MINIMUM", "_MAXIMUM", "np", "math")]
    return EndpointBindings(module, tuple(entries))


def _unchanged(bindings):
    return type(bindings) is EndpointBindings and all(
        getattr(owner, name) is value and getattr(value, "__code__", None) is code
        and getattr(value, "__defaults__", None) is defaults
        for owner, name, value, code, defaults in bindings.entries)


def _plain_state(state):
    for kind, seal, descriptors in _STATE_TYPES:
        if type(state) is kind:
            if not all(kind.__dict__.get(name) is descriptor
                       for name, descriptor in zip(_DESCRIPTORS, descriptors)):
                return False
            return state._seal is seal
    return False


def dispatch(state, lower, upper, intercept, slope, *, first, bindings):
    native = _NATIVE_ENDPOINT
    if (type(native) is BuiltinFunctionType and native.__self__ is _native
            and _unchanged(bindings) and _plain_state(state) and type(first) is bool
            and type(state.origin) is tuple and type(state.tangent) is tuple
            and len(state.origin) == len(state.tangent) == 2
            and all(type(value) is float and math.isfinite(value)
                    for value in state.origin + state.tangent + (lower, upper, intercept, slope))):
        result = native(state.origin, state.tangent, lower, upper, intercept, slope, first)
        if result is not None:
            return result
    return bindings.module._endpoint_scalar(state, lower, upper, intercept, slope, first=first)
