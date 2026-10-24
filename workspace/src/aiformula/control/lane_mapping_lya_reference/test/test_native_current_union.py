"""Native current-frame union contract and independent oracle tests.

Pure-Python projection/union below are a literal numeric old-body oracle from
finite_support SHA 89f19b88d12d5f442aeb1ada6404dd087bcb18251dff1fa81be7f4c4e477b417.
Only helper names are changed; no production/native8 projection is called.
The native ABI deliberately validates scalar component bounds, not hypot:
public advance_domain continues to own its stricter unit-vector admission.
Missing native9 skips optional native tests, never passes a required build.
"""
from concurrent.futures import ThreadPoolExecutor
from fractions import Fraction
import hashlib
import importlib
import math
import os
import struct

import numpy as np
import pytest


def native_or_none():
    try:
        module = importlib.import_module("_lane_mapping_lya_native")
    except ImportError:
        module = None
    okay = (module is not None and
            getattr(module, "FINITE_CURRENT_UNION_API_VERSION", None) == 1 and
            callable(getattr(module, "finite_current_union", None)))
    if not okay and os.environ.get("LANE_REQUIRE_NATIVE") == "1":
        raise RuntimeError("LANE_REQUIRE_NATIVE=1 requires actual native9 finite_current_union")
    return module if okay else None


_NATIVE9 = native_or_none()


@pytest.fixture
def native9():
    if _NATIVE9 is None:
        pytest.skip("native9 not built; this is not a passed native test")
    return _NATIVE9


def _nextafter_binary64(value, direction):
    """One IEEE binary64 step, including signed zero, on Python 3.8.

    This replaces only the scalar operation, not the interval arithmetic order
    or its conservative outward rounding. Integer bit stepping avoids creating
    a NumPy error-state context for every scalar subnormal operation.
    """
    if math.isnan(value) or math.isnan(direction):
        return value + direction
    if value == direction:
        return direction
    if value == 0.:
        return math.copysign(float.fromhex('0x0.0000000000001p-1022'), direction)
    bits = struct.unpack('>Q', struct.pack('>d', value))[0]
    bits += 1 if (direction > value) == (value > 0.) else -1
    return struct.unpack('>d', struct.pack('>Q', bits))[0]


# Foxy's Python 3.8 has no stdlib nextafter. Both implementations have the
# same bit-exact contract, tested independently against NumPy's IEEE oracle.
_nextafter = getattr(math, 'nextafter', _nextafter_binary64)


def _outward(value):
    if not math.isfinite(value):
        raise ValueError('nonfinite interval arithmetic')
    lower, upper = _nextafter(value, -math.inf), _nextafter(value, math.inf)
    if not math.isfinite(lower) or not math.isfinite(upper):
        raise ValueError('unbounded interval arithmetic')
    return lower, upper


def _sub_bounds(first, second):
    if first == second:
        return 0., 0.
    return _outward(first - second)


def _multiply_bounds(interval, scalar):
    if scalar == 0. or interval == (0., 0.):
        return 0., 0.
    lower, upper = interval if scalar > 0. else interval[::-1]
    return _outward(lower * scalar)[0], _outward(upper * scalar)[1]


def oracle_projection(point, origin, tangent):
    """IEEE enclosure of the exact dot product of the supplied float64 values.

    Each nontrivial subtraction, multiplication and sum is rounded OUTWARD.
    This numerical enclosure is subsequently used to shrink, never extend,
    a measured segment's supported interval.
    """
    parts = [_multiply_bounds(_sub_bounds(float(point[i]), float(origin[i])), float(tangent[i]))
             for i in (0, 1)]
    if parts[0] == (0., 0.):
        return parts[1]
    if parts[1] == (0., 0.):
        return parts[0]
    return _outward(parts[0][0] + parts[1][0])[0], _outward(parts[0][1] + parts[1][1])[1]



def oracle_connected_spans(edges, projected):
    """Inner projection of each EXACT physical-endpoint connected component.

    A continuous affine image of a connected union of closed segments is an
    interval. Thus only its exterior extrema need inward rounding. Shrinking
    each A-B and B-C independently invents an unsupported strip at their shared
    B, even though the original observed union contains that point. No distance
    tolerance, approximate join, historical vote or normal inference is used.
    """
    parents, ranks = {}, {}

    def root(vertex):
        if vertex not in parents:
            parents[vertex], ranks[vertex] = vertex, 0
        while parents[vertex] != vertex:
            parents[vertex] = parents[parents[vertex]]
            vertex = parents[vertex]
        return vertex

    for first, second in edges:
        a, b = root(first), root(second)
        if a == b:
            continue
        if ranks[a] < ranks[b]:
            a, b = b, a
        parents[b] = a
        if ranks[a] == ranks[b]:
            ranks[a] += 1
    extrema = {}
    for vertex in parents:
        owner = root(vertex)
        lower, upper = projected[vertex]
        if owner not in extrema:
            extrema[owner] = [upper, lower]
        else:
            extrema[owner][0] = min(extrema[owner][0], upper)
            extrema[owner][1] = max(extrema[owner][1], lower)
    return [(lower, upper) for lower, upper in extrema.values() if lower < upper]


def oracle_current_union(segments, normals, reliable, origin, tangent, cosine):
    # The validated chart is binary64. Reuse the same two-element conversion
    # that np.dot previously made for every endpoint; keep each dot separate.
    normal = np.asarray((-tangent[1], tangent[0]), dtype=np.float64)
    vertices, projected, edges, trusted_edges = {}, [], [], []
    for segment, directions, trusted in zip(segments, normals, reliable):
        if (float(np.dot(directions[0], normal)) < cosine or
                float(np.dot(directions[1], normal)) < cosine):
            continue
        pair = []
        for point in segment:
            # Numeric equality of both finite binary64 coordinates, never a
            # rounded spatial bin. Signed zeros denote the same physical point.
            key = (float(point[0]), float(point[1]))
            vertex = vertices.get(key)
            if vertex is None:
                vertex = len(projected)
                vertices[key] = vertex
                projected.append(oracle_projection(point, origin, tangent))
            pair.append(vertex)
        edge = tuple(pair)
        edges.append(edge)
        if trusted:
            trusted_edges.append(edge)
    # Reliability has its OWN connectivity: an unreliable A-B bridge must not
    # join two otherwise separated trusted domains. All frame counts still
    # reduce to a boolean union before the one-per-source-frame vote below.
    all_trusted = len(edges) == len(trusted_edges)
    spans = [(lower, upper, all_trusted) for lower, upper in oracle_connected_spans(edges, projected)]
    if trusted_edges and not all_trusted:
        spans.extend((lower, upper, True) for lower, upper in oracle_connected_spans(trusted_edges, projected))
    events = {}
    for lower, upper, trusted in spans:
        for edge, count, rel in ((lower, 1, int(trusted)), (upper, -1, -int(trusted))):
            row = events.setdefault(edge, [0, 0]); row[0] += count; row[1] += rel
    boundaries = sorted(events)
    pieces, count, trusted = [], 0, 0
    for index, lower in enumerate(boundaries[:-1]):
        count += events[lower][0]; trusted += events[lower][1]
        if count:
            upper, rel = boundaries[index+1], trusted > 0
            if pieces and pieces[-1][1] == lower and pieces[-1][2] == rel:
                pieces[-1] = (pieces[-1][0], upper, rel)
            else:
                pieces.append((lower, upper, rel))
    return pieces




def pieces_bytes(pieces):
    return tuple((struct.pack("=d", lo), struct.pack("=d", hi), type(flag), flag)
                 for lo, hi, flag in pieces)


def unit_normals(count, tangent=(1., 0.)):
    return np.broadcast_to((-tangent[1], tangent[0]), (count, 2, 2)).copy()


def old_admission(normals, tangent, cosine):
    normal = np.asarray((-tangent[1], tangent[0]), dtype=np.float64)
    return bytes(not (float(np.dot(row[0], normal)) < cosine or
                      float(np.dot(row[1], normal)) < cosine) for row in normals)


def native_args(segments, admitted=None, reliable=None, origin=(0., 0.),
                tangent=(1., 0.)):
    values = np.asarray(segments, dtype=np.float64).reshape(-1, 2, 2)
    count = len(values)
    return (values.tobytes(), bytes([1])*count if admitted is None else bytes(admitted),
            bytes([1])*count if reliable is None else bytes(reliable), count,
            float(origin[0]), float(origin[1]), float(tangent[0]), float(tangent[1]))


def assert_contract(result):
    assert type(result) is tuple
    for row in result:
        assert type(row) is tuple and len(row) == 3
        assert type(row[0]) is float and type(row[1]) is float
        assert type(row[2]) is bool
        assert math.isfinite(row[0]) and math.isfinite(row[1]) and row[0] < row[1]


def compare_case(native9, segments, admitted=None, reliable=None,
                 origin=(0., 0.), tangent=(1., 0.)):
    points = np.asarray(segments, dtype=np.float64).reshape(-1, 2, 2)
    count = len(points)
    accept = np.ones(count, bool) if admitted is None else np.asarray(admitted, bool)
    trust = np.ones(count, bool) if reliable is None else np.asarray(reliable, bool)
    normals = unit_normals(count, tangent)
    # Encode an already-declared independent mask through signed normals only
    # for the literal old-body comparison. Native never redoes this gate.
    normals[~accept] *= -1.
    expected = oracle_current_union(points, normals, trust, origin, tangent, .5)
    args = native_args(points, accept, trust, origin, tangent)
    before = tuple(hashlib.sha256(item).digest() for item in args[:3])
    result = native9.finite_current_union(*args)
    assert_contract(result)
    assert pieces_bytes(result) == pieces_bytes(expected)
    assert before == tuple(hashlib.sha256(item).digest() for item in args[:3])
    assert_fraction_subset(result, points, accept, trust, origin, tangent)
    return result


def _fraction_union(intervals):
    answer = []
    for lo, hi in sorted(intervals):
        if lo >= hi:
            continue
        if answer and lo <= answer[-1][1]:
            answer[-1] = (answer[-1][0], max(hi, answer[-1][1]))
        else:
            answer.append((lo, hi))
    return answer


def exact_segment_union(segments, admitted, reliable, origin, tangent, trusted=False):
    # Independent rational affine-image oracle: the image of each REAL segment
    # is exactly the interval between its endpoints. Union cannot invent a gap.
    result = []
    for pair, allowed, is_trusted in zip(segments, admitted, reliable):
        if not allowed or (trusted and not is_trusted):
            continue
        values = [sum((Fraction(float(p[i]))-Fraction(float(origin[i]))) *
                      Fraction(float(tangent[i])) for i in (0, 1)) for p in pair]
        result.append((min(values), max(values)))
    return _fraction_union(result)


def assert_fraction_subset(pieces, segments, admitted, reliable, origin, tangent):
    base = exact_segment_union(segments, admitted, reliable, origin, tangent)
    trusted = exact_segment_union(segments, admitted, reliable, origin, tangent, True)
    for lo, hi, rel in pieces:
        left, right = Fraction(lo), Fraction(hi)
        assert any(a <= left and right <= b for a, b in base)
        if rel:
            assert any(a <= left and right <= b for a, b in trusted)


CASES = [
    ("empty", [], [], []),
    ("none-admitted", [[[0., 0.], [1., 0.]]], [0], [1]),
    ("one", [[[0., 0.], [1., 0.]]], [1], [1]),
    ("duplicates", [[[0., 0.], [1., 0.]], [[0., 0.], [1., 0.]]], [1, 1], [0, 1]),
    ("reverse", [[[1., 0.], [0., 0.]], [[1., 0.], [2., 0.]]], [1, 1], [1, 1]),
    ("abc", [[[0., 0.], [1., 0.]], [[1., 0.], [2., 0.]]], [1, 1], [1, 1]),
    ("trust-islands", [[[0., 0.], [1., 0.]], [[1., 0.], [2., 0.]],
                       [[2., 0.], [3., 0.]]], [1, 1, 1], [1, 0, 1]),
    ("branch", [[[0., 0.], [1., 0.]], [[1., 0.], [2., 1.]],
                [[1., 0.], [2., -1.]]], [1, 1, 1], [1, 0, 1]),
    ("loop", [[[0., 0.], [1., 1.]], [[1., 1.], [2., 0.]],
              [[2., 0.], [0., 0.]]], [1, 1, 1], [1, 1, 1]),
    ("projected-overlap", [[[0., 0.], [2., 0.]], [[1., 1.], [3., 1.]]],
                         [1, 1], [0, 1]),
    ("zero-extent", [[[0., 0.], [0., 1.]], [[1., 0.], [1., 0.]]], [1, 1], [1, 0]),
    ("one-ulp-gap", [[[0., 0.], [1., 0.]],
                     [[float.fromhex("0x1.0000000000001p+0"), 0.], [2., 0.]]],
                    [1, 1], [1, 1]),
    ("base-first-negative-zero", [[[-0., 0.], [1., 0.]], [[+0., -0.], [-1., 0.]],
                                  [[1., 0.], [2., 0.]]], [1, 1, 1], [0, 1, 1]),
    ("trusted-order", [[[2., 0.], [1., 0.]], [[-0., 0.], [-1., 0.]],
                       [[1., 0.], [+0., 0.]]], [1, 1, 1], [0, 1, 1]),
    ("rejected-prefix", [[[10., 10.], [11., 10.]], [[-0., 0.], [1., 0.]],
                         [[12., 0.], [13., 0.]]], [0, 1, 0], [1, 0, 1]),
]


@pytest.mark.parametrize("name,segments,admitted,reliable", CASES, ids=[c[0] for c in CASES])
def test_ordered_graphs_byte_exact_and_fraction_contained(native9, name, segments, admitted, reliable):
    compare_case(native9, segments, admitted, reliable)


@pytest.mark.parametrize("tangent", [(1., 0.), (-1., -0.), (0., 1.), (-0., -1.),
                                     (.6, .8), (-.6, .8),
                                     (1., float.fromhex("0x0.0000000000001p-1022"))])
@pytest.mark.parametrize("origin", [(0., -0.), (1.e7, -1.e7), (-1.e7, 1.e7)])
def test_axes_cancellation_and_subnormal(native9, tangent, origin):
    segments = [[[0., -0.], [1., .5]], [[1., .5], [2., -.5]],
                [[-1.e7, 1.e7], [-9999999., 9999999.]]]
    compare_case(native9, segments, [1, 1, 1], [0, 1, 1], origin, tangent)


@pytest.mark.parametrize("seed", [7, 313, 999])
def test_fixed_seed_ordered_endpoint_pool(native9, seed):
    rng = np.random.RandomState(seed)
    pool = rng.randint(-16, 17, size=(32, 2)).astype(np.float64)/8.
    pool[:3] = [[-0., 0.], [0., -0.], [1., 0.]]
    for count in (1, 2, 7, 31, 128):
        ids = rng.randint(0, len(pool), size=(count, 2))
        points = pool[ids]
        admitted = rng.randint(0, 2, size=count)
        trusted = rng.randint(0, 2, size=count)
        for permutation in (np.arange(count), rng.permutation(count)):
            compare_case(native9, points[permutation], admitted[permutation],
                         trusted[permutation], (.25, -.125), (.6, .8))


def test_literal_projection_fraction_encloses_without_native():
    for point, origin, tangent in [
            ((1., 0.), (0., 0.), (1., 0.)),
            ((-0., 0.), (+0., -0.), (-1., -0.)),
            ((1.e7, -1.e7), (9999999., -9999999.), (.6, .8)),
            ((float.fromhex("0x0.0000000000001p-1022"), 1.), (0., 0.), (1., 0.))]:
        lo, hi = oracle_projection(point, origin, tangent)
        exact = sum((Fraction(point[i])-Fraction(origin[i]))*Fraction(tangent[i]) for i in (0, 1))
        assert Fraction(lo) <= exact <= Fraction(hi)


def test_native9_verifier_witness_real_call(native9):
    result = native9.finite_current_union(*native_args(
        [[[0., 0.], [1., 0.]], [[0., 0.], [1., 0.]]], [1, 1], [0, 1]))
    expected = ((0., float.fromhex("0x1.ffffffffffffep-1"), True),)
    assert pieces_bytes(result) == pieces_bytes(expected)


class BytesSubclass(bytes):
    pass


class IntSubclass(int):
    pass


class FloatSubclass(float):
    pass


@pytest.mark.parametrize("slot", [0, 1, 2])
@pytest.mark.parametrize("kind", ["bytearray", "view", "readonly-view", "subclass", "string"])
def test_ffi_exact_immutable_bytes(native9, slot, kind):
    args = list(native_args([[[0., 0.], [1., 0.]]]))
    value = args[slot]
    args[slot] = {"bytearray": lambda: bytearray(value),
                  "view": lambda: memoryview(value),
                  "readonly-view": lambda: memoryview(bytearray(value)).toreadonly(),
                  "subclass": lambda: BytesSubclass(value),
                  "string": lambda: ""}[kind]()
    with pytest.raises(TypeError):
        native9.finite_current_union(*args)


@pytest.mark.parametrize("value,error", [(True, TypeError), (np.int64(1), TypeError),
    (IntSubclass(1), TypeError), (1., TypeError), (-1, ValueError),
    (200001, ValueError), (10**100, ValueError)])
def test_ffi_count_exact_type_and_capacity(native9, value, error):
    args = list(native_args([])); args[3] = value
    with pytest.raises(error, match="count"):
        native9.finite_current_union(*args)


@pytest.mark.parametrize("slot", [0, 1, 2])
@pytest.mark.parametrize("delta", [-1, 1])
def test_ffi_lengths(native9, slot, delta):
    args = list(native_args([[[0., 0.], [1., 0.]]]))
    args[slot] = args[slot][:-1] if delta < 0 else args[slot]+b"\0"
    with pytest.raises(ValueError, match=("segments", "admitted", "reliable")[slot]):
        native9.finite_current_union(*args)


@pytest.mark.parametrize("slot", [4, 5, 6, 7])
@pytest.mark.parametrize("value", [True, 1, np.float64(1.), FloatSubclass(1.), "0"])
def test_ffi_chart_exact_floats(native9, slot, value):
    args = list(native_args([])); args[slot] = value
    with pytest.raises(TypeError, match="chart scalars"):
        native9.finite_current_union(*args)


@pytest.mark.parametrize("slot,value", [(4, math.inf), (5, math.nan),
    (6, -math.inf), (7, math.nan), (4, float.fromhex("0x1.312d000000001p+23")),
    (6, 1.01)])
def test_ffi_chart_finite_component_bounds_even_empty(native9, slot, value):
    args = list(native_args([])); args[slot] = value
    with pytest.raises(ValueError, match="chart scalars"):
        native9.finite_current_union(*args)


def test_ffi_does_not_add_hypot_policy(native9):
    # Private ABI permits these finite component values. Public Python owns
    # unit-normal admission. This zero chart creates no positive-length span.
    assert native9.finite_current_union(*native_args([[[0., 0.], [1., 1.]]],
                                                   tangent=(0., 0.))) == ()


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf,
                                   float.fromhex("0x1.312d000000001p+23")])
@pytest.mark.parametrize("coordinate", [0, 1, 2, 3])
def test_all_rejected_rows_still_validate_every_coordinate(native9, value, coordinate):
    segments = np.zeros((1, 2, 2)); segments.ravel()[coordinate] = value
    with pytest.raises(ValueError, match="segments must be finite and bounded"):
        native9.finite_current_union(*native_args(segments, [0], [0]))


@pytest.mark.parametrize("slot", [1, 2])
@pytest.mark.parametrize("value", [2, 255])
def test_all_rejected_rows_still_validate_canonical_masks(native9, slot, value):
    args = list(native_args([[[0., 0.], [1., 0.]]], [0], [0]))
    args[slot] = bytes([value])
    with pytest.raises(ValueError, match="zero or one"):
        native9.finite_current_union(*args)


def test_ffi_error_order(native9):
    args = list(native_args([[[math.nan, 0.], [1., 0.]]], [2], [255]))
    args[3] = True
    with pytest.raises(TypeError, match="count"):
        native9.finite_current_union(*args)
    args[3] = 1; args[0] = b""
    with pytest.raises(ValueError, match="segments must contain"):
        native9.finite_current_union(*args)
    args = list(native_args([[[math.nan, 0.], [1., 0.]]], [2], [255]))
    args[4] = "bad"
    with pytest.raises(TypeError, match="chart scalars"):
        native9.finite_current_union(*args)
    args[4] = 0.
    with pytest.raises(ValueError, match="segments must be finite"):
        native9.finite_current_union(*args)
    args[0] = np.zeros((1, 2, 2)).tobytes()
    with pytest.raises(ValueError, match="zero or one"):
        native9.finite_current_union(*args)


@pytest.mark.parametrize("count", [199999, 200000])
def test_capacity_boundary_repeated_identical_segment(native9, count):
    # Do not make the independent Python oracle loop 200000 times. Same-frame
    # duplicates have exactly the same domain as this separately-oracled row.
    points = np.tile(np.array([[[0., 0.], [1., 0.]]]), (count, 1, 1))
    result = native9.finite_current_union(*native_args(points))
    expected = oracle_current_union(points[:1], unit_normals(1), np.ones(1, bool),
                                    (0., 0.), (1., 0.), .5)
    assert pieces_bytes(result) == pieces_bytes(expected)


def test_immutable_concurrent_calls_and_return_lifetime(native9):
    args = native_args([[[0., 0.], [1., 0.]], [[1., 0.], [2., 1.]],
                        [[2., 1.], [3., 0.]]], [1, 1, 1], [1, 0, 1])
    before = tuple(hashlib.sha256(value).digest() for value in args[:3])
    expected = native9.finite_current_union(*args)
    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(lambda _: native9.finite_current_union(*args), range(20)))
    assert all(pieces_bytes(result) == pieces_bytes(expected) for result in results)
    assert before == tuple(hashlib.sha256(value).digest() for value in args[:3])
    del args
    assert_contract(results[-1])
    with pytest.raises(TypeError):
        results[-1][0] = (0., 0., False)
