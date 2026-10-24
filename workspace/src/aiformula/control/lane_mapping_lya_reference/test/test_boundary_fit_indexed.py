"""Indexed global-source bounds versus frozen selected and padded expressions.

The candidate is imported from the package normally. Both numerical oracles and
the original seeded factory are local literals, so no Git, checkout hash, private
diagnostic fixture, native binary identity or machine-specific path is required.
The selected literal freezes the pre-indexed arithmetic/error contract; the R16
padded literal freezes ordinary IEEE results, not unused-padding warning counts.
"""
import ast
import copy
import inspect
import textwrap
import unittest
import warnings

import numpy as np

from lane_mapping_lya_reference import paint_geometry as paint


def _literal_selected_bounds(origins, normals, scales, centers, segments,
                             arc_offsets, weights, radius):
    """Frozen pre-indexed selected arithmetic; never call a candidate helper."""
    count = len(origins)
    answer = np.full((count, 2), np.nan)
    if not count or not weights.shape[1]:
        return answer
    selected = np.asarray(weights, dtype=bool)
    nonempty = np.any(selected, axis=1)
    first = np.argmax(selected, axis=1)
    last = selected.shape[1]-1-np.argmax(selected[:, ::-1], axis=1)
    contiguous = np.sum(selected, axis=1) == last-first+1
    rows, columns = np.nonzero(selected)
    if not len(rows):
        return answer
    face_centers, faces = centers[rows, columns], segments[rows, columns]
    offsets = arc_offsets[rows, columns]
    connected = np.ones(count, dtype=bool)
    adjacent = (rows[:-1] == rows[1:]) & (columns[1:] == columns[:-1]+1)
    linked = np.all(faces[:-1, 1] == faces[1:, 0], axis=1)
    np.logical_and.at(connected, rows[1:][adjacent], linked[adjacent])
    finite = (np.all(np.isfinite(face_centers), axis=1)
              & np.all(np.isfinite(faces), axis=(1, 2))
              & np.isfinite(offsets) & (np.abs(offsets) <= radius))
    delta = faces-face_centers[:, None, :]
    lengths = np.linalg.norm(delta, axis=2)
    finite &= (np.all(lengths > 0., axis=1)
               & (np.linalg.norm(faces[:, 1]-faces[:, 0], axis=1) > 0.))
    row_finite = np.ones(count, dtype=bool)
    np.logical_and.at(row_finite, rows, finite)
    good = (nonempty & contiguous & connected & row_finite
            & np.all(np.isfinite(origins), axis=1)
            & np.all(np.isfinite(normals), axis=1)
            & np.isfinite(scales) & (scales > 0.))
    if not np.any(good):
        return answer
    remaining = np.stack((radius+offsets, radius-offsets), axis=1)
    with np.errstate(invalid='ignore', divide='ignore', over='ignore'):
        fraction = np.minimum(1., remaining/lengths)
        clipped = face_centers[:, None, :] + fraction[:, :, None]*delta
        clipped = np.where((fraction == 1.)[:, :, None], faces, clipped)
        tangents = np.column_stack((normals[:, 1], -normals[:, 0]))
        values = np.sum((clipped-origins[rows, None, :])
                        * tangents[rows, None, :], axis=2)/scales[rows, None]
    np.logical_and.at(good, rows, np.all(np.isfinite(values), axis=1))
    lower, upper = np.full(count, np.inf), np.full(count, -np.inf)
    np.minimum.at(lower, rows, np.min(values, axis=1))
    np.maximum.at(upper, rows, np.max(values, axis=1))
    zero_rows = np.flatnonzero(good & ((lower == 0.) | (upper == 0.)))
    if len(zero_rows):
        slots = np.full(count, -1, dtype=np.int64)
        slots[zero_rows] = np.arange(len(zero_rows))
        keep = slots[rows] >= 0
        padded = np.full((len(zero_rows), selected.shape[1], 2), np.inf)
        padded[slots[rows[keep]], columns[keep]] = values[keep]
        lower[zero_rows] = np.min(padded, axis=(1, 2))
        padded[~selected[zero_rows]] = -np.inf
        upper[zero_rows] = np.max(padded, axis=(1, 2))
    answer[good, 0], answer[good, 1] = lower[good], upper[good]
    return answer


def _literal_bounds(origins, normals, scales, centers, segments,
                    arc_offsets, weights, radius):
    # Frozen original R16 expressions, including padding and reduction order.
    count = len(origins)
    answer = np.full((count, 2), np.nan)
    if not count or not weights.shape[1]:
        return answer
    selected = np.asarray(weights, dtype=bool)
    nonempty = np.any(selected, axis=1)
    first = np.argmax(selected, axis=1)
    last = selected.shape[1]-1-np.argmax(selected[:, ::-1], axis=1)
    contiguous = np.sum(selected, axis=1) == last-first+1
    adjacent = selected[:, :-1] & selected[:, 1:]
    linked = np.all(segments[:, :-1, 1] == segments[:, 1:, 0], axis=2)
    connected = np.all(~adjacent | linked, axis=1)
    finite = (np.all(np.isfinite(centers), axis=2)
              & np.all(np.isfinite(segments), axis=(2, 3))
              & np.isfinite(arc_offsets) & (np.abs(arc_offsets) <= radius))
    delta = segments-centers[:, :, None, :]
    lengths = np.linalg.norm(delta, axis=3)
    finite &= (np.all(lengths > 0., axis=2)
               & (np.linalg.norm(segments[:, :, 1]-segments[:, :, 0], axis=2) > 0.))
    good = (nonempty & contiguous & connected & np.all(~selected | finite, axis=1)
            & np.all(np.isfinite(origins), axis=1)
            & np.all(np.isfinite(normals), axis=1)
            & np.isfinite(scales) & (scales > 0.))
    if not np.any(good):
        return answer
    remaining = np.stack((radius+arc_offsets, radius-arc_offsets), axis=2)
    with np.errstate(invalid='ignore', divide='ignore', over='ignore'):
        fraction = np.minimum(1., remaining/lengths)
        clipped = centers[:, :, None, :] + fraction[:, :, :, None]*delta
        clipped = np.where((fraction == 1.)[:, :, :, None], segments, clipped)
        tangents = np.column_stack((normals[:, 1], -normals[:, 0]))
        values = np.sum((clipped-origins[:, None, None, :])
                        * tangents[:, None, None, :], axis=3)/scales[:, None, None]
    finite_projection = np.all(np.isfinite(values), axis=2)
    good &= np.all(~selected | finite_projection, axis=1)
    lower = np.min(np.where(selected[:, :, None], values, np.inf), axis=(1, 2))
    upper = np.max(np.where(selected[:, :, None], values, -np.inf), axis=(1, 2))
    answer[good, 0], answer[good, 1] = lower[good], upper[good]
    return answer


def _inputs(seed, count=8, width=17):
    rng = np.random.RandomState(seed)
    vertices = np.cumsum(rng.uniform([.001, -.003], [.008, .003],
                                    (count, width+1, 2)), axis=1)
    vertices += rng.uniform(-3., 3., (count, 1, 2))
    segments = np.stack((vertices[:, :-1], vertices[:, 1:]), axis=2)
    fractions = rng.uniform(.1, .9, (count, width, 1))
    centers = segments[:, :, 0] + fractions*(segments[:, :, 1]-segments[:, :, 0])
    angles = rng.uniform(-np.pi, np.pi, count)
    normals = np.column_stack((np.sin(angles), np.cos(angles)))
    offsets = np.broadcast_to((np.arange(width)-width//2)*.005, (count, width)).copy()
    weights = np.zeros((count, width), dtype=bool)
    for row in range(count):
        if row % 4 == 1:
            weights[row] = True
        elif row % 4 == 2:
            start = max(0, width//2-3)
            weights[row, start:min(width, start+7)] = True
        elif row % 4 == 3:
            weights[row] = rng.rand(width) > .3
    return dict(origins=rng.uniform(-2., 2., (count, 2)), normals=normals,
                scales=rng.uniform(.1, 2., count), centers=centers,
                segments=segments, arc_offsets=offsets, weights=weights, radius=.2)


FROZEN_SELECTED = _literal_selected_bounds
FROZEN_PADDED = _literal_bounds
FROZEN_INPUTS = _inputs
CANDIDATE = paint._finite_face_fit_bounds_indexed


def _function_node(function):
    tree = ast.parse(textwrap.dedent(inspect.getsource(function)))
    definitions = [node for node in tree.body if isinstance(node, ast.FunctionDef)]
    if len(definitions) != 1:
        raise RuntimeError("expected one source function for structural oracle")
    return definitions[0]


def _body_without_docstring(node):
    body = node.body
    if (body and isinstance(body[0], ast.Expr)
            and isinstance(body[0].value, ast.Constant)
            and isinstance(body[0].value.value, str)):
        body = body[1:]
    return copy.deepcopy(body)


def _array_snapshot(arguments):
    return {name: (value.shape, value.dtype.str, value.strides,
                   bool(value.flags.writeable), value.tobytes())
            for name, value in arguments.items()
            if isinstance(value, np.ndarray)}


def _dense_arguments(indexed):
    """Expected-side full gathers, independent of candidate selection/packing."""
    return dict(origins=indexed["origins"], normals=indexed["normals"],
                scales=indexed["scales"],
                centers=indexed["points"][indexed["member_ids"]],
                segments=indexed["segments"][indexed["member_ids"]],
                arc_offsets=indexed["arc_offsets"], weights=indexed["weights"],
                radius=indexed["radius"])


def _indexed_arguments(case, seed=7301):
    """Reindex dense support into a permuted global source without fitting it."""
    q, k = case["weights"].shape
    count = q * k
    order = np.random.RandomState(seed).permutation(count)
    inverse = np.empty(count, dtype=np.int64)
    inverse[order] = np.arange(count, dtype=np.int64)
    points = case["centers"].reshape((count, 2))[order].copy()
    segments = case["segments"].reshape((count, 2, 2))[order].copy()
    return dict(origins=case["origins"], normals=case["normals"],
                scales=case["scales"], points=points, segments=segments,
                member_ids=inverse.reshape((q, k)),
                arc_offsets=case["arc_offsets"], weights=case["weights"],
                radius=case["radius"])


def _simple_case(width=9, count=3):
    """Dyadic connected faces with deliberately asymmetric projected centers."""
    vertices = np.zeros((count, width + 1, 2), dtype=np.float64)
    vertices[:, :, 0] = (np.arange(width + 1, dtype=np.float64)[None, :] / 8.
                         + np.arange(count, dtype=np.float64)[:, None] * 4.)
    segments = np.stack((vertices[:, :-1], vertices[:, 1:]), axis=2)
    centers = segments[:, :, 0] + .25 * (segments[:, :, 1] - segments[:, :, 0])
    origins = (centers[:, width // 2].copy() if width
               else np.zeros((count, 2), dtype=np.float64))
    offsets = np.broadcast_to(
        (np.arange(width, dtype=np.float64) - width // 2) / 8.,
        (count, width)).copy()
    return dict(origins=origins,
                normals=np.tile(np.array([[0., 1.]]), (count, 1)),
                scales=np.ones(count), centers=centers, segments=segments,
                arc_offsets=offsets, weights=np.ones((count, width), dtype=bool),
                radius=float(width + 1))


def _layout(value, name):
    if name == "fortran":
        return np.asfortranarray(value)
    if name == "strided":
        backing = np.empty(value.shape[:-1] + (value.shape[-1] * 2,),
                           dtype=value.dtype)
        backing[..., ::2] = value
        return backing[..., ::2]
    raise ValueError("unknown diagnostic layout")


def _capture(function, arguments, mode):
    result, error = None, None
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with np.errstate(all=mode):
            try:
                result = function(**arguments)
            except Exception as exception:
                error = (type(exception).__module__, type(exception).__qualname__,
                         str(exception))
    messages = [(item.category.__module__, item.category.__qualname__,
                 str(item.message)) for item in caught]
    return result, error, messages


class IndexedBoundsTest(unittest.TestCase):
    def assert_bytes(self, actual, expected):
        self.assertEqual(actual.dtype, np.dtype(np.float64))
        self.assertEqual(actual.dtype.str, expected.dtype.str)
        self.assertEqual(actual.shape, expected.shape)
        self.assertEqual(np.isnan(actual).tobytes(), np.isnan(expected).tobytes())
        self.assertEqual(np.isfinite(actual).tobytes(),
                         np.isfinite(expected).tobytes())
        self.assertEqual(actual.tobytes(), expected.tobytes())

    def check_indexed(self, indexed):
        before = _array_snapshot(indexed)
        dense = _dense_arguments(indexed)
        dense_before = _array_snapshot(dense)
        with np.errstate(all="ignore"):
            selected = FROZEN_SELECTED(**dense)
            padded = FROZEN_PADDED(**dense)
            actual = CANDIDATE(**indexed)
        self.assert_bytes(actual, selected)
        self.assert_bytes(actual, padded)
        self.assertEqual(_array_snapshot(indexed), before)
        self.assertEqual(_array_snapshot(dense), dense_before)
        return actual

    def check_case(self, case, seed=7301):
        before = _array_snapshot(case)
        indexed = _indexed_arguments(case, seed)
        dense = _dense_arguments(indexed)
        self.assertEqual(dense["centers"].tobytes(), case["centers"].tobytes())
        self.assertEqual(dense["segments"].tobytes(), case["segments"].tobytes())
        result = self.check_indexed(indexed)
        self.assertEqual(_array_snapshot(case), before)
        return result

    def check_profiles(self, case):
        indexed = _indexed_arguments(case)
        dense = _dense_arguments(indexed)
        before = _array_snapshot(indexed)
        dense_before = _array_snapshot(dense)
        results = {}
        for mode in ("warn", "raise"):
            with self.subTest(mode=mode):
                expected = _capture(FROZEN_SELECTED, dense, mode)
                actual = _capture(CANDIDATE, indexed, mode)
                self.assertEqual(actual[1], expected[1])
                self.assertEqual(actual[2], expected[2])
                if expected[1] is None:
                    self.assert_bytes(actual[0], expected[0])
                else:
                    self.assertIsNone(actual[0])
                self.assertEqual(_array_snapshot(indexed), before)
                self.assertEqual(_array_snapshot(dense), dense_before)
                results[mode] = actual
        return results

    def test_frozen_selected_literal_matches_retained_dense_entry_body(self):
        literal = _function_node(FROZEN_SELECTED)
        retained = _function_node(paint._finite_face_fit_bounds)
        self.assertEqual(ast.dump(literal.args, include_attributes=False),
                         ast.dump(retained.args, include_attributes=False))
        dump = lambda node: ast.dump(node, include_attributes=False)
        self.assertEqual([dump(node) for node in _body_without_docstring(literal)],
                         [dump(node) for node in _body_without_docstring(retained)])
        self.assertIs(CANDIDATE, paint._finite_face_fit_bounds_indexed)
        self.assertIsNot(FROZEN_SELECTED, paint._finite_face_fit_bounds)
        self.assertIsNot(FROZEN_SELECTED, CANDIDATE)
        self.assertIsNot(FROZEN_PADDED, CANDIDATE)

    def test_ast_body_changes_only_selected_source_gather_and_signature(self):
        original = _function_node(FROZEN_SELECTED)
        candidate = _function_node(CANDIDATE)
        self.assertEqual([item.arg for item in candidate.args.args],
                         ["origins", "normals", "scales", "points", "segments",
                          "member_ids", "arc_offsets", "weights", "radius"])
        self.assertFalse(candidate.args.defaults)
        self.assertFalse(candidate.args.kwonlyargs)
        self.assertIsNone(candidate.args.vararg)
        self.assertIsNone(candidate.args.kwarg)
        body = _body_without_docstring(candidate)
        old_body = _body_without_docstring(original)
        inserted = ast.parse("face_ids = member_ids[rows, columns]").body[0]
        changed = ast.parse(
            "face_centers, faces = points[face_ids], segments[face_ids]").body[0]
        old_gather = ast.parse(
            "face_centers, faces = centers[rows, columns], segments[rows, columns]").body[0]
        dump = lambda node: ast.dump(node, include_attributes=False)
        positions = [index for index, node in enumerate(body)
                     if dump(node) == dump(inserted)]
        self.assertEqual(len(positions), 1)
        position = positions[0]
        self.assertEqual(dump(body[position + 1]), dump(changed))
        body[position:position + 2] = [old_gather]
        self.assertEqual([dump(node) for node in body],
                         [dump(node) for node in old_body])

    def test_frozen_random_factory_byte_equivalence(self):
        for seed in range(12):
            width = (1, 7, 33, 512)[seed % 4]
            with self.subTest(seed=seed, width=width):
                result = self.check_case(FROZEN_INPUTS(seed, width=width))
                self.assertTrue(np.isfinite(result).any())

    def test_row_batch_boundaries(self):
        for count in (0, 1, 127, 128, 129):
            with self.subTest(count=count):
                self.check_case(FROZEN_INPUTS(281, count=count, width=9))

    def test_original_window_widths_and_empty_dimensions(self):
        for count, width in ((0, 0), (0, 17), (3, 0), (4, 1), (4, 2),
                             (4, 3), (4, 8), (4, 17), (4, 64), (4, 512)):
            with self.subTest(count=count, width=width):
                self.check_case(FROZEN_INPUTS(91, count=count, width=width))

    def test_empty_all_false_and_mixed_rows(self):
        case = _simple_case(count=5)
        case["weights"][:] = False
        self.assertTrue(np.isnan(self.check_case(case)).all())
        case["weights"][1] = True
        case["weights"][3, 2:6] = True
        result = self.check_case(case)
        self.assertTrue(np.isfinite(result[[1, 3]]).all())
        self.assertTrue(np.isnan(result[[0, 2, 4]]).all())

    def test_single_face_and_leading_trailing_contiguous_ranges(self):
        case = _simple_case(count=5)
        case["weights"][:] = False
        case["weights"][0, 0] = True
        case["weights"][1, -1] = True
        case["weights"][2, 3] = True
        case["weights"][3, :5] = True
        case["weights"][4, -4:] = True
        self.assertTrue(np.isfinite(self.check_case(case)).all())

    def test_arbitrary_global_source_permutations(self):
        case = _simple_case(width=17, count=4)
        for seed in (0, 1, 371, 2**31 - 1):
            with self.subTest(seed=seed):
                result = self.check_case(case, seed=seed)
                self.assertTrue(np.isfinite(result).all())

    def test_duplicate_global_ids_between_rows_with_distinct_charts(self):
        case = _simple_case(width=7, count=4)
        for row in (1, 3):
            case["centers"][row] = case["centers"][0]
            case["segments"][row] = case["segments"][0]
            case["arc_offsets"][row] = case["arc_offsets"][0]
        indexed = _indexed_arguments(case)
        indexed["member_ids"][[1, 3]] = indexed["member_ids"][0]
        indexed["normals"][1] = [0., -1.]
        indexed["scales"][3] = 2.
        self.assertTrue(np.isfinite(self.check_indexed(indexed)).all())

    def test_closed_contour_seam_keeps_directed_id_order(self):
        vertices = np.array([[0., 0.], [2., 0.], [2., 2.], [0., 2.], [0., 0.]])
        segments = np.stack((vertices[:-1], vertices[1:]), axis=1)
        points = np.mean(segments, axis=1)
        indexed = dict(origins=np.zeros((2, 2)),
                       normals=np.array([[0., 1.], [1., 0.]]),
                       scales=np.array([1., 2.]), points=points, segments=segments,
                       member_ids=np.array([[3, 0, 1], [2, 3, 0]], dtype=np.int64),
                       arc_offsets=np.zeros((2, 3)),
                       weights=np.ones((2, 3), dtype=bool), radius=20.)
        self.assertTrue(np.isfinite(self.check_indexed(indexed)).all())
        indexed["member_ids"][0] = [0, 3, 1]
        result = self.check_indexed(indexed)
        self.assertTrue(np.isnan(result[0]).all())
        self.assertTrue(np.isfinite(result[1]).all())

    def test_gaps_noncontiguity_nextafter_and_reversed_faces(self):
        for kind in ("internal_selection_gap", "endpoint_gap", "nextafter_gap",
                     "reversed_face"):
            with self.subTest(kind=kind):
                case = _simple_case(width=7, count=3)
                if kind == "internal_selection_gap":
                    case["weights"][0, 3] = False
                elif kind == "endpoint_gap":
                    case["segments"][0, 3, 0, 0] += .125
                elif kind == "nextafter_gap":
                    case["segments"][0, 3, 0, 0] = np.nextafter(
                        case["segments"][0, 3, 0, 0], np.inf)
                else:
                    case["segments"][0, 3] = case["segments"][0, 3, ::-1].copy()
                result = self.check_case(case)
                self.assertTrue(np.isnan(result[0]).all())
                self.assertTrue(np.isfinite(result[1:]).all())

    def test_unselected_nonfinite_extreme_and_degenerate_padding(self):
        for kind in ("nan", "inf", "extreme", "degenerate"):
            with self.subTest(kind=kind):
                case = _simple_case()
                case["weights"][:] = False
                case["weights"][:, 3:6] = True
                if kind == "degenerate":
                    case["segments"][:, :3] = case["centers"][:, :3, None, :]
                else:
                    value = {"nan": np.nan, "inf": np.inf, "extreme": 1.e200}[kind]
                    case["centers"][:, :3] = value
                    case["segments"][:, :3] = value
                    case["arc_offsets"][:, :3] = value
                self.assertTrue(np.isfinite(self.check_case(case)).all())

    def test_selected_nonfinite_geometry_and_offsets(self):
        for field in ("centers", "segments", "arc_offsets"):
            for value in (np.nan, np.inf, -np.inf):
                with self.subTest(field=field, value=value):
                    case = _simple_case()
                    case[field][0, 3] = value
                    result = self.check_case(case)
                    self.assertTrue(np.isnan(result[0]).all())
                    self.assertTrue(np.isfinite(result[1:]).all())

    def test_selected_zero_faces_and_centers_at_endpoint(self):
        for kind in ("zero_at_center", "zero_away", "center_at_first",
                     "center_at_last"):
            with self.subTest(kind=kind):
                case = _simple_case(width=1, count=3)
                if kind == "zero_at_center":
                    case["segments"][0, 0] = case["centers"][0, 0]
                elif kind == "zero_away":
                    case["segments"][0, 0] = [2., 0.]
                else:
                    index = 0 if kind == "center_at_first" else 1
                    case["centers"][0, 0] = case["segments"][0, 0, index]
                result = self.check_case(case)
                self.assertTrue(np.isnan(result[0]).all())
                self.assertTrue(np.isfinite(result[1:]).all())

    def test_radius_endpoint_nextafter_and_asymmetric_clipping(self):
        case = _simple_case(width=1, count=6)
        radius = 1. / 16.
        case["radius"] = radius
        case["arc_offsets"][:, 0] = [0., radius, -radius,
                                      np.nextafter(radius, np.inf),
                                      np.nextafter(-radius, -np.inf), 0.]
        case["centers"][5, 0] = (case["segments"][5, 0, 0]
                                 + .75 * (case["segments"][5, 0, 1]
                                          - case["segments"][5, 0, 0]))
        result = self.check_case(case)
        self.assertTrue(np.isfinite(result[[0, 1, 2, 5]]).all())
        self.assertTrue(np.isnan(result[[3, 4]]).all())

    def test_invalid_origins_normals_and_scales(self):
        mutations = (("origins", np.nan), ("origins", np.inf),
                     ("normals", np.nan), ("normals", -np.inf),
                     ("scales", np.nan), ("scales", np.inf),
                     ("scales", 0.), ("scales", -1.))
        for field, value in mutations:
            with self.subTest(field=field, value=value):
                case = _simple_case()
                case[field][0] = value
                result = self.check_case(case)
                self.assertTrue(np.isnan(result[0]).all())
                self.assertTrue(np.isfinite(result[1:]).all())

    def test_extreme_finite_coordinates_and_subnormal_scale(self):
        for kind in ("huge_geometry", "tiny_scale", "huge_normal"):
            with self.subTest(kind=kind):
                case = _simple_case()
                with np.errstate(all="ignore"):
                    if kind == "huge_geometry":
                        for field in ("origins", "centers", "segments"):
                            case[field][0] *= 1.e200
                    elif kind == "tiny_scale":
                        case["scales"][0] = np.nextafter(0., 1.)
                    else:
                        case["normals"][0] = [1.e308, -1.e308]
                self.check_case(case)

    def test_signed_zero_original_width_fallback(self):
        for width in (1, 2, 3, 8, 17, 64, 512):
            with self.subTest(width=width):
                case = FROZEN_INPUTS(512, count=4, width=width)
                vertices = np.zeros((4, width + 1, 2))
                vertices[:, :, 1] = np.arange(width + 1) * .01
                vertices[:, ::2, 0] = -0.
                case["segments"] = np.stack((vertices[:, :-1], vertices[:, 1:]), axis=2)
                case["centers"] = np.mean(case["segments"], axis=2)
                case["normals"][:] = [0., 1.]
                case["origins"][:] = [0., 0.]
                case["origins"][1::2, 0] = -0.
                case["arc_offsets"][:] = 0.
                case["weights"][:] = False
                case["weights"][:, width // 2] = True
                case["weights"][1] = True
                case["radius"] = 100.
                self.assertTrue(np.all(self.check_case(case) == 0.))

    def test_zero_bounds_interleaved_with_nonzero_bad_and_empty_rows(self):
        case = _simple_case(width=7, count=6)
        for row in (0, 2, 4):
            case["segments"][row, :, :, 0] = -0.
            case["segments"][row, :, 0, 1] = np.arange(7) / 8.
            case["segments"][row, :, 1, 1] = np.arange(1, 8) / 8.
            case["centers"][row] = np.mean(case["segments"][row], axis=1)
            case["origins"][row] = [-0., 0.]
            case["normals"][row] = [-0., 1.]
        case["weights"][3] = False
        case["scales"][4] = 0.
        case["weights"][0, :2] = False
        case["weights"][2, 4:] = False
        result = self.check_case(case)
        self.assertTrue(np.all(result[[0, 2]] == 0.))
        self.assertTrue(np.isfinite(result[[1, 5]]).all())
        self.assertTrue(np.isnan(result[[3, 4]]).all())

    def test_strided_and_fortran_source_ids_weights_offsets(self):
        for layout in ("strided", "fortran"):
            with self.subTest(layout=layout):
                indexed = _indexed_arguments(FROZEN_INPUTS(92, width=33))
                for name, value in list(indexed.items()):
                    if isinstance(value, np.ndarray):
                        indexed[name] = _layout(value, layout)
                self.check_indexed(indexed)

    def test_readonly_input_views(self):
        for layout in ("strided", "fortran"):
            with self.subTest(layout=layout):
                indexed = _indexed_arguments(_simple_case(width=17))
                for name, value in list(indexed.items()):
                    if isinstance(value, np.ndarray):
                        indexed[name] = _layout(value, layout)
                        indexed[name].setflags(write=False)
                self.assertTrue(np.isfinite(self.check_indexed(indexed)).all())

    def test_existing_radius_limits_are_not_new_gates(self):
        for radius in (0., -0., -1., np.nan, np.inf, .0625, 1.e308):
            with self.subTest(radius=radius):
                case = _simple_case(width=1)
                case["radius"] = radius
                self.check_case(case)

    def test_warning_order_and_first_error_match_frozen_selected(self):
        for kind in ("norm_overflow", "invalid_difference", "remaining_overflow"):
            with self.subTest(kind=kind):
                case = _simple_case(width=1)
                if kind == "norm_overflow":
                    case["segments"][0] *= 1.e200
                    case["centers"][0] *= 1.e200
                    case["origins"][0] *= 1.e200
                elif kind == "invalid_difference":
                    case["segments"][0] = np.inf
                    case["centers"][0] = np.inf
                else:
                    case["radius"] = 1.e308
                    case["arc_offsets"][0, 0] = 1.e308
                results = self.check_profiles(case)
                self.assertIsNotNone(results["raise"][1])
                self.assertEqual(results["raise"][1][1], "FloatingPointError")
                self.assertTrue(results["warn"][2])

    def test_underflow_warning_and_first_error_match_frozen_selected(self):
        case = _simple_case(width=1)
        for field in ("origins", "centers", "segments"):
            case[field] *= 1.e-200
        case["radius"] = 1.e-200
        results = self.check_profiles(case)
        self.assertIsNotNone(results["raise"][1])
        self.assertEqual(results["raise"][1][1], "FloatingPointError")
        self.assertTrue(results["warn"][2])

    def test_unused_padding_arithmetic_remains_unused_in_strict_mode(self):
        for value in (np.nan, np.inf, 1.e200):
            with self.subTest(value=value):
                case = _simple_case()
                case["weights"][:] = False
                case["weights"][:, 3:6] = True
                case["centers"][:, :3] = value
                case["segments"][:, :3] = value
                case["arc_offsets"][:, :3] = value
                self.check_case(case)
                results = self.check_profiles(case)
                self.assertIsNone(results["raise"][1])
                self.assertFalse(results["warn"][2])


if __name__ == "__main__":
    unittest.main(verbosity=2)
