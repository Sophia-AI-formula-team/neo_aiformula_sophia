"""Independent scalar-negative and complete-path oracle for R30 batching.

Frozen scalar arithmetic and directed-path body are from the pre-batch
corner_brackets e0254dc2 / paint_geometry c57c1d53 source. No Git/private input
or old checkout is required. Every finite-input true negative has a Fraction
separating-band check; false means UNKNOWN, not a feasibility certificate.
The original 22 synthetic masks are reproduced and checked against their
frozen mask-byte hashes; no analytic truth enters extraction.
"""
from dataclasses import fields
from fractions import Fraction
import hashlib
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from lane_mapping_lya_reference import corner_brackets as subject
from lane_mapping_lya_reference import paint_geometry as geometry


def _literal_dot_enclosure(first, second):
    products = first*second
    lower = np.nextafter(products, -np.inf)
    upper = np.nextafter(products, np.inf)
    return (np.nextafter(lower[..., 0]+lower[..., 1], -np.inf),
            np.nextafter(upper[..., 0]+upper[..., 1], np.inf))


def _literal_negative(vertex, axes, normals, whites, blacks):
    # Complete old scalar operations, shapes and reduction order, not new code.
    with np.errstate(all='ignore'):
        white_lower, _ = _literal_dot_enclosure(normals[:, None, :], whites)
        _, black_upper = _literal_dot_enclosure(normals[:, None, :], blacks)
        origin_lower, origin_upper = _literal_dot_enclosure(normals, vertex)
        axes_lower, axes_upper = _literal_dot_enclosure(normals[:, None, :], axes[None, :, :])
        magnitudes = np.maximum(np.abs(axes_lower), np.abs(axes_upper))
        radius = np.nextafter(.5*np.nextafter(magnitudes[:, 0]+magnitudes[:, 1], np.inf), np.inf)
        footprint_lower = np.nextafter(origin_lower-radius, -np.inf)
        footprint_upper = np.nextafter(origin_upper+radius, np.inf)
        lower = np.max(white_lower, axis=1)
        upper = np.min(black_upper, axis=1)
    if not all(np.all(np.isfinite(value)) for value in (
            lower, upper, footprint_lower, footprint_upper)):
        return False
    return bool(np.any((lower > upper) | (lower > footprint_upper) | (upper < footprint_lower)))


def _fraction(value):
    return Fraction.from_float(float(value))


def _exact_dot(first, second):
    return sum((_fraction(a)*_fraction(b) for a, b in zip(first, second)), Fraction())


def _exact_separation(vertex, axes, normals, whites, blacks):
    """Independent necessary-condition witness, original closed P/all labels.

    No solver, outward-float operation, subject helper, fitted normal, physical
    line assumption or approximate residual is used to make this assertion.
    """
    for normal, white, black in zip(normals, whites, blacks):
        lower = max(_exact_dot(normal, point) for point in white)
        upper = min(_exact_dot(normal, point) for point in black)
        origin = _exact_dot(normal, vertex)
        radius = sum((abs(_exact_dot(normal, axis)) for axis in axes), Fraction()) / 2
        if lower > upper or lower > origin+radius or upper < origin-radius:
            return True
    return False


def _signature(value):
    if isinstance(value, np.ndarray):
        return value.dtype.str, value.shape, value.tobytes(), value.flags.writeable
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    return tuple(_signature(item) for item in value)


def _batch_inputs(rows=6, k=6):
    vertex = np.zeros((rows, 2), dtype=np.float64)
    axes = np.tile(np.eye(2), (rows, 1, 1))
    normals = axes.copy()
    whites = np.zeros((rows, 2, k, 2), dtype=np.float64)
    blacks = whites.copy()
    for row in range(rows):
        lower, upper = [(-.25, .25), (2., 1.), (2., 3.), (-3., -2.), (0., 0.), (-.5, .5)][row % 6]
        whites[row, 0, :, 0] = lower
        whites[row, 1, :, 1] = lower
        blacks[row, 0, :, 0] = upper
        blacks[row, 1, :, 1] = upper
    return [vertex, axes, normals, whites, blacks]


def _scalar_rows(args):
    return np.array([_literal_negative(*(value[row] for value in args))
                     for row in range(len(args[0]))], dtype=np.bool_)


def _scalar_direction_paths(points, segments, stable, paths, cfg, *, white_xy=None,
                            black_xy=None, cut_evidence=None):
    # Frozen complete c57 arithmetic/control body. Only its name/global source
    # and local certificate import spelling differ from the old function.
    if (white_xy is None) != (black_xy is None):
        raise ValueError('corner domains require both white and black labels')
    certify_corner_brackets = subject.certify_corner_brackets
    result = []
    k = cfg['normal_min_points']
    cosine = math.cos(geometry.CONSENSUS_DEFAULT_CONFIG['normal_max_angle_rad'])
    for path, closed in paths:
        count = len(path)
        if count < 2*k:
            result.append((path, closed))
            continue
        junctions = np.arange(count if closed else count-1)
        score = np.full(len(junctions), -np.inf)
        quality = np.full(len(junctions), np.inf)
        corner_normals = {}
        for start in range(0, len(junctions), 128):
            rows = junctions[start:start+128]
            slots = rows[:, None]+np.arange(-k+1, k+1)
            inside = np.ones(len(rows), dtype=bool) if closed else np.all(
                (slots >= 0) & (slots < count), axis=1)
            members = path[slots % count]
            usable = inside & np.all(stable[members], axis=1)
            rows, ids = rows[usable], members[usable].reshape(-1, k)
            if not len(rows):
                continue
            coordinates = points[ids]
            delta = coordinates-coordinates[:, :1]
            centered = delta-np.mean(delta, axis=1)[:, None]
            covariance = np.einsum('nki,nkj->nij', centered, centered)/k
            eigenvalues, eigenvectors = np.linalg.eigh(covariance)
            good = ((eigenvalues[:, 1] > 1.e-5)
                & (eigenvalues[:, 0] <= cfg['normal_max_thickness_m']**2)
                & (eigenvalues[:, 0] <= cfg['normal_max_variance_ratio']*eigenvalues[:, 1]))
            tangents = eigenvectors[:, :, 1]
            observed_direction = segments[ids[:, -1], 1]-segments[ids[:, 0], 0]
            sign = np.sum(tangents*observed_direction, axis=1)
            good &= np.isfinite(sign) & (sign != 0.)
            tangents[sign < 0.] *= -1.
            pair = tangents.reshape(-1, 2, 2)
            agreement = np.sum(pair[:, 0]*pair[:, 1], axis=1)
            accepted = np.all(good.reshape(-1, 2), axis=1) & (agreement < cosine)
            means = np.mean(coordinates, axis=1).reshape(-1, 2, 2)
            cross = lambda a, b: a[..., 0]*b[..., 1]-a[..., 1]*b[..., 0]
            determinant = cross(pair[:, 0], pair[:, 1])
            junction = segments[path[rows], 1]
            first = segments[path[rows], 1]-segments[path[rows], 0]
            next_ids = path[(rows+1) % count]
            second = segments[next_ids, 1]-segments[next_ids, 0]
            footprint_determinant = cross(first, second)
            with np.errstate(divide='ignore', invalid='ignore', over='ignore'):
                along = cross(means[:, 1]-means[:, 0], pair[:, 1])/determinant
                intersection = means[:, 0]+along[:, None]*pair[:, 0]
                offset = intersection-junction
                u = cross(offset, second)/footprint_determinant
                v = cross(first, offset)/footprint_determinant
            if white_xy is None:
                accepted &= (np.isfinite(u) & np.isfinite(v) & (np.abs(u) <= .5)
                             & (np.abs(v) <= .5))
            else:
                side_ids = ids.reshape(-1, 2, k)
                for slot in np.flatnonzero(accepted):
                    normals = np.column_stack((pair[slot, :, 1], -pair[slot, :, 0]))
                    white = white_xy[side_ids[slot]]
                    black = black_xy[side_ids[slot]]
                    outward = np.sum(black-white, axis=1)
                    normals[np.sum(normals*outward, axis=1) < 0.] *= -1.
                    accepted[slot] = certify_corner_brackets(junction[slot],
                        np.stack((first[slot], second[slot])), normals, white, black) is not None
                    if cut_evidence is not None and accepted[slot]:
                        corner_normals[int(rows[slot])] = normals.copy()
            score[rows[accepted]] = 1.-agreement[accepted]
            ratio = eigenvalues[:, 0]/eigenvalues[:, 1]
            quality[rows[accepted]] = np.sum(ratio.reshape(-1, 2), axis=1)[accepted]
        peaks = np.flatnonzero(np.isfinite(score))
        extended = np.r_[peaks-count, peaks, peaks+count] if closed else peaks
        cuts = []
        for index in peaks:
            left = np.searchsorted(extended, index-k, side='right')
            right = np.searchsorted(extended, index+k, side='left')
            neighbours = extended[left:right] % count
            best = neighbours[score[neighbours] == np.max(score[neighbours])]
            best = best[quality[best] == np.min(quality[best])]
            if len(best) == 1 and best[0] == index:
                cuts.append(int(index))
        if not cuts:
            result.append((path, closed))
            continue
        cuts.sort()
        if cut_evidence is not None and white_xy is not None:
            for cut in cuts:
                side_ids = path[(cut+np.arange(-k+1, k+1)) % count].reshape(2, k)
                cut_evidence.append((side_ids.copy(), corner_normals[cut]))
        if closed:
            for start, stop in zip(cuts, cuts[1:]+[cuts[0]+count]):
                result.append((path[np.arange(start+1, stop+1) % count], False))
        else:
            starts = [0]+[cut+1 for cut in cuts]
            stops = [cut+1 for cut in cuts]+[count]
            result.extend((path[a:b], False) for a, b in zip(starts, stops))
    return result


def _path_inputs(closed=False, shift=0, angle=0., labelled=True):
    corners = [(0., 0.), (1.6, 0.), (1.6, 1.2)]
    counts = [40, 30]
    if closed:
        corners += [(0., 1.2), (0., 0.)]
        counts += [40, 30]
    vertices = [np.array(corners[0])]
    for first, last, count in zip(corners[:-1], corners[1:], counts):
        first, last = np.array(first), np.array(last)
        vertices.extend(first+(last-first)*fraction
                        for fraction in np.linspace(0., 1., count+1)[1:])
    vertices = np.array(vertices)
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    vertices = vertices @ rotation.T + [1.37, -.91]
    segments = np.stack((vertices[:-1], vertices[1:]), axis=1)
    points = np.mean(segments, axis=1)
    path = np.roll(np.arange(len(points), dtype=np.int64), shift) if closed else np.arange(len(points), dtype=np.int64)
    result = dict(points=points, segments=segments, stable=np.ones(len(points), dtype=bool),
                  paths=[(path, closed)], cfg=dict(geometry.DEFAULT_CONFIG))
    if labelled:
        delta = segments[:, 1]-segments[:, 0]
        normals = np.column_stack((delta[:, 1], -delta[:, 0])) / np.linalg.norm(delta, axis=1)[:, None]
        result.update(white_xy=points-.005*normals, black_xy=points+.005*normals)
    return result


MASK_SHA256 = {
    'longcap_phase_0.0': '672748b217aebdd71dfc0110d7af1e32adcf58e0310f994946bd966139504be2',
    'longcap_phase_0.35': '2cacf98e83f6d70caaa220515386acbfcab47eded5b7948d19259d0f8f2fd1b2',
    'shortcap_1_phase_0.0': 'd7eab3897feac7c6456e37d80c616afacb5b99c93645242fad9d131591280d68',
    'shortcap_1_phase_0.35': '0ee6b867ed51b186527b4708d1e9cf4d57f7e68014c4a0763c2aa5f5ea941fff',
    'shortcap_3_phase_0.0': '5d0fc2fa7a6696827c912b619553996720645aaa0ef5cc12633a8399b4b26d54',
    'shortcap_3_phase_0.35': '127db6adccc14a6804cc37d53fd99bb801ba978af7dfd334ea890f7a2e59c184',
    'shortcap_5_phase_0.0': '53a4ee2095117f09dfa16d6bf9cefe013dd98bfa7f7e9d5fe7b1386578cad478',
    'shortcap_5_phase_0.35': '3fbed7a0d33bd618c170cbf473f463a81a91ba9ad21b606026915953d0678d9b',
    'L_corner_phase_0.0': 'e6a827613db135d5fdb83fe1e20a80b42b1adbf341977f1b52f82e85b3b1b30e',
    'L_corner_phase_0.35': '5a84a12c2b7f088f9c0d9a0c63bbdcdc2673a3450b4ebb7faeea96d585eeccbc',
    'narrow_U_phase_0.0': 'f585909d28be8a6079ae29e65532480ed16743d869e0b72585866bc0cc468d26',
    'narrow_U_phase_0.35': '239da7942890670bd3b65d06390f4878cd77ff94e61aec17a04b15ebf303027c',
    'parallel_phase_0.0': 'a9bff6ea7f42ff595c717093fd278f57624c104cbc0fa65185f602d4a713cff8',
    'parallel_phase_0.35': '856e73073714ba1983d5aea23e2a6834cf93a504bbc05818fc11ce65113edbe4',
    'circle_0.6_phase_0.0': '3e3b9e4e0bffb4379fbffc1b44d27d89f8414460dedc449fc48db2804357cd52',
    'circle_0.6_phase_0.35': 'fc8f08364db064b1ce11cf17bc5cb21617ecdb447bb0c164e2ef32e9f723f118',
    'arc_0.6_phase_0.0': 'ce30fd99997042a8a2c2053c99aa292a933452c861d543c5f28f0cc18b1a1f29',
    'arc_0.6_phase_0.35': '70e62d0a2032a1878ce26bae141ea1fd16702693838c3c205288908740f10288',
    'circle_8.0_phase_0.0': '4ebc253092a1a0fdde2399fff90701934fb7a20924c9443b0b3dbac25bab0758',
    'circle_8.0_phase_0.35': '39aec9b16da7e87032a51e3742ced28b3c494d28d5d7ffc7163ef494c7a117f2',
    'arc_8.0_phase_0.0': '835bbb2b05f43a860ab8d9b42fcbaab9650ba3b757205f8c2dc8268788d2a3f1',
    'arc_8.0_phase_0.35': 'b4a6b9ac2ea03d039e199e5d9ea980cb176b5638cce0111f165086d2f32abe41',
}


def _polygon(vertices, angle=0.):
    theta = math.radians(angle)
    r = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    return dict(kind='polygon', vertices=np.asarray(vertices, float) @ r.T)


def _rectangle(length, width, angle=0., shift=0.):
    vertices = np.array([[-length/2., -width/2.], [length/2., -width/2.],
                         [length/2., width/2.], [-length/2., width/2.]])
    vertices[:, 1] += shift
    return _polygon(vertices, angle)


def _fixed_masks():
    # R20 fixed_cases/render pixel expressions; unused scoring primitives are
    # omitted. SHA pins below independently reject any pixel-factory drift.
    bases = [('longcap', 160, .02, [_rectangle(2.4, .32, 23.)])]
    for width in (.02, .06, .10):
        bases.append(('shortcap_'+str(int(round(width/.02))), 160, .02, [_rectangle(2.4, width)]))
    bases.append(('L_corner', 160, .02, [_polygon([
        [-.8, -.8], [.8, -.8], [.8, -.64], [-.64, -.64], [-.64, .8], [-.8, .8]])]))
    bases.append(('narrow_U', 160, .02, [_polygon([
        [-.12, -.8], [.12, -.8], [.12, .8], [.04, .8], [.04, -.72],
        [-.04, -.72], [-.04, .8], [-.12, .8]])]))
    bases.append(('parallel', 160, .02, [_rectangle(2., .08, shift=-.08), _rectangle(2., .08, shift=.08)]))
    for radius, width, size, scale in ((.6, .12, 128, .02), (8., .16, 256, .08)):
        for finite in (False, True):
            lo, hi = ((math.radians(-110), math.radians(110)) if radius == .6 else
                      (math.radians(-70), math.radians(70))) if finite else (-math.pi, math.pi)
            bases.append((('arc_' if finite else 'circle_')+str(radius), size, scale,
                [dict(kind='annulus', inner=radius-width/2., outer=radius+width/2.,
                      lo=lo, hi=hi, full=not finite)]))
    for name, size, scale, shapes in bases:
        for phase in (0., .35):
            yy, xx = np.indices((size, size))
            shift = np.array([phase*scale, -phase*scale])
            points = (np.asarray(np.stack((xx, yy), axis=-1), dtype=float)-size/2.)*np.array([scale, -scale])-shift
            inside = np.zeros((size, size), dtype=bool)
            for shape in shapes:
                if shape['kind'] == 'polygon':
                    polygon_inside = np.zeros(points.shape[:-1], dtype=bool)
                    x, y = points[..., 0], points[..., 1]
                    vertices = shape['vertices']
                    for a, b in zip(vertices, np.roll(vertices, -1, axis=0)):
                        if a[1] == b[1]:
                            continue
                        polygon_inside ^= ((a[1] > y) != (b[1] > y)) & (x < a[0]+(y-a[1])*(b[0]-a[0])/(b[1]-a[1]))
                    inside |= polygon_inside
                else:
                    radius = np.linalg.norm(points, axis=-1)
                    angle = np.arctan2(points[..., 1], points[..., 0])
                    angle_inside = np.ones_like(angle, dtype=bool) if shape['full'] else ((angle >= shape['lo']) & (angle <= shape['hi']))
                    inside |= (radius >= shape['inner']) & (radius <= shape['outer']) & angle_inside
            yield name+'_phase_'+str(phase), size, scale, inside.astype(np.uint8)*255


def _calibration(size, scale):
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = 1.
    return SimpleNamespace(width=size, height=size,
        camera_matrix=np.array([[1./scale, 0., size/2.], [0., 1./scale, size/2.], [0., 0., 1.]]),
        base_from_camera=transform, max_sensitivity_m_per_px=.5, plane_offset_m=0.)


class CornerBracketBatchTests(unittest.TestCase):
    def _check_batch(self, args, exact=True):
        before = _signature(args)
        expected = _scalar_rows(args)
        actual = subject._definitely_disjoint_batch(*args)
        self.assertIsInstance(actual, np.ndarray)
        self.assertEqual(actual.dtype, np.dtype(np.bool_))
        self.assertEqual(actual.shape, expected.shape)
        self.assertEqual(actual.tobytes(), expected.tobytes())
        self.assertEqual(_signature(args), before)
        if exact:
            for row in np.flatnonzero(actual):
                original = [value[row] for value in args]
                self.assertTrue(all(np.isfinite(value).all() for value in original))
                self.assertTrue(_exact_separation(*original))
        return actual

    def _check_paths(self, args):
        before = {key: _signature(value) for key, value in args.items() if key != 'cfg'}
        cfg = dict(args['cfg'])
        old_evidence, new_evidence = [], []
        expected = _scalar_direction_paths(**args, cut_evidence=old_evidence)
        actual = geometry._direction_paths(**args, cut_evidence=new_evidence)
        self.assertEqual(_signature(actual), _signature(expected))
        self.assertEqual(_signature(new_evidence), _signature(old_evidence))
        self.assertEqual(before, {key: _signature(value) for key, value in args.items() if key != 'cfg'})
        self.assertEqual(args['cfg'], cfg)
        original_ids = sorted(int(face) for path, _ in args['paths'] for face in path)
        self.assertEqual(sorted(int(face) for path, _ in actual for face in path), original_ids)
        return actual

    def test_empty_batch_has_empty_boolean_answer(self):
        self._check_batch(_batch_inputs(0, 6))

    def test_one_row_preserves_both_boolean_outcomes(self):
        args = _batch_inputs(1, 1)
        self.assertFalse(self._check_batch(args)[0])
        args[3][:] = 2.
        args[4][:] = 1.
        self.assertTrue(self._check_batch(args)[0])

    def test_128_mixed_rows_keep_original_order(self):
        result = self._check_batch(_batch_inputs(128, 6))
        self.assertEqual(result[:6].tolist(), [False, True, True, True, False, False])

    def test_k_1_6_512_retains_every_original_label(self):
        for k in (1, 6, 512):
            with self.subTest(k=k):
                args = _batch_inputs(3, k)
                args[3][:] = 0.
                args[4][:] = 0.
                args[3][1, 0, -1, 0] = 1.
                args[4][2, 1, -1, 1] = -1.
                self.assertEqual(self._check_batch(args).tolist(), [False, True, True])

    def test_strided_reversed_and_fortran_layouts(self):
        base = _batch_inputs(8, 6)
        for mode in ('stride', 'reverse', 'fortran'):
            with self.subTest(mode=mode):
                args = []
                for value in base:
                    if mode == 'stride':
                        packed = np.empty(value.shape[:-1]+(4,), dtype=np.float64)
                        packed[..., ::2] = value
                        packed[..., 1::2] = 19.
                        value = packed[..., ::2]
                    elif mode == 'reverse':
                        value = value[::-1]
                    else:
                        value = np.asfortranarray(value)
                    args.append(value)
                self._check_batch(args)

    def test_readonly_inputs_have_no_writes(self):
        args = _batch_inputs(9, 6)
        for value in args:
            value.setflags(write=False)
        self._check_batch(args)

    def test_signed_zero_inputs_preserve_all_bits(self):
        args = _batch_inputs(6, 6)
        for value in args:
            value[value == 0.] = -0.
        self._check_batch(args)

    def test_subnormal_and_closed_half_face_boundaries(self):
        tiny = float.fromhex('0x0.0000000000001p-1022')
        for scale in (2*tiny, float.fromhex('0x1p-1022'), 1., float.fromhex('0x1p500')):
            with self.subTest(scale=scale):
                args = _batch_inputs(3, 6)
                args[1] *= scale
                args[3][:] = 0.
                args[4][:] = 0.
                args[3][0, 0, :, 0] = .5*scale
                args[4][0, 0, :, 0] = .5*scale
                args[3][1, 0, :, 0] = np.nextafter(.5*scale, np.inf)
                args[4][1, 0, :, 0] = np.nextafter(.5*scale, np.inf)
                self._check_batch(args)

    def test_nonfinite_rows_match_scalar_and_exact_checker_rejects(self):
        for position in range(5):
            for bad in (np.nan, np.inf, -np.inf):
                with self.subTest(position=position, bad=bad):
                    args = _batch_inputs(3, 6)
                    args[position][0].flat[0] = bad
                    # The literal scalar checks derived bounds, not every
                    # input element. nextafter(+/-Inf) can yield a finite
                    # extreme and prove its necessary negative predicate.
                    # Do not apply a Fraction proof to nonfinite inputs or
                    # invent a stronger False contract than the old route.
                    actual = self._check_batch(args, exact=False)
                    self.assertIsNone(subject.certify_corner_brackets(
                        *(value[0] for value in args)))
                    if np.isnan(bad):
                        self.assertFalse(actual[0])
                    self.assertTrue(actual[1])
                    self.assertTrue(actual[2])
                    for row in (1, 2):
                        self.assertTrue(_exact_separation(*(value[row] for value in args)))

    def test_finite_overflow_and_cancellation_are_unknown(self):
        args = _batch_inputs(4, 6)
        huge = np.finfo(np.float64).max
        args[0][:2] = [[huge, huge], [huge, -huge]]
        args[2][:2] = [[1., 1.], [-1., 1.]]
        args[3][:2] = args[0][:2, None, None, :]
        args[4][:2] = args[0][:2, None, None, :]
        actual = self._check_batch(args)
        self.assertFalse(actual[0])
        self.assertFalse(actual[1])

    def test_wrong_shapes_and_dtype_refuse_without_mutation(self):
        base = _batch_inputs(3, 6)
        for index in range(5):
            for dtype in (np.float32, np.int64, np.bool_, object):
                with self.subTest(index=index, dtype=dtype):
                    args = [value.copy() for value in base]
                    args[index] = args[index].astype(dtype)
                    before = _signature(args)
                    self.assertIsNone(subject._definitely_disjoint_batch(*args))
                    self.assertEqual(_signature(args), before)
        for index, shape in ((0, (3, 3)), (1, (3, 2, 3)), (2, (2, 2, 2)),
                             (3, (3, 2, 0, 2)), (3, (3, 2, 513, 2)),
                             (4, (3, 2, 7, 2)), (4, (3, 6, 2))):
            with self.subTest(index=index, shape=shape):
                args = [value.copy() for value in base]
                args[index] = np.zeros(shape, dtype=np.float64)
                self.assertIsNone(subject._definitely_disjoint_batch(*args))

    def test_over_128_rows_and_non_native_float_are_explicit_fallbacks(self):
        self.assertIsNone(subject._definitely_disjoint_batch(*_batch_inputs(129, 6)))
        base = _batch_inputs(2, 6)
        opposite = '>f8' if np.little_endian else '<f8'
        for index in range(5):
            with self.subTest(index=index):
                args = [value.copy() for value in base]
                args[index] = args[index].astype(opposite)
                before = _signature(args)
                self.assertIsNone(subject._definitely_disjoint_batch(*args))
                self.assertEqual(_signature(args), before)

    def test_strict_numpy_error_settings_are_restored(self):
        for args in (_batch_inputs(6, 6), _batch_inputs(0, 6)):
            with np.errstate(all='raise'):
                self._check_batch(args)
                self.assertEqual(np.geterr(), dict(divide='raise', over='raise', under='raise', invalid='raise'))

    def test_false_is_not_a_feasibility_or_physical_corner_claim(self):
        args = _batch_inputs(1, 6)
        args[1][0] = [[1., 0.], [2., 0.]]
        self.assertFalse(self._check_batch(args)[0])
        self.assertIsNone(subject.certify_corner_brackets(*(value[0] for value in args)))
        with patch.object(subject, 'certify_corner_brackets', side_effect=AssertionError('batch called exact solver')):
            self._check_batch(args)

    def test_seeded_fraction_negative_oracle(self):
        for seed in range(12):
            rng = np.random.RandomState(28300+seed)
            args = _batch_inputs(9, [1, 6, 17][seed % 3])
            for value in args:
                value[:] = rng.uniform(-2., 2., value.shape)
            self._check_batch(args)

    def test_complete_labelled_paths_and_cut_evidence_match_scalar(self):
        for closed in (False, True):
            for shift in ((0, 1, 39, 77) if closed else (0,)):
                for angle in (0., .37, 1.13):
                    with self.subTest(closed=closed, shift=shift, angle=angle):
                        self._check_paths(_path_inputs(closed, shift, angle))

    def test_analytic_unlabelled_path_keeps_original_branch(self):
        for closed in (False, True):
            self._check_paths(_path_inputs(closed=closed, angle=.71, labelled=False))

    def test_invalid_faces_and_complete_batch_fallback_preserve_paths(self):
        args = _path_inputs(closed=True, shift=21, angle=.37)
        args['stable'][36:43] = False
        self._check_paths(args)
        with patch.object(subject, '_definitely_disjoint_batch', return_value=None):
            self._check_paths(_path_inputs(closed=True, shift=19, angle=.43))

    def test_survivors_call_original_exact_checker_in_original_order(self):
        args = _path_inputs(closed=True, shift=7, angle=.61)
        original = subject.certify_corner_brackets
        old_calls, new_calls = [], []

        def old_spy(*packet):
            old_calls.append((_signature(packet), _literal_negative(*packet)))
            return original(*packet)

        def new_spy(*packet):
            new_calls.append(_signature(packet))
            return original(*packet)

        with patch.object(subject, 'certify_corner_brackets', side_effect=old_spy):
            _scalar_direction_paths(**args, cut_evidence=[])
        with patch.object(subject, 'certify_corner_brackets', side_effect=new_spy):
            geometry._direction_paths(**args, cut_evidence=[])
        self.assertGreater(len(old_calls), 0)
        self.assertEqual(new_calls, [packet for packet, negative in old_calls if not negative])

    def test_float32_fallback_preserves_original_exact_packets_and_order(self):
        args = _path_inputs(closed=True, shift=7, angle=.61)
        for name in ('points', 'segments', 'white_xy', 'black_xy'):
            args[name] = args[name].astype(np.float32)
        original = subject.certify_corner_brackets
        old_calls, new_calls = [], []

        def old_spy(*packet):
            old_calls.append(_signature(packet))
            return original(*packet)

        def new_spy(*packet):
            new_calls.append(_signature(packet))
            return original(*packet)

        old_evidence, new_evidence = [], []
        with patch.object(subject, 'certify_corner_brackets', side_effect=old_spy):
            expected = _scalar_direction_paths(**args, cut_evidence=old_evidence)
        with patch.object(subject, 'certify_corner_brackets', side_effect=new_spy):
            actual = geometry._direction_paths(**args, cut_evidence=new_evidence)
        self.assertGreater(len(old_calls), 0)
        self.assertEqual(new_calls, old_calls)
        self.assertEqual(_signature(actual), _signature(expected))
        self.assertEqual(_signature(new_evidence), _signature(old_evidence))

    def test_original_22_mask_inputs_all_29_plus_5_outputs_are_byte_exact(self):
        derived = ('model_points', 'model_valid', 'model_sensitivity', 'model_reliable', 'model_normals')
        seen = []
        for name, size, scale, mask in _fixed_masks():
            with self.subTest(name=name):
                seen.append(name)
                self.assertEqual(hashlib.sha256(mask.tobytes()).hexdigest(), MASK_SHA256[name])
                mask_before = _signature(mask)
                with patch.object(geometry, '_direction_paths', _scalar_direction_paths):
                    old = geometry.extract_boundary_observation(mask, _calibration(size, scale), [0., 0., 1.], 1)
                new = geometry.extract_boundary_observation(mask, _calibration(size, scale), [0., 0., 1.], 1)
                self.assertEqual(len(fields(old)), 29)
                self.assertEqual([field.name for field in fields(old)], [field.name for field in fields(new)])
                for field in fields(old):
                    self.assertEqual(_signature(getattr(old, field.name)), _signature(getattr(new, field.name)), field.name)
                for prop in derived:
                    self.assertEqual(_signature(getattr(old, prop)), _signature(getattr(new, prop)), prop)
                self.assertEqual(_signature(mask), mask_before)
        self.assertEqual(seen, list(MASK_SHA256))
        self.assertEqual(len(seen), 22)


if __name__ == '__main__':
    unittest.main()
