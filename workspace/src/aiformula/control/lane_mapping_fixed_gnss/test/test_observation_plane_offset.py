"""R10A focused plan: explicit plane calibration, not a vehicle change.

An independent scalar world-ray oracle checks both offset signs and body tilt,
J4 inactive neighbours, empty/full masks and the original gate. Separate tests
cover keyword-only strict Real validation, exact static cache identity, immutable
K/T and zero-offset legacy byte equivalence. No trajectory-derived calibration,
truth-fed estimator, ROS default, safety gate or simulation parameter is changed.
"""
from fractions import Fraction
import math
from unittest.mock import patch

import numpy as np
import pytest

from lane_mapping_lya_reference.mapping_core import GroundLookup, ProjectionCounts
from lane_mapping_lya_reference.tilted_projection import TiltedGroundLookup


def calibration(origin=(0.21, -0.08, 0.655)):
    width, height = 15, 11
    camera = np.array([[9., .17, 7.2], [0., 8., 2.3], [0., 0., 1.]])
    transform = np.eye(4)
    transform[:3, :3] = [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]
    transform[:3, 3] = origin
    return width, height, camera, transform


def rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rx = np.array([[1., 0., 0.], [0., cr, -sr], [0., sr, cr]])
    ry = np.array([[cp, 0., sp], [0., 1., 0.], [-sp, 0., cp]])
    rz = np.array([[cy, -sy, 0.], [sy, cy, 0.], [0., 0., 1.]])
    return rz @ ry @ rx


def scalar_oracle(mask, threshold, camera, transform, world_from_body, offset, limit):
    """Rotate camera to a horizontal world; intersect z=offset, pixel by pixel.

    No production basis/intersection/cache helper is called. K solve is scalar
    upper-triangular substitution and all dot products are literal math.fsum.
    """
    height, width = mask.shape
    origin = [math.fsum(world_from_body[i, j] * transform[j, 3] for j in range(3))
              for i in range(3)]
    if origin[2] - offset <= 0.:
        raise ValueError("oracle camera is not above the plane")
    optical_to_world = [[math.fsum(world_from_body[i, k] * transform[k, j] for k in range(3))
                         for j in range(3)] for i in range(3)]
    heading = math.atan2(world_from_body[1, 0], world_from_body[0, 0])
    ch, sh = math.cos(heading), math.sin(heading)
    pixels = {}
    for row in range(height):
        for column in range(width):
            optical_y = (row - camera[1, 2]) / camera[1, 1]
            optical_x = (column - camera[0, 2] - camera[0, 1] * optical_y) / camera[0, 0]
            ray = [math.fsum(optical_to_world[i][j] * (optical_x, optical_y, 1.)[j]
                             for j in range(3)) for i in range(3)]
            if ray[2] >= -1.e-9:
                continue
            distance = (offset - origin[2]) / ray[2]
            if not math.isfinite(distance) or distance <= 0.:
                continue
            point = [origin[i] + distance * ray[i] for i in range(3)]
            xy = (ch * point[0] + sh * point[1], -sh * point[0] + ch * point[1])
            if all(math.isfinite(value) for value in xy):
                pixels[row, column] = xy
    points, sensitivity = [], []
    raw = metric = 0
    for row in range(height):
        for column in range(width):
            if mask[row, column] <= int(threshold):
                continue
            raw += 1
            if (row, column) not in pixels:
                continue
            metric += 1
            centre = pixels[row, column]
            differences = [0.]
            for neighbour in ((row, column - 1), (row, column + 1),
                              (row - 1, column), (row + 1, column)):
                if neighbour in pixels:
                    other = pixels[neighbour]
                    differences.append(math.hypot(centre[0] - other[0], centre[1] - other[1]))
            j4 = max(differences)
            if math.isfinite(j4) and j4 <= limit:
                points.append(centre)
                sensitivity.append(j4)
    return (np.asarray(points, dtype=np.float64).reshape(-1, 2),
            np.asarray(sensitivity, dtype=np.float64), ProjectionCounts(raw, metric, len(points)))


def assert_oracle(actual, expected):
    assert actual[2] == expected[2]
    np.testing.assert_allclose(actual[0], expected[0], rtol=3.e-13, atol=3.e-13)
    np.testing.assert_allclose(actual[1], expected[1], rtol=4.e-13, atol=4.e-13)


@pytest.mark.parametrize("offset", (.007, -.013))
@pytest.mark.parametrize("angles", ((0., 0., 0.), (.06, -.04, .9), (-.08, .03, -1.3)))
@pytest.mark.parametrize("layout", ("full", "strided"))
@pytest.mark.parametrize("limit", (.08, .5, 100.))
def test_signed_plane_scalar_world_oracle(offset, angles, layout, limit):
    width, height, camera, transform = calibration()
    body = rotation(*angles)
    if layout == "full":
        mask = np.full((height, width), 255, np.uint8)
    else:
        storage = np.zeros((2 * height, 2 * width), np.uint8)
        mask = storage[::2, ::2]
        mask[:] = np.random.RandomState(819).randint(0, 256, mask.shape)
    lookup = TiltedGroundLookup(width, height, camera, transform, limit, plane_offset_m=offset)
    assert_oracle(lookup.project_mask_with_sensitivity(mask, 127, body[2]),
                  scalar_oracle(mask, 127, camera, transform, body, offset, limit))


@pytest.mark.parametrize("offset", (.007, -.013))
@pytest.mark.parametrize("row,column", ((0, 0), (10, 14), (8, 0), (6, 7)))
def test_offset_sparse_j4_uses_inactive_neighbours_and_no_spatial_roi(offset, row, column):
    width, height, camera, transform = calibration()
    lookup = TiltedGroundLookup(width, height, camera, transform, 100., plane_offset_m=offset)
    body = rotation(.03, -.02, .6)
    mask = np.zeros((height, width), np.uint8)
    mask[row, column] = 255
    expected = {row * width + column}
    for r, c in ((row, column - 1), (row, column + 1), (row - 1, column), (row + 1, column)):
        if 0 <= r < height and 0 <= c < width:
            expected.add(r * width + c)
    with patch.object(lookup, "_intersect_indices", wraps=lookup._intersect_indices) as probe:
        actual = lookup.project_mask_with_sensitivity(mask, 127, body[2])
    np.testing.assert_array_equal(probe.call_args.args[0], sorted(expected))
    assert len(probe.call_args.args[0]) <= 5
    assert_oracle(actual, scalar_oracle(mask, 127, camera, transform, body, offset, 100.))
    if len(actual[1]):
        assert actual[1][0] > 0.


@pytest.mark.parametrize("normal", ([0., 0., 1.], rotation(.02, -.03, .8)[2]))
def test_nonzero_offset_empty_mask_and_shape_contract(normal):
    width, height, camera, transform = calibration()
    lookup = TiltedGroundLookup(width, height, camera, transform, .5, plane_offset_m=.007)
    with patch.object(lookup, "_intersect_indices", side_effect=AssertionError("empty geometry")):
        points, sensitivity, counts = lookup.project_mask_with_sensitivity(
            np.zeros((height, width), np.uint8), 127, normal)
    assert points.shape == (0, 2) and points.dtype == np.float64
    assert sensitivity.shape == (0,) and sensitivity.dtype == np.float64
    assert counts == ProjectionCounts(0, 0, 0)
    with pytest.raises(ValueError, match="shape"):
        lookup.project_mask(np.zeros((width, height), np.uint8), 127, normal)


@pytest.mark.parametrize("offset", (.007, -.013))
def test_level_nonzero_offset_uses_active_path_and_original_inclusive_j4_gate(offset):
    width, height, camera, transform = calibration()
    mask = np.zeros((height, width), np.uint8)
    mask[8, 7] = 255
    lookup = TiltedGroundLookup(width, height, camera, transform, 100., plane_offset_m=offset)
    with patch.object(lookup._level, "project_mask_with_sensitivity", side_effect=AssertionError("zero fallback")):
        actual = lookup.project_mask_with_sensitivity(mask, 127, [0., 0., 1.])
    limit = actual[1][0]
    at = TiltedGroundLookup(width, height, camera, transform, limit, plane_offset_m=offset)
    below = TiltedGroundLookup(width, height, camera, transform, np.nextafter(limit, 0.), plane_offset_m=offset)
    assert at.project_mask(mask, 127, [0., 0., 1.])[1].stable_pixels == 1
    assert below.project_mask(mask, 127, [0., 0., 1.])[1].stable_pixels == 0


@pytest.mark.parametrize("offset", (0., -0.))
@pytest.mark.parametrize("limit", (.05, .5, 100.))
def test_default_and_explicit_zero_are_legacy_byte_exact(offset, limit):
    width, height, camera, transform = calibration()
    mask = np.random.RandomState(14).randint(0, 256, (height, width)).astype(np.uint8)[::-1]
    old = GroundLookup(width, height, camera, transform, limit)
    expected = old.project_mask_with_sensitivity(mask, 127)
    for lookup in (TiltedGroundLookup(width, height, camera, transform, limit),
                   TiltedGroundLookup(width, height, camera, transform, limit, plane_offset_m=offset)):
        actual = lookup.project_mask_with_sensitivity(mask, 127, [0., 0., 1.])
        assert actual[2] == expected[2]
        assert all(a.tobytes() == b.tobytes() for a, b in zip(actual[:2], expected[:2]))


@pytest.mark.parametrize("offset", (0., -0.))
def test_explicit_zero_tilt_is_bit_exact_with_original_tilt_path(offset):
    width, height, camera, transform = calibration()
    mask = np.full((height, width), 255, np.uint8)
    normal = rotation(.07, -.08, 1.2)[2]
    a = TiltedGroundLookup(width, height, camera, transform, .5).project_mask_with_sensitivity(mask, 127, normal)
    b = TiltedGroundLookup(width, height, camera, transform, .5, plane_offset_m=offset).project_mask_with_sensitivity(mask, 127, normal)
    assert a[2] == b[2]
    assert all(x.tobytes() == y.tobytes() for x, y in zip(a[:2], b[:2]))


@pytest.mark.parametrize("height", (0., -.02))
def test_level_zero_keeps_legacy_empty_for_nonpositive_camera_height(height):
    width, rows, camera, transform = calibration(origin=(.21, -.08, height))
    lookup = TiltedGroundLookup(width, rows, camera, transform, .5)
    mask = np.full((rows, width), 255, np.uint8)
    assert lookup.project_mask(mask, 127, [0., 0., 1.])[1] == ProjectionCounts(rows * width, 0, 0)


@pytest.mark.parametrize("normal", ([0., 0., 1.], [.6, 0., .8]))
@pytest.mark.parametrize("above_delta", (0., .01))
@pytest.mark.parametrize("empty", (False, True))
def test_nonzero_plane_on_or_above_camera_rejects_before_mask_shortcut(normal, above_delta, empty):
    width, height, camera, transform = calibration()
    offset = float(np.asarray(normal) @ transform[:3, 3]) + above_delta
    lookup = TiltedGroundLookup(width, height, camera, transform, .5, plane_offset_m=offset)
    with pytest.raises(ValueError, match="Camera must be above"):
        lookup.project_mask(np.full((height, width), 0 if empty else 255, np.uint8), 127, normal)


def test_camera_below_body_origin_but_above_negative_plane_is_valid():
    width, height, camera, transform = calibration(origin=(.21, -.08, -.01))
    lookup = TiltedGroundLookup(width, height, camera, transform, .5, plane_offset_m=-.03)
    mask = np.full((height, width), 255, np.uint8)
    actual = lookup.project_mask_with_sensitivity(mask, 127, [0., 0., 1.])
    assert actual[2].stable_pixels > 0
    assert_oracle(actual, scalar_oracle(mask, 127, camera, transform, np.eye(3), -.03, .5))


@pytest.mark.parametrize("offset", (True, False, np.bool_(True), "0.007", None,
                                    1j, np.nan, np.inf, -np.inf, [0.], np.array(0.), 10 ** 1000))
def test_invalid_offset_rejects_before_lookup_allocation_and_never_matches(offset):
    width, height, camera, transform = calibration()
    valid = TiltedGroundLookup(width, height, camera, transform, .5)
    with patch("lane_mapping_lya_reference.tilted_projection.GroundLookup", side_effect=AssertionError("allocated")):
        with pytest.raises(ValueError, match="plane_offset_m"):
            TiltedGroundLookup(width, height, camera, transform, .5, plane_offset_m=offset)
    assert not valid.matches(width, height, camera, transform, .5, plane_offset_m=offset)


@pytest.mark.parametrize("offset", (0, -.007, np.float32(.007), np.float64(-.013), Fraction(7, 1000)))
def test_finite_signed_real_identity_property_and_keyword_only(offset):
    width, height, camera, transform = calibration()
    lookup = TiltedGroundLookup(width, height, camera, transform, .5, plane_offset_m=offset)
    assert lookup.plane_offset_m == float(offset)
    assert lookup.matches(width, height, camera, transform, .5, plane_offset_m=offset)
    assert not lookup.matches(width, height, camera, transform, .5,
                              plane_offset_m=np.nextafter(float(offset), np.inf))
    with pytest.raises(AttributeError):
        lookup.plane_offset_m = .02
    with pytest.raises(TypeError):
        TiltedGroundLookup(width, height, camera, transform, .5, offset)
    with pytest.raises(TypeError):
        lookup.matches(width, height, camera, transform, .5, offset)


def test_static_offset_does_not_rewrite_calibration_or_retain_dynamic_inputs():
    width, height, camera, transform = calibration()
    camera_before, transform_before = camera.copy(), transform.copy()
    lookup = TiltedGroundLookup(width, height, camera, transform, .5, plane_offset_m=.007)
    mask = np.full((height, width), 255, np.uint8)
    normal = rotation(.07, -.04, .5)[2]
    first = lookup.project_mask_with_sensitivity(mask, 127, normal)
    keys = tuple(sorted(lookup.__dict__))
    camera[:] = -1.
    transform[:] = -1.
    mask[:] = 0
    normal[:] = 0
    lookup.project_mask(mask, 127, [0., 0., 1.])
    last = lookup.project_mask_with_sensitivity(np.full((height, width), 255, np.uint8), 127,
                                                rotation(.07, -.04, .5)[2])
    assert all(a.tobytes() == b.tobytes() for a, b in zip(first[:2], last[:2]))
    assert first[2] == last[2]
    assert tuple(sorted(lookup.__dict__)) == keys
    assert lookup.camera_matrix.tobytes() == camera_before.tobytes()
    assert lookup.base_from_camera.tobytes() == transform_before.tobytes()
    assert lookup.matches(width, height, camera_before, transform_before, .5, plane_offset_m=.007)
    assert not lookup.matches(width, height, camera_before, transform_before, .5)
    for array in (lookup.camera_matrix, lookup.base_from_camera):
        with pytest.raises(ValueError):
            array.setflags(write=True)
