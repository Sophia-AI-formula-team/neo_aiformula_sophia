"""Independent 3-D geometry checks, not a vehicle/attitude/runtime validation.

The oracle moves the entire calibrated camera into a horizontal world, solves
each optical ray there, then rotates into the projected vehicle heading. It
does not call the implementation's body-plane/basis helpers or lookup tables.
No recorded trajectory, image, truth attitude or learned parameter is used.
"""

from concurrent.futures import ThreadPoolExecutor
import math
from unittest.mock import patch

import numpy as np
import pytest

from lane_mapping_lya_reference.mapping_core import GroundLookup, ProjectionCounts
from lane_mapping_lya_reference.tilted_projection import TiltedGroundLookup


def calibration(width=13, height=9, origin=(0.23, -0.12, 0.65)):
    camera = np.array([[8.0, 0.23, 6.2], [0.0, 7.0, 2.1], [0.0, 0.0, 1.0]])
    transform = np.eye(4)
    # ROS optical right/down/forward -> body forward/left/up.
    transform[:3, :3] = [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
    transform[:3, 3] = origin
    return width, height, camera, transform


def rotation(roll, pitch, yaw=0.0):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def world_oracle(mask, threshold, camera, transform, world_from_body, limit):
    height, width = mask.shape
    world_origin = world_from_body @ transform[:3, 3]
    world_from_optical = world_from_body @ transform[:3, :3]
    body_forward_world = world_from_body[:, 0]
    heading = math.atan2(body_forward_world[1], body_forward_world[0])
    world_to_heading = np.array([[math.cos(heading), math.sin(heading)],
                                 [-math.sin(heading), math.cos(heading)]])
    cloud = np.full((height, width, 2), np.nan)
    valid = np.zeros((height, width), dtype=bool)
    for row in range(height):
        for col in range(width):
            optical_ray = np.linalg.solve(camera, np.array([col, row, 1.0]))
            world_ray = world_from_optical @ optical_ray
            if world_ray[2] < -1.0e-9 and world_origin[2] > 0.0:
                scale = -world_origin[2] / world_ray[2]
                if math.isfinite(scale) and scale > 0:
                    world_point = world_origin + scale * world_ray
                    cloud[row, col] = world_to_heading @ world_point[:2]
                    valid[row, col] = np.all(np.isfinite(cloud[row, col]))
    output, sensitivity = [], []
    raw = metric = 0
    for row in range(height):
        for col in range(width):
            if not mask[row, col] > int(threshold):
                continue
            raw += 1
            if not valid[row, col]:
                continue
            metric += 1
            changes = [0.0]
            for other_row, other_col in ((row, col - 1), (row, col + 1),
                                         (row - 1, col), (row + 1, col)):
                if (0 <= other_row < height and 0 <= other_col < width
                        and valid[other_row, other_col]):
                    changes.append(math.dist(cloud[row, col], cloud[other_row, other_col]))
            j4 = max(changes)
            if math.isfinite(j4) and j4 <= limit:
                output.append(cloud[row, col])
                sensitivity.append(j4)
    return (np.asarray(output, dtype=np.float64).reshape(-1, 2),
            np.asarray(sensitivity, dtype=np.float64), ProjectionCounts(raw, metric, len(output)))


def assert_projection(actual, expected):
    assert actual[2] == expected[2]
    np.testing.assert_allclose(actual[0], expected[0], rtol=2e-13, atol=2e-13)
    np.testing.assert_allclose(actual[1], expected[1], rtol=3e-13, atol=3e-13)


@pytest.mark.parametrize("layout", ("c", "fortran", "strided", "reversed"))
@pytest.mark.parametrize("limit", (0.05, 0.5, 100.0))
def test_exact_level_delegates_bit_for_bit(layout, limit):
    width, height, camera, transform = calibration()
    old = GroundLookup(width, height, camera, transform, limit)
    new = TiltedGroundLookup(width, height, camera, transform, limit)
    rng = np.random.RandomState(3187)
    mask = rng.randint(0, 256, (height, width)).astype(np.uint8)
    if layout == "fortran":
        mask = np.asfortranarray(mask)
    elif layout == "strided":
        storage = np.zeros((height * 2, width * 2), dtype=np.uint8)
        storage[::2, ::2] = mask
        mask = storage[::2, ::2]
    elif layout == "reversed":
        mask = mask[::-1, ::-1]
    actual = new.project_mask_with_sensitivity(mask, 127, [0, 0, 1])
    expected = old.project_mask_with_sensitivity(mask, 127)
    assert actual[2] == expected[2]
    for lhs, rhs in zip(actual[:2], expected[:2]):
        assert lhs.dtype == rhs.dtype
        assert lhs.shape == rhs.shape
        assert lhs.tobytes() == rhs.tobytes()


@pytest.mark.parametrize("angles", ((0.0, 0.07, 0.0), (0.0, -0.07, 0.4),
                                   (0.11, 0.0, -0.9), (-0.11, 0.0, 0.0),
                                   (0.08, -0.05, 1.2), (-0.08, 0.05, -2.3)))
@pytest.mark.parametrize("limit", (0.05, 0.5, 100.0))
def test_independent_world_ray_plane_oracle_with_nonzero_camera_offsets(angles, limit):
    width, height, camera, transform = calibration()
    new = TiltedGroundLookup(width, height, camera, transform, limit)
    body_rotation = rotation(*angles)
    mask = np.random.RandomState(93).randint(0, 256, (height, width)).astype(np.uint8)
    actual = new.project_mask_with_sensitivity(mask, 127, body_rotation[2])
    expected = world_oracle(mask, 127, camera, transform, body_rotation, limit)
    assert_projection(actual, expected)


def test_skew_and_nontrivial_fixed_camera_installation_are_not_rewritten():
    width, height, camera, transform = calibration()
    transform[:3, :3] = rotation(0.04, -0.12, 0.03) @ transform[:3, :3]
    new = TiltedGroundLookup(width, height, camera, transform, 0.5)
    body_rotation = rotation(-0.035, 0.056, 1.7)
    mask = np.full((height, width), 255, dtype=np.uint8)
    assert_projection(new.project_mask_with_sensitivity(mask, 127, body_rotation[2]),
                      world_oracle(mask, 127, camera, transform, body_rotation, 0.5))
    assert new.camera_matrix.tobytes() == camera.tobytes()
    assert new.base_from_camera.tobytes() == transform.tobytes()


def test_roll_plane_mirror_has_equal_x_opposite_y_and_identical_counts():
    width, height, camera, transform = calibration(origin=(0.23, 0.0, 0.65))
    camera[0, 1] = 0.0
    camera[0, 2] = (width - 1) / 2.0
    new = TiltedGroundLookup(width, height, camera, transform, 100.0)
    left_normal = rotation(0.08, 0.04)[2]
    right_normal = rotation(-0.08, 0.04)[2]
    for row, col in ((0, 0), (2, 12), (3, 5), (5, 1), (8, 9)):
        mask = np.zeros((height, width), dtype=np.uint8)
        mask[row, col] = 255
        lhs = new.project_mask_with_sensitivity(mask, 127, left_normal)
        rhs = new.project_mask_with_sensitivity(mask[:, ::-1], 127, right_normal)
        assert lhs[2] == rhs[2]
        np.testing.assert_allclose(lhs[0], rhs[0] * [1.0, -1.0], atol=1e-13)
        np.testing.assert_allclose(lhs[1], rhs[1], atol=1e-13)


@pytest.mark.parametrize("row,col", ((0, 0), (0, 12), (8, 0), (8, 12),
                                     (4, 0), (4, 12), (4, 6)))
def test_sparse_pixels_use_exact_in_image_four_neighbours_including_inactive(row, col):
    width, height, camera, transform = calibration()
    new = TiltedGroundLookup(width, height, camera, transform, 100.0)
    body_rotation = rotation(0.03, -0.06)
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[row, col] = 255
    with patch.object(new, "_intersect_indices", wraps=new._intersect_indices) as probe:
        actual = new.project_mask_with_sensitivity(mask, 127, body_rotation[2])
    expected_indices = {row * width + col}
    for r, c in ((row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)):
        if 0 <= r < height and 0 <= c < width:
            expected_indices.add(r * width + c)
    assert probe.call_count == 1
    np.testing.assert_array_equal(probe.call_args.args[0], sorted(expected_indices))
    assert_projection(actual, world_oracle(mask, 127, camera, transform, body_rotation, 100.0))
    if actual[1].size:
        assert actual[1][0] > 0.0  # A single active pixel still has a nonzero J4.


def test_pixel_union_is_bounded_unique_and_never_reprojects_the_entire_frame_if_sparse():
    width, height, camera, transform = calibration(131, 91)
    new = TiltedGroundLookup(width, height, camera, transform, 0.5)
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[20:23, 50:53] = 255
    with patch.object(new, "_intersect_indices", wraps=new._intersect_indices) as probe:
        new.project_mask_with_sensitivity(mask, 127, rotation(0.03, 0.02)[2])
    indices = probe.call_args.args[0]
    assert len(indices) <= 5 * 9
    assert len(indices) < width * height
    assert len(indices) == len(np.unique(indices))
    assert np.all((indices >= 0) & (indices < width * height))


def test_inactive_neighbour_can_make_active_point_unstable():
    width, height, camera, transform = calibration()
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[4, 6] = 255
    body_rotation = rotation(0.01, 0.02)
    oracle = world_oracle(mask, 127, camera, transform, body_rotation, 100.0)
    assert oracle[2] == ProjectionCounts(1, 1, 1)
    assert oracle[1][0] > 0.5
    new = TiltedGroundLookup(width, height, camera, transform, 0.5)
    assert new.project_mask_with_sensitivity(mask, 127, body_rotation[2])[2] == ProjectionCounts(1, 1, 0)


def test_isolated_metric_pixel_keeps_original_zero_j4_convention():
    width = height = 2
    camera = np.array([[1.0, 0.0, 0.5], [0.0, 1.0, 0.8], [0.0, 0.0, 1.0]])
    _, _, _, transform = calibration(origin=(0.0, 0.0, 0.65))
    body_rotation = rotation(0.5, 0.0)
    new = TiltedGroundLookup(width, height, camera, transform, 0.5)
    mask = np.full((height, width), 255, dtype=np.uint8)
    actual = new.project_mask_with_sensitivity(mask, 127, body_rotation[2])
    assert_projection(actual, world_oracle(mask, 127, camera, transform, body_rotation, 0.5))
    assert actual[2] == ProjectionCounts(4, 1, 1)
    assert actual[1].tolist() == [0.0]


def test_dynamic_gate_is_inclusive_without_changing_threshold():
    width, height, camera, transform = calibration()
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[7, 6] = 255
    normal = rotation(0.02, 0.04)[2]
    generous = TiltedGroundLookup(width, height, camera, transform, 100.0)
    limit = generous.project_mask_with_sensitivity(mask, 127, normal)[1][0]
    at_limit = TiltedGroundLookup(width, height, camera, transform, limit)
    below = TiltedGroundLookup(width, height, camera, transform, np.nextafter(limit, 0.0))
    assert at_limit.project_mask_with_sensitivity(mask, 127, normal)[2].stable_pixels == 1
    assert below.project_mask_with_sensitivity(mask, 127, normal)[2].stable_pixels == 0


def test_near_level_is_not_flattened_or_sent_to_old_projection():
    width, height, camera, transform = calibration()
    new = TiltedGroundLookup(width, height, camera, transform, 100.0)
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[6, 5] = 255
    old = new.project_mask_with_sensitivity(mask, 127, [0.0, 0.0, 1.0])
    body_rotation = rotation(0.0, 1.0e-10)
    with patch.object(new._level, "project_mask_with_sensitivity", side_effect=AssertionError("flat fallback")):
        actual = new.project_mask_with_sensitivity(mask, 127, body_rotation[2])
    assert actual[0].tobytes() != old[0].tobytes()
    assert_projection(actual, world_oracle(mask, 127, camera, transform, body_rotation, 100.0))


def test_horizon_and_above_plane_rays_do_not_create_huge_accepted_cloud():
    width, height, camera, transform = calibration()
    camera[1, 2] = 3.0
    normal = np.array([-1.0e-10, 0.0, 1.0])
    new = TiltedGroundLookup(width, height, camera, transform, 0.5)
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[:4, :] = 255  # Row 3 is only 1e-10 below the tilted horizon.
    points, sensitivity, counts = new.project_mask_with_sensitivity(mask, 127, normal)
    assert counts == ProjectionCounts(4 * width, 0, 0)
    assert points.shape == (0, 2)
    assert sensitivity.shape == (0,)


def test_just_below_horizon_is_metric_but_rejected_by_original_j4_gate():
    width, height, camera, transform = calibration()
    camera[1, 2] = 3.0
    new = TiltedGroundLookup(width, height, camera, transform, 0.5)
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[3, :] = 255
    actual = new.project_mask_with_sensitivity(mask, 127, [-2.0e-9, 0.0, 1.0])
    assert actual[2] == ProjectionCounts(width, width, 0)
    assert not actual[0].size


@pytest.mark.parametrize("normal", ([0, 0, 0], [0, 0, 2], [0, 0, -1], [0, 1, 0],
                                    [1.0, 0.0, 1e-12], [0, 0, np.nan],
                                    [np.inf, 0, 1], [0, 0, 1e308], [0, 1],
                                    [[0, 0, 1]], [False, False, True],
                                    [0j, 0j, 1 + 0j], ["0", "0", "1"], None))
def test_invalid_normal_rejected_even_on_empty_mask_without_fallback(normal):
    width, height, camera, transform = calibration()
    new = TiltedGroundLookup(width, height, camera, transform, 0.5)
    with patch.object(new._level, "project_mask_with_sensitivity", side_effect=AssertionError("flat fallback")):
        with pytest.raises(ValueError):
            new.project_mask_with_sensitivity(np.zeros((height, width), np.uint8), 127, normal)


def test_camera_below_tilted_plane_rejects_instead_of_changing_height():
    width, height, camera, transform = calibration(origin=(1.0, 0.0, 0.65))
    new = TiltedGroundLookup(width, height, camera, transform, 0.5)
    with pytest.raises(ValueError, match="Camera must be above"):
        new.project_mask_with_sensitivity(np.full((height, width), 255), 127, [-0.8, 0.0, 0.6])
    assert new.base_from_camera[2, 3] == 0.65


def test_float32_unit_normal_roundoff_is_supported_without_mutating_input():
    width, height, camera, transform = calibration()
    new = TiltedGroundLookup(width, height, camera, transform, 0.5)
    normal = rotation(0.12, -0.09)[2].astype(np.float32)
    before = normal.tobytes()
    mask = np.full((height, width), 255, dtype=np.uint8)
    a = new.project_mask_with_sensitivity(mask, 127, normal)
    exact_normal = normal.astype(np.float64)
    exact_normal /= np.linalg.norm(exact_normal)
    b = new.project_mask_with_sensitivity(mask, 127, exact_normal)
    assert_projection(a, b)
    assert normal.tobytes() == before


@pytest.mark.parametrize("normal", ([0.0, 0.0, 1.0], rotation(0.03, -0.04)[2]))
def test_empty_and_shape_contract(normal):
    width, height, camera, transform = calibration()
    new = TiltedGroundLookup(width, height, camera, transform, 0.5)
    with patch.object(new, "_intersect_indices", side_effect=AssertionError("empty geometry")):
        points, sensitivity, counts = new.project_mask_with_sensitivity(
            np.zeros((height, width), dtype=np.uint8), 127, normal)
    assert counts == ProjectionCounts(0, 0, 0)
    assert points.shape == (0, 2) and points.dtype == np.float64
    assert sensitivity.shape == (0,) and sensitivity.dtype == np.float64
    with pytest.raises(ValueError):
        new.project_mask_with_sensitivity(np.zeros((width, height), np.uint8), 127, normal)
    with pytest.raises(ValueError):
        new.project_mask_with_sensitivity(np.zeros((height, width, 1), np.uint8), 127, normal)


def test_dynamic_threshold_and_c_order_on_strided_mask():
    width, height, camera, transform = calibration()
    new = TiltedGroundLookup(width, height, camera, transform, 0.5)
    storage = np.zeros((height * 2, width * 2), dtype=np.uint8)
    mask = storage[::2, ::2]
    mask[4:8, 2:10] = np.arange(32).reshape(4, 8) + 110
    body_rotation = rotation(0.01, -0.04)
    actual = new.project_mask_with_sensitivity(mask, 127, body_rotation[2])
    assert actual[2].raw_pixels == int(np.count_nonzero(mask > 127))
    assert_projection(actual, world_oracle(mask, 127, camera, transform, body_rotation, 0.5))


def test_only_static_immutable_calibration_is_retained_and_calls_are_independent():
    width, height, camera, transform = calibration()
    new = TiltedGroundLookup(width, height, camera, transform, 0.5)
    saved_camera, saved_transform = camera.copy(), transform.copy()
    mask = np.full((height, width), 255, dtype=np.uint8)
    first_normal, second_normal = rotation(0.07, 0.05)[2], rotation(-0.08, -0.09)[2]
    first = new.project_mask_with_sensitivity(mask, 127, first_normal)
    state_before = tuple(sorted(new.__dict__))
    camera[:] = -99
    transform[:] = -99
    first_normal[:] = 0
    mask[:] = 0
    new.project_mask_with_sensitivity(mask, 127, second_normal)
    mask[:] = 255
    last = new.project_mask_with_sensitivity(mask, 127, rotation(0.07, 0.05)[2])
    assert first[0].tobytes() == last[0].tobytes()
    assert first[1].tobytes() == last[1].tobytes()
    assert first[2] == last[2]
    assert tuple(sorted(new.__dict__)) == state_before
    assert new.matches(width, height, saved_camera, saved_transform, 0.5)
    assert not new.matches(width, height, camera, transform, 0.5)
    for array in (new.camera_matrix, new.base_from_camera, new._base_rays, new._origin):
        with pytest.raises(ValueError):
            array.setflags(write=True)


def test_concurrent_calls_cannot_share_or_overwrite_attitude_state():
    width, height, camera, transform = calibration()
    new = TiltedGroundLookup(width, height, camera, transform, 0.5)
    mask = np.full((height, width), 255, dtype=np.uint8)
    normals = [rotation(sign * 0.08, sign * 0.04)[2] for sign in (1, -1)] * 4
    sequential = [new.project_mask_with_sensitivity(mask, 127, n) for n in normals]
    with ThreadPoolExecutor(max_workers=2) as pool:
        concurrent = list(pool.map(lambda n: new.project_mask_with_sensitivity(mask, 127, n), normals))
    for a, b in zip(sequential, concurrent):
        assert a[0].tobytes() == b[0].tobytes()
        assert a[1].tobytes() == b[1].tobytes()
        assert a[2] == b[2]


def test_convenience_projection_requires_explicit_normal_and_keeps_counts():
    width, height, camera, transform = calibration()
    new = TiltedGroundLookup(width, height, camera, transform, 0.5)
    mask = np.full((height, width), 255, dtype=np.uint8)
    normal = rotation(0.01, -0.04)[2]
    expected = new.project_mask_with_sensitivity(mask, 127, normal)
    points, counts = new.project_mask(mask, 127, normal)
    assert points.tobytes() == expected[0].tobytes() and counts == expected[2]
    with pytest.raises(TypeError):
        new.project_mask(mask, 127)


@pytest.mark.parametrize("width,height", ((0, 9), (13, 1), (13.1, 9), (True, 9)))
def test_invalid_dimensions_rejected_before_allocating(width, height):
    _, _, camera, transform = calibration()
    with pytest.raises(ValueError):
        TiltedGroundLookup(width, height, camera, transform, 0.5)


@pytest.mark.parametrize("limit", (0.0, -0.5, np.inf, np.nan))
def test_invalid_sensitivity_configuration_is_not_a_silent_accept_all(limit):
    width, height, camera, transform = calibration()
    with pytest.raises(ValueError):
        TiltedGroundLookup(width, height, camera, transform, limit)
