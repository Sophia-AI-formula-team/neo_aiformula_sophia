"""ROS-independent tests for the shared, live localization contract."""

import ast
from pathlib import Path
from types import SimpleNamespace as Obj

import numpy as np
import pytest

from lane_mapping_lya_reference.mapping_core import VectorNavLocalizer


def common(stamp=1000000000, latitude=35.0, longitude=139.0, yaw=90.0,
           north=0.0, east=0.0):
    return Obj(header=Obj(stamp=Obj(sec=stamp // 1000000000,
                                    nanosec=stamp % 1000000000)),
               group_fields=0x10C8, position=Obj(x=latitude, y=longitude, z=20.0),
               velocity=Obj(x=north, y=east), yawpitchroll=Obj(x=yaw),
               insstatus=Obj(mode=2, gps_fix=True, time_error=False,
                             imu_error=False, gps_error=False))


def test_stationary_attitude_heading_is_available_without_course():
    localizer = VectorNavLocalizer()
    sample = localizer.accept(common(yaw=0), 1000000000)
    assert sample.speed_mps == 0
    assert sample.heading_rad == pytest.approx(np.pi / 2)
    assert localizer.origin_lla == [35.0, 139.0, 20.0]


def test_persisted_origin_gives_identical_coordinates_after_restart():
    first = VectorNavLocalizer()
    first.accept(common(), 1000000000)
    second = VectorNavLocalizer(origin_lla=first.origin_lla)
    next_msg = common(stamp=2000000000, longitude=139.000001, east=1.0)
    one = first.accept(next_msg, 2000000000)
    two = second.accept(next_msg, 2000000000)
    assert one.east_m > 0
    assert [one.east_m, one.north_m] == pytest.approx([two.east_m, two.north_m])


def test_mount_yaw_offset_is_explicit():
    sample = VectorNavLocalizer(yaw_offset_rad=0.2).accept(common(), 1000000000)
    assert sample.heading_rad == pytest.approx(0.2)


@pytest.mark.parametrize("field", [0x8, 0x40, 0x80, 0x1000])
def test_required_fields_cannot_be_silently_zero_filled(field):
    message = common()
    message.group_fields &= ~field
    with pytest.raises(ValueError, match="missing_vectornav"):
        VectorNavLocalizer().accept(message, 1000000000)


@pytest.mark.parametrize("attribute,value", [("mode", 1), ("mode", 0), ("mode", 3),
                                              ("gps_fix", False), ("time_error", True),
                                              ("imu_error", True), ("gps_error", True)])
def test_unhealthy_ins_cannot_drive_position(attribute, value):
    message = common()
    setattr(message.insstatus, attribute, value)
    localizer = VectorNavLocalizer()
    with pytest.raises(ValueError, match="unhealthy"):
        localizer.accept(message, 1000000000)
    assert localizer.origin_lla is None


@pytest.mark.parametrize("now", [0, 900000000, 1300000000])
def test_invalid_or_stale_clock_rejected(now):
    with pytest.raises(ValueError, match="clock|stale_or_future"):
        VectorNavLocalizer().accept(common(), now)


def test_time_regression_and_duplicate_both_rejected():
    localizer = VectorNavLocalizer()
    localizer.accept(common(), 1000000000)
    for stamp in (1000000000, 999000000):
        with pytest.raises(ValueError, match="nonincreasing"):
            localizer.accept(common(stamp=stamp), 1000000000)


def test_position_jump_does_not_mutate_last_good_sample():
    localizer = VectorNavLocalizer()
    previous = localizer.accept(common(), 1000000000)
    with pytest.raises(ValueError, match="position_jump"):
        localizer.accept(common(stamp=1100000000, longitude=139.001), 1100000000)
    assert localizer.last_sample is previous


def test_heading_discontinuity_is_rejected_without_mutating_pose():
    localizer = VectorNavLocalizer()
    previous = localizer.accept(common(), 1000000000)
    with pytest.raises(ValueError, match="heading_jump"):
        localizer.accept(common(stamp=1005000000, yaw=45.0), 1005000000)
    assert localizer.last_sample is previous


def test_heading_wrap_does_not_look_like_a_full_turn():
    localizer = VectorNavLocalizer()
    localizer.accept(common(yaw=-89.9), 1000000000)
    sample = localizer.accept(common(stamp=1010000000, yaw=269.9), 1010000000)
    assert abs(sample.heading_rad) > 3.0


@pytest.mark.parametrize("kwargs", [{"latitude": float("nan")}, {"latitude": 91},
                                    {"longitude": 181}, {"yaw": float("nan")},
                                    {"east": 9.0}])
def test_nonphysical_input_rejected(kwargs):
    with pytest.raises(ValueError):
        VectorNavLocalizer().accept(common(**kwargs), 1000000000)


def test_recorder_source_never_publishes_twist_or_launches_commands():
    source = Path(__file__).parents[1] / "lane_mapping_lya_reference" / "recorder_node.py"
    module = ast.parse(source.read_text(encoding="utf-8"), feature_version=(3, 8))
    for node in ast.walk(module):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            if node.func.attr == "create_publisher":
                assert not (isinstance(node.args[0], ast.Name) and node.args[0].id == "Twist")
            assert node.func.attr not in ("Popen", "system", "kill", "terminate")


def test_source_and_vendored_core_parse_as_python38():
    directory = Path(__file__).parents[1] / "lane_mapping_lya_reference"
    for filename in ("recorder_node.py", "mapping_core.py"):
        ast.parse((directory / filename).read_text(encoding="utf-8"), feature_version=(3, 8))
