#!/usr/bin/env python3
"""Compare installed neo wiring and motor codec to this checkout before commissioning."""
import hashlib
from importlib.util import find_spec
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory

ROOT = Path(__file__).resolve().parents[2] / 'workspace/src/aiformula'


def digest(path):
    return hashlib.sha256(path.read_bytes().replace(b'\r\n', b'\n')).hexdigest()


def main():
    files = []
    for package, prefix, paths in (
        ('launchers', 'launchers', ('launch/allnodes.launch.py', 'launch/vectornav.launch.py',
                                     'config/topic_list.yaml', 'config/joy.yaml', 'config/gamepad/dualshock4.yaml')),
        ('vehicle', 'vehicle', ('config/wheel.yaml', 'config/zedx/common.yaml', 'config/zedx/zedx.yaml')),
        ('motor_controller', 'control/motor_controller', ('config/motor_controller.yaml',)),
    ):
        share = Path(get_package_share_directory(package))
        files.extend((ROOT / prefix / p, share / p) for p in paths)
    script = 'l2_throttle_joy_remap.py'
    files.append((ROOT/'launchers/scripts'/script,
                  Path(get_package_prefix('launchers'))/'lib/launchers'/script))
    for name in ('gate', 'motor_controller'):
        spec = find_spec('motor_controller.' + name)
        files.append((ROOT/'control/motor_controller/motor_controller'/(name + '.py'),
                      Path(spec.origin) if spec and spec.origin else None))
    mismatches = 0
    for source, installed in files:
        match = installed and installed.is_file() and digest(source) == digest(installed)
        print(('OK       ' if match else 'DIFF/MISSING '), installed or source.name)
        mismatches += int(not match)
    return int(mismatches != 0)


if __name__ == '__main__':
    sys.exit(main())
