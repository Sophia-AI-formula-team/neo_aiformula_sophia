#!/usr/bin/env python3
"""Scoped offline bundle installer. Builds two packages; never starts ROS nodes."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import uuid

PACKAGES = ('motor_controller', 'semantic_local_planner_ros')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def live_processes():
    result = []
    executables = ('motor_controller', 'trajectory_follower', 'ros2_socketcan',
                   'semantic_local_planner_ros', 'semantic_vehicle_wiring', 'semantic_drive_bridge')
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            args = path.read_bytes().split(b'\0')
        except OSError:
            continue
        command = b' '.join(args[:4]).decode(errors='replace')
        if (any('/lib/' + name + '/' in command for name in executables)
                or 'ros2 launch launchers allnodes' in command
                or 'ros2 run semantic_vehicle_wiring semantic' in command):
            result.append(command)
    return result


def preflight(bundle, workspace):
    manifest = json.loads((bundle / 'manifest.json').read_text())
    source = workspace / 'src/aiformula'
    for name, record in manifest['files'].items():
        path = Path(name)
        if path.is_absolute() or '..' in path.parts:
            raise RuntimeError('Unsafe manifest path: ' + name)
        if digest(bundle / 'payload' / name) != record['after']:
            raise RuntimeError('Bundle checksum mismatch: ' + name)
        if not (source / name).is_file() or digest(source / name) != record['before']:
            raise RuntimeError('Vehicle source changed; inspect before deploying: ' + name)
    for name, expected in manifest['guards'].items():
        if not (source / name).is_file() or digest(source / name) != expected:
            raise RuntimeError('Vehicle prerequisite differs: ' + name)
    processes = live_processes()
    if processes:
        raise RuntimeError('Exit the driving/semantic stack before deployment: ' + repr(processes))
    return manifest


def build_environment(workspace):
    result = subprocess.run(['/bin/bash', '-e', '-c',
        '. /opt/ros/foxy/setup.bash\n. "$1/install/setup.bash"\n/usr/bin/env -0',
        'setup', str(workspace)], check=True, stdout=subprocess.PIPE)
    return dict(os.fsdecode(item).split('=', 1) for item in result.stdout.split(b'\0') if b'=' in item)


def build(workspace, environment, log):
    with log.open('w') as output:
        subprocess.run(['colcon', 'build', '--base-paths', 'src/aiformula', '--packages-select',
                        *PACKAGES, '--symlink-install'], cwd=str(workspace), env=environment,
                       stdout=output, stderr=subprocess.STDOUT, check=True)


def verify_installed(workspace, manifest):
    modules = {
        'motor_controller': ('motor_controller/gate.py', 'control/motor_controller/motor_controller/gate.py'),
        'semantic_local_planner_ros': ('semantic_local_planner_ros/map_assist.py',
            'perception/semantic_planning_system/semantic_local_planner_ros/semantic_local_planner_ros/map_assist.py'),
    }
    for package, (relative, source) in modules.items():
        matches = list((workspace / 'install' / package / 'lib').glob('python*/site-packages/' + relative))
        if len(matches) != 1 or digest(matches[0]) != manifest['files'][source]['after']:
            raise RuntimeError('Installed module mismatch: ' + package)


def apply(bundle, workspace, backup, manifest, environment):
    source = workspace / 'src/aiformula'
    # Make all backups before changing source or removing built packages.
    backup.mkdir(parents=True, exist_ok=False)
    for name in manifest['files']:
        target = backup / 'source' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source / name, target)
    for section in ('build', 'install'):
        for package in PACKAGES:
            current = workspace / section / package
            if not current.is_dir():
                raise RuntimeError('Missing installed baseline: ' + str(current))
            shutil.copytree(current, backup / section / package, symlinks=True)
    shutil.copy2(bundle / 'manifest.json', backup / 'manifest.json')
    # Recheck immediately before mutations; never kill user processes.
    preflight(bundle, workspace)
    try:
        for name in manifest['files']:
            shutil.copy2(bundle / 'payload' / name, source / name)
        for section in ('build', 'install'):
            for package in PACKAGES:
                shutil.rmtree(workspace / section / package)
        build(workspace, environment, backup / 'build.log')
        verify_installed(workspace, manifest)
        receipt = dict(status='installed', packages=manifest['versions'], backup=str(backup),
                       nodes_started=False, hardware_commands_sent=False)
    except BaseException:
        for name in manifest['files']:
            shutil.copy2(backup / 'source' / name, source / name)
        for section in ('build', 'install'):
            for package in PACKAGES:
                current = workspace / section / package
                if current.exists():
                    shutil.rmtree(current)
                shutil.copytree(backup / section / package, current, symlinks=True)
        (backup / 'receipt.json').write_text(json.dumps(dict(status='rolled_back_after_failure'), indent=2))
        raise
    (backup / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path('/home/workspace'))
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    bundle = Path(__file__).resolve().parent
    manifest = preflight(bundle, args.workspace)
    if args.check_only:
        print('Payload, baseline hashes and idle process state verified; nothing changed.')
        return
    environment = build_environment(args.workspace)
    name = datetime.utcnow().strftime('%Y%m%dT%H%M%SZ') + '_' + uuid.uuid4().hex[:8]
    backup = Path('/home/control/aiformula_backups') / ('motor_green_resume_' + name)
    print(json.dumps(apply(bundle, args.workspace, backup, manifest, environment), indent=2))


if __name__ == '__main__':
    main()
