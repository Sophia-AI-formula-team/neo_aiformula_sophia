"""Build a small source deployment bundle from the reviewed repository revision."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile

REPO = Path(__file__).resolve().parents[2]
BASE = '09f3110'
SOURCE = 'workspace/src/aiformula/'
SCOPES = ('control/motor_controller/', 'perception/semantic_planning_system/semantic_local_planner_ros/')


def git(*args):
    return subprocess.check_output(['git', '-C', str(REPO), *args])


def sha(content):
    return hashlib.sha256(content).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    name = 'motor_green_resume_0.4.1_semantic_0.2.2'
    stage = args.output / name
    stage.mkdir(exist_ok=False)
    changed = git('diff', '--name-only', BASE, '--', *(SOURCE + scope for scope in SCOPES)).decode().splitlines()
    manifest = dict(base_commit=git('rev-parse', BASE).decode().strip(),
                    source_commit=git('rev-parse', 'HEAD').decode().strip(),
                    versions=dict(motor_controller='0.4.1', semantic_local_planner_ros='0.2.2'),
                    files={}, guards={})
    for path in changed:
        relative = path[len(SOURCE):]
        old = git('show', BASE + ':' + path)
        new = (REPO / path).read_bytes()
        destination = stage / 'payload' / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(new)
        manifest['files'][relative] = dict(before=sha(old), after=sha(new))
    for relative in ('control/motor_controller/motor_controller/motor_controller.py',
                     'control/semantic_vehicle_wiring/semantic_vehicle_wiring/session.py',
                     'control/semantic_vehicle_wiring/package.xml', 'launchers/launch/semantic_vehicle.launch.py'):
        manifest['guards'][relative] = sha(git('show', BASE + ':' + SOURCE + relative))
    if not manifest['files']:
        raise RuntimeError('Empty deployment bundle')
    (stage / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    for filename in ('install.py', 'README.md', 'before_reproduction.json', 'test_receipt.json', 'dds_acceptance_pending.py'):
        shutil.copy2(Path(__file__).with_name(filename), stage / filename)
    archive = args.output / (name + '.tar.gz')
    with tarfile.open(archive, 'w:gz') as output:
        output.add(stage, arcname=name)
    checksum = sha(archive.read_bytes())
    archive.with_suffix(archive.suffix + '.sha256').write_text(checksum + '  ' + archive.name + '\n')
    print(json.dumps(dict(archive=str(archive.resolve()), sha256=checksum,
                         files=len(manifest['files']), source_commit=manifest['source_commit']), indent=2))


if __name__ == '__main__':
    main()
