"""Export the pinned neo source verbatim; keep the historical Unity source separate."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

PIN = '3c6fbac2ff9f8dd23b247a187e966648876400b1'
SEMANTIC = 'workspace/src/aiformula/perception/semantic_planning_system'
GATE = 'workspace/src/aiformula/control/motor_controller/motor_controller/gate.py'

def git(repo, *args):
    return subprocess.check_output(['git', '-C', str(repo), *args])

def build(repo, out):
    if out.exists():
        raise ValueError('Choose a fresh output directory; preserve previous bundles')
    assert git(repo, 'rev-parse', PIN).decode().strip() == PIN
    entries = git(repo, 'ls-tree', '-r', '-z', PIN, '--', SEMANTIC).decode().strip('\0').split('\0')
    hashes = {}
    for entry in entries:
        meta, path = entry.split('\t', 1)
        oid = meta.split()[2]
        rel = Path(path).relative_to(SEMANTIC).as_posix()
        # Source/config/docs only. Refuse unexpected weight or recording payloads.
        if Path(rel).suffix.lower() in ('.pt', '.pth', '.onnx', '.mp4', '.db3'):
            raise ValueError('Unexpected model or recording in source tree: ' + rel)
        data = git(repo, 'cat-file', 'blob', oid)
        target = out / 'source' / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        hashes['source/' + rel] = hashlib.sha256(data).hexdigest()
    data = git(repo, 'cat-file', 'blob', PIN + ':' + GATE)
    (out / 'motor_gate.py').write_bytes(data)
    hashes['motor_gate.py'] = hashlib.sha256(data).hexdigest()
    receipt = dict(source_repository='https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia',
                   source_commit=PIN, source_unmodified=True, package_version='0.1.0',
                   upstream_baseline='ed8e397ae41732877dfef334f0b89f98c0b20ae8',
                   map_assist_version='0.3.1', production_fix='Signed progress accepted after closed-route match; schema 1',
                   policy='stop_wait', bbox_distance_enabled=False, signal_approach_enabled=False,
                   transport='Unity stdio ROS-message adapter; not native ROS or DDS',
                   checkpoint='Explicit Windows/simulation adapter; original production schema and restore checks',
                   files=hashes)
    (out / 'provenance.json').write_text(json.dumps(receipt, indent=2), encoding='utf-8')
    print(json.dumps({'output':str(out), 'files':len(hashes), 'source_commit':PIN}))

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--repo', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a=p.parse_args()
    build(a.repo.resolve(), a.out.resolve())
