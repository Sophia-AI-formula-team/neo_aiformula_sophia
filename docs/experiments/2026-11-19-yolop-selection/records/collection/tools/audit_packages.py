import csv
import difflib
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import subprocess
import sys
from collections import Counter, defaultdict
from xml.etree import ElementTree

stage, dest, repo = (Path(value).absolute() for value in sys.argv[1:4])
vehicle = stage / 'vehicle'
result = dest / 'neo-diff'
result.mkdir(exist_ok=True)
base = 'home/workspace/src/aiformula'
control = 'home/control'
runtime = control + '/aiformula_runtime'
concept_src = control + '/src/neo-concept-v6275'
semantic_src = control + '/semantic_045_ws/source/workspace/src/aiformula'
source_bases = {
    'base': [base, 'home/dependencies'],
    'semantic': [semantic_src],
    'concept': [concept_src + '/vehicle_concept', concept_src + '/workspace/src/aiformula/launchers'],
    'concept-field-20261119': [runtime + '/concept-field-20261119/deployed-src/vehicle_concept'],
    'concept-drive-fix-20261119-1603': [runtime + '/concept-drive-fix-20261119-1603/srcstage/vehicle_concept'],
    'concept-field-1635': [runtime + '/concept-field-1635/srcstage/vehicle_concept'],
}
excluded_dirs = {'.git', '__pycache__', '.pytest_cache', 'build', 'install', 'log', '.venv', 'venv'}
excluded_models = {'.pt', '.pth', '.onnx', '.engine', '.safetensors'}
text_suffixes = {'.py', '.sh', '.bash', '.c', '.cc', '.cpp', '.cu', '.h', '.hpp', '.json', '.yaml', '.yml',
                 '.xml', '.xacro', '.md', '.txt', '.cfg', '.ini', '.toml', '.msg', '.srv', '.action', '.rst',
                 '.desktop', '.launch', '.rviz', '.dsv', '.patch', '.cmake', '.urdf', '.cfg', '.lua', '.rules'}
config_dirs = ('launch', 'config', 'urdf', 'xacro', 'params', 'param', 'shellscript', 'scripts', 'rviz')
baselines = {
    'main': 'origin/main',
    'experiment': 'origin/experiment/yolop-selection-20261119',
    'concept': 'origin/feat/concept-v626-live',
    'mapassist-experiment': 'origin/experiment/map-assist-auto-warm-detour-20261119',
}

def git(*args, **kwargs):
    return subprocess.run(['git', '-C', str(repo), *args], check=True, **kwargs)

def resolve_vehicle(path):
    path = Path(path)
    parts = path.relative_to(vehicle).parts
    for _ in range(35):
        current = vehicle
        for index, component in enumerate(parts):
            current = current / component
            if current.is_symlink():
                target = current.readlink()
                target = vehicle / str(target).lstrip('/') if target.is_absolute() else current.parent / target
                target = Path(os.path.normpath(target.joinpath(*parts[index+1:])))
                if not target.is_relative_to(vehicle):
                    return None
                parts = target.relative_to(vehicle).parts
                break
        else:
            return current
    return None

def walk(path):
    for folder, dirs, files in os.walk(path):
        dirs[:] = sorted(d for d in dirs if d not in excluded_dirs and not (Path(folder)/d).is_symlink())
        yield from (Path(folder)/name for name in sorted(files))

def package_roots(roots):
    index = defaultdict(list)
    for name in roots:
        for p in walk(vehicle/name):
            if p.name != 'package.xml':
                continue
            try:
                package_name = ElementTree.parse(p).findtext('name')
            except (ElementTree.ParseError, OSError):
                continue
            if package_name:
                index[package_name].append(p.parent)
    return index

source_index = {key: package_roots(paths) for key, paths in source_bases.items()}
cmake = {}
cmake_file = dest / 'cmake-source-roots.tsv'
if cmake_file.is_file():
    for row in cmake_file.read_text().splitlines():
        prefix, name, root = row.split('\t')
        cmake[prefix, name] = vehicle / root.lstrip('/')

def choose_source(name, prefix):
    layer = 'base'
    if 'semantic_045_ws/install/' in prefix:
        layer = 'semantic'
    elif 'concept_v6275_ws/install/' in prefix:
        layer = 'concept'
    else:
        for key in source_bases:
            if prefix.startswith('/' + runtime + '/' + key + '/install/'):
                layer = key
                break
    src = cmake.get((prefix, name))
    if src and (src/'package.xml').is_file():
        return src, layer, 'CMake build directory'
    installed_xml = vehicle / prefix.lstrip('/') / 'share' / name / 'package.xml'
    resolved = resolve_vehicle(installed_xml)
    if resolved and resolved != installed_xml and any(resolved.parent == p for p in source_index[layer].get(name, [])):
        return resolved.parent, layer, 'installed manifest symlink'
    candidates = source_index[layer].get(name, [])
    canonical = [p for p in candidates if not any('bkup' in part or 'backup' in part for part in p.relative_to(vehicle).parts)]
    if len(canonical) == 1:
        return canonical[0], layer, 'package.xml in selected source layer'
    if len(candidates) == 1:
        return candidates[0], layer, 'package.xml in selected source layer'
    return None, layer, 'no unique source in selected layer'

modes = {}
for row in (dest/'ament-prefixes.tsv').read_text().splitlines():
    mode, paths = row.split('\t')
    prefix_map = {}
    for prefix in paths.split(':'):
        if not prefix.startswith('/home/'):
            continue
        markers = vehicle / prefix.lstrip('/') / 'share/ament_index/resource_index/packages'
        for marker in sorted(markers.glob('*')):
            target = resolve_vehicle(marker)
            if target and target.is_file():
                prefix_map.setdefault(marker.name, prefix)
    modes[mode] = prefix_map

resolution = dest / 'vehicle-package-resolution.tsv'
if resolution.is_file():
    modes = defaultdict(dict)
    with resolution.open() as stream:
        for row in csv.DictReader(stream, delimiter='\t'):
            modes[row['mode']][row['package']] = row['prefix']

variants = defaultdict(list)
for mode, packages in modes.items():
    for name, prefix in packages.items():
        variants[name,prefix].append(mode)

source_files = {}
runtime_overrides = []
inventory = []

def git_blob_id(path):
    digest = hashlib.sha1(usedforsecurity=False)
    digest.update(f'blob {path.stat().st_size}\0'.encode())
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            digest.update(block)
    return digest.hexdigest()

def source_tree(path):
    found = {}
    for p in walk(path):
        if p.suffix.lower() in excluded_models or p.suffix == '.pyc':
            continue
        resolved = resolve_vehicle(p)
        if resolved and resolved.is_file():
            rel = p.relative_to(path).as_posix()
            found[rel] = (p, resolved, git_blob_id(resolved))
    return found

for (name,prefix), active in sorted(variants.items()):
    source, layer, reason = choose_source(name,prefix)
    install = vehicle / prefix.lstrip('/')
    manifest = resolve_vehicle(install/'share'/name/'package.xml')
    version = ElementTree.parse(manifest).findtext('version') if manifest and manifest.is_file() else '?'
    key = name if sum(other == name for other, _ in variants) == 1 else f'{name}@{layer}'
    inventory.append((key, name, layer, prefix, ','.join(active), version, str(source.relative_to(vehicle)) if source else '', reason))
    if not source:
        continue
    original = source_tree(source)
    effective = dict(original)
    checks = [(install/'share'/name/dirname, dirname) for dirname in config_dirs]
    for python_dir in sorted(install.glob('lib/python*/site-packages')):
        checks.append((python_dir/name,name))
    for dirname, source_prefix in checks:
        copy = resolve_vehicle(dirname)
        if not copy or not copy.is_dir():
            continue
        for p in walk(copy):
            relative = f'{source_prefix}/{p.relative_to(copy).as_posix()}'
            resolved = resolve_vehicle(p)
            if not resolved or not resolved.is_file() or relative not in original or p.suffix not in text_suffixes:
                continue
            oid = git_blob_id(resolved)
            if oid != original[relative][2]:
                runtime_overrides.append((key, relative, str(p.relative_to(vehicle)), original[relative][2], oid))
                effective[relative] = (p, resolved, oid)
    source_files[key] = effective

def write_tsv(path, headers, rows):
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream, delimiter='\t', lineterminator='\n')
        writer.writerow(headers)
        writer.writerows(rows)

write_tsv(result/'active-packages.tsv', ('variant','package','layer','installed_prefix','modes','version','source','source_evidence'), inventory)
write_tsv(result/'installed-overrides.tsv', ('variant','file','installed_file','source_git_blob','installed_git_blob'), runtime_overrides)
write_tsv(result/'active-modes.tsv', ('mode','package_count'), [(key,len(value)) for key,value in modes.items()])

def match_repo_package(name, vehicle_source, roots):
    candidate_paths = list(roots.get(name, []))
    if not candidate_paths:
        return None
    for anchor in ('workspace/src/', 'vehicle_concept/src/'):
        if anchor in vehicle_source:
            relative = vehicle_source[vehicle_source.index(anchor):]
            if relative in candidate_paths:
                return relative
    canonical = [p for p in candidate_paths if not any('bkup' in part or 'backup' in part for part in p.split('/'))]
    return canonical[0] if len(canonical) == 1 else (candidate_paths[0] if len(candidate_paths) == 1 else None)

all_comparisons = {}
blobs_to_read = set()
for baseline in baselines:
    tree = {}
    candidate_roots = defaultdict(list)
    tree_file = stage / f'neo-{baseline}-tree.nul'
    for item in tree_file.read_bytes().split(b'\0'):
        if not item:
            continue
        meta, path = item.decode().split('\t',1)
        mode, kind, oid = meta.split()
        if kind != 'blob':
            continue
        tree[path] = (mode,oid)
        if path.endswith('/package.xml'):
            root = str(PurePosixPath(path).parent)
            package = PurePosixPath(root).name
            candidate_roots['zed_interfaces' if package == 'zed-ros2-interfaces' else package].append(root)
    comparisons = []
    files = []
    for key,name,layer,prefix,active,version,source,reason in inventory:
        if baseline == 'concept' and layer not in {'concept','concept-field-20261119','concept-drive-fix-20261119-1603','concept-field-1635'}:
            continue
        if baseline == 'mapassist-experiment' and (layer == 'concept' or name not in {
            'lane_points', 'launchers', 'obsticle_avoidence', 'semantic_local_planner_ros',
            'semantic_planner_core', 'semantic_vehicle_wiring', 'trajectory_follower'
        }):
            continue
        refpath = match_repo_package(name,source,candidate_roots)
        if key not in source_files:
            comparisons.append((key,'source unresolved',refpath or '',0,0,0))
            continue
        old = {}
        if refpath:
            old = {p[len(refpath)+1:]:value for p,value in tree.items()
                   if p.startswith(refpath+'/') and PurePosixPath(p).suffix.lower() not in excluded_models}
        new = source_files[key]
        counts = Counter()
        for rel in sorted(old.keys() | new.keys()):
            before = old.get(rel)
            after = new.get(rel)
            if before and after and before[1] == after[2]:
                continue
            status = 'M' if before and after else 'A' if after else 'D'
            counts[status] += 1
            files.append((key, refpath, rel, status, before, after))
            p = PurePosixPath(rel)
            if before and (p.suffix.lower() in text_suffixes or '.' not in p.name):
                blobs_to_read.add(before[1])
        comparisons.append((key,'changed' if counts and refpath else 'identical' if refpath else 'vehicle only',
                            refpath or '',counts['M'],counts['A'],counts['D']))
    write_tsv(result/f'{baseline}-packages.tsv', ('variant','status','neo_path','modified','added','deleted'), comparisons)
    write_tsv(result/f'{baseline}-neo-only-packages.tsv', ('source_directory_name','neo_path'),
              [(n,p) for n,roots in sorted(candidate_roots.items()) if n not in {k[0] for k in variants} for p in roots])
    all_comparisons[baseline] = (comparisons,files)

print(f'{len(variants)} installed variants in {len(modes)} modes, {len(runtime_overrides)} installed overrides, {len(blobs_to_read)} baseline text objects', flush=True)
blob_ids = sorted(blobs_to_read)
blob_input = ''.join(oid+'\n' for oid in blob_ids).encode()
env = os.environ | {'GIT_NO_LAZY_FETCH':'1'}
check = git('cat-file','--batch-check',input=blob_input,stdout=subprocess.PIPE,env=env)
missing = [row.split()[0] for row in check.stdout.splitlines() if row.endswith(b' missing')]
if missing:
    print(f'Fetching {len(missing)} missing textual blobs in one batch',flush=True)
    git('fetch','--no-tags','--no-write-fetch-head','origin','--stdin',input=b'\n'.join(missing)+b'\n')
data = git('cat-file','--batch',input=blob_input,stdout=subprocess.PIPE,env=env).stdout
objects = {}
reader = io.BytesIO(data)
for oid in blob_ids:
    fields = reader.readline().decode().split()
    if not fields or fields[0] != oid or fields[1] != 'blob':
        raise RuntimeError((oid,fields))
    objects[oid] = reader.read(int(fields[2]))
    if reader.read(1) != b'\n':
        raise RuntimeError(oid)

for baseline,(comparisons,files) in all_comparisons.items():
    directory = result / baseline
    directory.mkdir(exist_ok=True)
    details = []
    groups = defaultdict(list)
    for key,refpath,rel,status,before,after in files:
        old = objects.get(before[1]) if before else b''
        path = PurePosixPath(rel)
        new = (after[1].read_bytes() if after and after[1].stat().st_size < 2_000_000 and
               (path.suffix.lower() in text_suffixes or '.' not in path.name) else None if after else b'')
        textual = old is not None and new is not None and b'\0' not in old and b'\0' not in new
        if textual:
            try:
                old_lines = old.decode().splitlines(keepends=True)
                new_lines = new.decode().splitlines(keepends=True)
            except UnicodeDecodeError:
                textual = False
        if textual:
            old_path = f'a/{refpath}/{rel}' if before else '/dev/null'
            new_path = f'b/{refpath or key}/{rel}' if after else '/dev/null'
            groups[key].extend(difflib.unified_diff(old_lines,new_lines,old_path,new_path))
        details.append((key,status,rel,before[1] if before else '',after[2] if after else '', 'text' if textual else 'binary or no inline preview'))
    for key,lines in groups.items():
        (directory/f'{key}.patch').write_text(''.join(line if line.endswith('\n') else line+'\n\\ No newline at end of file\n' for line in lines))
    write_tsv(result/f'{baseline}-changed-files.tsv', ('variant','status','relative_path','neo_git_blob','vehicle_git_blob','format'), details)
    print(baseline,dict(Counter(row[1] for row in comparisons)),len(files),'file differences',flush=True)

(result/'baselines.json').write_text(json.dumps({
    label:{'ref':ref,'commit':git('rev-parse',ref,stdout=subprocess.PIPE,text=True).stdout.strip()}
    for label,ref in baselines.items()
},indent=2)+'\n')
