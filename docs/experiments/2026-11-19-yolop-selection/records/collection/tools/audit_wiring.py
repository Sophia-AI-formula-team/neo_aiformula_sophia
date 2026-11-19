import csv
import difflib
from pathlib import Path
import subprocess
import sys

vehicle, out, repo = (Path(v).absolute() for v in sys.argv[1:4])
out.mkdir(exist_ok=True)
home = Path('home/control')
commissioning = home / 'semantic_045_ws/commissioning'
native_launch = home / 'semantic_045_ws/source/workspace/src/aiformula/launchers/launch/semantic_vehicle_045.launch.py'
launcher_root = 'workspace/src/aiformula/launchers'
inputs = [
    (native_launch, f'{launcher_root}/launch/semantic_vehicle_045.launch.py'),
    (home/'semantic_045_ws/activate_semantic.bash', ''),
    (home/'concept_v6275_ws/activate_concept.bash', ''),
]
for path in sorted((vehicle/commissioning).glob('*.py')):
    location = f'{launcher_root}/launch/{path.name}' if path.name.endswith('.launch.py') else f'{launcher_root}/shellscript/{path.name}'
    inputs.append((path.relative_to(vehicle), location))
for filename in ('allnodes', 'lya', 'oa_2', 'oa-avoidance', 'semantic', 'mapassist-live', 'concept-live',
                 'yolop-launch'):
    inputs.append((home/'.local/bin'/filename, ''))
for path in sorted((vehicle/home/'.xdg-hidden/Desktop').glob('*.desktop')):
    if path.name in ('Allnodes.desktop','Concept.desktop','LYA.desktop','LYA MapAssist.desktop',
                     'MapAssist 正式接线.desktop','OA 避障.desktop','Semantic MapAssist.desktop','YOLOP.desktop'):
        inputs.append((path.relative_to(vehicle), ''))
a3 = home/'aiformula_runtime/yolop_a3_v14_20261012'
selector = home/'aiformula_runtime/2026-11-19-yolop-selection'
inputs.extend([(a3/'run_a3.py', ''), (selector/'yolop.launch','')])

for suffix, ref in [('main','origin/main'), ('experiment','origin/experiment/yolop-selection-20261119'),
                    ('mapassist-experiment','origin/experiment/map-assist-auto-warm-detour-20261119')]:
    with (out/f'{suffix}-runtime-wiring.tsv').open('w',newline='') as table, (out/f'{suffix}-runtime-wiring.patch').open('w') as patch:
        writer = csv.writer(table, delimiter='\t',lineterminator='\n')
        writer.writerow(['vehicle_path','neo_path','status'])
        for name, counterpart in inputs:
            path = vehicle/name
            if path.is_symlink():
                writer.writerow([name,counterpart,'vehicle symlink to '+str(path.readlink())])
                continue
            if not path.is_file():
                writer.writerow([name,counterpart,'missing from archive'])
                continue
            before = None
            if counterpart:
                check = subprocess.run(['git','-C',str(repo),'cat-file','-e',f'{ref}:{counterpart}'],
                                       stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                if check.returncode == 0:
                    before = subprocess.check_output(['git','-C',str(repo),'show',f'{ref}:{counterpart}']).decode()
            after = path.read_text()
            if before == after:
                writer.writerow([name,counterpart,'identical'])
                continue
            writer.writerow([name,counterpart if before is not None else '', 'changed' if before is not None else 'vehicle only'])
            lines = difflib.unified_diff((before or '').splitlines(keepends=True),after.splitlines(keepends=True),
                                         f'a/{counterpart}' if before is not None else '/dev/null',f'b/{name}')
            for line in lines:
                patch.write(line if line.endswith('\n') else line+'\n\\ No newline at end of file\n')
