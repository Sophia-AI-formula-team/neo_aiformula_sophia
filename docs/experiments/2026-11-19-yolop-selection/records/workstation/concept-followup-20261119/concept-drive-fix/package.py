"""Package the small field report and its linked evidence, including patch and settings tests."""
import re
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
base=Path(__file__).resolve().parent
workspace=base.parents[2]
root=base.parent
daily=workspace/'integration/yolop-selection-20261119.0N90OJ/repo/docs/experiments/2026-11-19-yolop-selection'
target=daily/'2026-11-19-concept-drive-fix.zip'
included=set()
pending=[base/f for f in (
    'README.md','field-fixes.patch','settings-events.jsonl','saved-vehicle-settings.json',
    'verified-desktop-sources.json','final-unit-dds.log','final-dds-retest.log','deployment.log',
    'baseline-summary.json','physical-handoff.log','installed-native-tests.log',
    'bench.py','run.bash','bench.launch.py','summarize.py')]
if (base/"final-bench.json").stat().st_size:
    pending.append(base/"final-bench.json")
while pending:
    source=pending.pop().resolve()
    if source in included:continue
    assert source.is_file() and root in source.parents and source.stat().st_size<1024*1024,source
    included.add(source)
    if source.suffix=='.md':
        for link in re.findall(r'!?\[[^\]]*?\]\(([^()]+)\)',source.read_text()):
            if '://' not in link and not link.startswith('#'):
                pending.append(source.parent/link.split('#')[0])
with ZipFile(target,'w',compression=ZIP_DEFLATED,compresslevel=9) as z:
    for p in sorted(included):
        z.write(p,p.relative_to(root))
with ZipFile(target) as z:
    assert z.testzip() is None
    assert not any(x.endswith(('.db3','.mcap','.mp4','.avi')) for x in z.namelist())
print(target,len(included),target.stat().st_size)
