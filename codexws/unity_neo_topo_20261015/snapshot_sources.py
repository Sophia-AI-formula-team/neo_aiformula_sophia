"""Read-only scoped simulator source hash snapshot, not runtime parameter evidence."""
import argparse,hashlib,json
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('--simulator',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
if a.output.exists(): raise ValueError('Preserve previous snapshot')
root=a.simulator.resolve()
entries=[]
for tree in ('Assets/AIFormula','Assets/StreamingAssets','Assets/Scenes','ProjectSettings'):
    for file in sorted((root/tree).rglob('*')):
        if file.is_file() and file.suffix in ('.cs','.py','.json','.yaml','.osm','.unity','.asset'):
            entries.append({'path':file.relative_to(root).as_posix(),'sha256':hashlib.sha256(file.read_bytes()).hexdigest()})
a.output.write_text(json.dumps(entries,indent=2),encoding='utf-8')
print(json.dumps({'files':len(entries),'output':str(a.output)}))
