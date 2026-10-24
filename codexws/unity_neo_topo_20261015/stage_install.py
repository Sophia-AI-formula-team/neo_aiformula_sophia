"""Build a new hashable install tree from pinned Git blobs and this package."""
import argparse, hashlib, json, shutil
from pathlib import Path
from prepare_bundle import build

def stage(repo, output):
    if output.exists(): raise ValueError('Choose a fresh output directory')
    package=Path(__file__).resolve().parent
    worker=output/'Assets/StreamingAssets/SemanticPlanningNeo20261015'
    build(repo,worker)
    for name in ('semantic_worker.py','sim_checkpoint_adapter.py'):
        shutil.copyfile(package/'runtime'/name,worker/name)
    for source,rel in (
        ('runtime/AIFormulaSemanticPlanning.cs','Assets/AIFormula/Runtime/Control/AIFormulaSemanticPlanning.cs'),
        ('trials/AIFormulaNeoTopoTrial.cs','Assets/AIFormula/Editor/Validation/AIFormulaNeoTopoTrial.cs')):
        dest=output/rel
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(package/source,dest)
    files={p.relative_to(output).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
           for p in output.rglob('*') if p.is_file()}
    receipt=output.with_name(output.name+'_manifest.json')
    if receipt.exists(): raise ValueError('Manifest already exists')
    receipt.write_text(json.dumps(files,indent=2),encoding='utf-8')
    print(json.dumps({'stage':str(output),'files':len(files),'manifest':str(receipt)}))

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--repo',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    stage(a.repo.resolve(),a.output.resolve())
