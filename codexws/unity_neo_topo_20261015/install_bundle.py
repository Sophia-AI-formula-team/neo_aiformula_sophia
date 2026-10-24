"""Hash-guarded reversible Unity wiring install. Call only in owned clean Edit mode."""
import argparse, hashlib, json, shutil
from pathlib import Path

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def install(stage, simulator, backup, baseline):
    if backup.exists():
        raise ValueError('Backup already exists; preserve it and choose a fresh receipt directory')
    files = sorted(p for p in stage.rglob('*') if p.is_file())
    allowed = ('Assets/StreamingAssets/SemanticPlanningNeo20261015/',)
    exact = {'Assets/AIFormula/Runtime/Control/AIFormulaSemanticPlanning.cs',
             'Assets/AIFormula/Editor/Validation/AIFormulaNeoTopoTrial.cs'}
    expected = {row['path']:row['sha256'] for row in json.loads(baseline.read_text(encoding='utf-8-sig'))}
    plan=[]
    for file in files:
        rel=file.relative_to(stage).as_posix()
        if rel not in exact and not any(rel.startswith(root) for root in allowed):
            raise ValueError('Out of scope: ' + rel)
        dest=simulator / rel
        if dest.exists() and (rel not in expected or sha(dest) != expected[rel]):
            raise ValueError('Unknown/changed destination; preserve it: ' + rel)
        plan.append(dict(path=rel, before=sha(dest) if dest.exists() else None, after=sha(file)))
    if not plan:
        raise ValueError('Empty install plan')
    # Also detect unrelated source changes before installation.
    drift=[rel for rel,value in expected.items() if not (simulator/rel).exists() or sha(simulator/rel)!=value]
    if drift:
        raise ValueError('Baseline drift before install: ' + repr(drift))
    backup.mkdir(parents=True)
    (backup/'plan.json').write_text(json.dumps(plan, indent=2), encoding='utf-8')
    for row in plan:
        target=simulator/row['path']
        if row['before'] is not None:
            saved=backup/'files'/row['path']
            saved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(target,saved)
    for row in plan:
        target=simulator/row['path']
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(stage/row['path'], target)
        assert sha(target)==row['after']
    (backup/'receipt.json').write_text(json.dumps(dict(installed=True, files=plan), indent=2), encoding='utf-8')
    print(json.dumps(dict(installed=True, files=len(plan), backup=str(backup))))

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('stage','simulator','backup','baseline'):
        p.add_argument('--'+name, type=Path, required=True)
    a=p.parse_args()
    install(*(getattr(a,name).resolve() for name in ('stage','simulator','backup','baseline')))
