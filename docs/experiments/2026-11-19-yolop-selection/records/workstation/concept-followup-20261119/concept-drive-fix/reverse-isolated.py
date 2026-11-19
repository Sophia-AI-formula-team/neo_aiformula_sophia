"""Install source candidate in the test interpreter's import table only."""
from pathlib import Path
import runpy, sys, importlib.util

root=Path(__file__).resolve().parent
def install(name, filename):
    spec=importlib.util.spec_from_file_location(name,str(root/filename))
    module=importlib.util.module_from_spec(spec)
    sys.modules[name]=module
    spec.loader.exec_module(module)
for name, fname in [
    ('concept_feasibility.response61515','response61515.py'),
    ('concept_feasibility.reverse_model617','reverse_model617.py'),
    ('concept_candidates.backtrack_forward617','backtrack_forward617.py'),
]:
    install(name, fname)
if sys.argv[1]=='pytest':
    import pytest
    sys.exit(pytest.main(['-q',str(root/'test_reverse_sweep.py')]))
runpy.run_path(str(root/'reverse-parity-bench.py'),run_name='__main__')
