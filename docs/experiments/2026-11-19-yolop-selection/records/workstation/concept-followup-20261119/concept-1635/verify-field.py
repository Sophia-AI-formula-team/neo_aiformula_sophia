import hashlib,importlib.util,json
from pathlib import Path
from ament_index_python.packages import get_package_prefix
root=Path.home()/'aiformula_runtime/concept-field-1635'
manifest=json.loads((root/'source-manifest.json').read_text())
def hashpath(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for relative,h in manifest.items():
 pkg,sub,filename=relative.split('/')
 assert Path(get_package_prefix(pkg))==root/'install'/pkg
 assert hashpath(root/'srcstage/vehicle_concept/src'/relative)==h,relative
 if filename.endswith('.py'):
    path=Path(importlib.util.find_spec(pkg).origin).with_name(filename)
    assert hashpath(path)==h,(path,h)
native=Path(importlib.util.find_spec('concept_feasibility._motion').origin)
assert root/'install' in native.parents
motor=Path(importlib.util.find_spec('motor_controller.motor_controller').origin)
assert hashpath(motor)=='7a6282c6a7e8f0215477da917f654fc39d15891c76a0b4209edeeb859519fd7d'
from concept_vehicle_state.core import decode_honda_rpm
import struct
assert decode_honda_rpm(0x711,struct.pack('<ii',276,363))==(363,276)
print(json.dumps(dict(verified=manifest,native=str(native),motor_sha256=hashpath(motor),persisted={p.stem:json.loads(p.read_text()) for p in (Path.home()/'.config/concept/speed').glob('*.json')}),indent=2))
