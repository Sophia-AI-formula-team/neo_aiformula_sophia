import hashlib, importlib.util, json
from pathlib import Path
from ament_index_python.packages import get_package_prefix, get_package_share_directory
root=Path.home()/'aiformula_runtime/concept-drive-fix-20261119-1603'
draft=root/'srcstage/vehicle_concept/src'
shas={}
for name, paths in {
    'concept': ['concept/bringup.py','launch/concept.launch.py'],
    'concept_candidates':['concept_candidates/trajectory_batch.py','concept_candidates/backtrack_forward617.py'],
    'concept_feasibility':['concept_feasibility/response61515.py','concept_feasibility/reverse_model617.py'],
    'concept_drive':['concept_drive/node.py','concept_drive/transport.py'],
    'concept_panel':['concept_panel/main.py','concept_panel/window.py'],
    'concept_control_gate':['concept_control_gate/node.py','concept_control_gate/speed_setting.py'],
    'concept_vehicle_state':['concept_vehicle_state/core.py'],
    'concept_yolop':['concept_yolop/inference.py','concept_yolop/node.py'],
}.items():
    assert Path(get_package_prefix(name)) == root/'install'/name, (name,get_package_prefix(name))
    for relative in paths:
        installed=(Path(get_package_share_directory(name))/relative
                   if relative.startswith('launch/') else
                   Path(importlib.util.find_spec(name).origin).parent/Path(relative).name)
        old=(draft/name/relative).read_bytes()
        assert old==installed.read_bytes(),relative
        shas[f'{name}/{relative}']=hashlib.sha256(old).hexdigest()
native=importlib.util.find_spec('concept_feasibility._motion').origin
assert str(root/'install') in native, native
motor=Path(importlib.util.find_spec('motor_controller.motor_controller').origin)
shas['installed_motor']=hashlib.sha256(motor.read_bytes()).hexdigest()
assert shas['installed_motor']=='7a6282c6a7e8f0215477da917f654fc39d15891c76a0b4209edeeb859519fd7d', shas['installed_motor']
print(json.dumps(dict(modules=shas,native=native,motor=str(motor)),indent=2))
