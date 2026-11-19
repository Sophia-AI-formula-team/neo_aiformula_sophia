import cProfile,pstats
from pathlib import Path
import runpy
from concept_drive.core import Drive
f=Path.home()/'aiformula_runtime/concept-field-20261119/deployed-src/vehicle_concept/src/concept_drive/test/test_frozen_driver.py'
prof=cProfile.Profile()
fn=Drive.step
def step(*args):
    return prof.runcall(fn,*args)
Drive.step=step
test=runpy.run_path(str(f))['test_five_archived_frames_through_real_perception_wire']
for _ in range(3): test()
pstats.Stats(prof).strip_dirs().sort_stats('cumtime').print_stats(50)
