"""Compare the Concept field transfer with the converter recovered from the car."""
import ast
import json
import math
import sys
from hashlib import sha256
from pathlib import Path
from typing import List

import numpy as np
from concept_drive.transport import ros_command
from concept_feasibility.response61515 import can_command, reachable_commands

src, observations = map(Path, sys.argv[1:])
motor = next(n for n in ast.parse(src.read_bytes()).body if getattr(n,'name','')=='MotorController')
motor.bases = []
motor.body = [f for f in motor.body if getattr(f,'name','') in ('toRefRPM','_inv_sigmoid','toCanCmd')]
env = {'np':np,'math':math,'List':List}
exec(compile(ast.fix_missing_locations(ast.Module(body=[motor],type_ignores=[])),str(src),'exec'),env)
convert = env['MotorController']()
convert.tread,convert.diameter,convert.gear_ratio = .6,.254,1.1
convert.Lp,convert.kp,convert.x0p,convert.Cp = 1.53908994,3.15496243,3.36664054,-.0621119
convert.Ln,convert.kn,convert.x0n,convert.Cn = 1.68283261,2.91627673,-3.22124235,1.6954783
decisions=json.loads(observations.read_text())

def compare(drive, steer):
    output=ros_command((drive,steer),physical=True)
    expected=tuple(round(v) for v in convert.toRefRPM(output['linear_x_mps'],output['angular_z_radps']))
    actual=tuple(int(v) for v in can_command(drive,steer,True))
    return expected, actual

ranges={}
for run,values in [
        ('recorded_1550',[(r['speed'],-r['ros_yaw']) for r in decisions for _ in range(r['count'])]),
        ('motion_grid',[(s,y) for s in [-1.,-.2,-.1,0.,.05,.1,.2,1.,3.2,4.] for y in np.r_[
           np.linspace(-1.55,1.55,101),0.,-1e-9,1e-9,np.nextafter(1e-9,np.inf),np.nextafter(-1e-9,-np.inf)]]),
        ('steering_intervals',[(s,y) for s in [1.,2.2,3.2,4.] for y in reachable_commands(s,True)])]:
    mismatches=[]
    for s,y in values:
        expected,actual=compare(s,y)
        if expected!=actual:
            mismatches.append(dict(drive=s,steer=y,field=expected,model=actual))
    ranges[run]=dict(pairs=len(values),mismatches=len(mismatches),examples=mismatches[:3])
print(json.dumps(dict(installed_motor_sha256=sha256(src.read_bytes()).hexdigest(),results=ranges,
    straight_reverse_frozen=tuple(map(int,can_command(-.2,0.))),
    straight_reverse_field=tuple(map(int,can_command(-.2,0.,True)))),indent=2))
if any(r['mismatches'] for r in ranges.values()):
    sys.exit(1)
