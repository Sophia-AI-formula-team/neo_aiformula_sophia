"""Runtime source extracted from the accepted V6.26 freeze: experiments/v617_regression/reverse_model617.py. """

from functools import lru_cache
import json

from pathlib import Path

import numpy as np

from concept_feasibility.response61515 import features, MODEL, physical_motor_enabled

from concept_feasibility.reverse_response import step as original

DATA=json.loads(Path(__file__).with_suffix('.json').read_bytes())

@lru_cache(maxsize=32)
def _reverse_targets(drive, steer, dt, physical):
    # A powered interval holds one checked speed and steering request.
    # Its fitted targets and decay factors are independent of vehicle state.
    f = features(drive, steer, MODEL['yaw'][4], MODEL['yaw'][5], physical=physical)
    v, w = DATA['models']['v'], DATA['models']['w']
    return float(f @ np.asarray(v[1:])), float(f @ np.asarray(w[1:])), np.exp(-dt/v[0]), np.exp(-dt/w[0])


def step(speed,yaw,drive,steer,dt,zero_age):
    if drive>=0:return original(speed,yaw,drive,steer,dt,zero_age)
    if drive!=DATA['action_speed'] or not DATA['steer_min']-1e-8<=steer<=DATA['steer_max']+1e-8:
        raise ValueError('Reverse action outside validated simulator range')
    vtarget, wtarget, vdecay, wdecay = _reverse_targets(drive, steer, dt, physical_motor_enabled())
    return vtarget + (speed-vtarget)*vdecay, wtarget + (yaw-wtarget)*wdecay, 0.
