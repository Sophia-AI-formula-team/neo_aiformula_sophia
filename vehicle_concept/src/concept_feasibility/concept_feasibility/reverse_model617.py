"""Runtime source extracted from the accepted V6.26 freeze: experiments/v617_regression/reverse_model617.py. """

import json

from pathlib import Path

import numpy as np

from concept_feasibility.response61515 import features, MODEL

from concept_feasibility.reverse_response import step as original

DATA=json.loads(Path(__file__).with_suffix('.json').read_bytes())

def step(speed,yaw,drive,steer,dt,zero_age):
    if drive>=0:return original(speed,yaw,drive,steer,dt,zero_age)
    if drive!=DATA['action_speed'] or not DATA['steer_min']-1e-8<=steer<=DATA['steer_max']+1e-8:
        raise ValueError('Reverse action outside validated simulator range')
    f=features(drive,steer,MODEL['yaw'][4],MODEL['yaw'][5]);v=DATA['models']['v'];w=DATA['models']['w']
    vtarget=float(f@np.asarray(v[1:]));wtarget=float(f@np.asarray(w[1:]))
    return vtarget+(speed-vtarget)*np.exp(-dt/v[0]),wtarget+(yaw-wtarget)*np.exp(-dt/w[0]),0.
