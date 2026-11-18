"""Runtime source extracted from the accepted V6.26 freeze: experiments/v616_decision/reverse_response.py. """

import json

from pathlib import Path

import numpy as np

from concept_feasibility.response61515 import advance, MODEL

MODEL_PATH=Path(__file__).with_suffix('.json')

def step(speed,yaw,drive,steer,dt,zero_age):
    if drive>0:return advance(speed,yaw,drive,steer,dt,zero_age)
    if drive==0:
        brake=MODEL['stop']
        active=max(0.,zero_age+dt-brake['delay'])-max(0.,zero_age-brake['delay'])
        velocity=np.sign(speed)*max(0.,abs(speed)-brake['deceleration']*active)
        gyro=yaw*min(1.,abs(velocity)/max(abs(speed),1e-8)) if abs(speed)>1e-8 else 0.
        return velocity,gyro,zero_age+dt
    if drive!=-.2 or steer!=0:raise ValueError('Uncalibrated reverse action')
    model=json.loads(MODEL_PATH.read_bytes())
    velocity=model['speed_target']+(speed-model['speed_target'])*np.exp(-dt/model['speed_tau'])
    gyro=model['yaw_target']+(yaw-model['yaw_target'])*np.exp(-dt/model['yaw_tau'])
    return velocity,gyro,0.
