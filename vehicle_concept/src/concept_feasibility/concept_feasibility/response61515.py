"""Runtime source extracted from the accepted V6.26 freeze: experiments/v615_recovery/response61515.py. """

import json

from contextlib import contextmanager
from contextvars import ContextVar
from functools import lru_cache

from pathlib import Path

import numpy as np

PATH=Path(__file__).with_suffix('.json')

MODEL=json.loads(PATH.read_bytes()) if PATH.exists() else None

try:
    from . import _motion
except ImportError:  # Unbuilt source checkout.
    _motion = None

_SPEED = np.asarray(MODEL['speed']) if MODEL else None
_YAW = np.asarray(MODEL['yaw']) if MODEL else None
# 16:35:33 manual forward segments, wheel feedback and VectorNav gyro.
# Fit 17 x 0.6 s windows; separate 25-window error 0.112 rad/s.
FIELD_YAW_TAU_S = .525
_FIELD_YAW = np.r_[FIELD_YAW_TAU_S, _YAW[1:]] if MODEL else None

def forward_yaw_model():
    return _FIELD_YAW if physical_motor_enabled() else _YAW


# Each driver callback selects its converter. Historical replay uses the
# Archived1030 transfer, while field rollouts use the actual car's transfer.
_physical_motor = ContextVar('physical_motor', default=False)
physical_motor_enabled = _physical_motor.get
_applied = ContextVar('physical_applied_command', default=(0., 0.))
physical_applied_command = _applied.get


@contextmanager
def motor_profile(physical, applied=(0., 0.)):
    token = _physical_motor.set(physical)
    applied_token = _applied.set(applied)
    try:
        yield
    finally:
        _applied.reset(applied_token)
        _physical_motor.reset(token)


def motor_steering(w, physical=False):
    """Stock converter's differential input for a ROS body-yaw request."""
    pos=w>=0
    limit=np.where(pos,1.53908994,1.68283261)
    gain=np.where(pos,3.15496243,2.91627673)
    offset=np.where(pos,3.36664054,-3.22124235)
    bias=np.where(pos,-.0621119,1.6954783)
    bounded=np.clip(w,-bias+1e-6,limit-bias-1e-6)
    log_term=np.log(limit/(bounded+bias)-1)
    if physical:
        # Concept releases zero both axes before the real motor conversion.
        # The installed motor bypasses its asymmetric steering fits at zero.
        ros=offset-(1./gain)*log_term
        ros=np.where(abs(w)<=1e-9, 0., ros)
    else:
        ros=offset-log_term/gain
    return ros


def can_command(speed_command,yaw_command,physical=False):
    """Right/left RPM targets under the selected motor conversion."""
    dtype=float if physical else np.float32
    v,w=np.broadcast_arrays(np.asarray(speed_command,dtype=dtype).astype(float),
                            -np.asarray(yaw_command,dtype=dtype).astype(float))
    ros=motor_steering(w,physical)
    if physical:
        ros=np.where(v==0.,0.,ros)
        v=np.where((abs(v)<=1e-9) & (abs(w)<=1e-9), 0., v)
    right,left=v/.127+.6/.254*ros,v/.127-.6/.254*ros
    right=np.sign(right)*(abs(right)/.834+2.76)
    left=np.sign(left)*(abs(left)/.844+2.81)
    if physical:
        return np.rint(right*(60/(2*np.pi))*1.1),np.rint(left*(60/(2*np.pi))*1.1)
    factor=60/(2*np.pi)*1.1
    return np.rint(right*factor),np.rint(left*factor)

def features(speed_command,yaw_command,knot,saturation,physical=None):
    if physical is None:
        physical=physical_motor_enabled()
    right,left=can_command(speed_command,yaw_command,physical)
    common=(right+left)*.5
    ratio=np.divide(right-left,common,out=np.zeros_like(common),where=abs(common)>1)
    offset=np.clip(ratio,-saturation,saturation)-knot
    return np.stack([common/100,common/100*np.minimum(offset,0),
                     common/100*np.maximum(offset,0)],axis=-1)

def target(speed_command,yaw_command,parameters,physical=None):
    # tau, three gains, hinge location, saturation. Units are stored in JSON.
    return features(speed_command,yaw_command,parameters[4],parameters[5],physical)@np.asarray(parameters[1:4])

@lru_cache(maxsize=32)
def reachable_commands(speed_command,physical=False):
    """All intervals of constant integer CAN, derived from encoding boundaries.

    This is the actuator's finite reachable set, not additional road paths or
    an arbitrary denser steering grid. ArchivedYaw has a real discontinuity at
    zero; the strongest turn need not be the largest absolute input command.
    """
    v=float(speed_command if physical else np.float32(speed_command));points=[-1.55,0.,1.55]
    if physical:
        points.extend([-1e-9,1e-9])
    probes=np.array([-1.55,-1e-7,0.,1e-7,1.55])
    extrema=can_command(v,probes,physical)
    for side,(gain,deadband,direction) in enumerate([(.834,2.76,1),(.844,2.81,-1)]):
        # CAN changes at n+.5, or when the wheel angular input crosses zero.
        extrema_side=extrema[side]
        cuts=np.arange(np.floor(extrema_side.min())-1,np.ceil(extrema_side.max())+1)+.5
        shaft=cuts/(60/(2*np.pi)*1.1)
        shaft=shaft[abs(shaft)>=deadband]
        wheel=np.r_[np.sign(shaft)*(abs(shaft)-deadband)*gain,0.]
        archived=(wheel-v/.127)/(direction*.6/.254)
        for limit,k,offset,bias,positive in [(1.53908994,3.15496243,3.36664054,-.0621119,True),
                                            (1.68283261,2.91627673,-3.22124235,1.6954783,False)]:
            ros=limit/(1+np.exp(np.clip(-k*(archived-offset),-700,700)))-bias
            u=-ros
            valid=(ros>=0 if positive else ros<0)&(u>=-1.55)&(u<=1.55)
            points.extend(u[valid].tolist())
    cuts=np.unique(points)
    candidates=np.r_[cuts,(cuts[:-1]+cuts[1:])*.5].astype(np.float32).astype(float)
    # Adjacent float32 values also cover endpoint rounding without assuming
    # that a mathematical half-integer remains exact after the Unity cast.
    f=candidates.astype(np.float32)
    candidates=np.unique(np.r_[candidates,np.nextafter(f,np.float32(-np.inf)),np.nextafter(f,np.float32(np.inf))])
    candidates=candidates[(candidates>=-1.55)&(candidates<=1.55)]
    # Keep representatives in every interval, including disjoint intervals
    # producing the same CAN. Slew constraints can exclude one but not another.
    order=np.lexsort((candidates,abs(candidates)))
    return candidates[order]

@lru_cache(maxsize=32)
def reachable_response(speed,parameters,physical=False):
    commands=reachable_commands(speed,physical)
    return commands,target(speed,commands,parameters,physical)

@lru_cache(maxsize=32)
def response_intervals(speed,parameters,physical=False):
    commands,values=reachable_response(speed,parameters,physical)
    order=np.argsort(commands)
    commands,values=commands[order],values[order]
    starts=np.r_[0,np.flatnonzero(values[1:]!=values[:-1])+1,len(values)].astype(np.int64)
    groups=np.repeat(np.arange(len(starts)-1,dtype=np.int64),np.diff(starts))
    return commands,values[starts[:-1]],order.astype(np.int64),groups,starts,int(np.searchsorted(commands,0.))


def inverse_yaw(desired,speed_command,lower=-1.55,upper=1.55,current_yaw=None,dt=.2):
    if MODEL is None:raise RuntimeError('Fit response61515.json before use')
    desired,q,lower,upper=np.broadcast_arrays(desired,speed_command,lower,upper)
    if np.any(lower>upper):raise ValueError('Empty steering command interval')
    if current_yaw is not None:
        decay=np.exp(-dt/MODEL['yaw'][0])
        desired=(desired-decay*np.asarray(current_yaw))/(1-decay)
    flat_d,flat_q=desired.ravel(),q.ravel();result=np.clip(np.zeros(flat_d.shape),lower.ravel(),upper.ravel())
    if _motion is not None:
        lo_flat, hi_flat = lower.ravel(), upper.ravel()
        endpoints = target(np.concatenate((flat_q, flat_q)), np.concatenate((lo_flat, hi_flat)), MODEL['yaw'])
        all_lo, all_hi = np.split(endpoints, 2)
    physical=physical_motor_enabled()
    for speed in np.unique(flat_q[flat_q>0]):
        active=flat_q==speed;lo,hi=lower.ravel()[active],upper.ravel()[active]
        model_speed=float(speed if physical else np.float32(speed))
        commands,reachable=reachable_response(model_speed,tuple(MODEL['yaw']),physical)
        if _motion is not None:
            intervals=response_intervals(model_speed,tuple(MODEL['yaw']),physical)
            result[active] = _motion.closest(*intervals, flat_d[active], lo, hi,
                                             all_lo[active], all_hi[active])
            continue
        proposals=np.broadcast_to(commands[:,None],(len(commands),len(lo)))
        proposals=np.vstack([proposals,lo,hi])
        predictions=np.vstack([np.broadcast_to(reachable[:,None],(len(commands),len(lo))),
                                target(speed,lo,MODEL['yaw']),target(speed,hi,MODEL['yaw'])])
        legal=(proposals>=lo)&(proposals<=hi)
        errors=np.where(legal,abs(predictions-flat_d[active]),np.inf)
        result[active]=np.take_along_axis(proposals,np.argmin(errors,axis=0)[None,:],axis=0)[0]
    return result.reshape(q.shape)

def advance(speed,yaw,speed_command,yaw_command,dt,zero_age):
    if MODEL is None:raise RuntimeError('Fit response61515.json before use')
    speed,yaw,q,u=np.broadcast_arrays(speed,yaw,speed_command,yaw_command)
    pv,pw=MODEL['speed'],forward_yaw_model();brake=MODEL['stop']
    if _motion is not None and speed.ndim == 1:
        age = np.broadcast_to(zero_age, q.shape)
        return _motion.advance(np.asarray(speed, float), np.asarray(yaw, float), np.asarray(q, float),
                               np.asarray(u, float), dt, np.asarray(age, float), _SPEED, pw,
                               brake['delay'], brake['deceleration'], physical_motor_enabled())
    velocity_target=np.maximum(0,target(q,u,pv))
    positive=velocity_target+(speed-velocity_target)*np.exp(-dt/pv[0])
    active=np.maximum(0,zero_age+dt-brake['delay'])-np.maximum(0,zero_age-brake['delay'])
    stopped=np.maximum(0,speed-brake['deceleration']*active)
    next_speed=np.where(q>0,positive,stopped)
    turn_target=u if physical_motor_enabled() else target(q,u,pw)
    driven=turn_target+(yaw-turn_target)*np.exp(-dt/pw[0])
    released=yaw*np.clip(next_speed/np.maximum(speed,1e-8),0,1)
    next_yaw=np.where(q>0,driven,np.where(speed>1e-8,released,0))
    return next_speed,next_yaw,np.where(q>0,0,zero_age+dt)
