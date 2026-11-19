import numpy as np
import pytest

from concept_feasibility import geometry61514, response61515
from concept_feasibility.geometry import road_clearance
from concept_road.onboard625 import tread_clearance


@pytest.mark.parametrize('physical', [False, True])
def test_native_integer_steering_search_matches_all_representatives_and_endpoints(monkeypatch, physical):
    with response61515.motor_profile(physical):
        check_integer_steering_search(monkeypatch)


def check_integer_steering_search(monkeypatch):
    module = pytest.importorskip('concept_feasibility._motion')
    rng = np.random.default_rng(626)
    for pace in (.2,1.,2.4,3.4,5.99):
        speed = rng.choice(np.array([0.,.25,.55,.8,1.])*pace, 300)
        lower = rng.uniform(-1.55,1.54,len(speed))
        upper = rng.uniform(lower,1.55)
        desired = rng.uniform(-6.,6.,len(speed))
        current = rng.uniform(-2.,2.,len(speed))
        # Exercise degenerate intervals, zero yaw, and equal-response ties.
        lower[::6] = upper[::6]
        desired[::7] = 0.
        monkeypatch.setattr(response61515, '_motion', None)
        expected = response61515.inverse_yaw(desired,speed,lower,upper,current)
        monkeypatch.setattr(response61515, '_motion', module)
        np.testing.assert_array_equal(response61515.inverse_yaw(desired,speed,lower,upper,current),expected)


def test_field_motor_exact_straight_and_zero_drive_release():
    speed = np.array([-.2, .2, -.2, .2, 0., 0.])
    steer = np.array([0., 0., 1e-9, -1e-9, .55, -.17])
    right, left = response61515.can_command(speed, steer, physical=True)
    for wheel in (right, left):
        np.testing.assert_array_equal(wheel, [-49, 49, -49, 49, 0, 0])
    legacy = response61515.can_command(speed, steer)
    assert legacy[0][0] != right[0] and legacy[1][0] != left[0]


def test_field_native_rollouts_match_reference_at_the_steering_discontinuity(monkeypatch):
    module = pytest.importorskip('concept_feasibility._motion')
    speed = np.array([1., .8, .2, .2, .2, .2])
    steer = np.array([0., 1e-9, -1e-9, 1.1e-9, -1.1e-9, .17])
    stopped = np.zeros_like(speed)
    model = response61515.MODEL

    with response61515.motor_profile(True):
        monkeypatch.setattr(response61515, '_motion', None)
        expected = response61515.advance(stopped, stopped, speed, steer, .05, stopped)
        monkeypatch.setattr(response61515, '_motion', module)
        calculated = response61515.advance(stopped, stopped, speed, steer, .05, stopped)
        np.testing.assert_allclose(calculated, expected, rtol=0, atol=1e-12)
        controls = np.stack((speed,steer), axis=-1)[None]
        state = module.integrate(np.zeros((len(speed),8)), controls, .05,
                                 np.array(model['speed']), response61515.forward_yaw_model(),
                                 model['stop']['delay'], model['stop']['deceleration'],True)
        np.testing.assert_allclose(state[1,:,3:5].T, expected[:2], rtol=0, atol=1e-12)
        np.testing.assert_array_equal(
            response61515.inverse_yaw(stopped,speed,0.,0.), stopped)
    assert not response61515.physical_motor_enabled()


def test_compiled_tyre_contacts_include_exact_road_end_and_inner_paint():
    pytest.importorskip('concept_feasibility._tread')
    station = np.linspace(-1.,13.,71)
    road = np.c_[station, -1.+.045*station**2, 2.+.015*station, np.full(71,.03), np.full(71,.05)]
    z=np.linspace(-1.1,13.1,501)
    x=np.interp(z,station,road[:,1])+.3-.055
    yaw=np.linspace(-.1,.1,len(z))
    for got,ref in zip(road_clearance(road,z,x,yaw),tread_clearance(road,z,x,yaw)):
        np.testing.assert_allclose(got,ref,rtol=0,atol=1e-12)


def test_native_mixed_obstacle_search_includes_masked_times_and_singular_covariance(monkeypatch):
    pytest.importorskip('concept_feasibility._risk')
    rng = np.random.default_rng(626)
    n,ticks=16,14
    states=[]
    for i in range(ticks):
        z=rng.uniform(0,5,n)+i*.1
        x=rng.uniform(-3,3,n)
        h=rng.uniform(-.4, .4,n)
        states.append((z,x,h,i*.05,np.full(n,i*.1),np.full(n,i*.02)))
    active = rng.uniform(size=(ticks,n))>.3
    objects=np.zeros((3,77))
    for i,row in enumerate(objects):
        row[:5]=[1.6+i,.15-i*.3,-.32,.6,.14]
        row[25]=.1+i*.3
        row[26:30]=row[46:50]=row[:4]
        row[30:46]=np.diag([.0,.015,.03,.10]).ravel()
        row[50:66]=np.diag([.015,.18,.05,.29]).ravel()
        row[66:69]=[.3,.1,.04]
        row[69:73]=[.05,.02,.02,.2]
        row[75]=3.
    fast=geometry61514.path_obstacle_clearance(objects,states,active)
    monkeypatch.setattr(geometry61514,'compiled_path_clearance',None)
    expected=geometry61514.path_obstacle_clearance(objects,states,active)
    np.testing.assert_allclose(fast[0],expected[0],rtol=0,atol=1e-12)
    np.testing.assert_array_equal(fast[1],expected[1])
