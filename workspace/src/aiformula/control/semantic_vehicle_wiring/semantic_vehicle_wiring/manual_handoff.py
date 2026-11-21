"""Read stamped handoff state from the vehicle's motor and OA publishers."""

import json


MOTOR_STATUS_TOPIC = "/aiformula_control/motor_controller/gate_status"
OA_STATUS_TOPIC = "/oa/status"
MAX_AGE_NS = 600_000_000


def read_state(text, now_ns, state_key):
    try:
        payload = json.loads(text)
        state, transition, stamp = (
            payload[state_key], payload["manual_transition_ns"], payload["stamp_ns"]
        )
    except (KeyError, TypeError, ValueError):
        return None
    if (type(state) is not bool or type(transition) is not int or type(stamp) is not int
            or not 0 <= transition <= stamp or not 0 <= now_ns - stamp <= MAX_AGE_NS):
        return None
    return state, transition
