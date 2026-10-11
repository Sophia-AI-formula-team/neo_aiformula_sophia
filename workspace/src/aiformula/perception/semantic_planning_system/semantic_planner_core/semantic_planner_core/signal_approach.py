"""Image-observation validation and a conservative longitudinal stop envelope.

Distances are from the vehicle front to the signal's longitudinal plane at the
image capture time, not slant range or the bottom of an elevated lamp box.
This module does not estimate distance or decide whether a turn is permitted.
"""
from dataclasses import dataclass
import json
import math
from typing import Optional


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(name + " must be a finite number")
    return float(value)


@dataclass(frozen=True)
class SignalObservation:
    state: str
    sequence: int
    age_sec: float
    distance_m: Optional[float]
    uncertainty_m: float
    range_reason: str


def parse_signal_observation(payload: str) -> SignalObservation:
    """Reject invalid frame metadata; retain a real color even without range."""
    data = json.loads(payload)
    if not isinstance(data, dict):
        raise ValueError("Signal observation must be a JSON object")
    sequence = data.get("sequence")
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ValueError("sequence must be a nonnegative integer")
    state = data.get("state")
    if state not in ("red", "green", "unknown"):
        raise ValueError("state must be red, green or unknown")
    age = _number(data.get("age_sec"), "age_sec")
    if age < 0:
        raise ValueError("age_sec must be nonnegative")
    distance, uncertainty, reason = None, 0.0, "signal_range_missing"
    if data.get("distance_m") is not None:
        try:
            distance = _number(data["distance_m"], "distance_m")
            uncertainty = _number(data.get("uncertainty_m", 0.0), "uncertainty_m")
            if distance <= 0 or uncertainty < 0:
                raise ValueError("Invalid distance or uncertainty")
            reason = "ok"
        except ValueError:
            distance, uncertainty, reason = None, 0.0, "signal_range_invalid"
    return SignalObservation(state, sequence, age, distance, uncertainty, reason)


def stop_envelope(distance_m: float, uncertainty_m: float, age_sec: float,
                  speed_bound_mps: float, actual_speed_mps: float,
                  target_distance_m: float, max_speed_mps: float,
                  deceleration_mps2: float, latency_sec: float) -> dict:
    """Bound speed by v*latency + v**2/(2*a) <= remaining distance.

The caller supplies the maximum measured forward speed over the observation's
age, including the sample just before capture. Using only the latest speed
while braking would underestimate travel since capture.
"""
    distance_lower = distance_m - uncertainty_m - max(0.0, speed_bound_mps) * age_sec
    remaining = distance_lower - target_distance_m
    a = deceleration_mps2
    limit = min(max_speed_mps, max(0.0, math.sqrt((a * latency_sec) ** 2 +
                                                2.0 * a * max(0.0, remaining)) - a * latency_sec))
    speed = max(0.0, actual_speed_mps)
    braking_distance = speed * latency_sec + speed * speed / (2.0 * a)
    return {
        "distance_lower_m": distance_lower,
        "remaining_m": remaining,
        "speed_limit_mps": limit,
        "braking_distance_m": braking_distance,
        "braking_required": braking_distance > max(0.0, remaining) + 1e-9,
    }
