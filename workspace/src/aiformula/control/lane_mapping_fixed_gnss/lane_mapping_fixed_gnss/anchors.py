"""Optional endpoint GNSS for repeat-start proximity, NEVER for building a map.

The local map, route, heading and motion origin are independent of this class.
START records a geographic reference; END only checks whether the stopped car
is near that reference before fixed-route repeat. Missing/bad GNSS disables
that repeat permission, not map construction or the validity of saved geometry.
No endpoint translates, rotates, scales or closes the map. The wrapper freezes
its map before open_end and subscribes ONLY while an endpoint window is open.
"""
from dataclasses import dataclass
import math
from numbers import Integral, Real
from types import MappingProxyType

import numpy as np


DEFAULT_CONFIG = {"window_s": 2.0, "max_fix_age_s": 0.5,
                  "max_horizontal_sigma_m": 1.5, "max_endpoint_distance_m": 3.0}


def _time(value, label):
    if isinstance(value, bool) or not isinstance(value, Integral) or not 0 < value < 2 ** 63:
        raise ValueError("invalid_" + label)
    return int(value)


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError("invalid_" + label)
    return float(value)


@dataclass(frozen=True)
class Fix:
    stamp_ns: int
    received_steady_ns: int
    lat: float
    lon: float
    alt: float
    status: int
    covariance_type: int
    covariance: tuple

    def __post_init__(self):
        object.__setattr__(self, "covariance", tuple(self.covariance))


def decode_vectornav_gps(message, receipt_ns):
    """Decode ONLY GpsGroup GPS position/uncertainty, not CommonGroup INS.

    The checked driver default BO1.gpsField=0x210 lacks POSLLA. Runtime must
    explicitly configure 0x230 (FIX|POSLLA|POSU), or this decoder rejects it.
    VectorNav posu is N/E/D standard deviation; Fix covariance is ENU variance.
    Fix==3 means 3D fix here, not RTK. No satellite or RTK quality is invented.
    """
    try:
        fields, status = message.group_fields, message.fix
        if (isinstance(fields, bool) or not isinstance(fields, Integral)
                or not 0 <= fields <= 65535 or fields & 0x0230 != 0x0230):
            raise ValueError("gps_required_fields_missing")
        if isinstance(status, bool) or not isinstance(status, Integral) or status != 3:
            raise ValueError("gps_requires_3d_fix")
        sec, nanosec = message.header.stamp.sec, message.header.stamp.nanosec
        if (isinstance(sec, bool) or not isinstance(sec, Integral) or sec < 0
                or isinstance(nanosec, bool) or not isinstance(nanosec, Integral)
                or not 0 <= nanosec < 1000000000):
            raise ValueError("invalid_gps_stamp")
        stamp = _time(int(sec) * 1000000000 + int(nanosec), "gps_stamp")
        receipt = _time(receipt_ns, "gps_receipt")
        latitude = _finite(message.poslla.x, "latitude")
        longitude = _finite(message.poslla.y, "longitude")
        altitude = _finite(message.poslla.z, "altitude")
        if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
            raise ValueError("invalid_geographic_coordinate")
        north, east, down = [_finite(value, "gps_uncertainty") for value in
                              (message.posu.x, message.posu.y, message.posu.z)]
        if not all(0 < value <= 100000 for value in (north, east, down)):
            raise ValueError("gps_uncertainty_must_be_positive")
        covariance = (east ** 2, 0.0, 0.0, 0.0, north ** 2, 0.0, 0.0, 0.0, down ** 2)
        return Fix(stamp, receipt, latitude, longitude, altitude, 0, 2, covariance)
    except AttributeError:
        raise ValueError("invalid_vectornav_gps_message")


def _validated_fix(fix, maximum_sigma):
    if not isinstance(fix, Fix):
        raise ValueError("fix_type_required")
    _time(fix.stamp_ns, "fix_stamp")
    _time(fix.received_steady_ns, "fix_receipt")
    latitude = _finite(fix.lat, "latitude")
    longitude = _finite(fix.lon, "longitude")
    _finite(fix.alt, "altitude")
    if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
        raise ValueError("invalid_geographic_coordinate")
    if (isinstance(fix.status, bool) or not isinstance(fix.status, Integral)
            or fix.status not in (0, 1, 2)):
        raise ValueError("no_valid_gnss_fix")
    if (isinstance(fix.covariance_type, bool) or not isinstance(fix.covariance_type, Integral)
            or fix.covariance_type not in (1, 2, 3)):
        raise ValueError("unknown_gnss_covariance")
    if len(fix.covariance) != 9:
        raise ValueError("invalid_covariance_shape")
    covariance = np.asarray([_finite(v, "covariance") for v in fix.covariance]).reshape(3, 3)
    if (not np.allclose(covariance, covariance.T, atol=1e-9, rtol=0)
            or np.min(np.linalg.eigvalsh(covariance)) < -1e-9
            or np.any(np.diag(covariance) <= 0)):
        raise ValueError("invalid_or_zero_gnss_covariance")
    sigma = math.sqrt(max(0.0, float(np.linalg.eigvalsh(covariance[:2, :2])[-1])))
    if sigma > maximum_sigma:
        raise ValueError("gnss_horizontal_uncertainty_limit")
    return sigma


def horizontal_distance_m(start, end):
    """Short-distance endpoint check on a sphere; no geometry alignment."""
    lat1, lat2 = math.radians(start.lat), math.radians(end.lat)
    dlat, dlon = lat2 - lat1, math.radians(end.lon - start.lon)
    haversine = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 6371008.8 * 2 * math.asin(math.sqrt(min(1.0, max(0.0, haversine))))


class EndpointAnchors:
    """One-shot stationary windows, fail-closed for repeat permission only.

    open_start(now_ns, steady_ns, stopped) -> start_window.
    observe_fix(fix, now_ns, steady_ns, stopped=True) -> teach or ready.
    mark_teach_started() closes START forever, with or without a usable fix.
    open_end(now_ns, steady_ns, stopped) -> end_window (caller first freezes map).
    check_window(...) must run from the wrapper watchdog even without GNSS.
    Any stale/invalid fix, movement, clock regression or window expiry latches
    a GNSS fault. An explicit mark_teach_started may retire a failed START
    attempt and begin GNSS-independent teaching, but it cannot retry GNSS or
    recover an END attempt. Quality-valid END fixes always finish the check;
    distance failure is metadata, not a map failure or an exception. Only
    snapshot()["ready_for_repeat"] grants the endpoint prerequisite. Neither
    that flag nor phase="ready" replaces fresh mask localization / safety
    interlocks. Outside-window fixes are rejected before reading them.
    """

    def __init__(self, config=None):
        if config is not None and not isinstance(config, dict):
            raise ValueError("configuration_must_be_dict")
        cfg = dict(DEFAULT_CONFIG)
        if config:
            unknown = set(config) - set(cfg)
            if unknown:
                raise ValueError("unknown_configuration: " + ",".join(sorted(unknown)))
            cfg.update(config)
        for key in cfg:
            cfg[key] = _finite(cfg[key], key)
            if cfg[key] <= 0:
                raise ValueError("invalid_" + key)
        if (cfg["window_s"] > 10 or cfg["max_fix_age_s"] > 1
                or cfg["max_horizontal_sigma_m"] > 10 or cfg["max_endpoint_distance_m"] > 20):
            raise ValueError("unsafe_configuration_range")
        self.config = MappingProxyType(cfg)
        self.phase = "unopened"
        self.reason = "waiting_for_explicit_stopped_start"
        self._opened_ns = self._opened_steady_ns = None
        self._last_now_ns = self._last_steady_ns = None
        self._start = self._end = None
        self._start_metadata = self._end_metadata = None
        self._teach_started = False
        self._end_window_used = False
        self._start_unavailable_reason = None
        self.outside_window_rejected = 0

    def _fault(self, reason):
        self.phase, self.reason = "fault", str(reason)
        raise ValueError(str(reason))

    def _clocks(self, now_ns, steady_ns):
        try:
            now, steady = _time(now_ns, "now"), _time(steady_ns, "steady_now")
        except ValueError as error:
            self._fault(str(error))
        if (self._last_now_ns is not None and now < self._last_now_ns
                or self._last_steady_ns is not None and steady < self._last_steady_ns):
            self._fault("anchor_clock_regressed")
        self._last_now_ns, self._last_steady_ns = now, steady
        return now, steady

    def _open(self, phase, now_ns, steady_ns, stopped):
        if stopped is not True:
            self._fault("anchor_window_requires_stopped")
        now, steady = self._clocks(now_ns, steady_ns)
        self._opened_ns, self._opened_steady_ns = now, steady
        self.phase, self.reason = phase, "waiting_for_one_fresh_gnss_fix"

    def open_start(self, now_ns, steady_ns, stopped):
        if self.phase != "unopened":
            raise ValueError("start_window_already_used_or_faulted")
        self._open("start_window", now_ns, steady_ns, stopped)

    def mark_teach_started(self):
        """Explicitly start local mapping without requiring geographic data.

        This is a one-way boundary: even an outstanding or failed START window
        is retired, so a queued fix cannot become a mid-lap GNSS input. It does
        not reset motion, accept a substitute anchor, or authorize repeat.
        Repeated calls during teaching are harmless; END cannot be rewound.
        """
        if self._end_window_used or self.phase not in (
                "unopened", "start_window", "teach", "teach_without_start", "fault"):
            raise ValueError("teach_boundary_cannot_rewind_endpoint_session")
        if self._teach_started:
            return
        if self._start is None:
            self._start_unavailable_reason = (self.reason if self.phase == "fault" else
                "start_window_closed_without_fix" if self.phase == "start_window" else
                "start_window_not_requested")
            self.phase, self.reason = "teach_without_start", "local_mapping_without_gnss_reference"
        else:
            self.phase, self.reason = "teach", "local_mapping_gnss_not_consumed"
        self._opened_ns = self._opened_steady_ns = None
        self._teach_started = True

    def open_end(self, now_ns, steady_ns, stopped):
        if self.phase not in ("teach", "teach_without_start") or self._end_window_used:
            raise ValueError("end_window_requires_teach")
        self._end_window_used = True
        self._open("end_window", now_ns, steady_ns, stopped)

    def cancel_window(self, reason):
        """Revoke an open endpoint window without throwing in a safety callback.

        The wrapper calls this before destroying its temporary subscription on
        stop/fault/estop. Retained or already queued fixes cannot reopen it.
        Outside a window this is a no-op, preserving accepted endpoint evidence
        and the first latched failure reason. Return whether a window closed.
        """
        if self.phase not in ("start_window", "end_window"):
            return False
        self.phase, self.reason = "fault", str(reason)
        return True

    def check_window(self, now_ns, steady_ns, stopped):
        if self.phase not in ("start_window", "end_window"):
            return
        now, steady = self._clocks(now_ns, steady_ns)
        if stopped is not True:
            self._fault("movement_during_anchor_window")
        window_ns = int(self.config["window_s"] * 1e9)
        if now - self._opened_ns > window_ns or steady - self._opened_steady_ns > window_ns:
            self._fault("anchor_window_expired")

    def observe_fix(self, fix, now_ns, steady_ns, stopped=False):
        if self.phase not in ("start_window", "end_window"):
            self.outside_window_rejected += 1
            raise ValueError("gnss_outside_endpoint_window")
        self.check_window(now_ns, steady_ns, stopped)
        try:
            sigma = _validated_fix(fix, self.config["max_horizontal_sigma_m"])
            if fix.stamp_ns <= self._opened_ns or fix.received_steady_ns < self._opened_steady_ns:
                raise ValueError("fix_predates_endpoint_window")
            max_age_ns = int(self.config["max_fix_age_s"] * 1e9)
            if not 0 <= now_ns - fix.stamp_ns <= max_age_ns:
                raise ValueError("fix_source_stale_or_future")
            if not 0 <= steady_ns - fix.received_steady_ns <= max_age_ns:
                raise ValueError("fix_receipt_stale_or_future")
            if self._start is not None and fix.stamp_ns <= self._start.stamp_ns:
                raise ValueError("fix_stamp_not_increasing")
        except (ValueError, TypeError, OverflowError) as error:
            self._fault(str(error))
        kind = "start" if self.phase == "start_window" else "end"
        metadata = {"kind": kind, "accepted": True, "stamp_ns": int(fix.stamp_ns),
                    "received_steady_ns": int(fix.received_steady_ns),
                    "lat": float(fix.lat), "lon": float(fix.lon), "alt": float(fix.alt),
                    "status": int(fix.status), "covariance_type": int(fix.covariance_type),
                    "covariance": [float(v) for v in fix.covariance], "horizontal_sigma_m": sigma,
                    "applied_to_geometry": False,
                    "purpose": "repeat_start_reference" if kind == "start" else "repeat_start_proximity"}
        if kind == "start":
            self._start, self._start_metadata = fix, metadata
            self.phase, self.reason = "teach", "start_fix_consumed_gnss_window_closed"
        else:
            distance = horizontal_distance_m(self._start, fix) if self._start is not None else None
            passed = distance is not None and distance <= self.config["max_endpoint_distance_m"]
            reason = ("missing_start_reference" if distance is None else
                      "within_start_proximity" if passed else "endpoint_distance_limit")
            metadata["validation"] = {"purpose": "repeat_start_proximity", "passed": passed,
                                      "reason": reason, "distance_m": distance,
                                      "max_distance_m": self.config["max_endpoint_distance_m"]}
            self._end, self._end_metadata = fix, metadata
            self.phase, self.reason = "ready", reason
        return fix

    def snapshot(self):
        # Fresh containers prevent callers mutating stored accepted metadata.
        import copy
        return {"phase": self.phase, "reason": self.reason, "policy": "start_and_end_only",
                "purpose": "repeat_start_proximity_only", "mapping_requires_gnss": False,
                "geometry_uses_gnss": False, "teach_started": self._teach_started,
                "ready_for_repeat": bool(self.phase == "ready" and self._end_metadata is not None
                                         and self._end_metadata["validation"]["passed"]),
                "start_unavailable_reason": self._start_unavailable_reason,
                "start": copy.deepcopy(self._start_metadata), "end": copy.deepcopy(self._end_metadata),
                "start_used": int(self._start is not None), "end_used": int(self._end is not None),
                "runtime_used": 0, "outside_window_rejected": self.outside_window_rejected,
                "geometry_changed_by_end": False, "configuration": dict(self.config)}
