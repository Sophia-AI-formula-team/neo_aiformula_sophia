"""Score closed-loop turning from the physical referee and actual CAN echo (offline only)."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def read(path):
    return json.loads(path.read_bytes())


def check_application_ticks(folder, events):
    delay_ms=read(folder/"host.json").get("actuation_delay_ms",0)
    if not delay_ms:
        return
    with (folder/"actuation.jsonl").open(encoding="utf8") as stream:
        for tick,line in enumerate(stream):
            state=json.loads(line)
            index=(tick-delay_ms//10)//20
            if index < 0:
                expected={"left_can_rpm":0,"right_can_rpm":0}
            else:
                expected=events[index]["command"]
            if (state["tick"]!=tick or state["applied_seq"]!=index
                    or any(state[k]!=expected[k] for k in ("left_can_rpm","right_can_rpm"))):
                raise ValueError("Wrong actual CAN at physics tick " + str(tick))


def summarize(folder, geometry):
    output = read(folder / "result.json")
    request = read(folder / "request.json")
    events = [json.loads(row) for row in (folder / "exchange.jsonl").read_bytes().splitlines()]
    if not events:
        raise ValueError("No actual camera exchange in " + str(folder))
    check_application_ticks(folder,events)
    for i, row in enumerate(events):
        previous = events[i-1]["command"] if i else {"right_can_rpm":0, "left_can_rpm":0}
        frame, command = row["frame"], row["command"]
        if (frame["seq"], command["seq"], frame["applied_seq"]) != (i, i, i-1):
            raise ValueError("Frame sequence differs in " + str(folder))
        for side in ("left", "right"):
            if (previous[side+"_can_rpm"] != frame["applied_"+side+"_can_rpm"]
                    or command[side+"_can_rpm"] != row["set_"+side+"_can"]):
                raise ValueError("CAN echo differs in " + str(folder))

    axis = [0, 2]
    # Unity writes Matrix4x4 flattened by column, hence transpose on import.
    matrix = np.array(output["surfaceGeometry"][0]["matrix"]).reshape(4, 4).T
    basis, origin = matrix[np.ix_(axis, axis)], matrix[axis, 3]
    rings = [np.ascontiguousarray(np.array(geometry[key]) @ basis.T + origin, dtype=np.float32)
             for key in ("outer", "inner")]
    paint = geometry["halfpaint"] * np.linalg.norm(basis[:, 0])

    def legal_distance(point):
        outer, inner = (cv2.pointPolygonTest(ring, tuple(point), True) for ring in rings)
        return float(min(outer, -inner) + paint)

    clearance = [[row["t"], *(legal_distance(p) for p in row["wheelXZ"][:2])]
                 for row in output["surfaceRows"]]
    actual = np.array(output["rows"])
    edge = np.array(request["edge"]) @ basis.T + origin
    directions = np.roll(edge, -1, axis=0) - np.roll(edge, 1, axis=0)
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    # Same lane construction as the Track referee, used only after the run.
    lane = edge - directions[:, ::-1] * [1, -1] * (3.43051 / 2)
    stations = np.r_[0., np.cumsum(np.linalg.norm(np.roll(lane, -1, axis=0) - lane, axis=1))]
    if abs(stations[-1] - output["length"]) > .01:
        raise ValueError("Wrong referee transform")
    tangents = np.unwrap(np.arctan2(directions[:, 0], directions[:, 1]))
    station0 = stations[np.searchsorted(stations, request["start"] * stations[-1])]
    if (folder/"pre_roll.json").exists():
        start_xy=actual[0,1:3]
        nearest=int(np.argmin(np.linalg.norm(lane-start_xy,axis=1)))
        station0=stations[nearest]+np.dot(start_xy-lane[nearest],directions[nearest])
    route = np.interp((station0 + actual[:, 7]) % stations[-1], stations[:-1], tangents)
    body = np.unwrap(actual[:, 3])
    relative_heading = np.rad2deg((body - route + np.pi) % (2*np.pi) - np.pi)
    w = np.array([row["command"]["angular_z_radps"] for row in events])
    v = np.array([row["command"]["linear_mps"] for row in events])
    t = np.array([row["frame"]["simulation_time_s"] for row in events])
    difference = np.array([row["set_right_can"]-row["set_left_can"] for row in events],dtype=int)
    prior = np.array([row["previous_right_can"]-row["previous_left_can"] for row in events],dtype=int)
    delta = difference-prior
    peak = int(np.argmax(abs(delta)))
    event = events[peak]
    gyro = [row["frame"]["gyro_ros_radps"] for row in events]
    # Count a hold for as long as that command was actually held by Unity.
    duration = np.diff(np.r_[t, output["seconds"]])
    significant = np.sign(w[np.abs(w) >= .05])
    wheel = np.min(np.array(clearance)[:, 1:], axis=1)
    # Interpolate the speed threshold on the referee's 0.1 s samples. Separate
    # actual standstill from an issued zero and from an actuator latching zero.
    still = 0.
    last_motion_end = actual[0, 0]
    for (t0, v0), (t1, v1) in zip(actual[:-1, [0,4]], actual[1:, [0,4]]):
        cuts=[0.,1.]
        if v1 != v0:
            cuts.extend(x for x in ((-.05-v0)/(v1-v0),(.05-v0)/(v1-v0)) if 0.<x<1.)
        cuts=sorted(cuts)
        for a,b in zip(cuts,cuts[1:]):
            if abs(v0+(a+b)*.5*(v1-v0))<.05:
                still+=(t1-t0)*(b-a)
            else:
                last_motion_end=t0+(t1-t0)*b
    actual_latch_delay = np.array([row.get("can_set_sim_s", row["frame"]["simulation_time_s"])
                                   for row in events])
    latch_durations = np.diff(np.r_[actual_latch_delay,output["seconds"]])
    return dict(run=folder.name, status=output["status"], seconds=output["seconds"], frames=len(events),
                first_jpeg_sha256=hashlib.sha256((folder/events[0]["camera_file"]).read_bytes()).hexdigest(),
                progress_m=output["progress"], stopped_command_s=float(duration[np.abs(v)<.01].sum()),
                zero_latched_from_command_s=float(latch_durations[np.abs(v)<.01].sum()),
                physical_below_005_s=float(still),
                terminal_physical_below_005_s=float(max(0.,actual[-1,0]-last_motion_end)),
                final_actual_forward_mps=float(actual[-1, 4]),
                first_complete_tread_off_s=output["firstCompletelyOut"],
                painted_wheel_center_m=dict(start=wheel[0], minimum=wheel.min(), final=wheel[-1]),
                cmd_left_positive_radps=dict(min=w.min(), max=w.max(), peak_abs=max(abs(w))),
                actual_left_positive_gyro_peak_radps=float(max(abs(actual[:, 14]))),
                largest_delta_right_minus_left_rpm=dict(seq=peak, signed_rpm=int(delta[peak]),
                    previous_rpm=int(prior[peak]), applied_rpm=int(difference[peak]),
                    capture_s=float(t[peak]), apply_s=event.get("can_set_sim_s",float(t[peak])),
                    body_at_capture_mps=event["body_forward_mps"], cmd_v_mps=float(v[peak]),
                    cmd_omega_radps=float(w[peak]), gyro_next_frame_radps=gyro[peak+1] if peak+1<len(gyro) else None,
                    selected=event["command"].get("selected_index"), feasible=event["command"].get("feasible_count")),
                rolling_peak_delta_rpm=int(np.max(abs(delta[np.array([abs(row["body_forward_mps"]) for row in events])>.1]),initial=0)),
                measured_peak_abs_speed_times_yaw_mps2=float(max(abs(actual[:, 4]*actual[:, 14]))),
                cmd_total_variation_radps=float(sum(abs(np.diff(w)))),
                largest_cmd_step_radps=float(max(abs(np.diff(w)), default=0)),
                cmd_sign_changes_outside_005=int(np.sum(significant[1:] != significant[:-1])),
                heading_right_of_road_deg=dict(start=relative_heading[0], final=relative_heading[-1],
                                               largest_abs=float(max(abs(relative_heading)))),
                wheel_time_series=clearance,
                timeseries=dict(s=t.tolist(), cmd_ros_w=w.tolist(), cmd_v=v.tolist(),
                                diff_can_rpm=difference.tolist(), delta_diff_can_rpm=delta.tolist(), measured_ros_gyro=gyro))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rings", required=True, type=Path, help="Private V6.25 referee-rings.json")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("runs", nargs="+", type=Path)
    args = parser.parse_args()
    geometry = read(args.rings)
    with args.out.open("x",encoding="utf8") as stream:
        json.dump([summarize(path, geometry) for path in args.runs], stream, indent=2, allow_nan=False)


if __name__ == "__main__":
    main()
