"""Score closed-loop turning from the physical referee and actual CAN echo (offline only)."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def read(path):
    return json.loads(path.read_bytes())


def summarize(folder, geometry):
    output = read(folder / "result.json")
    request = read(folder / "request.json")
    events = [json.loads(row) for row in (folder / "exchange.jsonl").read_bytes().splitlines()]
    if not events:
        raise ValueError("No actual camera exchange in " + str(folder))
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
    route = np.interp((station0 + actual[:, 7]) % stations[-1], stations[:-1], tangents)
    body = np.unwrap(actual[:, 3])
    relative_heading = np.rad2deg((body - route + np.pi) % (2*np.pi) - np.pi)
    w = np.array([row["command"]["angular_z_radps"] for row in events])
    v = np.array([row["command"]["linear_mps"] for row in events])
    t = np.array([row["frame"]["simulation_time_s"] for row in events])
    # Count a hold for as long as that command was actually held by Unity.
    duration = np.diff(np.r_[t, output["seconds"]])
    significant = np.sign(w[np.abs(w) >= .05])
    wheel = np.min(np.array(clearance)[:, 1:], axis=1)
    return dict(run=folder.name, status=output["status"], seconds=output["seconds"], frames=len(events),
                first_jpeg_sha256=hashlib.sha256((folder/events[0]["camera_file"]).read_bytes()).hexdigest(),
                progress_m=output["progress"], stopped_command_s=float(duration[np.abs(v)<.01].sum()),
                final_actual_forward_mps=float(actual[-1, 4]),
                first_complete_tread_off_s=output["firstCompletelyOut"],
                painted_wheel_center_m=dict(start=wheel[0], minimum=wheel.min(), final=wheel[-1]),
                cmd_left_positive_radps=dict(min=w.min(), max=w.max(), peak_abs=max(abs(w))),
                actual_left_positive_gyro_peak_radps=float(max(abs(actual[:, 14]))),
                measured_peak_abs_speed_times_yaw_mps2=float(max(abs(actual[:, 4]*actual[:, 14]))),
                cmd_total_variation_radps=float(sum(abs(np.diff(w)))),
                largest_cmd_step_radps=float(max(abs(np.diff(w)), default=0)),
                cmd_sign_changes_outside_005=int(np.sum(significant[1:] != significant[:-1])),
                heading_right_of_road_deg=dict(start=relative_heading[0], final=relative_heading[-1],
                                               largest_abs=float(max(abs(relative_heading)))),
                wheel_time_series=clearance,
                timeseries=dict(s=t.tolist(), cmd_ros_w=w.tolist(), cmd_v=v.tolist(),
                                measured_ros_gyro=[row["frame"]["gyro_ros_radps"] for row in events]))


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
