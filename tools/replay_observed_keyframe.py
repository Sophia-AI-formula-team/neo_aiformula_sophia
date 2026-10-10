"""Export/replay a portable, observed mask + wheel/gyro mapping regression.

No Unity, ROS transport, world pose, route prior, control publication or future
frame is used. Export admits only named sensor fields; replay recomputes odometry
and the one-time teach origin. New outputs are exclusive, never replacements.
The prefix is a regression case, not evidence of a completed physical lap.
"""
import argparse
import base64
import hashlib
import io
import json
import math
from pathlib import Path
import subprocess
import sys
from types import ModuleType

import numpy as np
from PIL import Image


CORE_NAMES = (
    "lane_mapping_lya_reference.mapping_core",
    "lane_mapping_lya_reference.mask_localization",
    "lane_mapping_fixed_gnss.motion",
    "lane_mapping_fixed_gnss.mapping",
)
CONTROL = "workspace/src/aiformula/control/"
EPOCH_NS = 1_000_000_000


def sha(data):
    return hashlib.sha256(data).hexdigest()


def source_hashes(data):
    normalized = data.replace(b"\r\n", b"\n")
    return dict(raw=sha(data), lf=sha(normalized), crlf=sha(normalized.replace(b"\n", b"\r\n")))


def source_path(name):
    package, module = name.split(".")
    return CONTROL + package + "/" + package + "/" + module + ".py"


def source_ns(seconds):
    return EPOCH_NS + round(seconds * 1e9)


def pose_digest(pose):
    # 摘要仅用于检验轮速积分没有漂移；不会把记录位姿作为算法输入。
    values = [0. if round(float(value), 10) == 0. else round(float(value), 10)
              for value in pose]
    return sha(json.dumps(values, separators=(",", ":")).encode())


def json_safe(value):
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def write_new(path, value):
    if not path.parent.is_dir():
        raise ValueError("output parent must already exist")
    with path.open("x", encoding="utf-8") as stream:
        json.dump(json_safe(value), stream, ensure_ascii=False, allow_nan=False,
                  separators=(",", ":"))
        stream.write("\n")


def export_fixture(backend, unity, output):
    records = [json.loads(line) for line in (backend / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    startup = next(row for row in records if row["event"] == "startup")
    provenance = startup["provenance"]
    camera = next(row["camera"] for row in records if row["event"] == "calibration")
    sensor = next(row["measured_values"] for row in records if row["event"] == "sensor_calibration")
    frames, mask_bytes = [], 0
    receipt_origin = None
    for row in records:
        if row["event"] != "step":
            continue
        response = row["response"]
        metrics = response["metrics"]
        receipt_origin = receipt_origin or row["received_monotonic_ns"]
        motion = []
        for sample in row["motion"]:
            if set(sample) != {"stamp", "left_rpm", "right_rpm", "gyro_z"}:
                raise ValueError("unexpected motion fields")
            motion.append(dict(sample))
        frame = dict(sequence=response["sequence"], image_stamp_ns=source_ns(response["sim_time"]),
                     receipt_ns=EPOCH_NS + row["received_monotonic_ns"] - receipt_origin,
                     motion=motion, raw_odom_digest=pose_digest(metrics["raw_odom_pose"]))
        if "mapping" in metrics:
            png = (unity / ("mask_%06d.png" % response["sequence"])).read_bytes()
            if sha(png) != metrics["mask_sha256"]:
                raise ValueError("mask hash mismatch at sequence " + str(response["sequence"]))
            mask_bytes += len(png)
            result = metrics["mapping"]
            frame.update(mask_png_base64=base64.b64encode(png).decode("ascii"),
                         mask_sha256=sha(png), prior_digest=pose_digest(metrics["map_frame_raw_prior"]),
                         recorded_decision=dict(accepted=result["accepted"], reason=result["reason"]))
        frames.append(frame)
        if "mapping" in metrics and not metrics["mapping"]["accepted"]:
            break
    mapped = [row for row in frames if "mask_png_base64" in row]
    if not mapped or mapped[-1]["recorded_decision"]["accepted"]:
        raise ValueError("fixture must end at the first observed map rejection")
    fixture = dict(schema_version=1, kind="observed_mask_wheel_gyro_prefix",
                   sensor_source="Unity model-wheel RPM and body gyro; ideal-camera mask",
                   source_epoch_offset_ns=EPOCH_NS,
                   camera={key: camera[key] for key in ("width", "height", "K", "T_base_camera")},
                   sensor_calibration={key: sensor[key] for key in ("wheel_diameter_m", "motor_to_wheel_ratio")},
                   teach_map_config=provenance["teach_map_adapter_overrides"],
                   recorded_core_revision=provenance["repo_head"],
                   recorded_source_sha256={source_path(name): provenance["source_sha256"][source_path(name)]
                                           for name in CORE_NAMES},
                   evidence_sha256=dict(events=sha((backend / "events.jsonl").read_bytes())),
                   frames=frames,
                   scope="Observed prefix only; no world pose, GNSS, route, controller or future recovery proof",
                   receipt_note="Original per-batch receipt intervals, translated to a positive relative epoch")
    write_new(output, fixture)
    return dict(fixture=output.name, bytes=output.stat().st_size, sha256=sha(output.read_bytes()),
                batches=len(frames), motion_samples=sum(len(row["motion"]) for row in frames),
                mapping_frames=len(mapped), exact_png_bytes=mask_bytes,
                first_mapping_sequence=mapped[0]["sequence"], terminal_sequence=mapped[-1]["sequence"])


def load_core(repo, revision=None):
    """Load four real core files in memory, from worktree or immutable Git blobs."""
    if revision:
        revision = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "--verify",
                                            revision + "^{commit}"], text=True).strip()
    modules, hashes, text_hashes = {}, {}, {}
    for name in CORE_NAMES:
        relative = source_path(name)
        data = (subprocess.check_output(["git", "-C", str(repo), "show", revision + ":" + relative])
                if revision else (repo / relative).read_bytes())
        hashes[relative] = sha(data)
        text_hashes[relative] = source_hashes(data)
        package = name.split(".")[0]
        if package not in sys.modules:
            parent = ModuleType(package)
            parent.__path__ = []
            sys.modules[package] = parent
        module = ModuleType(name)
        module.__package__ = package
        module.__file__ = ("git:" + revision + ":" + relative) if revision else str(repo / relative)
        sys.modules[name] = module
        exec(compile(data, module.__file__, "exec"), module.__dict__)
        modules[name] = module
    return modules, hashes, text_hashes, revision


def map_prior(raw, origin):
    dx, dy = raw[:2] - origin[:2]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return np.array([c * dx + s * dy, -s * dx + c * dy,
                     math.atan2(math.sin(raw[2] - origin[2]), math.cos(raw[2] - origin[2]))])


def replay_fixture(fixture_path, repo, revision, output):
    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    if fixture["schema_version"] != 1 or fixture["kind"] != "observed_mask_wheel_gyro_prefix":
        raise ValueError("unsupported fixture")
    modules, hashes, text_hashes, resolved_revision = load_core(repo, revision)
    motion_module = modules["lane_mapping_fixed_gnss.motion"]
    map_module = modules["lane_mapping_fixed_gnss.mapping"]
    localizer = modules["lane_mapping_lya_reference.mask_localization"].LaneMapLocalizer
    camera, sensor = fixture["camera"], fixture["sensor_calibration"]
    odometry = motion_module.CausalWheelGyroOdometry({"wheel_diameter_m": sensor["wheel_diameter_m"]})
    engine = map_module.TeachMapEngine(config=fixture["teach_map_config"])
    default_matcher = dict(localizer(np.empty((0, 2))).config)
    lookup = modules["lane_mapping_lya_reference.mapping_core"].GroundLookup(
        camera["width"], camera["height"], camera["K"], np.asarray(camera["T_base_camera"]).reshape(4, 4), .5)
    origin, results = None, []
    sample_count, prior_checks = 0, 0
    for frame in fixture["frames"]:
        stamp, receipt = frame["image_stamp_ns"], frame["receipt_ns"]
        for sample in frame["motion"]:
            sensor_stamp = source_ns(sample["stamp"])
            if sensor_stamp > stamp:
                raise ValueError("future motion relative to image")
            odometry.accept_gyro(sensor_stamp, receipt, sample["gyro_z"])
            accepted = odometry.accept_wheels(sensor_stamp, receipt,
                sample["left_rpm"] / sensor["motor_to_wheel_ratio"],
                sample["right_rpm"] / sensor["motor_to_wheel_ratio"], stamp, receipt)
            sample_count += 1
        raw = np.array([accepted.x, accepted.y, accepted.yaw])
        if pose_digest(raw) != frame["raw_odom_digest"]:
            raise ValueError("recomputed raw odometry differs at sequence " + str(frame["sequence"]))
        if "mask_png_base64" not in frame:
            continue
        if origin is None:
            origin = raw.copy()
        prior = map_prior(raw, origin)
        if pose_digest(prior) != frame["prior_digest"]:
            raise ValueError("recomputed map prior differs at sequence " + str(frame["sequence"]))
        prior_checks += 1
        png = base64.b64decode(frame["mask_png_base64"], validate=True)
        if sha(png) != frame["mask_sha256"]:
            raise ValueError("fixture mask hash mismatch")
        with Image.open(io.BytesIO(png)) as image:
            if image.format != "PNG" or image.size != (camera["width"], camera["height"]):
                raise ValueError("invalid fixture PNG")
            points, sensitivity, _ = lookup.project_mask_with_sensitivity(np.asarray(image.convert("L")), 127)
        result = engine.process(points, sensitivity, prior, stamp)
        if engine._matcher is not None and dict(engine._matcher.config) != default_matcher:
            raise ValueError("matcher gates differ from production defaults")
        frontier = result["matched_against_frontier_ns"]
        if frontier is not None and frontier >= stamp:
            raise ValueError("current or future keyframe frontier")
        results.append(dict(sequence=frame["sequence"], recorded_decision=frame["recorded_decision"],
                            mask_sha256=frame["mask_sha256"], result=json_safe(result)))
        if not result["accepted"]:
            break
    expected_count = sum("mask_png_base64" in frame for frame in fixture["frames"])
    equivalence = {}
    for name, expected in fixture["recorded_source_sha256"].items():
        actual = text_hashes[name]
        equivalence[name] = ("raw_identical" if actual["raw"] == expected else
                             "line_endings_only" if expected in (actual["lf"], actual["crlf"]) else
                             "different_content")
    report = dict(schema_version=1, kind="offline_observed_prefix_core_regression",
                  fixture_sha256=sha(fixture_path.read_bytes()), source_sha256=hashes,
                  source_line_ending_hashes=text_hashes, source_equivalence_to_recording=equivalence,
                  source_revision=resolved_revision, source_mode="git_blob" if revision else "worktree",
                  baseline_hashes_match=hashes == fixture["recorded_source_sha256"],
                  baseline_content_matches_ignoring_only_line_endings=all(
                      value != "different_content" for value in equivalence.values()),
                  motion_config=dict(odometry.config), teach_config=dict(engine.config),
                  matcher_config=default_matcher, matcher_gates_unchanged=True,
                  recomputed_motion_samples=sample_count, checked_recomputed_priors=prior_checks,
                  expected_mapping_frames=expected_count, actual_mapping_frames=len(results),
                  all_prefix_frames_accepted=len(results) == expected_count and all(r["result"]["accepted"] for r in results),
                  observed_decisions_reproduced=len(results) == expected_count and all(
                      r["result"]["accepted"] == r["recorded_decision"]["accepted"] and
                      r["result"]["reason"] == r["recorded_decision"]["reason"] for r in results),
                  results=results, odometry_reset_count=odometry.reset_count,
                  limits=["Real captured simulation sensors, not physical vehicle data.",
                          "No adapter teacher/control/transport/stopping/route/repeat exercised.",
                          "This fixed recorded prefix is not the future trajectory after a changed command.",
                          "Real default clocks and processing budgets; offline latency is not runtime performance."])
    write_new(output, report)
    return dict(report=output.name, all_prefix_frames_accepted=report["all_prefix_frames_accepted"],
                observed_decisions_reproduced=report["observed_decisions_reproduced"],
                mapping_frames=len(results), last_sequence=results[-1]["sequence"],
                last_reason=results[-1]["result"]["reason"], sha256=sha(output.read_bytes()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="operation", required=True)
    export = sub.add_parser("export")
    export.add_argument("--backend", type=Path, required=True)
    export.add_argument("--unity", type=Path, required=True)
    replay = sub.add_parser("replay")
    replay.add_argument("--fixture", type=Path, required=True)
    replay.add_argument("--repo", type=Path, required=True)
    replay.add_argument("--revision", help="Optional immutable Git source; otherwise current worktree")
    for child in (export, replay):
        child.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists")
    if args.operation == "export":
        result = export_fixture(args.backend, args.unity, args.output)
    else:
        result = replay_fixture(args.fixture, args.repo, args.revision, args.output)
    print(json.dumps(result, allow_nan=False))


if __name__ == "__main__":
    main()
