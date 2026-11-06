"""Paired Unity evaluation of camera lane association and risk-conditioned speed."""
import argparse
from contextlib import contextmanager
import hashlib
import json
import os
import socket
import struct
import subprocess
import sys
import time

from run import HERE, Lab, SIM, write

ROOT = HERE / "risk_revision"
PRACTICE = (41021, 5948, 33551)
CHECK = (78441, 96527)
BLIND = (93677, 20318, 57846, 81052)
INITIAL = json.loads((HERE / "evidence/camera_policy.json").read_bytes())


@contextmanager
def detector(lab, observer, name, film=False):
    folder = lab.work / "camera" / name
    folder.mkdir(exist_ok=True, parents=True)
    ready = folder / "ready.json"
    ready.unlink(missing_ok=True)
    cfg = json.loads((SIM / "Simulator/UserSettings/AIFormulaRoadDetector.json").read_bytes())
    release = SIM / "Simulator/Assets/StreamingAssets/RoadDetector/a4"
    args = [cfg["python"], "-B", str(HERE / "vision.py"), str(release), str(ready)]
    if not film:
        args.append("--no-samples")
    env = dict(os.environ, AIFORMULA_LANE_OBSERVER=observer)
    with (folder / "server.log").open("w", encoding="utf8") as log:
        proc = subprocess.Popen(args, stdout=log, stderr=log, env=env, creationflags=subprocess.CREATE_NO_WINDOW)
        try:
            start = time.monotonic()
            while not ready.exists():
                if proc.poll() is not None or time.monotonic() - start > 100:
                    raise RuntimeError("A4 did not start: " + str(folder))
                time.sleep(.15)
            meta = json.loads(ready.read_bytes())
            if observer != meta.get("laneObserver", meta.get("lane_observer", "legacy")):
                raise ValueError("Observer mode mismatch: " + str(meta))
            model = "a4/single_frame/" + meta["sha256"] + "/" + observer
            if observer != "legacy":
                model += "/" + hashlib.sha256((HERE / "vision.py").read_bytes()).hexdigest()[:12]
            yield meta["port"], model
        finally:
            if ready.exists() and proc.poll() is None:
                with socket.create_connection(("127.0.0.1", json.loads(ready.read_bytes())["port"]), timeout=3) as sock:
                    sock.sendall(struct.pack("<I", 0))
            try:
                proc.wait(8)
            except subprocess.TimeoutExpired:
                proc.terminate()
                proc.wait()


def run_group(lab, name, weights, sources, observer, film=False):
    with detector(lab, observer, name, film) as (port, model):
        for seed, traffic in sources:
            trial = name if film else f"{name}_{seed}_{'traffic' if traffic else 'clear'}"
            lab.run(trial, weights, seed, traffic, port, film, capture=.15, model=model)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("action", choices=["baseline","practice","qualify","blind","film"])
    p.add_argument("--name", default="challenger")
    p.add_argument("--weights", nargs="+", type=float)
    p.add_argument("--observer", default="outer-edge")
    p.add_argument("--seed", type=int, default=93677)
    p.add_argument("--clear", action="store_true")
    args = p.parse_args()
    baseline = args.action == "baseline"
    lab = Lab(SIM, ROOT, INITIAL["solver"] if baseline else None)
    weights = INITIAL["weights"] if baseline else args.weights
    if args.action in ("blind","film"):
        frozen = json.loads((ROOT / "frozen.json").read_bytes())
        weights = frozen["weights"]
        if lab.build != frozen["solver"]:raise RuntimeError("Frozen controller changed")
    if not weights:raise ValueError("Candidate weights required")
    if args.action == "film":
        run_group(lab, args.name, weights, [(args.seed, not args.clear)], args.observer, True)
        return
    if baseline or args.action == "blind":
        jobs = [(s, True) for s in BLIND] + [(s, False) for s in BLIND[:2]]
    elif args.action == "practice":
        jobs = [(s, True) for s in PRACTICE]
    else:
        jobs = [(s, True) for s in CHECK]
    run_group(lab, "prior" if baseline else args.name, weights, jobs, "legacy" if baseline else args.observer)


if __name__ == "__main__":
    main()
