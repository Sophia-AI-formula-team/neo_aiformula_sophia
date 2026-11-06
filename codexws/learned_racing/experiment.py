"""Run the saved race, or regenerate both the fitted plant and the driving policies."""
import argparse
import hashlib
import json
import tempfile
from pathlib import Path

from course import Course, build_course
from driving import SPEED_FEATURES, STEER_FEATURES, practice
from dynamics import TRAIN, TEST, fit


HERE = Path(__file__).resolve().parent


def write(path, obj):
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf8")


def train_course(course, plant, args, seed, robust):
    first, best, history = practice(course, plant, seed, args.generations, args.population, robust)
    return {"seed": seed, "robust_practice": robust, "first": first.tolist(), "best": best.tolist(), "history": history}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=HERE/"demo")
    parser.add_argument("--surface", type=Path, help="Simulator/Assets/TrainingEnvironment/Surface")
    parser.add_argument("--extracted", type=Path, help="oct27_dynamics_20261104/extracted")
    parser.add_argument("--bags", type=Path, help="original Oct 27 recording directory, for hash verification")
    parser.add_argument("--train-bags", nargs="+", default=TRAIN, help="recording names used for identification")
    parser.add_argument("--test-bags", nargs="+", default=TEST, help="separate recording names for evaluation")
    parser.add_argument("--train", action="store_true", help="repeat simulated practice")
    parser.add_argument("--seed", type=int, default=41)
    parser.add_argument("--generations", type=int, default=26)
    parser.add_argument("--population", type=int, default=80)
    args = parser.parse_args()
    if bool(args.extracted) != bool(args.bags):
        parser.error("--extracted and --bags must be supplied together")
    if (args.surface or args.extracted) and not args.train:
        parser.error("regenerating inputs requires --train")
    out = args.output
    out.mkdir(exist_ok=True, parents=True)
    source = HERE/"demo" if args.train else out
    geometry = (build_course(args.surface) if args.surface else
                json.loads((source/"course.json").read_text(encoding="utf8")))
    plant = (fit(args.extracted, args.bags, args.train_bags, args.test_bags) if args.extracted else
             json.loads((source/"dynamics.json").read_text(encoding="utf8")))
    signature = hashlib.sha256(json.dumps([geometry, plant], sort_keys=True).encode()).hexdigest()
    cases = {"outer": Course(geometry), "gates": Course(geometry, gates=True),
             "tight": Course(geometry, gates=True, tight=True)}
    if args.train:
        policy = {"speed_inputs": SPEED_FEATURES, "steering_inputs": STEER_FEATURES,
                  "inputs_sha256": signature, "courses": {}}
        for index, (name, course) in enumerate(cases.items()):
            # Each layout starts from zero weights. The first visit has no practice memory.
            print(name, flush=True)
            policy["courses"][name] = train_course(course, plant, args, args.seed+index, name=="tight")
    else:
        policy = json.loads((out/"policy.json").read_text(encoding="utf8"))
        if policy["inputs_sha256"] != signature:
            raise ValueError("Policy, vehicle parameters and layout must come from the same training run")
    from report import report
    # A failed training generation or render must not replace a usable saved demo.
    with tempfile.TemporaryDirectory(dir=out) as staging:
        stage = Path(staging)
        if args.train:
            write(stage/"course.json", geometry)
            write(stage/"dynamics.json", plant)
            write(stage/"policy.json", policy)
        report(stage, cases, plant, policy)
        for file in stage.iterdir():
            file.replace(out/file.name)


if __name__ == "__main__":
    main()
