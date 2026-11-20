"""Prepare or verify the private OA1B control overlay against the 11/20 installed files."""
import argparse
import hashlib
import importlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

SITE = "lib/python3.8/site-packages"
PLANNER = "semantic_local_planner_ros"
MOTOR = "motor_controller"
SHARE = "launchers/share/launchers"
REQUIRED = {
    "launch/semantic_045_commissioning.launch.py": "bf2246348605be59552e0f71a34125f0a8f2bccca6d3cbb0edde1f4277d6fcc1",
    "launch/semantic_vehicle_045.launch.py": "409e91fb39a57379fa3e23cbaf36bfddd6bf930a39c488930946b4971997106b",
    "launch/obstacle_avoidance.launch.py": "2090ced81475c25ee4ff42fcd5d05742441dcec8dc4b85aa56165215bc81208e",
    "shellscript/mapassist_045/pixel_stop_flag.py": "cd89891cc1627205228a78d8642df638dc3037056a771b0df9f42899c3c0d148",
    "shellscript/mapassist_045/fresh_oa_switch.py": "c47cd40ae9719bd16c0c51b5ed58c9087462f7c46e7d091110ad1d0b59e1b518",
    "shellscript/mapassist_045/guarded_stop_wait.py": "3a31a1b44581d5348e098f2615c79a72cc021699f387f683d126f34c3cc1d7cb",
    "shellscript/run_obstacle_avoidance.py": "707930f6b7f0d81125d799a0030bdad1c70185f786aed1c4bc4f513b0edf8d93",
    "shellscript/oa_2_1.py": "5f0cde0f0ddc7fee7dd87542c9d6a85783875241ddaf6df800cfe1d36286111d",
}
PLANNER_BASE = "eda0194f818958ab22a2fd7a3e0121bfb1a228a370c22ef7f2c175e569d93095"
MOTOR_BASE = "7a6282c6a7e8f0215477da917f654fc39d15891c76a0b4209edeeb859519fd7d"
GATE = "5051626c0497763d8aa6baab307d96b1330219876358968d1ecb110699905ac1"
PLANNER_TESTED = "dab225f79f86015896e6fcabb2f48f1be0ba526e17245c659fbea030ceb4b84e"
MOTOR_TESTED = "7c4447a24e88f99014230e27d556a06c836afd6c759642f4544804ea03e01b17"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(path, expected):
    actual = sha(path)
    if actual != expected:
        raise ValueError(f"Unexpected source {path}: {actual}, expected {expected}")


def wire(source, target, replacements):
    contents = source.read_text(encoding="utf8")
    for old, new in replacements:
        if contents.count(old) != 1:
            raise ValueError(f"Could not bind {old} in {source}")
        contents = contents.replace(old, new)
    target.write_bytes(contents.encode("utf8"))


def verified_files(root, receipt):
    for path, expected in receipt["private_files"].items():
        require(root / path, expected)
    for path, expected in REQUIRED.items():
        require(Path(receipt["field_install"]) / SHARE / path, expected)
    require(Path(receipt["field_install"]) / PLANNER / SITE / PLANNER / "map_assist.py", PLANNER_BASE)
    require(Path(receipt["motor_source"]) / "motor_controller.py", MOTOR_BASE)
    require(Path(receipt["motor_source"]) / "gate.py", GATE)


def prepare(field, motor, dest):
    if field in dest.parents or motor in dest.parents or dest in (field, motor):
        raise ValueError("OA1B needs a private prefix outside the vehicle install")
    planner = field / PLANNER / SITE / PLANNER
    share = field / SHARE
    for path, expected in REQUIRED.items():
        require(share / path, expected)
    require(planner / "map_assist.py", PLANNER_BASE)
    require(motor / "motor_controller.py", MOTOR_BASE)
    require(motor / "gate.py", GATE)
    patch = Path(__file__).with_name("red_flag.patch")
    dest.parent.mkdir(parents=True, exist_ok=True)
    # Build a complete immutable tree beside the destination. A failed check leaves
    # both the existing vehicle install and the last good OA1B install available.
    with tempfile.TemporaryDirectory(dir=dest.parent) as tmp:
        stage = Path(tmp) / "control"
        private = stage / "python"
        private.mkdir(parents=True)
        shutil.copytree(planner, private / PLANNER, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        shutil.copytree(motor, private / MOTOR, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        for name, original in [("map_assist.py", planner), ("motor_controller.py", motor)]:
            shutil.copyfile(original / name, Path(tmp) / name)
        subprocess.run(["git","-c","core.autocrlf=false","apply","--no-index",str(patch.resolve())],
                       cwd=tmp, check=True)
        for file, package, digest in [("map_assist.py",PLANNER,PLANNER_TESTED),
                                      ("motor_controller.py",MOTOR,MOTOR_TESTED)]:
            require(Path(tmp) / file, digest)
            shutil.copyfile(Path(tmp) / file, private / package / file)
        launch = stage / "launch"
        launch.mkdir()
        wire(share / "launch/semantic_045_commissioning.launch.py", launch / "oa1b.launch.py", [
            ('share / "launch" / "semantic_vehicle_045.launch.py"', 'Path(__file__).with_name("semantic_vehicle_045.launch.py")'),
            ('directory / "pixel_stop_flag.py"', 'Path(__file__).with_name("pixel_stop_flag.py")')])
        wire(share / "launch/semantic_vehicle_045.launch.py", launch / "semantic_vehicle_045.launch.py", [
            ('"/aiformula_perception/yolop_a3/traffic_light/state"', '"/oa_1b/perception/traffic_light/state"')])
        wire(share / "shellscript/mapassist_045/pixel_stop_flag.py", launch / "pixel_stop_flag.py", [
            ('"/aiformula_perception/yolop_a3/detections"', '"/oa_1b/perception/detections"')])
        # Commit the exact Python entry modules and wired scripts. No shared underlay file is modified.
        receipt = dict(field_install=str(field), motor_source=str(motor), red_flag_patch=sha(patch),
                       private_files={p.relative_to(stage).as_posix():sha(p)
                                      for p in sorted(stage.rglob("*")) if p.is_file()})
        receipt_file = "control-version.json"
        if dest.exists():
            if json.loads((dest / receipt_file).read_text()) != receipt:
                raise ValueError(f"{dest} already owns a different build; use a new OA1B prefix")
            verified_files(dest, receipt)
            return
        (stage / receipt_file).write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf8")
        os.rename(stage, dest)


def verify(dest, role, entrypoint):
    receipt = json.loads((dest / "control-version.json").read_text())
    verified_files(dest, receipt)
    package, module, attr = ((MOTOR, MOTOR, "main") if role == "motor" else (PLANNER, "map_assist", "main"))
    imported = importlib.import_module(f"{package}.{module}")
    if Path(imported.__file__).resolve() != dest / "python" / package / (module + ".py"):
        raise ValueError(f"Another overlay supplies {imported.__file__}")
    if role == "motor":
        gate = importlib.import_module(MOTOR + ".gate")
        if Path(gate.__file__).resolve() != dest / "python" / MOTOR / "gate.py":
            raise ValueError(f"Another overlay supplies {gate.__file__}")
    if role == "planner":
        from ament_index_python.packages import get_package_share_directory
        for pkg in ("launchers", PLANNER):
            expected = Path(receipt["field_install"]) / pkg / "share" / pkg
            if Path(get_package_share_directory(pkg)).resolve() != expected.resolve():
                raise ValueError(f"Wrong ament prefix for {pkg}, expected {expected}")
    if entrypoint == "console":
        # Foxy launches setuptools console scripts. Also resolve the entry point used by
        # those scripts, rather than assuming an import name proves the executable binding.
        import pkg_resources
        ep = pkg_resources.get_entry_info(package.replace("_","-"), "console_scripts", module)
        if ep is None or ep.resolve() is not getattr(imported, attr):
            raise ValueError(f"Vehicle console entry point does not resolve to the private {package}.{module}")
    print(f"OA1B {role}: {imported.__file__} {sha(Path(imported.__file__))}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest="action", required=True)
    build = subs.add_parser("build")
    build.add_argument("--field-install", type=Path, required=True)
    build.add_argument("--motor-package", type=Path, required=True)
    build.add_argument("--dest", type=Path, required=True)
    check = subs.add_parser("verify")
    check.add_argument("--dest", type=Path, required=True)
    check.add_argument("--role", choices=("motor","planner"), required=True)
    check.add_argument("--entrypoint", choices=("console","module"), default="console",
                       help="The vehicle uses console. The Unity amd64 motor runs as a Python module.")
    args = parser.parse_args()
    if args.action == "build":
        prepare(args.field_install.resolve(),args.motor_package.resolve(),args.dest.resolve())
    else:
        verify(args.dest.resolve(),args.role,args.entrypoint)


if __name__ == "__main__":
    main()
