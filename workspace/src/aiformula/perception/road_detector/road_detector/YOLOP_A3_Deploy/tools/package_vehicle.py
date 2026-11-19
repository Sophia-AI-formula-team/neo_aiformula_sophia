"""Build two standalone source archives. No system installs or model downloads."""
import argparse
import hashlib
from pathlib import Path
import sys
from zipfile import ZIP_DEFLATED, ZipFile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from run_vehicle import WEIGHT_SHA256, profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    weight = ROOT / "weights/shared_best.pth"
    if hashlib.sha256(weight.read_bytes()).hexdigest() != WEIGHT_SHA256:
        raise ValueError("Unexpected checkpoint")
    files = [ROOT / name for name in ("run_vehicle.py", "README.md", "DEPLOYMENT.md", "JETSON.md",
                                     "TEMPORAL.md", "MODE_GUIDE_ZH.md", "LICENSE",
                                     "THIRD_PARTY_NOTICES.md", "pyproject.toml",
                                     "tools/compare_lean.py", "tools/verify_weights.py")]
    for directory in ("yolop_a3_runtime", "ros2", "weights"):
        files.extend(p for p in (ROOT / directory).rglob("*")
                     if p.is_file() and "__pycache__" not in p.parts and p.suffix not in {".pyc", ".pyo"})
    args.out.mkdir(parents=True, exist_ok=True)
    for name in ("A4", "A4_lite"):
        config = ROOT / f"profiles/{name}.json"
        profile(config, name)
        path = args.out / f"{name}-neo-vehicle.zip"
        with ZipFile(path, "w", compression=ZIP_DEFLATED) as archive:
            for src in sorted(files):
                archive.write(src, f"{name}/{src.relative_to(ROOT).as_posix()}")
            archive.write(config, f"{name}/profile.json")
        print(hashlib.sha256(path.read_bytes()).hexdigest(), path)


if __name__ == "__main__":
    main()
