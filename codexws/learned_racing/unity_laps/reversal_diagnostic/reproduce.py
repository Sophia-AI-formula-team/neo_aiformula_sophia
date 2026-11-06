"""Run the diagnostic with its archived solver and the existing private A4 Unity host."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import run
from risk_trials import run_group


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    out = parser.parse_args().output.resolve()
    if out.exists():
        raise FileExistsError("Select a new output directory so archived measurements are preserved")
    protocol = json.loads((HERE / "protocol.json").read_bytes())
    observer_hash = hashlib.sha256((HERE.parent / "vision.py").read_bytes()).hexdigest()
    if observer_hash != protocol["observerHash"]:
        raise ValueError("Observer source must match the frozen diagnostic")
    # Source selection is local to this diagnostic process, away from the active controller.
    run.HERE = HERE / "solver"
    source = b"".join(p.read_text(encoding="utf8").encode("utf8") for p in sorted(run.HERE.glob("*.cs")))
    if hashlib.sha256(source).hexdigest()[:14] != protocol["candidateSolver"]:
        raise ValueError("Archived solver does not match measured source")
    lab = run.Lab(run.SIM, out)
    run_group(lab, "reversal", protocol["weights"], [(s, True) for s in protocol["diagnosticSeeds"]], "outer-edge")
