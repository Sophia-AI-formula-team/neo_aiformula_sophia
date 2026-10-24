"""File-scoped, hash-guarded trial deployment. Never refreshes or runs Unity."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parent
WORKER_SHA = "bf50643792b2c998a179bdab47ddc9e9400ee3d009de704a1e14d1e3cd328469"
PREREQUISITES = {
    "signal_panel_range.py": "baf09f79cd0649b05e8d1acd90e6a9395bd7361d00e2b8ff51f6f20dd54d4ce7",
    "signal_panel_rgb.py": "aeb7b9c23d019b67fcf795a3a5067d9de55eacb33af9e0220f7095cb7dd61326",
}


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def worker_after(raw):
    old = b"from signal_panel_range import estimate_panel_range"
    if sha(raw) != WORKER_SHA or raw.count(old) != 1:
        raise ValueError("worker drift: inspect the new source; do not force")
    return raw.replace(old, b"from signal_bbox_range import estimate_panel_range")


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--simulator", type=Path, required=True)
    parser.add_argument("--action", choices=("inspect", "apply", "rollback"), default="inspect")
    parser.add_argument("--receipt", type=Path)
    parser.add_argument("--exclusive-clean-edit", action="store_true")
    parser.add_argument("--enable-trial", action="store_true")
    args = parser.parse_args()
    target = args.simulator.resolve() / "Assets/StreamingAssets/RoadDetector"
    if not target.is_dir():
        raise ValueError("not a compatible Simulator project")
    if args.action == "rollback":
        if not args.exclusive_clean_edit or args.receipt is None:
            raise ValueError("rollback requires exclusive clean Edit and original receipt")
        receipt = json.loads(args.receipt.read_text(encoding="utf-8"))
        if receipt["target"] != str(target) or receipt["status"] != "applied":
            raise ValueError("wrong or incomplete deployment receipt")
        # Validate all three targets before modifying any of them.
        expected_names = {"worker.py", "signal_bbox_range.py", "signal_bbox_range.json"}
        if set(receipt["after"]) != expected_names:
            raise ValueError("unexpected rollback paths")
        for name, digest in receipt["after"].items():
            if sha((target / name).read_bytes()) != digest:
                raise ValueError("target changed since apply: " + name)
        backup = args.receipt.parent / "worker.before.py"
        if sha(backup.read_bytes()) != WORKER_SHA:
            raise ValueError("backup changed")
        shutil.copyfile(backup, target / "worker.py")
        for name in ("signal_bbox_range.py", "signal_bbox_range.json"):
            (target / name).unlink()
        # Do not remove Unity .meta files or refresh while another chat owns it.
        print(json.dumps(dict(status="rolled_back", target=str(target))))
        return
    for name, digest in PREREQUISITES.items():
        if sha((target / name).read_bytes()) != digest:
            raise ValueError("prerequisite drift: " + name)
    original = (target / "worker.py").read_bytes()
    payloads = {"worker.py": worker_after(original)}
    for name in ("signal_bbox_range.py", "signal_bbox_range.json"):
        if (target / name).exists():
            raise ValueError("target already exists: " + name)
        payloads[name] = (ROOT / "overlay" / name).read_bytes()
    manifest = json.loads((ROOT / "manifest.json").read_text(encoding="utf-8"))
    for name in ("signal_bbox_range.py", "signal_bbox_range.json"):
        if sha(payloads[name]) != manifest["overlay_sha256"][name]:
            raise ValueError("payload drift: " + name)
    if args.enable_trial:
        config = json.loads(payloads["signal_bbox_range.json"])
        config["enabled"] = True
        payloads["signal_bbox_range.json"] = (json.dumps(config, indent=2) + "\n").encode()
    receipt = dict(status="prepared", target=str(target), before_worker_sha256=sha(original),
                   after={name: sha(raw) for name, raw in payloads.items()}, completed=[])
    if args.action == "inspect":
        print(json.dumps(receipt, indent=2))
        return
    if not args.exclusive_clean_edit or args.receipt is None:
        raise ValueError("apply requires exclusive clean Edit and a fresh receipt path")
    # One attempt per new directory. Persist partial receipts, never auto-retry.
    args.receipt.parent.mkdir(parents=True, exist_ok=False)
    (args.receipt.parent / "worker.before.py").write_bytes(original)
    def save():
        args.receipt.write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    save()
    for name in ("signal_bbox_range.py", "signal_bbox_range.json", "worker.py"):
        (target / name).write_bytes(payloads[name])
        receipt["completed"].append(name)
        save()
    for name, digest in receipt["after"].items():
        if sha((target / name).read_bytes()) != digest:
            raise ValueError("post-write mismatch: " + name)
    receipt["status"] = "applied"
    save()
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
