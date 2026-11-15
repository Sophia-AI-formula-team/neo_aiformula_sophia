"""Hash-guarded manifest-scoped Unity adapter installation. Default is read-only."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def within(root, relative):
    path = (root / relative).resolve()
    if root not in path.parents:
        raise ValueError("Path escapes root: " + relative)
    return path


def atomic_write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=path.name + ".", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(data)
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulator", type=Path, required=True)
    parser.add_argument("--action", choices=("check", "install", "rollback"), default="check")
    parser.add_argument("--apply", action="store_true", help="Write the reviewed plan; omit for read-only comparison")
    args = parser.parse_args()
    if args.apply and args.action == "check":
        parser.error("--apply requires --action install or rollback")
    root = Path(__file__).resolve().parent
    simulator = args.simulator.resolve(strict=True)
    if not (simulator / "Assets").is_dir() or not (simulator / "ProjectSettings/ProjectVersion.txt").is_file():
        raise ValueError("Expected an existing Unity project")
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    for relative, expected in manifest["archive_files"].items():
        if sha(within(root, relative)) != expected:
            raise ValueError("Archive payload hash changed: " + relative)
    plan = []
    for entry in manifest["unity_files"]:
        live = within(simulator, entry["path"])
        current = sha(live)
        if live.exists() and not live.is_file():
            raise ValueError("Target is not a regular file: " + str(live))
        before, after = entry["before_sha256"], entry["after_sha256"]
        state = "before" if current == before else "after" if current == after else "conflict"
        desired = before if args.action == "rollback" else after
        action = "unchanged" if current == desired else "remove" if desired is None else "write"
        plan.append({"path": entry["path"], "state": state, "current_sha256": current,
                     "before_sha256": before, "after_sha256": after, "planned_action": action})
    receipt = {"action": args.action, "applied": False, "simulator": str(simulator), "files": plan}
    if any(row["state"] == "conflict" for row in plan):
        print(json.dumps(receipt, indent=2))
        raise SystemExit("Refusing unknown live bytes; preserve other edits and review first")
    if args.apply:
        # Preflight all files before any write, then recheck each immediately before mutation.
        for entry, row in zip(manifest["unity_files"], plan):
            live = within(simulator, entry["path"])
            if sha(live) != row["current_sha256"]:
                raise RuntimeError("Live file changed after preflight: " + str(live))
            if row["planned_action"] == "unchanged":
                continue
            if row["planned_action"] == "remove":
                live.unlink()  # Only the exact known after-hash harness; no recursive deletion or .meta edits.
            else:
                source = entry["before_path"] if args.action == "rollback" else entry["after_path"]
                atomic_write(live, within(root, source).read_bytes())
            expected = entry["before_sha256"] if args.action == "rollback" else entry["after_sha256"]
            if sha(live) != expected:
                raise IOError("Post-write verification failed: " + str(live))
        receipt["applied"] = True
    print(json.dumps(receipt, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
