"""Byte-exact overlay for the separate Unity left-turn investigation archive.

Read-only by default. Python 3.9+. No Git reads, downloads, Unity operation,
recursive removal, force replacement or modification of vendored semantic core.
Adapted from ../unity_semantic_integration/install.py under the repository's
GNU AGPL v3 license; see ../../LICENSE. The original archive remains frozen.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import stat
import sys
import tempfile


FORMAT = "neo-unity-left-turn-investigation-v1"
BEFORE = AFTER = "63eec2da9f9d2ef90e1db29ce2c6c0b3c1a18a96"
BASE = "434fdc43d00399808adf7e548d491a5dfdc5bffa"
ALLOWED = frozenset((
    "Assets/AIFormula/Runtime/Control/AIFormulaSemanticPlanning.cs",
    "Assets/AIFormula/Runtime/Control/AIFormulaVisionLaneFollower.cs",
    "Assets/AIFormula/Runtime/Perception/AIFormulaRoadDetector.cs",
    "Assets/AIFormula/Editor/Validation/AIFormulaSemanticLeftTrial.cs",
    "Assets/AIFormula/Editor/Validation/AIFormulaSemanticLeftTrial.cs.meta",
    "Assets/StreamingAssets/VisionLaneFollower/upstream_adapter.py",
    "Assets/StreamingAssets/SemanticPlanning/semantic_worker.py",
    "Assets/StreamingAssets/SemanticPlanning/fusion_snapshot_capture.py",
    "Assets/StreamingAssets/SemanticPlanning/fusion_snapshot_capture.py.meta",
    "Assets/StreamingAssets/SemanticPlanning/calibrated_topology.py",
    "Assets/StreamingAssets/SemanticPlanning/calibrated_transport.py",
    "Assets/StreamingAssets/SemanticPlanning/continuous_detour_tracker.py",
    "Assets/StreamingAssets/SemanticPlanning/unity_east_detour_inner_v1.json",
    "Assets/StreamingAssets/SemanticPlanning/unity_east_detour_outer_v1.json",
))


def digest(data):
    return hashlib.sha256(data).hexdigest()


def safe_name(value):
    if not isinstance(value, str):
        raise ValueError("Relative path must be a string")
    parts = value.split("/")
    if (not value or "\\" in value or ":" in value or PurePosixPath(value).is_absolute()
            or any(ord(c) < 32 or ord(c) == 127 for c in value)
            or any(p in ("", ".", "..") or p.endswith((" ", "."))
                   or PureWindowsPath(p).is_reserved() for p in parts)):
        raise ValueError("Unsafe relative path: " + repr(value))
    return value


def _unlinked(path):
    try:
        info = path.lstat()
    except FileNotFoundError:
        return
    # Reject Windows junctions/reparse points even on Python versions without
    # Path.is_junction(), and ordinary file hardlinks on both Windows and POSIX.
    if (stat.S_ISLNK(info.st_mode)
            or getattr(info, "st_file_attributes", 0) & getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)
            or (stat.S_ISREG(info.st_mode) and info.st_nlink > 1)):
        raise ValueError("Linked path refused: " + str(path))


def unlinked_root(root):
    root = Path(os.path.abspath(os.fspath(root)))
    for entry in reversed((root,) + tuple(root.parents)):
        _unlinked(entry)
    if not root.is_dir():
        raise ValueError("Root must be an existing ordinary directory: " + str(root))
    return root


def inside(root, name):
    root = unlinked_root(root)
    current = root
    for part in safe_name(name).split("/"):
        current = current / part
        _unlinked(current)
    if not current.resolve().is_relative_to(root.resolve()):
        raise ValueError("Path escapes selected root")
    return current


def sha_file(path):
    _unlinked(path)
    if not path.exists():
        return None
    if not path.is_file():
        raise ValueError("Expected an ordinary file: " + str(path))
    checksum = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(chunk)
    return checksum.hexdigest()


def managed_name(name):
    if safe_name(name) not in ALLOWED:
        raise ValueError("Outside exact investigation write allowlist: " + name)


def _unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON key: " + key)
        result[key] = value
    return result


def load_manifest(bundle):
    raw = inside(bundle, "manifest.json").read_bytes()
    if len(raw) > 8 * 1024 * 1024:
        raise ValueError("Manifest exceeds size limit")
    checksum = inside(bundle, "manifest.sha256").read_text(encoding="ascii").split()[0]
    if not re.fullmatch("[0-9a-f]{64}", checksum) or digest(raw) != checksum:
        raise ValueError("Manifest checksum mismatch")
    manifest = json.loads(raw, object_pairs_hook=_unique)
    if (manifest.get("format") != FORMAT or manifest.get("before_semantic_commit") != BEFORE
            or manifest.get("frozen_semantic_commit") != AFTER or manifest.get("integration_base_commit") != BASE):
        raise ValueError("Wrong archive format or frozen revisions")
    rows = manifest.get("files")
    if not isinstance(rows, list):
        raise ValueError("Manifest files must be a list")
    seen = set()
    for row in rows:
        name = row["path"]
        managed_name(name)
        if name.casefold() in seen:
            raise ValueError("Duplicate/case-aliased managed path: " + name)
        seen.add(name.casefold())
        for side in ("before", "after"):
            sha, recipe = row[side + "_sha256"], row[side]
            if sha is not None and (not isinstance(sha, str) or not re.fullmatch("[0-9a-f]{64}", sha)):
                raise ValueError("Invalid payload SHA-256")
            if (sha is None) != (recipe is None):
                raise ValueError("Absent payload recipe/hash disagreement")
            if recipe is not None:
                expected = ("before/" if side == "before" else "overlay/") + name
                if recipe != {"kind": "file", "path": expected}:
                    raise ValueError("Only exact file payload recipes are permitted")
        if row["before"] is None and row["after"] is None:
            raise ValueError("A managed row cannot be absent on both sides")
    if seen != {name.casefold() for name in ALLOWED}:
        raise ValueError("Manifest must enumerate the entire fixed 14-file overlay")
    prerequisites = manifest.get("prerequisites")
    if not isinstance(prerequisites, dict) or not prerequisites:
        raise ValueError("Unchanged baseline prerequisites are required")
    for name, sha in prerequisites.items():
        safe_name(name)
        if (name.casefold() in seen or not isinstance(sha, str) or not re.fullmatch("[0-9a-f]{64}", sha)):
            raise ValueError("Duplicate/overlapping/invalid prerequisite: " + name)
        seen.add(name.casefold())
    return manifest


def payload(bundle, row, side):
    recipe = row[side]
    if recipe is None:
        return None
    data = inside(bundle, recipe["path"]).read_bytes()
    if digest(data) != row[side + "_sha256"]:
        raise ValueError("Payload bytes differ: " + side + "/" + row["path"])
    return data


def _prerequisite_errors(simulator, manifest):
    return [name + ": unchanged prerequisite differs or is missing"
            for name, expected in manifest["prerequisites"].items()
            if sha_file(inside(simulator, name)) != expected]


def execute(bundle, simulator=None, action="verify-payload", write=False, unity_closed=False):
    if action not in ("verify-payload", "verify-before", "verify-after", "inspect", "apply", "rollback"):
        raise ValueError("Unknown action")
    writing = action in ("apply", "rollback")
    if writing and not (write and unity_closed):
        raise ValueError("Writes require --write --unity-closed after closing Unity")
    if not writing and (write or unity_closed):
        raise ValueError("Write flags are not accepted for read-only actions")
    manifest = load_manifest(bundle)
    # ALL before/after payloads are checked and materialized before any write.
    data = {(row["path"], side): payload(bundle, row, side)
            for row in manifest["files"] for side in ("before", "after")}
    if action == "verify-payload":
        return dict(ok=True, action=action, writes=0, managed_files=len(manifest["files"]),
                    payload_sides=sum(value is not None for value in data.values()))
    if simulator is None or not inside(simulator, "Assets").is_dir():
        raise ValueError("--simulator must be an existing project with ordinary Assets")
    current_side = "before" if action in ("apply", "verify-before") else "after"
    desired_side = "after" if action == "apply" else "before"
    errors, states = [], []
    for row in manifest["files"]:
        actual = sha_file(inside(simulator, row["path"]))
        matches = [side for side in ("before", "after") if actual == row[side + "_sha256"]]
        state = "both" if len(matches) == 2 else matches[0] if matches else "drift"
        states.append(dict(path=row["path"], state=state, actual_sha256=actual))
        if action != "inspect" and current_side not in matches:
            errors.append(row["path"] + ": requires " + current_side + ", found " + state)
    errors.extend(_prerequisite_errors(simulator, manifest))
    mixed = {row["state"] for row in states} >= {"before", "after"}
    if action == "inspect":
        return dict(ok=not errors and not mixed and all(row["state"] != "drift" for row in states),
                    action=action, writes=0, mixed=mixed, states=states, errors=errors,
                    note="Mixed/partial states require review; there is no force or resume action")
    if errors:
        return dict(ok=False, action=action, writes=0, errors=errors)
    if not writing:
        return dict(ok=True, action=action, writes=0, managed_files=len(states),
                    prerequisites=len(manifest["prerequisites"]))
    completed, changed, unchanged = [], [], []
    in_progress = None
    try:
        for row in manifest["files"]:
            name = in_progress = row["path"]
            target = inside(simulator, name)
            if sha_file(target) != row[current_side + "_sha256"]:
                raise RuntimeError("Destination changed after full preflight: " + name)
            if _prerequisite_errors(simulator, manifest):
                raise RuntimeError("Prerequisite changed after full preflight")
            desired = data[(name, desired_side)]
            if row[current_side + "_sha256"] == row[desired_side + "_sha256"]:
                unchanged.append(name)
                continue
            if desired is None:
                target = inside(simulator, name)
                if sha_file(target) != row[current_side + "_sha256"]:
                    raise RuntimeError("Destination raced before exact-file deletion: " + name)
                target.unlink()
                changed.append(name)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                target = inside(simulator, name)
                fd, temporary = tempfile.mkstemp(prefix=".semantic-left-", suffix=".tmp", dir=target.parent)
                try:
                    with os.fdopen(fd, "wb") as stream:
                        stream.write(desired)
                        stream.flush()
                        os.fsync(stream.fileno())
                    target = inside(simulator, name)
                    if sha_file(target) != row[current_side + "_sha256"]:
                        raise RuntimeError("Destination raced before atomic replacement: " + name)
                    os.replace(temporary, target)
                    changed.append(name)
                finally:
                    if os.path.lexists(temporary):
                        os.unlink(temporary)
            if sha_file(inside(simulator, name)) != row[desired_side + "_sha256"]:
                raise RuntimeError("Post-write byte check failed: " + name)
            completed.append(name)
            in_progress = None
    except BaseException as error:
        return dict(ok=False, action=action, writes=len(changed), completed=completed, changed=changed,
                    unchanged=unchanged, in_progress=in_progress, error=type(error).__name__ + ": " + str(error),
                    recovery="Stop; retain this receipt, inspect exact hashes and review recovery. No force mode.")
    return dict(ok=True, action=action, writes=len(changed), completed=completed, changed=changed,
                unchanged=unchanged, note="Per-file atomic replacement; the multi-file overlay is not a transaction")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=Path(__file__).absolute().parent)
    parser.add_argument("--simulator", type=Path)
    parser.add_argument("--action", default="verify-payload", choices=(
        "verify-payload", "verify-before", "verify-after", "inspect", "apply", "rollback"))
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--unity-closed", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = execute(args.bundle, args.simulator, args.action, args.write, args.unity_closed)
    except Exception as error:
        result = dict(ok=False, writes=0, error=type(error).__name__ + ": " + str(error))
    # This stdout JSON is the completed-path receipt, including partial failures.
    # Capture it outside the Simulator; the installer writes no unrelated logs.
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 2


if __name__ == "__main__":
    sys.exit(main())
