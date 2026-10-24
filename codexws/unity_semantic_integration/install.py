"""Byte-exact, scoped deployment of the frozen final63 Unity integration.

Read-only by default. Requires Python 3.9+ and Git with both pinned commits.
No downloads, inference, Unity operation, recursive deletion, or force mode.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import subprocess
import sys
import tempfile


FORMAT = "neo-unity-semantic-archive-v1"
BEFORE = "5bc52e05329d7fe642f219d32a578d04d2b7f668"
AFTER = "63eec2da9f9d2ef90e1db29ce2c6c0b3c1a18a96"
NATIVE = "workspace/src/aiformula/perception/semantic_planning_system/"
VENDORED = "Assets/StreamingAssets/SemanticPlanning/source/"
ALLOWED = ("Assets/AIFormula/Editor/Validation/AIFormulaSemanticTrafficTrial.cs",
           "Assets/AIFormula/Runtime/Control/AIFormulaSemanticPlanning.cs",
           "Assets/AIFormula/Runtime/Control/AIFormulaVisionLaneFollower.cs",
           "Assets/AIFormula/Runtime/Perception/AIFormulaRoadDetector.cs")
STREAMING = ("Assets/StreamingAssets/RoadDetector/",
             "Assets/StreamingAssets/VisionLaneFollower/",
             "Assets/StreamingAssets/SemanticPlanning/")


def digest(data):
    return hashlib.sha256(data).hexdigest()


def safe_name(value):
    if not isinstance(value, str):
        raise ValueError("A relative path must be a string")
    name = PurePosixPath(value)
    if (not value or "\\" in value or ":" in value or name.is_absolute()
            or any(p in ("", ".", "..") for p in value.split("/"))):
        raise ValueError("Unsafe relative path: " + value)
    return value


def inside(root, name):
    name = safe_name(name)
    root = root.resolve()
    current = root
    for part in name.split("/"):
        current = current / part
        # is_junction is available in Python 3.12+. On Windows older Python
        # still rejects a junction resolving outside the chosen root below.
        if current.is_symlink() or (hasattr(current, "is_junction") and current.is_junction()):
            raise ValueError("Linked path refused: " + str(current))
    if not current.resolve().is_relative_to(root):
        raise ValueError("Path escapes root: " + name)
    return current


def sha_file(path):
    if not path.exists():
        return None
    if not path.is_file():
        raise ValueError("Expected an ordinary file: " + str(path))
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(chunk)
    return sha.hexdigest()


def managed_name(name):
    safe_name(name)
    lower = name.lower()
    if "laneteachrepeat" in lower or "/upstream/" in lower or "/a3-v14/" in lower:
        raise ValueError("Protected source cannot be managed: " + name)
    if not (name in ALLOWED or name.startswith(STREAMING)):
        raise ValueError("Outside integration write scope: " + name)
    base = name[:-5] if name.endswith(".meta") else name
    if PurePosixPath(base).suffix.lower() in {
        ".pt", ".pth", ".onnx", ".engine", ".dll", ".exe", ".pyc", ".zip",
        ".png", ".jpg", ".jpeg", ".mp4", ".fbx", ".unity", ".asset"}:
        raise ValueError("Binary/scene/settings payload refused: " + name)


def load_manifest(bundle):
    raw = inside(bundle, "manifest.json").read_bytes()
    expected = inside(bundle, "manifest.sha256").read_text(encoding="ascii").split()[0]
    if digest(raw) != expected:
        raise ValueError("Manifest checksum mismatch")
    manifest = json.loads(raw)
    if (manifest.get("format") != FORMAT
            or manifest.get("before_semantic_commit") != BEFORE
            or manifest.get("frozen_semantic_commit") != AFTER):
        raise ValueError("Wrong archive format or pinned revisions")
    seen = set()
    for row in manifest["files"]:
        name = row["path"]
        managed_name(name)
        if name.casefold() in seen:
            raise ValueError("Duplicate/case-aliased managed path: " + name)
        seen.add(name.casefold())
        for side in ("before", "after"):
            sha = row[side + "_sha256"]
            if sha is not None and not re.fullmatch("[0-9a-f]{64}", sha):
                raise ValueError("Invalid payload checksum")
            if (sha is None) != (row[side] is None):
                raise ValueError("Absent payload recipe/hash disagreement")
    for name, sha in manifest["prerequisites"].items():
        safe_name(name)
        if name.casefold() in seen or not re.fullmatch("[0-9a-f]{64}", sha):
            raise ValueError("Invalid/overlapping prerequisite: " + name)
        seen.add(name.casefold())
    return manifest


def payload(bundle, repo, row, side):
    recipe = row[side]
    if recipe is None:
        return None
    name = row["path"]
    if recipe["kind"] == "file":
        expected_path = ("before/" if side == "before" else "overlay/") + name
        if recipe["path"] != expected_path:
            raise ValueError("Payload file path mismatch: " + name)
        data = inside(bundle, expected_path).read_bytes()
    elif recipe["kind"] == "git_blob":
        if not name.startswith(VENDORED) or name.endswith(".meta"):
            raise ValueError("Git recipe is restricted to native semantic source")
        if recipe["commit"] != (BEFORE if side == "before" else AFTER):
            raise ValueError("Unexpected Git revision")
        path = safe_name(recipe["path"])
        if path != NATIVE + name[len(VENDORED):]:
            raise ValueError("Native source mapping mismatch")
        result = subprocess.run(
            ["git", "--no-replace-objects", "-C", str(repo), "cat-file", "blob",
             recipe["commit"] + ":" + path], capture_output=True, check=False)
        if result.returncode:
            raise ValueError("Required Git blob unavailable: " + recipe["commit"] + ":" + path
                             + "; use a full clone containing both pinned commits")
        data = result.stdout.replace(b"\r\n", b"\n")
        if b"\r" in data or b"\0" in data:
            raise ValueError("Unexpected native encoding: " + path)
        if recipe["newline"] == "crlf":
            data = data.replace(b"\n", b"\r\n")
        elif recipe["newline"] != "lf":
            raise ValueError("Unknown explicit newline policy")
    else:
        raise ValueError("Unknown payload recipe")
    if digest(data) != row[side + "_sha256"]:
        raise ValueError("Payload byte mismatch: " + side + "/" + name)
    return data


def execute(bundle, repo, simulator, action, write=False, unity_closed=False):
    writing = action in ("apply", "rollback")
    if writing and not (write and unity_closed):
        raise ValueError("Writes require --write --unity-closed after closing Unity")
    if not writing and (write or unity_closed):
        raise ValueError("Write flags are not accepted for read-only actions")
    manifest = load_manifest(bundle)
    # Materialize and hash ALL payloads before any destination check or write.
    data = {(row["path"], side): payload(bundle, repo, row, side)
            for row in manifest["files"] for side in ("before", "after")}
    if action == "verify-payload":
        return dict(ok=True, action=action, managed_files=len(manifest["files"]),
                    payload_sides=sum(v is not None for v in data.values()), writes=0)
    if simulator is None or not (simulator / "Assets").is_dir():
        raise ValueError("--simulator must identify an existing project containing Assets")
    current_side = "before" if action in ("apply", "verify-before") else "after"
    desired_side = "after" if action == "apply" else "before"
    errors, states = [], []
    for row in manifest["files"]:
        name = row["path"]
        actual = sha_file(inside(simulator, name))
        state = ("before" if actual == row["before_sha256"] else
                 "after" if actual == row["after_sha256"] else "drift")
        states.append(dict(path=name, state=state, actual_sha256=actual))
        if action != "inspect" and actual != row[current_side + "_sha256"]:
            errors.append(name + ": required " + current_side + ", found " + state)
    for name, expected in manifest["prerequisites"].items():
        if sha_file(inside(simulator, name)) != expected:
            errors.append(name + ": unchanged prerequisite differs or is missing")
    if action == "inspect":
        return dict(ok=not errors and all(s["state"] != "drift" for s in states),
                    action=action, writes=0, states=states, errors=errors,
                    note="A mix of before/after is reportable but apply/rollback refuses it")
    if errors:
        return dict(ok=False, action=action, writes=0, errors=errors)
    if not writing:
        return dict(ok=True, action=action, writes=0, managed_files=len(states),
                    prerequisites=len(manifest["prerequisites"]))
    completed = []
    try:
        for row in manifest["files"]:
            name = row["path"]
            target = inside(simulator, name)
            if sha_file(target) != row[current_side + "_sha256"]:
                raise RuntimeError("Destination changed after full preflight: " + name)
            desired = data[(name, desired_side)]
            if desired is None:
                target.unlink()  # Exactly one listed, hash-checked added file.
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                fd, temporary = tempfile.mkstemp(prefix=".semantic-", suffix=".tmp", dir=target.parent)
                try:
                    with os.fdopen(fd, "wb") as stream:
                        stream.write(desired)
                    if sha_file(inside(simulator, name)) != row[current_side + "_sha256"]:
                        raise RuntimeError("Destination raced before replacement: " + name)
                    os.replace(temporary, target)
                finally:
                    if os.path.exists(temporary):
                        os.unlink(temporary)
            if sha_file(inside(simulator, name)) != row[desired_side + "_sha256"]:
                raise RuntimeError("Post-write byte check failed: " + name)
            completed.append(name)
    except Exception as error:
        return dict(ok=False, action=action, writes=len(completed), completed=completed,
                    error=str(error), recovery="Stop; use inspect and review exact hashes. No force mode.")
    return dict(ok=True, action=action, writes=len(completed), completed=completed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--simulator", type=Path)
    parser.add_argument("--action", default="verify-payload", choices=(
        "verify-payload", "verify-before", "verify-after", "inspect", "apply", "rollback"))
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--unity-closed", action="store_true")
    args = parser.parse_args()
    try:
        result = execute(args.bundle.resolve(), args.repo.resolve(),
                         args.simulator.resolve() if args.simulator else None,
                         args.action, args.write, args.unity_closed)
    except Exception as error:
        result = dict(ok=False, writes=0, error=type(error).__name__ + ": " + str(error))
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 2


if __name__ == "__main__":
    sys.exit(main())
