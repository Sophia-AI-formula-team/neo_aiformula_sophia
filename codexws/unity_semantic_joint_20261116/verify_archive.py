"""Read-only archive integrity check; --staged also checks actual Git index blobs."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import subprocess


def digest(data):
    return hashlib.sha256(data).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--staged", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    repo = root.parent.parent
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    entries = {}
    for group in ("archive_files", "evidence_files", "documentation_files"):
        for relative, expected in manifest[group].items():
            if relative in entries:
                raise ValueError("Duplicate manifest path: " + relative)
            entries[relative] = expected
    actual = {p.relative_to(root).as_posix() for p in root.rglob("*") if p.is_file()}
    if actual != set(entries) | {"manifest.json"}:
        raise ValueError("Inventory mismatch: " + str(actual ^ (set(entries) | {"manifest.json"})))
    scripts = 0
    for relative, expected in entries.items():
        path = (root / relative).resolve()
        if root not in path.parents or digest(path.read_bytes()) != expected:
            raise ValueError("Archive hash/path mismatch: " + relative)
        if path.suffix == ".py":
            ast.parse(path.read_text(encoding="utf-8-sig"), filename=relative)
            scripts += 1
    if len(manifest["unity_files"]) != 6:
        raise ValueError("Final deployment must contain six files")
    for item in manifest["unity_files"]:
        if entries[item["after_path"]] != item["after_sha256"]:
            raise ValueError("Payload pin mismatch: " + item["path"])
        if item["before_path"] and entries[item["before_path"]] != item["before_sha256"]:
            raise ValueError("Rollback pin mismatch: " + item["path"])
    if (root / "overlay/Assets/StreamingAssets/RoadDetector/signal_model_identity.py").exists():
        raise ValueError("Retired helper must not be an active payload")
    paths = sorted((root / rel).relative_to(repo).as_posix() for rel in actual)
    listing = "\n".join(paths) + "\n"
    def git(*arguments, **kwargs):
        return subprocess.run(["git", *arguments], cwd=repo, check=True, capture_output=True, **kwargs).stdout
    # Default Git clean filtering must preserve exactly the same bytes as --no-filters.
    filtered = git("hash-object", "--stdin-paths", input=listing.encode()).splitlines()
    raw = git("hash-object", "--no-filters", "--stdin-paths", input=listing.encode()).splitlines()
    if filtered != raw or len(raw) != len(paths):
        raise ValueError("Git clean filters would alter archived bytes")
    if args.staged:
        for relative in paths:
            blob = git("show", ":" + relative)
            if blob != (repo / relative).read_bytes():
                raise ValueError("Staged Git blob differs: " + relative)
    print(json.dumps({"read_only": True, "files": len(actual), "manifest_entries": len(entries),
                      "python_ast_passed": scripts, "unity_payloads": 6,
                      "all_sha256_match": True, "git_clean_filter_bytes_unchanged": True,
                      "actual_staged_blobs_checked": args.staged}, indent=2))


if __name__ == "__main__":
    main()
