#!/usr/bin/env python3
"""Download and verify a category or one 2026-11-19 ROS bag using Python 3."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import tarfile
import tempfile
from urllib.request import Request, urlopen

def digest(path):
    checksum = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(block)
    return checksum.hexdigest()

def matches(path, expected_size, expected_hash):
    return path.is_file() and path.stat().st_size == expected_size and digest(path) == expected_hash

def restore(record, destination, asset_dir):
    relative = Path(record["directory"])
    folder = destination / relative
    folder.mkdir(parents=True, exist_ok=True)
    expected = {item["file"]: item for item in record["files"]}
    missing = set()
    for name, item in expected.items():
        path = folder / name
        if not path.exists():
            missing.add(name)
        elif not matches(path, item["bytes"], item["sha256"]):
            raise ValueError("Existing file differs: " + str(path))
    if not missing:
        print("Verified", relative)
        return
    with tempfile.TemporaryDirectory(dir=folder.parent) as scratch:
        stage = Path(scratch)
        if asset_dir:
            package = asset_dir / record["asset"]
        else:
            package = stage / "download.tar.gz"
            request = Request(record["url"], headers={"User-Agent": "Aiformula-field-archive"})
            with urlopen(request, timeout=120) as response, package.open("wb") as output:
                shutil.copyfileobj(response, output)
        if not matches(package, record["archive_bytes"], record["archive_sha256"]):
            raise ValueError("Archive checksum mismatch: " + record["asset"])
        received = set()
        with tarfile.open(package, "r:gz") as archive:
            for member in archive:
                name = Path(member.name).name
                if (name not in expected or name in received or not member.isfile()
                        or member.name != (relative / name).as_posix()):
                    raise ValueError("Unexpected archive member: " + member.name)
                item = expected[name]
                if member.size != item["bytes"]:
                    raise ValueError("Incorrect member size: " + name)
                with archive.extractfile(member) as source, (stage / name).open("xb") as output:
                    shutil.copyfileobj(source, output)
                if not matches(stage / name, item["bytes"], item["sha256"]):
                    raise ValueError("Member checksum mismatch: " + name)
                received.add(name)
        if received != set(expected):
            raise ValueError("Incomplete recording: " + str(relative))
        for name in sorted(missing):
            try:
                os.link(stage / name, folder / name)
            except FileExistsError:
                item = expected[name]
                if not matches(folder / name, item["bytes"], item["sha256"]):
                    raise
            except OSError:
                # exFAT has no hard links. An exclusive file avoids replacing an existing copy.
                path = folder / name
                try:
                    output = path.open("xb")
                except FileExistsError:
                    item = expected[name]
                    if not matches(path, item["bytes"], item["sha256"]):
                        raise
                else:
                    try:
                        with output, (stage / name).open("rb") as source:
                            shutil.copyfileobj(source, output)
                    except BaseException:
                        path.unlink()
                        raise
    print("Restored", relative)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("selection", help="semantic, concept, other, all, or an exact session name")
    parser.add_argument("--output", type=Path, default=Path.cwd(), help="parent of the restored bag folder")
    parser.add_argument("--asset-dir", type=Path, help="use previously downloaded Release attachments")
    args = parser.parse_args()
    records = json.loads(Path(__file__).with_name("index.json").read_text(encoding="utf-8"))
    chosen = [r for r in records if args.selection in ("all", r["category"], r["session"])]
    if not chosen:
        parser.error("Unknown category or session: " + args.selection)
    for record in chosen:
        restore(record, args.output, args.asset_dir)

if __name__ == "__main__":
    main()
