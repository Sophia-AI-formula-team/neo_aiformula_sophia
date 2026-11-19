"""Preview or overlay the pinned A4 update, without starting ROS."""
import argparse
from contextlib import ExitStack
import datetime
import hashlib
import os
from pathlib import Path
import stat
import subprocess
import tempfile

COMMIT = "220f8cba0b96c9bb9ac2873b2685c91da2277e03"
PACKAGE = "workspace/src/aiformula/perception/road_detector/road_detector/YOLOP_A3_Deploy"
WEIGHT = "b4d3556ded29ada232682b607a11296d723ffe74814a85cc4f2bcbf573e5b1e3"
PROFILES = ("A4", "A4_lite")
FILES = {
    "color_semantics.py": ("646b9ee52642f730379cc7067cec91c7d4e0fcf27c3378650782345b1f965173",
                           "5cf63086409f66515215091eb74d9b1ba1e5a7dc6e2ff431c933718d2fc48c0b"),
    "shared_fcos.py": ("4eb4ad6aa193014f3300f0f2c013b2d5a274c9d73ffea947febe8f098b47ca48",
                       "50e24bf8a1289b77892d437a37e27f498c0d9c0a9b8063dfa065e077cc9fdeaa"),
    "shared_runtime.py": ("c135ac385a5877f2477b389b21a95adf7c01ced95bcbac73d3106c2bd21d377e",
                          "0f3fd8db590aa1dfaaa3ebbccea2a1fa037833b237d7a130201d371b8991f2bc"),
}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def plan(checkout, root):
    changes = []
    for profile in PROFILES:
        bundle = root / "bundles" / profile
        weight = bundle / "weights/shared_best.pth"
        if weight.resolve() != weight or digest(weight.read_bytes()) != WEIGHT:
            raise ValueError("Unexpected A4 deployment weights: " + str(weight))
        for name, (old_sha, new_sha) in FILES.items():
            path = bundle / "yolop_a3_runtime" / name
            if path.resolve() != path or not path.is_file():
                raise ValueError("Not an ordinary installed runtime file: " + str(path))
            before = path.read_bytes()
            new = subprocess.check_output(["git", "-C", str(checkout), "show",
                                           COMMIT + ":" + PACKAGE + "/yolop_a3_runtime/" + name])
            if digest(new) != new_sha or digest(before) not in (old_sha, new_sha):
                raise ValueError("Unexpected source or installed content: " + str(path))
            if before != new:
                changes.append((path, before, new, stat.S_IMODE(path.stat().st_mode)))
    return changes


def replace(path, data, mode):
    fd, name = tempfile.mkstemp(dir=path.parent, prefix="yolop-")
    temp = Path(name)
    try:
        with os.fdopen(fd, "wb") as out:
            out.write(data)
            out.flush()
            os.fsync(out.fileno())
        temp.chmod(mode)
        os.replace(temp, path)
    finally:
        if temp.exists():
            temp.unlink()


def apply(root, changes):
    if not changes:
        return
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup = root / ("a4-before-1120-" + stamp)
    backup.mkdir()
    for path, before, _, mode in changes:
        copy = backup / path.relative_to(root)
        copy.parent.mkdir(parents=True, exist_ok=True)
        copy.write_bytes(before)
        copy.chmod(mode)
    completed = []
    try:
        for path, before, new, mode in changes:
            if path.read_bytes() != before:
                raise RuntimeError("File changed after backup: " + str(path))
            replace(path, new, mode)
            completed.append((path, before, mode))
    except BaseException:
        for path, before, mode in reversed(completed):
            replace(path, before, mode)
        raise
    print("Backup:", backup)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--checkout", required=True, type=Path,
                   help="neo clone that contains the pinned A4 commit")
    p.add_argument("--apply", action="store_true")
    args = p.parse_args()
    root = (Path.home() / "aiformula_runtime/2026-11-19-yolop-selection").resolve(strict=True)
    with ExitStack() as locks:
        if args.apply:
            import fcntl
            domain = os.environ.get("ROS_DOMAIN_ID", "0") or "0"
            if not domain.isdecimal():
                raise ValueError("Invalid ROS_DOMAIN_ID")
            for path in sorted(set(root.glob("selector-*.lock")) | {root / ("selector-" + domain + ".lock")}):
                if path.resolve() != path:
                    raise ValueError("Unexpected selector lock path")
                lock = locks.enter_context(path.open("a+"))
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            for process in Path("/proc").glob("[0-9]*/cmdline"):
                try:
                    argv = process.read_bytes().split(b"\0")
                except (FileNotFoundError, ProcessLookupError, PermissionError):
                    continue
                if any(v.endswith(b"/run_vehicle.py") and os.fsencode(root) in v for v in argv):
                    raise RuntimeError("Stop the active A4 bundle before updating")
        changes = plan(args.checkout, root)
        for path, _, _, _ in changes:
            print(path)
        print("Files requiring update:", len(changes))
        if args.apply:
            apply(root, changes)


if __name__ == "__main__":
    main()
