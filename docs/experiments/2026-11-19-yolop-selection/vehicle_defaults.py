"""Apply or roll back this experiment's two local launcher defaults."""
import argparse
from datetime import datetime
from pathlib import Path
import shutil
import tempfile
import os

ROOT = Path(__file__).resolve().parent
LAUNCHERS = Path("/home/workspace/src/aiformula/launchers/launch")
NAMES = ("allnodes.launch.py", "allnodes_2.launch.py")
OLD = b'DeclareLaunchArgument("launch_yolop_a3", default_value="true",'
NEW = OLD.replace(b'"true"', b'"false"')


def replace(path, data):
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as temp:
        staging = Path(temp.name)
    try:
        shutil.copystat(path, staging)
        staging.write_bytes(data)
        os.replace(staging, path)
    finally:
        if staging.exists():
            staging.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("apply", "rollback"))
    mode = parser.parse_args().mode
    backups = ROOT / "before"
    backups.mkdir(exist_ok=True)
    edits = []
    for name in NAMES:
        path = LAUNCHERS / name
        current = path.read_bytes()
        backup = backups / name
        if mode == "apply" and not backup.exists():
            if current.count(OLD) != 1:
                raise SystemExit(f"Expected one enabled A3 default in {path}")
            shutil.copy2(path, backup)
        original = backup.read_bytes()
        if original.count(OLD) != 1:
            raise SystemExit(f"Invalid rollback copy: {backup}")
        changed = original.replace(OLD, NEW)
        if current not in (original, changed):
            raise SystemExit(f"Launcher changed outside the experiment: {path}")
        wanted = changed if mode == "apply" else original
        if current != wanted:
            edits.append((path, current, wanted))
    applied = []
    try:
        for path, prior, wanted in edits:
            replace(path, wanted)
            applied.append((path, prior))
    except Exception:
        for path, prior in reversed(applied):
            replace(path, prior)
        raise
    with (ROOT / "timeline.log").open("a") as log:
        print(datetime.now().astimezone().isoformat(), mode,
              ", ".join(path.name for path, _ in applied) or "already set", file=log)
    print(f"{mode}: {len(applied)} launcher(s) changed")


if __name__ == "__main__":
    main()
