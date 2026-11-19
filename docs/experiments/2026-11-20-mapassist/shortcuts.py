"""Migrate the 11/19 vehicle user's shortcuts. Run without arguments to preview."""
import argparse
import fcntl
import hashlib
import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
# SHA256 of the four identical launchers installed by the original 1120 prepare.sh.
PREVIOUS_1120_DRIVER = "b8ad579bc753094cc28f9b08d157810f90426e4244fc6a5d75b2e7286eda30b9"
COMMANDS = ("allnodes", "yolop", "mapassist", "lya")


def migrate(home, desktop, apply, check=False):
    bin_dir = home / ".local/bin"
    old = json.loads((HERE / "legacy-sha256.json").read_text())
    runtime = home / "aiformula_runtime/2026-11-20-mapassist-av314"
    selector = home / "aiformula_runtime/2026-11-19-yolop-selection/yolop.launch"
    driver = (HERE / "vehicle-command").read_bytes()
    wanted = {bin_dir / name: driver for name in COMMANDS}
    for i, name in enumerate(COMMANDS, start=1):
        entry = ("[Desktop Entry]\nType=Application\nVersion=1.0\n"
                 f"Name=1120 {i} {name}\nExec=\"{bin_dir / name}\"\n"
                 "Terminal=true\nIcon=utilities-terminal\nCategories=Utility;\n")
        wanted[desktop / f"1120-{name}.desktop"] = entry.encode()
    old_paths = {bin_dir / name for name in old if not name.endswith(".desktop")}
    old_paths.update([bin_dir / "yolop", bin_dir / "yolop-launch"])
    menu = Path(os.environ.get("XDG_DATA_HOME") or home / ".local/share") / "applications"
    desktops = {desktop, home / "Desktop", home / ".xdg-hidden/Desktop", home / ".local/share/applications", menu}
    # Identify extra copies by content, including a 1120 desktop moved to another XDG directory.
    desktop_hashes = {value for key, value in old.items() if key.endswith(".desktop")}
    desktop_hashes.update(hashlib.sha256(data).hexdigest()
                          for path, data in wanted.items() if path.suffix == ".desktop")
    command_hashes = {value for key, value in old.items() if not key.endswith(".desktop")}
    command_hashes.add(PREVIOUS_1120_DRIVER)
    command_hashes.add(hashlib.sha256(driver).hexdigest())
    if bin_dir.is_dir():
        for path in bin_dir.iterdir():
            # The recorded launchers are small scripts. Avoid hashing unrelated executable packages.
            if path.is_file() and path.stat().st_size <= 65536:
                if ((path.is_symlink() and path.resolve() == selector.resolve())
                        or hashlib.sha256(path.read_bytes()).hexdigest() in command_hashes):
                    old_paths.add(path)
    for directory in desktops:
        if directory.is_dir():
            old_paths.update(directory.glob("*.desktop"))
    remove, create = [], {}
    for path in sorted(old_paths | set(wanted)):
        path.parent.resolve().relative_to(home)
        exists = os.path.lexists(path)
        if path.is_file() and not path.is_symlink() and path in wanted:
            if path.read_bytes() == wanted[path] and os.access(path, os.X_OK):
                continue
        known = False
        if path.parent == bin_dir and path.name in COMMANDS and path.is_file() and not path.is_symlink():
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            known = digest in command_hashes
        elif path.parent == bin_dir and path.is_symlink() and path.resolve() == selector.resolve():
            known = True
        elif path == bin_dir / "yolop" and path.is_symlink() and os.readlink(path) == "yolop-launch":
            known = True
        elif path.is_file() and (path.suffix == ".desktop" or path.parent == bin_dir):
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            if path.suffix == ".desktop":
                known = digest in desktop_hashes
            else:
                known = digest in command_hashes
        if known:
            remove.append(path)
        elif exists and path in wanted:
            raise SystemExit(f"Unrecognized shortcut occupies {path}; inspect before installing.")
        elif exists and path.name in ("Concept.desktop", "concept-live"):
            # Unknown content at a former Concept path belongs to the current Concept deployment.
            print("preserve Concept", path)
        elif exists and (path.name in old or path in (bin_dir / "yolop-launch", menu / "aiformula-yolop.desktop")):
            raise SystemExit(f"Changed 11/19 entry: {path}; inspect before migrating.")
        if path in wanted:
            create[path] = wanted[path]
    for path in remove:
        print("archive", path)
    for path in create:
        print("install", path)
    if check and (remove or create):
        raise SystemExit("Shortcut migration incomplete; see the entries above.")
    if not apply or not (remove or create):
        return
    if not selector.is_file() or not (runtime / "install/local_setup.bash").is_file():
        raise SystemExit("Missing 11/19 YOLOP runtime or 11/20 build. See README.md.")
    backup = Path(tempfile.mkdtemp(prefix="shortcut-backup-", dir=str(runtime)))
    moved, added = [], []
    try:
        for path in remove:
            target = backup / path.relative_to(home)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(path), str(target))
            moved.append((path, target))
        for path, content in create.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("xb") as handle:
                added.append(path)
                handle.write(content)
            path.chmod(0o755)
    except BaseException:
        for path in reversed(added):
            path.unlink()
        for path, target in reversed(moved):
            shutil.move(str(target), str(path))
        raise
    print("Backup:", backup)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--apply", action="store_true")
    mode.add_argument("--check", action="store_true", help="exit nonzero for missing or stale shortcuts")
    args = parser.parse_args()
    home = Path.home().resolve()
    desktop = Path(subprocess.check_output(["xdg-user-dir", "DESKTOP"], text=True).strip()).resolve()
    desktop.relative_to(home)
    if desktop == home:
        raise SystemExit("XDG desktop is disabled; select a desktop directory first.")
    lockdir = home / ".cache"
    lockdir.mkdir(exist_ok=True)
    with (lockdir / "aiformula-1120-shortcuts.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        migrate(home, desktop, args.apply, check=args.check)
