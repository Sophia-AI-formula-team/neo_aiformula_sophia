"""Migrate only the Concept terminal and desktop launchers of the vehicle user."""
import argparse
from dataclasses import dataclass
import fcntl
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
from typing import Optional


HERE = Path(__file__).resolve().parent
LEGACY = json.loads((HERE.parent / "legacy-sha256.json").read_text(encoding="utf-8"))
MODE = 0o755


@dataclass(frozen=True)
class Release:
    source: Path
    version: str
    state_dir: str
    previous: Optional["Release"] = None


V6278 = Release(HERE, "V6.27.8", "concept-shortcuts-v6278")


def digest(data):
    return hashlib.sha256(data).hexdigest()


def replace_file(path, data, mode):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(dir=str(path.parent), prefix=".concept-")
    try:
        with os.fdopen(fd, "wb") as output:
            output.write(data)
            os.fchmod(output.fileno(), mode)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.lexists(temporary):
            os.unlink(temporary)


def snapshot(path):
    if not os.path.lexists(path):
        return None
    mode = path.lstat().st_mode
    if stat.S_ISLNK(mode):
        return ["symlink", os.readlink(path)]
    if not stat.S_ISREG(mode):
        raise ValueError(f"Unexpected non-file: {path}")
    return [digest(path.read_bytes()), stat.S_IMODE(mode)]


def home_path(home, relative):
    path = home / relative
    if not relative or Path(relative).is_absolute() or ".." in Path(relative).parts:
        raise ValueError(f"Invalid home-relative path: {relative}")
    path.parent.resolve().relative_to(home)
    return path


def replace_link(path, target):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=str(path.parent), prefix=".concept-") as directory:
        link = Path(directory) / "link"
        link.symlink_to(target)
        os.replace(str(link), str(path))


def old_link(path, command, command_hashes):
    if path.parent != command.parent or not path.is_symlink():
        return False
    # Match the link identity, not the bytes of an unrelated external target.
    target = Path(os.path.abspath(str(path.parent / os.readlink(path))))
    if target == command:
        return path != command
    # A site may have moved the known wrapper to a second name in ~/.local/bin.
    return (target.parent == command.parent and not target.is_symlink() and target.is_file()
            and target.stat().st_size < 65536 and digest(target.read_bytes()) in command_hashes)


def known_file(path, hashes):
    return (not path.is_symlink() and path.is_file() and path.stat().st_size < 65536
            and digest(path.read_bytes()) in hashes)


def xdg_desktop(home):
    try:
        raw = subprocess.check_output(["xdg-user-dir", "DESKTOP"], text=True).strip()
    except (FileNotFoundError, subprocess.CalledProcessError):
        standard = home / "Desktop"
        return standard if standard.is_dir() else None
    if not raw:
        return None
    value = Path(raw).resolve()
    return value if value != home else None


def desktop_entry(command, version):
    # Exec is a desktop-entry string followed by the separate Exec quoting layer.
    # Protect literal %, quotation marks and reserved shell characters in home paths.
    executable = str(command).replace("%", "%%")
    for ch in ('\\', '"', '`', '$'):
        executable = executable.replace(ch, "\\" + ch)
    executable = executable.replace("\\", "\\\\")
    return ('[Desktop Entry]\nVersion=1.0\nType=Application\nName=Concept\n'
             f'Comment=Start Concept {version} and choose a tracking controller\n'
             f'Exec="{executable}"\nIcon=applications-engineering\nTerminal=true\n'
             'Categories=Science;\n').encode("utf-8")


def accepted_hashes(command, release):
    hashes = {name: {LEGACY[name]} for name in ("concept-live", "Concept.desktop")}
    if release.previous:
        old = release.previous
        hashes["concept-live"].add(digest((old.source / "concept-live").read_bytes()))
        hashes["Concept.desktop"].add(digest(desktop_entry(command, old.version)))
    return hashes


def targets(home, desktop, menu, release, hashes):
    command = home / ".local/bin/concept-live"
    entry = desktop_entry(command, release.version)
    dirs = {menu}
    if desktop is not None and desktop != home:
        dirs.add(desktop)
    known_dirs = (home / "Desktop", home / ".xdg-hidden/Desktop", home / ".local/share/applications")
    old_runtime = home / "aiformula_runtime/2026-11-20-mapassist-av314"
    # Older MapAssist migrations moved the original desktop item out of view.
    # Recognize only the exact archived Concept bytes at these known locations.
    # Leave those backups intact so each migration can roll back independently.
    for directory in known_dirs:
        rel = (directory / "Concept.desktop").relative_to(home)
        archived = old_runtime.glob("shortcut-backup-*/" + str(rel))
        if os.path.lexists(home/rel) or any(known_file(p, hashes["Concept.desktop"]) for p in archived):
            dirs.add(directory)
    driver = (release.source / "concept-live").read_bytes()
    wanted = {command: driver, **{folder/"Concept.desktop": entry for folder in dirs}}
    reserved = {"allnodes", "yolop", "mapassist", "lya"}
    if command.parent.is_dir():
        for path in command.parent.iterdir():
            if path.name not in reserved and (old_link(path, command, hashes["concept-live"])
                                              or known_file(path, hashes["concept-live"])):
                wanted[path] = driver
    protected_icons = {"1120-"+name+".desktop" for name in reserved}
    for directory in dirs | set(known_dirs):
        if directory.is_dir():
            for path in directory.glob("*.desktop"):
                if path.name not in protected_icons and known_file(path, hashes["Concept.desktop"]):
                    wanted[path] = entry
    return {str(p.relative_to(home)): data for p, data in wanted.items()}


def validate_runtime(release):
    subprocess.run(["bash", str(release.source / "concept-live"), "--verify-install"], check=True)


def migrate(home, desktop, menu, action, release=V6278):
    state = home_path(home, f".local/state/aiformula/{release.state_dir}/current.json")
    receipt = json.loads(state.read_text(encoding="utf-8")) if state.exists() else None
    if action == "rollback" and receipt is None:
        print("No active Concept migration to roll back.")
        return
    entries = receipt["entries"] if receipt else {}
    command = home / ".local/bin/concept-live"
    hashes = accepted_hashes(command, release) if action != "rollback" else {}
    wanted = {} if action == "rollback" else targets(home, desktop, menu, release, hashes)
    if release.previous and action != "rollback":
        previous_state = home_path(home, f".local/state/aiformula/{release.previous.state_dir}/current.json")
        if previous_state.is_file():
            previous = json.loads(previous_state.read_text(encoding="utf-8"))
            for key in previous["entries"]:
                path = home_path(home, key)
                if key not in wanted and path.parent == command.parent and known_file(path, hashes["concept-live"]):
                    wanted[key] = wanted[str(command.relative_to(home))]
                elif key not in wanted and path.suffix == ".desktop" and known_file(path, hashes["Concept.desktop"]):
                    wanted[key] = desktop_entry(command, release.version)
    if receipt:
        backup = home_path(home, receipt["backup"])
        # A changed XDG location must not orphan an earlier launch entry.
        if action != "rollback":
            for key in entries.keys() - wanted.keys():
                relpath = Path(key)
                if relpath.parent == Path(".local/bin"):
                    wanted[key] = wanted[".local/bin/concept-live"]
                elif relpath.suffix == ".desktop":
                    wanted[key] = next(v for k, v in wanted.items() if k.endswith("/Concept.desktop"))
                else:
                    raise ValueError(f"Unrecognized path in Concept receipt: {key}")

    changes, additions = [], {}
    keys = sorted(entries if action == "rollback" else wanted)
    for rel in keys:
        path = home_path(home, rel)
        current = snapshot(path)
        desired = entries[rel]["installed"] if action == "rollback" else [digest(wanted[rel]), MODE]
        if rel in entries:
            old = entries[rel]["original"]
            if entries[rel]["installed"] != desired:
                raise ValueError(f"Installer payload differs from existing migration: {path}")
            if current != desired and current != old:
                raise ValueError(f"Shortcut changed after backup: {path}; preserve and inspect it.")
        else:
            old = current
            known = hashes["concept-live" if path.parent == command.parent else "Concept.desktop"]
            if current and current[0] != desired[0] and current[0] not in known and not old_link(path, command, hashes["concept-live"]):
                raise ValueError(f"Unrecognized content at {path}; inspect before replacing.")
            additions[rel] = {"original": current, "installed": desired}
        if current != (old if action == "rollback" else desired):
            changes.append(rel)

    if action == "preview":
        for rel in changes:
            print("install", home / rel)
        if not changes:
            print(f"Concept shortcuts already match {release.version}.")
        return
    if action == "check":
        if changes:
            raise ValueError("Missing or stale Concept shortcut(s): " + ", ".join(changes))
        validate_runtime(release)
        print(f"Verified {release.version} overlay and Concept shortcuts.")
        return
    if action == "apply":
        validate_runtime(release)
        if receipt and not changes and not additions:
            print(f"Concept shortcuts already match {release.version}.")
            return
        if receipt is None:
            state.parent.mkdir(parents=True, exist_ok=True)
            backup = Path(tempfile.mkdtemp(prefix="backup-", dir=str(state.parent)))
            receipt = {"backup": str(backup.relative_to(home)), "entries": entries}
        for rel, metadata in additions.items():
            if metadata["original"] is not None:
                path = home_path(home, rel)
                original = metadata["original"]
                if snapshot(path) != original:
                    raise ValueError(f"Shortcut changed before backup: {path}")
                if original[0] == "symlink":
                    replace_link(backup/rel, original[1])
                else:
                    data = path.read_bytes()
                    if digest(data) != original[0]:
                        raise ValueError(f"Shortcut changed before backup: {path}")
                    replace_file(backup/rel, data, original[1])
        entries.update(additions)
        if additions:
            raw = (json.dumps(receipt, ensure_ascii=False, indent=2) + "\n").encode()
            replace_file(backup / "manifest.json", raw, 0o600)
            replace_file(state, raw, 0o600)
    else:
        # Verify every needed backup before restoring the first file.
        for rel in changes:
            original = entries[rel]["original"]
            if original is not None and snapshot(backup / rel) != original:
                raise ValueError(f"Original backup changed or missing: {backup / rel}")

    # Persistent receipt precedes the first replacement. Re-running apply
    # or rollback after interruption accepts only the original or installed state.
    for rel in changes:
        path = home_path(home, rel)
        original = entries[rel]["original"]
        expected = original if action == "apply" else entries[rel]["installed"]
        if snapshot(path) != expected:
            raise ValueError(f"Shortcut changed during migration: {path}")
        if action == "apply":
            replace_file(path, wanted[rel], MODE)
        elif original is None:
            path.unlink()
        elif original[0] == "symlink":
            replace_link(path, original[1])
        else:
            replace_file(path, (backup / rel).read_bytes(), original[1])
        print(action, path)
    if action == "rollback":
        state.unlink()
    print("Backup:", backup)


def main(release=V6278):
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    for mode in ("apply", "check", "rollback"):
        group.add_argument("--" + mode, dest="action", action="store_const", const=mode)
    args = parser.parse_args()
    home = Path.home().resolve()
    desktop = None if args.action == "rollback" else xdg_desktop(home)
    menu = Path(os.environ.get("XDG_DATA_HOME") or home / ".local/share") / "applications"
    lockpath = home_path(home, ".cache/aiformula-1120-shortcuts.lock")
    lockpath.parent.mkdir(parents=True, exist_ok=True)
    with lockpath.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        migrate(home, desktop, menu, args.action or "preview", release)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        sys.exit(str(exc))
