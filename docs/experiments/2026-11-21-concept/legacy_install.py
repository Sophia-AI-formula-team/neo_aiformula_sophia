"""Move Concept's Python dependencies forward and retire the two known installs."""
import os
from pathlib import Path
import shutil
import subprocess
import sys


REPOS = ("src/neo-concept-v6275", "src/neo-concept-1120-v6278")
WORKSPACES = ("concept_v6275_ws", "concept_1120_v6278_ws")
WORKSPACE_FILES = {"activate_concept.bash", "build", "install", "log", "venv", ".prepare-concept.lock"}
ORIGINS = {
    "https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia",
    "https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git",
    "git@github.com:Sophia-AI-formula-team/neo_aiformula_sophia.git",
}


def exact_path(path):
    if path.resolve() != path:
        raise ValueError(f"Expected a local path without symlink parents: {path}")
    return path


def mounted_under(path):
    for line in Path("/proc/self/mountinfo").read_text().splitlines():
        escaped = line.split()[4]
        mount = Path(escaped.replace(r"\040", " ").replace(r"\011", "\t")
                     .replace(r"\012", "\n").replace(r"\134", "\\"))
        if mount == path or path in mount.parents:
            raise ValueError(f"Refusing to retire mounted path: {mount}")


def installed_paths(home):
    for relative in REPOS + WORKSPACES:
        path = exact_path(home / relative)
        if not path.exists():
            continue
        mounted_under(path)
        if relative in REPOS:
            if not (path / ".git").is_dir() or not (path / "vehicle_concept/src/concept/package.xml").is_file():
                raise ValueError(f"Expected an independent Concept checkout: {path}")
            git = ["git", "-C", str(path)]
            origin = subprocess.check_output(git + ["remote", "get-url", "origin"], text=True).strip()
            if origin not in ORIGINS or subprocess.check_output(git + ["status", "--porcelain"], text=True).strip():
                raise ValueError(f"Review the old checkout or save its local changes before removal: {path}")
        else:
            if not path.is_dir():
                raise ValueError(f"Expected a Concept workspace directory: {path}")
            unexpected = {p.name for p in path.iterdir()} - WORKSPACE_FILES
            if unexpected:
                raise ValueError(f"Move unrecognized files before removal at {path}: {', '.join(sorted(unexpected))}")
        yield path


def python_value(python, expression):
    return subprocess.check_output([str(python), "-I", "-c", expression], text=True).strip()


def migrate_dependencies(home, workspace):
    # Use the Python packages from the 11/19 install. Recreate the interpreter
    # and activation scripts at the new path instead of copying their shebangs.
    old = exact_path(home / WORKSPACES[0] / "venv")
    new = exact_path(workspace / "venv")
    incomplete = exact_path(workspace / ".copying-legacy-venv")
    if incomplete.exists() and new.exists():
        mounted_under(new)
        shutil.rmtree(new)
    if new.exists() or not old.exists():
        return
    config = dict(tuple(part.strip() for part in line.split("=", 1))
                  for line in (old / "pyvenv.cfg").read_text().splitlines() if "=" in line)
    interpreter = old / "bin/python3"
    version = python_value(interpreter, "import sys; print('python{}.{}'.format(*sys.version_info[:2]))")
    if "home" not in config or not Path(config["home"]).is_absolute():
        raise ValueError(f"Missing absolute base interpreter directory in {old / 'pyvenv.cfg'}")
    base = (Path(config["home"]) / version).resolve(strict=True)
    legacy = [home / relative for relative in REPOS + WORKSPACES]
    if any(root in base.parents for root in legacy):
        raise ValueError(f"Python interpreter is installed inside an old workspace: {base}")
    origin = Path(python_value(old / "bin/python3", "import sysconfig; print(sysconfig.get_path('purelib'))"))
    if old not in origin.resolve().parents:
        raise ValueError(f"Expected packages inside the old Concept venv: {origin}")
    args = [str(base), "-m", "venv", "--without-pip"]
    if config.get("include-system-site-packages", "").lower() == "true":
        args.append("--system-site-packages")
    incomplete.touch()
    try:
        subprocess.run(args + [str(new)], check=True)
        dest = Path(python_value(new / "bin/python3", "import sysconfig; print(sysconfig.get_path('purelib'))"))
        if new not in dest.resolve().parents:
            raise ValueError(f"Expected packages inside the 1121 venv: {dest}")
        shutil.copytree(origin, dest, symlinks=True, dirs_exist_ok=True)
        for item in dest.rglob("*"):
            if item.is_symlink():
                target = item.resolve()
                if any(root == target or root in target.parents for root in legacy):
                    raise ValueError(f"Python dependency still points into an old install: {item}")
            elif item.suffix in (".pth", ".egg-link") and any(
                    str(root) in item.read_text(errors="replace") for root in legacy):
                raise ValueError(f"Editable Python dependency still points into an old install: {item}")
    except BaseException:
        if new.is_dir():
            shutil.rmtree(exact_path(new))
        raise
    incomplete.unlink()
    print(f"Moved Python dependencies into {new}", flush=True)


def retire(home, workspace, verifier):
    """Hide the old paths, verify the installed release, then delete old trees."""
    stash = exact_path(workspace / ".retiring-legacy")
    confirmed = stash / ".verified"
    names = {Path(rel).name: home / rel for rel in REPOS + WORKSPACES}
    if confirmed.exists():
        mounted_under(stash)
        shutil.rmtree(stash)
    if stash.exists():
        mounted_under(stash)
        for entry in stash.iterdir():
            original = exact_path(names[entry.name])
            if original.exists() or entry.is_symlink():
                raise ValueError(f"Review the interrupted Concept retirement at {entry}")
            entry.rename(original)
        stash.rmdir()

    old = list(installed_paths(home))
    if old:
        stash.mkdir(mode=0o700)
        try:
            for path in old:
                path.rename(stash / path.name)
            subprocess.run(verifier, check=True)
        except BaseException:
            for path in old:
                pending = stash / path.name
                if pending.exists():
                    pending.rename(path)
            stash.rmdir()
            raise
        confirmed.touch()
        mounted_under(stash)
        shutil.rmtree(stash)
        for path in old:
            print(f"Removed old Concept installation: {path}")
    (workspace / ".legacy-retired").touch()


if __name__ == "__main__":
    home = Path.home().resolve()
    workspace = exact_path(home / "concept_1121_ws")
    action, *command = sys.argv[1:]
    try:
        if action == "migrate-dependencies":
            migrate_dependencies(home, workspace)
        elif action == "preflight":
            list(installed_paths(home))
        elif action == "retire" and command:
            retire(home, workspace, command)
        else:
            raise ValueError("Expected migrate-dependencies, preflight or retire with a verifier")
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        sys.exit(f"Concept cleanup: {exc}")
