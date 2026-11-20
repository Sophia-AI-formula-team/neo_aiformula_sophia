"""Install the mapassist_b shortcut without replacing any existing command or icon."""
import os
from pathlib import Path
import subprocess
import sys


def desktop_arg(value):
    escaped = str(value).replace("%", "%%")
    escaped = "".join("\\" + char if char in '\\"$`' else char for char in escaped)
    return '"' + escaped.replace("\\", "\\\\") + '"'


def main():
    release = Path(sys.argv[1]).resolve()
    command = release / "mapassist_b"
    if (release / "READY").read_text().strip() != release.name or not os.access(command, os.X_OK):
        sys.exit("Prepare the versioned OA 1B release first.")
    home = Path.home().resolve()
    bindir = home / ".local/bin"
    desktop = Path(subprocess.check_output(["xdg-user-dir", "DESKTOP"], text=True).strip()).resolve()
    if bindir.resolve() != bindir or home not in desktop.parents:
        sys.exit("Check the vehicle user's bin and desktop directories.")
    link = bindir / "mapassist_b"
    icon = desktop / "1121-mapassist_b.desktop"
    content = (
        "[Desktop Entry]\nType=Application\nName=1121 MapAssist B\nTerminal=true\nIcon=utilities-terminal\n"
        f"Exec={desktop_arg(link)}\nActions=Inner;\n\n[Desktop Action Inner]\n"
        f"Name=Inner\nExec={desktop_arg(link)} inner\n"
    ).encode("utf8")
    create_link = not os.path.lexists(link)
    create_icon = not os.path.lexists(icon)
    if not create_link and (not link.is_symlink() or Path(os.readlink(link)) != command):
        sys.exit(f"{link} is occupied by another entry.")
    if not create_icon and (icon.is_symlink() or not icon.is_file() or icon.read_bytes() != content):
        sys.exit(f"{icon} is occupied by another desktop entry.")
    bindir.mkdir(exist_ok=True, parents=True)
    desktop.mkdir(exist_ok=True, parents=True)
    if create_link:
        link.symlink_to(command)
    try:
        if create_icon:
            with os.fdopen(os.open(icon, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o755), "wb") as stream:
                stream.write(content)
    except Exception:
        if create_link:
            link.unlink()
        raise


if __name__ == "__main__":
    main()
