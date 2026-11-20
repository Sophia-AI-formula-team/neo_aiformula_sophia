"""Install only the OA 1B command names and desktop entries, after a prepared release exists."""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


COMMANDS = {"allnodes":"oa1b-allnodes","yolop":"oa1b-yolop","oa1b":"oa1b","lya":"oa1b-lya",
            "stop":"oa1b-stop","verify":"oa1b-verify"}


def desktop_arg(path):
    value=str(path).replace("%","%%")
    for char in ('\\','"','`','$'):
        value=value.replace(char,'\\'+char)
    return '"'+value.replace('\\','\\\\')+'"'


def entries(release, home, desktop):
    result={}
    for stage,name in COMMANDS.items():
        path=home/".local/bin"/name
        wrapper=f'#!/bin/sh\nexec {shlex.quote(str(release/"entry"))} {stage} "$@"\n'
        result[path]=wrapper.encode()
    for i, stage in enumerate(("allnodes","yolop","oa1b","lya"),1):
        path=home/".local/bin"/COMMANDS[stage]
        content=("[Desktop Entry]\nType=Application\nTerminal=true\nIcon=utilities-terminal\n"
                 f"Name=1121 OA 1B {i} {stage}\nExec={desktop_arg(path)}\n")
        if stage=="oa1b":
            content+="Actions=Inner;\n\n[Desktop Action Inner]\nName=1121 OA 1B Inner\nExec="+desktop_arg(path)+" inner\n"
        result[desktop/f"1121-oa1b-{i}-{stage}.desktop"]=content.encode()
    return result


def apply(wanted, action):
    missing=[]
    for path,data in wanted.items():
        if not os.path.lexists(path):
            missing.append(path)
        elif path.is_symlink() or not path.is_file() or path.read_bytes()!=data or not os.access(path,os.X_OK):
            raise ValueError(f"Name already occupied or changed: {path}. Inspect before changing it.")
    if action=="check" and missing:
        raise ValueError("Missing OA 1B shortcuts: "+", ".join(map(str,missing)))
    if action=="remove":
        for path in wanted.keys()-set(missing):
            path.unlink()
    if action=="install":
        created=[]
        try:
            for path in missing:
                path.parent.mkdir(parents=True,exist_ok=True)
                with tempfile.NamedTemporaryFile(dir=path.parent,delete=False) as tmp:
                    tmp.write(wanted[path])
                tmpfile=Path(tmp.name)
                try:
                    tmpfile.chmod(0o755)
                    os.link(tmpfile,path)  # Refuse to overwrite concurrent or foreign entries.
                    created.append(path)
                finally:
                    tmpfile.unlink()
        except BaseException:
            for path in created:
                if path.is_file() and not path.is_symlink() and path.read_bytes()==wanted[path]:
                    path.unlink()
            raise
    for path in wanted:
        print(action,path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action",choices=["preview","install","check","remove"])
    parser.add_argument("release",type=Path)
    parser.add_argument("--desktop",type=Path)
    args=parser.parse_args()
    release=args.release.resolve()
    home=Path.home().resolve()
    if release.parent != home/"aiformula_runtime/2026-11-21-oa1b":
        parser.error("Expected the versioned 1121 OA 1B release under the vehicle user's home")
    if args.action!="remove" and (not (release/"READY").is_file() or (release/"READY").read_text().strip()!=release.name
                                 or not os.access(release/"entry",os.X_OK)):
        parser.error("Build and verify this release first")
    bindir = home / ".local/bin"
    if bindir.resolve() != bindir:
        parser.error("The command directory is linked elsewhere. Inspect it before installing OA 1B.")
    desktop=args.desktop or Path(subprocess.check_output(["xdg-user-dir","DESKTOP"],text=True).strip())
    desktop=desktop.resolve()
    if desktop==home or home not in desktop.parents:
        parser.error("Select a desktop below the vehicle user's home")
    apply(entries(release,home,desktop),args.action)


if __name__=="__main__":
    main()
