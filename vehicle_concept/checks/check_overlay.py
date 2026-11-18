#!/usr/bin/env python3
"""Verify the active ROS and Python installation before starting any nodes."""
import argparse
from importlib import import_module
from importlib.util import find_spec
from pathlib import Path
import sys
from xml.etree import ElementTree

from ament_index_python.packages import get_package_prefix


SOURCE = Path(__file__).resolve().parents[1] / "src"
NATIVE = {
    "concept_candidates": ("_progress",),
    "concept_feasibility": ("_motion", "_risk", "_tread"),
    "concept_road": ("_markings", "_memory"),
}


def verify(workspace):
    install = workspace / "install"
    for manifest in sorted(SOURCE.glob("*/package.xml")):
        package = manifest.parent.name
        prefix = Path(get_package_prefix(package)).resolve()
        if prefix not in (install, install / package):
            raise ValueError(f"{package} resolves to {prefix}; expected {install}")
        installed_manifest = prefix / "share" / package / "package.xml"
        if installed_manifest.read_text(encoding="utf8") != manifest.read_text(encoding="utf8"):
            raise ValueError(f"{package} manifest differs from {manifest}")

        # --symlink-install may load modules from this checkout or its build tree.
        roots = (prefix, workspace / "build" / package, manifest.parent)
        spec = find_spec(package)
        if spec is None or not spec.origin:
            raise ValueError(f"Python cannot locate {package}")
        py_dir = Path(spec.origin).resolve().parent
        if not any(root in py_dir.parents for root in roots):
            raise ValueError(f"Python resolves {package} to {py_dir}")
        for module in NATIVE.get(package, ()):
            path = Path(import_module(f"{package}.{module}").__file__).resolve()
            if not any(root in path.parents for root in roots):
                raise ValueError(f"{package}.{module} resolves to {path}")
        version = ElementTree.parse(manifest).getroot().findtext("version")
        print(f"{package} {version}  ROS={prefix}  Python={py_dir}")

    prefix = Path(get_package_prefix("concept"))
    launch = Path("launch/concept.launch.py")
    if (prefix / "share/concept" / launch).read_text(encoding="utf8") != (
            SOURCE / "concept" / launch).read_text(encoding="utf8"):
        raise ValueError("Installed concept.launch.py differs from the checkout; rebuild concept")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("workspace", type=Path)
    try:
        verify(parser.parse_args().workspace.resolve())
    except (ImportError, LookupError, OSError, ValueError) as exc:
        sys.exit(f"Concept deployment check failed: {exc}")
