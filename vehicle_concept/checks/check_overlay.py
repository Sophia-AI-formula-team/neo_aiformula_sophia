#!/usr/bin/env python3
"""Verify the active ROS and Python installation before starting any nodes."""
import argparse
from importlib import import_module
from importlib.machinery import EXTENSION_SUFFIXES
from importlib.util import find_spec
from pathlib import Path
import sys
from xml.etree import ElementTree

from ament_index_python.packages import get_package_prefix


SOURCE = Path(__file__).resolve().parents[1] / "src"
NATIVE = {
    "concept_candidates": ("_progress",),
    "concept_feasibility": ("_motion", "_risk", "_tread"),
    "concept_road": ("_markings", "_memory", "field_scan"),
}
EXPECTED_VERSIONS = dict(concept="6.27.8", concept_panel="6.27.8", concept_drive="6.27.8",
                         concept_road="6.27.7", concept_candidates="6.27.8", concept_ranker="6.27.6",
                         concept_obstacles="6.27.6",
                         concept_control_gate="6.27.4",
                         concept_visualization="6.27.4", concept_yolop="6.27.7")


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
            if not any(root in path.parents for root in roots) or not any(
                    path.name.endswith(suffix) for suffix in EXTENSION_SUFFIXES):
                raise ValueError(f"{package}.{module} resolves to {path}")
        version = ElementTree.parse(installed_manifest).getroot().findtext("version")
        if package in EXPECTED_VERSIONS and version != EXPECTED_VERSIONS[package]:
            raise ValueError(f"Expected {package} {EXPECTED_VERSIONS[package]}, installed manifest reports {version}")
        print(f"{package} {version}  ROS={prefix}  Python={py_dir}")

    prefix = Path(get_package_prefix("concept"))
    launch = Path("launch/concept.launch.py")
    if (prefix / "share/concept" / launch).read_text(encoding="utf8") != (
            SOURCE / "concept" / launch).read_text(encoding="utf8"):
        raise ValueError("Installed concept.launch.py differs from the checkout; rebuild concept")

    from concept_candidates import curve_road617
    from concept_candidates.candidate_merge import distinct_requests
    from concept_road import _markings, material

    if not callable(getattr(_markings, "material_sections", None)) or (
            getattr(material, "native_sections", None) is not _markings.material_sections):
        raise ValueError("Installed concept_road lacks the active V6.27.2 native material scanner")
    # Test both axes independently, including a one-ULP lateral difference.
    if getattr(curve_road617, "distinct_requests", None) is not distinct_requests:
        raise ValueError("Installed forward planner does not use the V6.27.2 exact request grouping")
    representatives, inverse = distinct_requests(
        [1.0, 1.0, float.fromhex("0x1.0000000000001p+0"), 1.0],
        [0.5, 0.5, 0.5, float.fromhex("0x1.0000000000001p-1")])
    if list(representatives) != [0, 2, 3] or list(inverse) != [0, 0, 1, 2]:
        raise ValueError("Installed candidate merge did not preserve distinct float64 requests")
    print("material_sections=native candidate_merge=exact")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("workspace", type=Path)
    try:
        verify(parser.parse_args().workspace.resolve())
    except (ImportError, LookupError, OSError, ValueError) as exc:
        sys.exit(f"Concept deployment check failed: {exc}")
