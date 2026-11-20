"""Use the 1120 Concept shortcut migrator with a separate rollback receipt."""
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "2026-11-20-mapassist/concept"))
from shortcuts import Release, V6278, main


if __name__ == "__main__":
    try:
        if "--rollback" in sys.argv[1:] and (Path.home() / "concept_1121_ws/.legacy-retired").exists():
            raise ValueError("The old Concept installation was removed. Reinstall it from its pinned GitHub source to revert.")
        main(Release(HERE, "1121 experiment", "concept-shortcuts-1121", previous=V6278))
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        sys.exit(str(exc))
