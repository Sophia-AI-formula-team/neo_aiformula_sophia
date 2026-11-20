"""Use the 1120 Concept shortcut migrator with a separate rollback receipt."""
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "2026-11-20-mapassist/concept"))
from shortcuts import Release, V6278, main


if __name__ == "__main__":
    try:
        main(Release(HERE, "1121 experiment", "concept-shortcuts-1121", previous=V6278))
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        sys.exit(str(exc))
