"""Archive the report, reproducible fixture and small source evidence."""
from pathlib import Path
import re
import sys
from zipfile import ZipFile, ZIP_DEFLATED

base = Path(__file__).resolve().parent
target = Path(sys.argv[1]).resolve()
queue = [base / name for name in (
    "README.md", "trace/README.md", "objects/README.md", "control/README.md",
    "objects/frame-0287.jpg", "objects/frame-0288.jpg", "objects/report-figure.py",
    "objects/timeline.log", "trace/timeline.log", "control/scene.json",
    "control/replay-ranked-start.py", "control/control.jsonl.gz", "trace/plot.R",
    "trace/run-stationary.bash", "trace/stationary-replay.launch.py",
    "trace/stationary-inputs.json.gz", "trace/export-stationary-inputs.py",
    "final-build-tests.log", "completion-processes.log", "activated.log",
    "source-manifest.json", "verify-field.py", "collect_samples.py", "package.py",
)]
queue.extend(sorted((base / "fixture").glob("sample-*")))
files = set()
while queue:
    file = queue.pop().resolve()
    if file in files:
        continue
    if base not in file.parents or not file.is_file() or file.stat().st_size > 1_000_000:
        raise ValueError(f"Missing, outside report, or too large: {file}")
    files.add(file)
    if file.suffix == ".md":
        for link in re.findall(r"!?\[[^\]]*\]\(([^()]+)\)", file.read_text()):
            if "://" not in link and not link.startswith("#"):
                queue.append(file.parent / link.partition("#")[0])

temporary = target.with_suffix(".tmp")
with ZipFile(temporary, "w", compression=ZIP_DEFLATED, compresslevel=9, strict_timestamps=False) as archive:
    for file in sorted(files):
        archive.write(file, base.name / file.relative_to(base))
with ZipFile(temporary) as archive:
    assert archive.testzip() is None
    assert len(archive.namelist()) == len(files)
    assert not any(name.endswith((".db3", ".mcap", ".mp4", ".avi")) for name in archive.namelist())
temporary.replace(target)
print(f"{len(files)} files, {target.stat().st_size:,} bytes")
