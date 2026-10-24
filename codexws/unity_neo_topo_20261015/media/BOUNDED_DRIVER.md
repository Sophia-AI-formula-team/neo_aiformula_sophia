# One-attempt Windows media supervisor

Status: prepared statically; this driver has not been executed or tested. Root must review and grant its runtime before use. It never starts Unity, perception, a model or a planner.

Example, using paths chosen on the executing host:

```powershell
python -B .\run_bounded.py --plan PLAN_JSON --python RENDER_PYTHON_EXE --receipt-directory NEW_EXECUTION_DIRECTORY --max-wall-sec 120
```

The selected rendering Python must have Pillow. Python 3.11+ is required. FFmpeg is read from the sealed plan, built using `make_plan.py --ffmpeg` (or its documented environment/PATH resolution). There are no user-specific runtime paths in the driver or its helper. Linux/macOS are rejected: this supervisor intentionally requires Windows Job Objects rather than silently using weaker containment.

`--builder` defaults to the neighboring `build_video.py`. To reproduce a preserved v02 plan, pass the preserved `frozen_v02/build_video.py`; its same-directory script hashes must match that original plan. New v03 plans use the current builder.

The explicitly requested `--max-wall-sec` bounds the child-tree primary operation; five seconds maximum are reserved for cleanup. Input SHA verification before launch and after the complete owned tree stops are separate bookkeeping phases. All timings are reported. Root should additionally account for that small file-hashing cost when granting a total CPU window.

The supervisor creates a new receipt directory, refuses existing media output, and never automatically retries. It launches Python suspended, assigns it to a Job with KILL_ON_JOB_CLOSE before Python can spawn descendants, finds and validates the unique initial thread, then resumes it and opens a kernel-event gate. This avoids the earlier venv-launcher race. The actual interpreter writes its own PID and creation FILETIME before `site.main`, Pillow or the builder is loaded.

CREATE_SUSPENDED uses the explicit Windows value `0x00000004`, because this Python 3.13 does not export that flag on `subprocess`. The source-level repair is recorded in `DRIVER_CREATE_SUSPENDED_PROBLEM_04.md`. An approved repair may be followed by a new named attempt; “no automatic retry” prohibits an unattended retry loop, not fixing a reported defect. Preserve prior output and receipt directories.

The `WindowsOwnedJob` class was copied verbatim from the previously used fixture driver03 (source file SHA256 `c70c04b7d67b417cf3854619a945b5aaf3ca785953c401c79e35737100a61722`). Only an imports/docstring prefix was added to the portable `windows_owned_job.py`; its full SHA256 is `aa6f98f52029502a5ad4452acafc7829f99d1a31e55a7d1b518b21446f98fd24` and the supervisor checks it before use. This provenance does not claim the new supervisor itself has been exercised.

The parent, suspended launcher, initial thread and actual interpreter have kernel creation tokens. Every FILETIME serialized by the supervisor or bootstrap is a decimal **string**, preserving all 64 bits. The inherited helper's internal integer is converted before receipt serialization. Job members are sampled every 50 ms; short-lived processes may be missed. The receipt explicitly avoids claiming a complete per-PID enumeration. The Job's final ActiveProcesses=0 is the whole-tree cleanup check. Independently opened known PID/birth tokens must be absent, exited, or a reused PID belonging to a different birth token. The driver never kills by name or PID discovered outside its owned Job.

`receipt.json` includes launch/isolation, timings, full source hash equality, actual interpreter identity, known process identities and independent exit/absence checks. `source_hashes_before.json` and `source_hashes_after.json` hold literal source/dependency/script hashes. Child stdout/stderr and partial output remain after failures. A successful child exit is insufficient: the supervisor reads the builder's `verification.json`, requires its full-decode per-frame timing verification, and independently checks the video hash.

The builder still owns complete FFmpeg decode verification and frame-source timeline creation. Visual QA remains separate and pending even after a successful machine receipt. Root should inspect decoded red/green, closure, restart and failure-caption samples, then record its own visual-QA result.
