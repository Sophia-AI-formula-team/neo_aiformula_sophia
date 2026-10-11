# Deploying the frozen Unity source overlay

This procedure updates an existing compatible `Demo_LaneLineTrainer/Simulator`.
It does not create a Unity project or install the native ROS stack. For the
active ROS source, use the documentation in the package tree. Keep native ROS
acceptance and the recorded Unity acceptance separate.

## Prerequisites

- A full local clone of `Sophia-AI-formula-team/neo_aiformula_sophia` containing
  commits `5bc52e05329d7fe642f219d32a578d04d2b7f668` and
  `63eec2da9f9d2ef90e1db29ce2c6c0b3c1a18a96`. The script reads these immutable
  Git objects. A downloaded source ZIP, unavailable history, or a shallow clone
  omitting either commit is insufficient. Restore the relevant history using
  your normal Git workflow; the installer never fetches or changes Git state.
- On Windows, use a short checkout path. The nested overlay can exceed Git's
  default path limit; enable long paths for this repository with
  `git config --local core.longpaths true`, or clone using
  `git clone -c core.longpaths=true <repository-url> <short-path>`.
  This is a repository-local setting, not a global Git configuration change.
- Existing Simulator project with Unity **2022.3.22f1 (887be4894c44)** and the
  saved `Assets/Scenes/MotorControllerValidation.unity` scene. Its camera,
  vehicle physics, traffic-signal assets, A3 runtime and pinned vision/LYA
  copies must match the manifest prerequisites. Unity, Git and Python are
  external prerequisites, not included binaries.
- The existing project package lock. The recorded project uses Input System
  1.7.0, TextMeshPro 3.0.6, Barracuda 3.0.2 and UGUI 1.0.0, plus Newtonsoft.Json
  supplied through the existing project dependencies. MCP for Unity is needed
  only for MCP orchestration; retain the baseline `Packages/packages-lock.json`
  resolution rather than updating its `main`-named package source.
- A compatible Python interpreter for the frozen worker: **Python 3.11 or
  newer** because `RoadDetector/worker.py` calls `hashlib.file_digest`.
  The A3 runtime's broader 3.10+ support is not sufficient for this frozen
  worker. The installer itself uses only Python's standard library and Git
  (Python 3.9+); use the tested 3.12 environment for both when available.
- Existing checkpoint `Assets/StreamingAssets/RoadDetector/a3-v14/weights/shared_best.pth`,
  79,763,687 bytes, SHA256
  `a411c983b661d6b26217019a968251c89a6b66c95400e4e7cb78f04a644ca10c`.
  Keep its weight manifest, runtime and licenses. This archive neither embeds
  nor downloads weights. The recognition worker validates checkpoint identity.
- Existing upstream vision copy pinned to
  `05f1651c8a14d56ab9a86a9767c52793403dcfdc`. Do not replace its LLP C++,
  `lane_0215.py`, `withoutkalman_0312.py`, `lya_0221.py`, camera transforms or
  gains with current unrelated repository versions.

The recorded machine used CPython 3.12.10 at
`E:/Mess/Projects/Programming/aiformula/YOLOP/YOLOP_A2/.venv/Scripts/python.exe`.
Installed package metadata at handoff: torch 2.11.0+cu130, torchvision
0.26.0+cu130, numpy 2.4.4, opencv-python 5.0.0.93, pillow 12.2.0, scipy 1.18.0,
PyYAML 6.0.3, tqdm 4.70.0 and yacs 0.1.8. These are the existing desktop
environment, not a newly tested install recipe or a Jetson/Foxy compatibility
claim. Provision a matching environment from the model's own deployment
instructions. The frozen native semantic packages separately retain Python
3.8 syntax support; that does not lower this Unity worker's Python requirement.

## Inspect and install

Run from the neo repository root. Replace the Simulator and interpreter paths
for another machine; do not paste the original user's local paths blindly.
The examples below use the original recorded locations.

```powershell
$repoRoot = (Get-Location).Path
$archiveRoot = Join-Path $repoRoot 'codexws\unity_semantic_integration'
$simulatorRoot = 'C:\Users\PC\OneDrive - Sophia Univ. Students\Main\SOPHIA\Y4T1\Research\Demo_LaneLineTrainer\Simulator'
$pythonExe = 'E:\Mess\Projects\Programming\aiformula\YOLOP\YOLOP_A2\.venv\Scripts\python.exe'
& $pythonExe -B "$archiveRoot\install.py" --repo $repoRoot --action verify-payload
& $pythonExe -B "$archiveRoot\install.py" --repo $repoRoot --simulator $simulatorRoot --action inspect
& $pythonExe -B "$archiveRoot\install.py" --repo $repoRoot --simulator $simulatorRoot --action verify-before
```

All outputs are JSON; failure exits with status 2. `verify-payload` checks every
before/after byte recipe without reading a Simulator. `verify-before` requires
the original bytes for all 116 managed paths and all 305 unchanged prerequisites.
It does not require the original 8.3 MB ZIP. A current final63 installation
should instead pass `verify-after` and needs no reapplication.

Before writing, obtain exclusive ownership of this project, stop active trials
and both Python workers, and close Unity. Preserve any unrelated work by the
other agent. Then explicitly apply:

```powershell
& $pythonExe -B "$archiveRoot\install.py" --repo $repoRoot --simulator $simulatorRoot --action apply --write --unity-closed
& $pythonExe -B "$archiveRoot\install.py" --repo $repoRoot --simulator $simulatorRoot --action verify-after
```

The confirmation flag is a caller assertion, not an automatic process check.
The installer materializes and verifies all payloads, checks all current
destination/prerequisite hashes, and only then writes manifest-listed files.
It preserves exact LF/CRLF bytes, refuses path traversal, links, protected
upstream/weight/scene/settings writes and unexpected changes. Only the four
new task-owned `.meta` files are installed; existing GUID metadata stays in
the compatible baseline. No UserSettings, scene or physics preset is changed.

Each file replacement is atomic; the multi-file operation is not a transaction.
An I/O interruption reports completed paths. A mixed before/after state is
visible with `inspect` and refused by both apply and rollback. Stop and review
the exact per-file hashes before a narrowly scoped manual recovery. There is
no `--force`, bulk reset or recursive deletion fallback. Verification is a
source-identity check; it does not launch inference or establish functional
acceptance for a different machine.

## Run the recorded scenario

Reopen the project in the pinned Unity version, wait for compilation, and open
the saved `MotorControllerValidation` scene in clean GUI Edit Mode. This
harness refuses batch mode, dirty scenes and an already-running trial. Set the
existing road-detector UI's Python executable to the compatible local path;
the baseline's local UserSettings are not installed by this archive. Retain
the original camera, physics, signal geometry and masks. Compare the scenario
settings with `evidence/configuration.json` before claiming comparability.

Execute one Editor C# call at a time through the existing MCP C# endpoint or
the project's own Editor execution facility. Use new output directories; do
not overwrite the recorded final63 data. Example for the inner red case:

```csharp
return AIFormula.Editor.AIFormulaSemanticTrafficTrial.Start(
    @"D:\trial-output\new_inner_red", "new_inner_red",
    68.76f, -10f, true, true, 10f, true, true);
```

Arguments after the two coordinates are `initialRed`, `releaseGreen`,
`perceptionHz`, `panelRgb`, `signalApproach`. The four recorded combinations are:

| Case | laneX | initialRed | releaseGreen | perceptionHz / panelRgb / signalApproach |
| --- | ---: | --- | --- | --- |
| Inner red | 68.76 | true | true | 10 / true / true |
| Inner green | 68.76 | false | false | 10 / true / true |
| Outer red | 72.53 | true | true | 10 / true / true |
| Outer green | 72.53 | false | false | 10 / true / true |

Read `AIFormula.Editor.AIFormulaSemanticTrafficTrial.Status()` without starting
another trial. To stop intentionally, call
`AIFormula.Editor.AIFormulaSemanticTrafficTrial.Stop("Operator stopped")`.
Wait for natural completion, worker cleanup and return to clean Edit Mode
before starting the next case. Coordinate Unity ownership with other agents;
do not run another model, test suite or video encoder during a control trial.

`signalApproach=true` explicitly enables the RGB range approach and the current
component/corridor wiring. It also opts this automated trial out of the
Follower's application-focus requirement and enables first-frame headroom.
These runtime settings are saved/restored by the harness; defaults remain
conservative outside this trial. Freshness, emergency stop, ownership, UI
blocking, loss-of-line and driver-neutral guards remain active.

The Simulator's opt-in approach uses an 8.0 m conservative lower-bound target,
3.0 m/s cap and 1.5 m/s² assumed deceleration. The requested **measured fixed-plane
front gap is 8–9 m**, not 8.0 m exactly. Native package defaults remain disabled
with 8.5 m / 0.8 m/s / 1.0 m/s² parameters. Do not transfer these simulation
calibrations directly to a real vehicle.

Inspect `result.json`, stop/release events, actual motion, raw RGB/overlay,
candidate support/source/age, semantic Traffic versus Gate status, and terminal
cleanup. `accepted=true` is only the harness result. For a comparable new
acceptance, also audit each requested lane and the actual vehicle trajectory;
the broad region guard alone is not a strict lane check. Keep failed runs and
their stopped tails. Do not replace them with a later successful trial.

The current postprocessing scripts and full frame data remain at the local
diagnostic root in README, under `staging/video/`; they are intentionally not
duplicated in this source overlay. Their exact identities and the original
outputs are recorded in `evidence/final_media_manifest.json`. Video re-encoding
requires those separately retained inputs and tools. Source installation does
not depend on them.

## Roll back

Close Unity and stop the workers first. Rollback accepts only an unmodified
final63 managed state and matching prerequisites:

```powershell
& $pythonExe -B "$archiveRoot\install.py" --repo $repoRoot --simulator $simulatorRoot --action verify-after
& $pythonExe -B "$archiveRoot\install.py" --repo $repoRoot --simulator $simulatorRoot --action rollback --write --unity-closed
& $pythonExe -B "$archiveRoot\install.py" --repo $repoRoot --simulator $simulatorRoot --action verify-before
```

Rollback restores exact original wiring bytes and native 5bc52e0 byte recipes,
including the baseline's mixed per-file newline policies. It deletes only the
eight files added by this task, individually and after hash checks. Empty
directories may remain. It does not remove peer work, reset a Git checkout,
restore settings or assets, or change another process. An edited managed file
causes refusal before any writes rather than loss of that edit.

For archive-tool validation only, with no Unity/model active:

```powershell
Push-Location $archiveRoot
& $pythonExe -B -m unittest test_install -v
Pop-Location
```

Tests create a new temporary project and check payload drift, path scope,
write opt-in, collisions, prerequisite drift, exact roundtrip and interruption
handling. They do not touch the original Simulator.
