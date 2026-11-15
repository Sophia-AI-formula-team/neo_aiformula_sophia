# Deployment and rollback

This manifest contains six final panel-only C#/Python payload files. Its single
request field is `signal_panel_only`; install the complete reviewed payload
together. Joint requires RGB recognition, rejects a disabled RGB configuration,
and restores prior settings. Missing calibration yields unknown, without model
fallback. An enabled archived identity-guard request is explicitly rejected.
The installer accepts only original `before/` bytes or the exact final payload.
Intermediate v1/v2 bytes need an explicit reviewed migration receipt; they are
not extra accepted installation baselines. Migration from v2 also removes only
its exact known-hash `signal_model_identity.py`; the root owns that live action.
Clean-baseline installation never creates this historical helper. The source,
weights, camera, scene, vehicle physics and render cap remain outside this install.

Use the original LYA Simulator, not the independent `concept_modules_v4` UnityLab.
Its current local path is
`E:/Mess/School/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator`.
The old OneDrive path was migrated. Read its parent `AGENTS.md`, full
`workspace/CODING_MASTERPROMPT.md`, and current `workspace/handoff.md` first.
Coordinate exclusive ownership of that Editor before changing its scoped scripts or
running Play; another project's CPU load also affects timing evidence.

The existing simulator must already have functioning RGB perception and original
LLP/LYA. Current model identity is recorded during validation; this archive neither
selects nor installs model weights. Python uses the existing detector environment.

## Compare, install, and compile

The installer defaults to a read-only comparison. Paths below are examples for
PowerShell, relative to this archive:

```powershell
$simulator = 'E:/Mess/School/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator'
python ./install.py --simulator $simulator
python ./install.py --simulator $simulator --action install
```

Review the before/after hashes in the output and `manifest.json`. With the owned
Editor in Edit Mode, the explicit mutation command is:

```powershell
python ./install.py --simulator $simulator --action install --apply
```

The deployment scope is exactly these four C# and two Python files:

1. `Assets/AIFormula/Runtime/Control/AIFormulaSemanticPlanning.cs`
2. `Assets/AIFormula/Editor/Validation/AIFormulaJointTopoTrial.cs`
3. `Assets/AIFormula/Runtime/Control/AIFormulaVisionLaneFollower.cs`
4. `Assets/AIFormula/Runtime/Perception/AIFormulaRoadDetector.cs`
5. `Assets/StreamingAssets/RoadDetector/worker.py`
6. `Assets/StreamingAssets/RoadDetector/signal_panel_rgb.py`

Unknown live bytes stop installation before any write. Already matching files
are left alone. Original bytes for the five replaced files are included; the new
harness was absent before this integration. Historical identity helper files are
preserved under `history/` and are not final deployment payloads.
No `.meta`, scene, prefab, settings, package,
weight, or camera file is installed. The installer does not control the Editor or
prove its ownership/Play state; the operator establishes those before `--apply`.

The detector's new conflict-rejection request flag defaults false and is not
serialized into scene or user settings. JointTrial explicitly sets it true and
restores the prior value; configuration and start events record the opt-in.
The corresponding Python perception payloads are included with reviewed exact
baseline/final hashes. The new helper imports only Python's `math`; it changes
neither A4 weights nor FCOS code. Default-off dictionaries match the original
pipeline on all 839 saved replay frames; opt-in changes and JPEG/timing limits
are disclosed in `evidence/staged_perception_verification.json` and the analysis.
Intermediate locally installed experiment versions are deliberately not accepted
as generic installer baselines: root upgrades them using a separate explicit
current-to-next SHA receipt. This installer supports the original archived
baseline, final matching state, and guarded rollback only.

Refresh using the existing Unity MCP connection, await compilation, and read the
console. If the bridge itself needs a local bootstrap, handle that separately and
remove it after use; a temporary bridge-start script is intentionally not part of
this runtime archive.

## Run through existing MCP

Open the saved `MotorControllerValidation.unity` scene in clean GUI Edit Mode.
Do not run a competing autonomous mode. The existing framed bridge script is
`E:/Mess/School/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/workspace/tools/invoke_unity_bridge.py`;
default loopback port is `6400`. The runner checks Editor state and sends the
existing `execute_code`/`manage_editor` MCP commands. It does not launch Unity or
use Computer Use.

```powershell
$archive = (Get-Location).Path
$repo = (Resolve-Path '../..').Path
$source = Join-Path $repo 'workspace/src/aiformula/perception/semantic_planning_system'
$bridge = 'E:/Mess/School/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/workspace/tools/invoke_unity_bridge.py'
$evidence = 'E:/Mess/Projects/Programming/aiformula/workspace/semantic_joint_trial_20261116/runs'
$manifest = Get-Content ./manifest.json -Raw -Encoding UTF8 | ConvertFrom-Json
$transportHash = $manifest.unity_files[0].after_sha256
$harnessHash = $manifest.unity_files[1].after_sha256
python ./trials/run_joint_trial.py inner_joint_01 --lane inner --output-root $evidence --source-root $source --worker-root (Join-Path $archive 'transport') --simulator $simulator --bridge-script $bridge --expected-harness-sha256 $harnessHash --expected-transport-sha256 $transportHash
```

After reviewing that run, use a new unique output name and `--lane outer` for the
other start. The runner never overwrites earlier evidence or retries a failed
case automatically. The default inner `component-only` runtime flags reproduce
the old accepted inner baseline (`RecoverProjectedComponents=true`,
`UseCurrentCorridor=false`); outer uses `baseline`. They are recorded/restored and
do not change LLP/LYA source. Explicit `--lane-recovery-mode` is available for a
separately named comparison. Starts are `(x,z)=(68.76,-10)` and `(72.53,-10)`, north.

Use the dedicated `transport/` as worker root. The runner hashes only its worker
file, the semantic source tree, and fixed simulator/LYA dependencies. Put runtime
evidence outside the semantic source tree. It preserves source hashes before and
after each case. Default wall timeout is 900 seconds; user video duration is an
editing target, not a simulation speed or time-scale override.

The fixed hash list retains the existing
`signal_panel_range.py`, and the actual A4 runtime's `shared_fcos.py`,
`shared_runtime.py`, and `color_semantics.py`. It does not recursively inventory
weights or change their existing provenance.

Run `inner_joint_04` temporarily changed only its own render cap to 30 FPS during
the run; see [RESULTS.md](RESULTS.md) for the exact transition and separate segment
metrics. It is not a fixed-cap baseline result. Root's proposed inner05 fixes that
render cap from startup while preserving time scale/physics step; verification
and results are pending. No render settings are installed by this archive.

## Evidence and acceptance

- `configuration.json`, `play_before/start/after.json`, camera/perception settings:
  start conditions, mode separation and preserved parameters.
- `trace.jsonl`, `motion.jsonl`, `semantic_ticks.jsonl`: real detector, physical
  motion and complete semantic outputs. `semantic.map_data` includes learned
  samples, reference/trust, graph nodes/edges/frame, and detour points. Show
  `official_unaligned` graph geometry distinctly from odometry geometry.
- `rgb_*.jpg`, `overlay_*.png`, `frames.jsonl`: genuine detector first-person RGB
  previews and overlays with actual capture time/sequence. A repeated preview
  timeline frame is not a new inference frame. No chase view substitutes for RGB.
- `events.jsonl`, `red_wait_snapshot.json`: closure, physical lap returns, physical
  lamp changes, received red, detour and measured turning, plus explicit failures.
- `result.json`: separate physical laps and map/detour/left-motion fields,
  `red_detour_failed`, any green recovery, cleanup and shutdown receipt.
- `integration_hashes*.json`: executable/configuration input preservation.

If red wait persists 12 simulated seconds, preserve that interval in the video
and identify subsequent movement as green recovery. A later completed lap does
not establish red-triggered turning. Physical loop closure and map fusion have
different acceptance meanings. Keep failed attempts and report problems before
behavioral fixes. Do not weaken source gates or add simulation map calibration.

## Guarded rollback

In owned Edit Mode, first inspect the plan:

```powershell
python ./install.py --simulator $simulator --action rollback
python ./install.py --simulator $simulator --action rollback --apply
```

Rollback requires the exact archived after hashes or already restored before
state. It restores only the five original replaced files and removes only the
known new harness; no recursive cleanup occurs. Refresh and verify compilation
through MCP afterward. Preserve all evidence and unrelated changes.
