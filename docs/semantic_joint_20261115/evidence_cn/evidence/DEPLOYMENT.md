# Deployment, reproduction and guarded rollback

The final install consists of **four C# and two perception Python files**, pinned in `manifest.json`. It changes the explicit Joint integration only and preserves source semantic 0.4.0, original LLP/LYA, weights, camera calibration, vehicle physics, scenes and project settings. The final policy is `calibrated_rgb_panel_only`: nonpersistent C# `UseCalibratedSignalPanelOnly`, request `signal_panel_only`, default false. Joint requires RGB mode, enables only this policy and restores the prior settings. Missing calibration yields unknown; an enabled old identity-guard request is rejected rather than silently combining modes.

## Preconditions

Use the existing original-LYA Simulator at `E:/Mess/School/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator`, not the separate UnityLab project. Read its parent `AGENTS.md`, full `workspace/CODING_MASTERPROMPT.md` and current `workspace/handoff.md`; coordinate exclusive ownership of this Editor. Existing RGB perception, A4 model and original LLP/LYA must already work. This archive installs no environment, model, ROS source or MCP bootstrap.

The source checkout is repository `workspace/src/aiformula/perception/semantic_planning_system`, original import commit `6b71267e24e81459220c7e6d30d84c3f7a00e591`. Use the existing detector Python environment and provided MCP bridge. The archive is `COLCON_IGNORE`, independent of ROS deployment.

## Read-only comparison, then explicit installation

Run from this archive in PowerShell:

```powershell
$archive = (Get-Location).Path
$simulator = 'E:/Mess/School/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator'
python ./install.py --simulator $simulator
python ./install.py --simulator $simulator --action install
```

Both commands are read-only. The installer checks every archived payload hash, resolves each target within the explicit Simulator root and preflights all six live files. Unknown bytes abort before mutation. Only original archived `before/` bytes or exact final `overlay/` bytes are accepted. After reviewing the plan, with the owned Editor in Edit Mode:

```powershell
python ./install.py --simulator $simulator --action install --apply
```

| Target | Baseline |
| --- | --- |
| `Assets/AIFormula/Runtime/Control/AIFormulaSemanticPlanning.cs` | Exact original restored by rollback |
| `Assets/AIFormula/Runtime/Control/AIFormulaVisionLaneFollower.cs` | Exact original restored by rollback |
| `Assets/AIFormula/Runtime/Perception/AIFormulaRoadDetector.cs` | Exact original restored by rollback |
| `Assets/AIFormula/Editor/Validation/AIFormulaJointTopoTrial.cs` | Previously absent; guarded rollback removes only this known file |
| `Assets/StreamingAssets/RoadDetector/worker.py` | Exact original restored by rollback |
| `Assets/StreamingAssets/RoadDetector/signal_panel_rgb.py` | Exact original restored by rollback |

Intermediate v1/v2 bytes are deliberately not accepted as generic baselines. Their historical `signal_model_identity.py` is not installed by the final payload. An existing v2 installation needs a separately reviewed exact-current-hash migration and exact-hash removal of its helper; never recursively clean a directory or accept unknown bytes. The completed local migration is indexed in evidence. `.meta`, scene, prefab, weight, settings and camera files are outside installation scope. The installer cannot establish Editor ownership or Play state.

Refresh/compile with the existing Unity MCP connection and inspect the console. A local temporary MCP bootstrap, if required, is separately owned and removed after use; it is not part of this runtime archive.

## Reproduce the actual MCP trials

Open saved `Assets/Scenes/MotorControllerValidation.unity` in clean GUI Edit Mode and disable competing autonomous owners. For the final experiment the owned Editor was set through MCP to `Application.targetFrameRate=30`, `QualitySettings.vSyncCount=0`, `Time.timeScale=1`; the actual trial physics step was 0.01 s. These are explicit temporary runtime conditions, not installed project settings. Capture existing values and restore them afterward; final cleanup restored FPS -1, vSync 1 and fixed step 0.02 s. Keep independent GPU/CPU load in the run evidence.

```powershell
$repo = (Resolve-Path '../..').Path
$source = Join-Path $repo 'workspace/src/aiformula/perception/semantic_planning_system'
$bridge = 'E:/Mess/School/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/workspace/tools/invoke_unity_bridge.py'
$evidence = 'E:/Mess/Projects/Programming/aiformula/workspace/semantic_joint_trial_20261116/runs'
$manifest = Get-Content ./manifest.json -Raw -Encoding UTF8 | ConvertFrom-Json
$transportHash = ($manifest.unity_files | Where-Object path -eq 'Assets/AIFormula/Runtime/Control/AIFormulaSemanticPlanning.cs').after_sha256
$harnessHash = ($manifest.unity_files | Where-Object path -eq 'Assets/AIFormula/Editor/Validation/AIFormulaJointTopoTrial.cs').after_sha256
python ./trials/run_joint_trial.py inner_joint_repeat_01 --lane inner --output-root $evidence --source-root $source --worker-root (Join-Path $archive 'transport') --simulator $simulator --bridge-script $bridge --expected-harness-sha256 $harnessHash --expected-transport-sha256 $transportHash
```

Choose an unused output name, then repeat with `--lane outer` and a different unused name. The runner refuses overwrites, does not launch Unity, and uses framed MCP `execute_code`/`manage_editor` rather than Computer Use. Default bridge port is 6400 and wall bound 900 s. Video editing duration is independent of simulation duration/time scale.

Inner default `component-only` reproduces the existing accepted runtime lane configuration (`RecoverProjectedComponents=true`, `UseCurrentCorridor=false`); outer uses `baseline`. Starts are `(x,z)=(68.76,-10)` inner and `(72.53,-10)` outer, facing north. These settings are recorded and restored without changing original LLP/LYA source. No four-point fallback is installed. Use only dedicated `transport/` as worker root; never inventory mutable audit directories. The runner hashes the worker, semantic source and fixed dependencies, including existing range/A4 runtime files, without recursively hashing large weights.

The first lap uses physical green. Odometry closure plus a physical start-plane return schedules physical red; only genuine subsequent detections enter native control. A 12 s native red wait records `red_detour_failed=true` and a terminal failure snapshot before physical green recovery. Green recovery can complete the second lap but never converts failed detour acceptance to success. Original 0.5 s native capture/command and transport epoch/identity/validity gates remain.

## Evidence meanings

- `configuration.json`, events and source hashes: actual runtime settings and unchanged executable inputs.
- `frames.jsonl`, genuine first-person `rgb_*.jpg`, original `overlay_*.png`: capture-sequence/time evidence, not a chase camera or new inference for every video frame.
- `motion.jsonl`, `trace.jsonl`, worker semantic ticks: physical trajectory, actual ABC/LYA command, original semantic status and map data.
- Learned odometry reference and `official_unaligned` graph must be displayed separately. Official interior estimates are not observed road mapping.
- `result.json`: separate laps, official fusion, detour and measured turn, latched failure, recovery and cleanup; `integration_hashes_after.json` verifies source preservation.

Final inner07/outer03 each passed two laps/red-stop/stationary-green recovery and failed official topo/interior turning. Stop remained early; no 8–9 m or bbox-distance behavior was imported. The package has no checkpoint persistence, so clean shutdown does not validate restart recovery. See `RESULTS.md`, `validation.json` and `media/` for final evidence; raw logs/images/video binaries remain local.

## Guarded rollback

In owned Edit Mode, inspect before applying:

```powershell
python ./install.py --simulator $simulator --action rollback
python ./install.py --simulator $simulator --action rollback --apply
```

Rollback accepts only the exact final or already-original bytes, restores the five replaced originals and removes only the known new harness. No recursive cleanup occurs. Refresh/compile with MCP, preserve all evidence and unrelated edits, and restore runtime settings separately. Retain `.gitattributes`: `* -text` prevents Git autocrlf from changing the reviewed SHA256 bytes. Before publishing, compare every staged Git blob to its manifest hash; worktree-only hashing is insufficient.
