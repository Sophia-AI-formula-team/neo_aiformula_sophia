# Deployment: Unity simulation only

Do not deploy this adapter to the physical vehicle. The vehicle entry remains
the native neo `semantic_vehicle.launch.py` and its production packages.

Use source commit `3c6fbac2ff9f8dd23b247a187e966648876400b1` (MapAssist
0.3.1), based on PR25 `ed8e397ae41732877dfef334f0b89f98c0b20ae8`.
The required production fix changes only the checkpoint validator predicate for
signed closed-route progress. The original route geometry, closure thresholds,
traffic logic and motor gate remain unchanged. See the
[problem and regression archive](../../workspace/src/aiformula/perception/semantic_planning_system/semantic_local_planner_ros/docs/checkpoint_signed_progress_20261015/README.md).
All four final MCP cases passed on this pin; follow [RESULTS.md](RESULTS.md)
and the immutable per-case receipts. Native DDS/vehicle deployment remains
outside this Unity acceptance.

## Preconditions

1. Confirm the neo source commit and a clean scoped feature branch. Preserve any
   unrelated dirty checkout. Obtain actual ownership of the target Unity Editor.
2. Through MCP, confirm idle clean Edit mode, no active test/worker and no pending
   compile. Save the current scene path and a complete runtime-tuning/component
   snapshot; initialize the tuning bindings with SerializePreset before comparing.
3. Hash the existing simulator sources, scenes and ProjectSettings. Preserve a
   backup of every file to replace. Camera calibration, light pose, native LLP/LYA,
   inference weights, drive math and physics parameters remain unchanged.

## Package layout and installation

The source exporter `prepare_bundle.py` reads Git objects from the exact pinned
commit and writes a new bundle with source hashes. It does not export model
weights or recordings. Stage the integration workers next to that source under
`Assets/StreamingAssets/SemanticPlanningNeo20261015/`. Keep the historical
`Assets/StreamingAssets/SemanticPlanning/` tree intact.

Only the existing C# semantic transport is extended with a default-off runtime
mode. Add the independent editor-only `AIFormulaNeoTopoTrial.cs`. The guarded
installer rejects an unknown or changed destination and writes a backup and
before/after receipt. Review the staged manifest and destination differences
before applying it.

From the repository root in PowerShell, prepare a fresh stage. The packaged
exporter must pin the fixed commit; the explicit check below rejects an older
exporter before installation. These commands assume the existing configured
Simulator and its detector/model environment; they do not install weights or
replace the original LLP/LYA runtime.

```powershell
$ErrorActionPreference = 'Stop'
$Repo = (Get-Location).Path
$Package = Join-Path $Repo 'codexws\unity_neo_topo_20261015'
$Python = 'C:\Users\PC\AppData\Local\Programs\Python\Python313\python.exe'
$Demo = 'C:\Users\PC\OneDrive - Sophia Univ. Students\Main\SOPHIA\Y4T1\Research\Demo_LaneLineTrainer'
$Simulator = Join-Path $Demo 'Simulator'
$Session = Join-Path $Repo ('local_artifacts\neo_topo_' + (Get-Date -Format 'yyyyMMdd_HHmmss'))
$Stage = Join-Path $Session 'install_tree'
$Baseline = Join-Path $Session 'source_before.json'
$Backup = Join-Path $Session 'install_backup'
$FixedPin = '3c6fbac2ff9f8dd23b247a187e966648876400b1'
& $Python -B (Join-Path $Package 'stage_install.py') --repo $Repo --output $Stage
if ($LASTEXITCODE -ne 0) { throw 'Staging failed' }
$Provenance = Get-Content -LiteralPath (Join-Path $Stage 'Assets\StreamingAssets\SemanticPlanningNeo20261015\provenance.json') -Raw | ConvertFrom-Json
if ($Provenance.source_commit -ne $FixedPin) { throw 'Wrong production source pin' }
```

After obtaining Editor ownership and the MCP/runtime snapshots above, capture
the current installed files plus protected source/scene/settings files. This is
an observed baseline, not permission to overwrite an unreviewed prior change.
Review the difference against the saved project state before the installer call.

```powershell
$ObservedFiles = @(
    Get-ChildItem -LiteralPath $Stage -Recurse -File | ForEach-Object {
        $Relative = $_.FullName.Substring($Stage.Length + 1)
        $Existing = Join-Path $Simulator $Relative
        if (Test-Path -LiteralPath $Existing -PathType Leaf) { Get-Item -LiteralPath $Existing }
    }
    foreach ($Relative in @('Assets\AIFormula', 'Assets\Scenes', 'Assets\StreamingAssets\SemanticPlanning', 'Assets\StreamingAssets\VisionLaneFollower', 'Assets\StreamingAssets\RoadDetector', 'ProjectSettings')) {
        $Protected = Join-Path $Simulator $Relative
        if (Test-Path -LiteralPath $Protected) {
            Get-ChildItem -LiteralPath $Protected -Recurse -File | Where-Object {
                $_.Extension -in @('.cs', '.py', '.json', '.yaml', '.yml', '.unity', '.asset', '.meta', '.shader', '.uxml', '.uss')
            }
        }
    }
)
$Rows = @($ObservedFiles | Sort-Object FullName -Unique | ForEach-Object {
    [ordered]@{
        path = $_.FullName.Substring($Simulator.Length + 1).Replace('\', '/')
        sha256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
})
$Rows | ConvertTo-Json -Depth 3 | Set-Content -LiteralPath $Baseline -Encoding UTF8
& $Python -B (Join-Path $Package 'install_bundle.py') --stage $Stage --simulator $Simulator --backup $Backup --baseline $Baseline
if ($LASTEXITCODE -ne 0) { throw 'Guarded installation failed; inspect receipt before continuing' }
```

After installation, request a Unity import/compile through MCP, wait for idle
state, and read Console errors. Open the saved MotorControllerValidation scene
only from clean Edit. Run cases into new uniquely named directories.

## MCP case commands

The runner invokes the installed Unity MCP bridge on port 6400. It requires the
GUI Editor in clean Edit mode with the saved MotorControllerValidation scene
already open. Confirm the active bridge/Editor identity before using the commands.
Run one case at a time while holding the shared Unity/CPU lease; inspect its
result and cleanup before the next case. No automatic retry loop is intended.

```powershell
$Worker = Join-Path $Simulator 'Assets\StreamingAssets\SemanticPlanningNeo20261015'
$Bridge = Join-Path $Demo 'workspace\tools\invoke_unity_bridge.py'
$ExpectedHarness = (Get-FileHash -LiteralPath (Join-Path $Package 'trials\AIFormulaNeoTopoTrial.cs') -Algorithm SHA256).Hash.ToLowerInvariant()
$Runs = Join-Path $Session 'runs'
$Common = @(
    '--output-root', $Runs,
    '--source-root', (Join-Path $Worker 'source'),
    '--motor-gate-source', (Join-Path $Worker 'motor_gate.py'),
    '--worker-root', $Worker, '--simulator', $Simulator,
    '--bridge-script', $Bridge, '--expected-harness-sha256', $ExpectedHarness
)
& $Python -B (Join-Path $Package 'trials\run_neo_trial.py') inner_traffic_01 --kind traffic --lane inner --lane-recovery-mode component-only @Common
if ($LASTEXITCODE -ne 0) { throw 'Case rejected or interrupted; preserve evidence and analyze' }
# Run separately after inspecting the previous case and its clean shutdown:
& $Python -B (Join-Path $Package 'trials\run_neo_trial.py') outer_traffic_01 --kind traffic --lane outer @Common
if ($LASTEXITCODE -ne 0) { throw 'Case rejected or interrupted; preserve evidence and analyze' }
```

Use `--kind closure` for a full-lap closure-only case. A recovery case includes
an actual partial-learning restart, continuation to closure, and a second restart
with the closed reference; it uses the same body pose, simulation clock and lane
epoch throughout. The explicit component-only option below is the analyzed trial
for the inner baseline support failure; it is not the default and is recorded in
the configuration. Run the two commands separately, after each cleanup review.
The full actual lap includes stationary holds for these recovery tests; it does
not reset odometry. A passing recovery case also establishes closure, so no
additional uninterrupted-lap case is required.

```powershell
& $Python -B (Join-Path $Package 'trials\run_neo_trial.py') inner_recovery_01 --kind recovery --lane inner --lane-recovery-mode component-only @Common
if ($LASTEXITCODE -ne 0) { throw 'Recovery not accepted; inspect result and preserve this run' }
& $Python -B (Join-Path $Package 'trials\run_neo_trial.py') outer_recovery_01 --kind recovery --lane outer @Common
if ($LASTEXITCODE -ne 0) { throw 'Recovery not accepted; inspect result and preserve this run' }
```

`--partial-restart-distance` defaults to 120 m and schedules only the test hold.
`--lane-recovery-mode baseline` preserves the original recovery flags.
`--no-panel-rgb` disables the existing optional traffic panel correction for that
case and must be treated as a different perception configuration. The default
records its active state. None of these options enables bbox distance.

Keep `result.json`, `configuration.json`, `integration_hashes*.json`, motion,
semantic/event traces, frames and checkpoint snapshots. A successful result must
also have clean shutdown, unchanged input-source hashes and real restore
receipts; compile, PID change or a saved checkpoint alone is insufficient.

## Runtime separation

Set `NeoStopWaitMode=true`, an explicit new `WorkerRoot`, `SourceRoot`,
`MotorGateSource`, `CheckpointRoot` and per-case `CheckpointPath`;
`TrafficPolicy=stop_wait`, `EnableSignalApproach=false`. The source worker rejects
range payloads. No second controller, command topic, manual route, teleport after
initial placement, or simulated traffic decision is injected.

Checkpoint paths must be absolute and isolated as `root/case/topo.json` (the
adapter also accepts the explicitly supported `checkpoint.json` filename).
Never use the vehicle's `.ros` path. Production intentionally disables its store
for use_sim_time; this integration keeps use_sim_time honest and uses an explicit
simulation initialization adapter. Windows uses byte locking, file fsync and
atomic replacement. POSIX directory fsync and power-loss durability are not
claimed. Production checksum, fingerprint, pose/time/frame continuity and restore
checks remain in force.

Schema remains 1. MapAssist 0.3.0 can reject a valid 0.3.1 snapshot containing
negative signed progress. Preserve an older compatible snapshot for rollback;
do not clamp, rewrite or relabel new evidence to make it pass an older reader.

Semantic-only restart requires a runtime DiagnosticHold and actual stationary
vehicle. Keep the detector, follower, lane epoch, pose and simulation clock
continuous. Wait for a clean checkpoint shutdown acknowledgement, a fresh boot
identity and production restored state. A process restart alone is insufficient.

## Cleanup and rollback

Stop the case through MCP, preserve results and failed runs, leave clean Edit,
verify worker PID plus process birth identity has exited, and compare source and
parameter snapshots. Restore the previously opened scene without saving runtime
trial state. Return Editor ownership with the actual state and changed-file list.

For rollback, restore only files listed in the install receipt after verifying
their current SHA still equals the installed SHA. Leave new bundle files archived
unless their explicit removal is required. Never replace an entire Assets tree,
reset another checkout, or overwrite a later editor/agent change.
