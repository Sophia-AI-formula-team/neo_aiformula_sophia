# Unity transport update problem analysis — 2026-10-15

This report precedes implementation. Scope: a simulation-only stdio transport
for the current neo MapAssist 0.3.0 stop_wait policy and original LYA outputs.
No semantic package, original LYA/LLP, camera, light, plant, model or bbox source
will be modified. No tests, Unity refresh/play, installation or model run have
been performed by this agent.

## Concrete incompatibilities in the existing Unity wiring

1. The current worker imports map_assist.py as one file but does not add the
   semantic_local_planner_ros package root. MapAssist 0.3.0 now imports TopoState
   and topo_checkpoint; the package must be imported intact.
2. The old command path routes MapAssist private output ABC through another LYA
   instance. That repeats the vehicle green-resume defect: topo's diagnostic
   readiness can block the existing fresh LYA stream. Current neo motor 0.4.1
   gates the original fresh LYA command using traffic_stop_required alone.
   The candidate will carry the existing detector LaneControl v/w and target,
   retain its sequence/epoch/source-age checks, and use the original MotorGate
   policy without importing motor wheel/USB sigmoid code. Topo ABC remains a
   read-only observer. Neither closed-loop map success nor a missing private
   ABC will fabricate or suppress an otherwise fresh LYA command.
3. Old --signal-approach and signal_range payloads are incompatible with this
   requested stop_wait mode. This candidate explicitly refuses signal approach
   and never sends or consumes signal_range. The separate bbox experiment stays
   preserved and unused.
4. Production TopoState intentionally disables checkpoints with use_sim_time.
   The simulation must not report a false real clock to evade this guard. A
   separately named simulation-only subclass will reproduce only the small
   checkpoint-initialization lifecycle with explicit isolated case paths. The
   production save/restore/continuity/diagnostic methods and thresholds remain
   unchanged. use_sim_time stays true; get_clock and odom stamp both use the
   unscaled actual Unity simulation time; all traffic arrival votes and timeout
   checks continue using real time.monotonic(). This is simulation-adapter
   checkpoint evidence, not native ROS / production use_sim_time acceptance.
5. Production topo_checkpoint imports POSIX fcntl and fsyncs its directory.
   Native Windows cannot import that module or perform this directory fsync.
   The simulation-only storage adapter will map the original nonblocking lock
   call to a one-byte Windows lock and override only save's platform durability
   step: validate the same state, encode the same schema/fingerprint/checksum,
   fsync the temporary file, then atomically replace within one directory. No
   directory-fsync or power-loss durability is claimed on Windows. The original
   load, validation, fingerprint, continuity and close methods remain in use.
6. Old Stop kills the Python worker without close_checkpoint. The new worker
   closes checkpoint in finally and supports a shutdown request; the Unity
   adapter waits a bounded interval for clean exit, records forced cleanup as a
   failure, and cannot label a killed worker's checkpoint as graceful recovery.

## Minimal runtime API and ownership

The new component retains Begin/Stop/TryGetCommand. Runtime-only properties:
SourceRoot, MotorGateSource, CheckpointRoot, CheckpointPath, TrafficPolicy
(only stop_wait), DiagnosticHold, WorkerGeneration and WorkerBootId.
NeoStopWaitMode is false by default; it and WorkerRoot must be explicitly set
to use the new separate SemanticPlanningNeo20261015 worker directory. The
existing SemanticPlanning directory and historical default lifecycle remain
available. Changing mode while a worker is requested is rejected.
RestartWorkerPreservingOdom stops only the semantic worker, waits for checkpoint
close/exit, and starts a new worker against the same case path. It never changes
the vehicle pose, velocities, Unity time, detector, lane epoch or LYA parameters.
DiagnosticHold causes TryGetCommand to return false so the existing follower
brakes during the controlled restart; it is explicit test harness state and
never a traffic decision. The old follower stays active throughout.

Per-case paths must be absolute, within an explicit isolated checkpoint root,
with a separate case subdirectory; the adapter refuses vehicle ~/.ros paths.
The harness must choose a fresh case directory for each new run. A worker restart
uses exactly the same case file, simulation clock, odom frame and source tree.

## Verification that remains pending

- Scoped synthetic transport tests: stop_wait is exclusive; small/no/range bbox
  payloads cannot influence traffic; fresh raw LYA forwards after green with no
  topo diagnostic ABC; no old command replay, duplicate votes, stale commands,
  source-epoch replay, time rewind, or topology-restart clock reset.
- Windows lock contention, identical format/hash/load, checkpoint persistence,
  clean close, restore under continuous odom, and rejection of mismatched pose,
  source time, config or frame; no substitute synthetic lap for Unity closure.
- Genuine MCP inner/outer runs: fresh RGB A3 → unchanged LLP/LYA, observed stable
  red → zero command + vehicle settled, fresh stable green → new LYA movement;
  full actual lap with closure checks and reference; stop/restart only semantic
  and prove sample/reference restoration before continuing. Record fusion
  separately: closure complete does not imply official map fusion available.
- Videos preserve actual recognition, LLP/LYA and semantic diagnostics with at
  least 75% of the main view unobscured. Every failure remains in evidence.

Source inspection only; no functionality has yet been accepted.

## Candidate review finding before its correction

The first unpublished adapter candidate passed capture-time backdated timestamps
to MotorGate.command. This does not match production's arrival-time contract.
MotorGate.hold(command_timeout) advances auto_after on every empty tick. A real
frame arriving with 0.1–0.2 s of capture age can therefore always predate that
barrier and never start, despite fresh source data. This is a transport defect,
not a MotorGate/semantic-source defect. The correction uses actual command
arrival for MotorGate and independently enforces source age in the adapter and
at Unity consumption. Source TTL is never refreshed by an odom-only tick.
Focused tests must include nonzero capture-age startup, timeout recovery and
red/green release, with unchanged production MotorGate logic.

The same review notes that restored can become saved before a status consumer
samples it. The transport will retain a read-only restore_event captured
immediately after the real production _on_odom restore callback and before the
status timer submits a subsequent save. It includes the actual source status,
sample count, reference length and triggering tick/odom stamp; it does not
manufacture restoration from saved alone.

## Atomic color freshness finding before candidate 02 correction

The first candidate used production _on_light_state, a legacy String callback.
Although the adapter rejects stale inputs at entry, the legacy callback discards
capture age: later stop_wait green release checks only age since arrival. Native
semantic_vehicle.launch uses _on_signal_observation; its later green-release
check adds original capture age to elapsed arrival time. A green frame already
0.45 s old can age beyond the 0.5 s limit while waiting for the full vote window.
Using the String callback could then release where the native atomic callback
continues waiting. This is a real transport mismatch, not a semantic-core issue.

Candidate 02 will call the unchanged atomic callback with the actual accepted
color, sequence and measured source age, distance_m=None, uncertainty_m=0. No
metric range, bbox measurement, confidence or class is fabricated. Signal
approach stays disabled and traffic_policy remains stop_wait. The previous 20
test results stay attributed to candidate 01, which is preserved before edits.

Native capture.py copies A3 traffic_light_state and source Header; bridge
SignalCapturePolicy checks identity/freshness and normalizes missing-range
uncertainty to zero. Neither schema has a confidence field or changes class
scores. Unity's existing detector can optionally use its pre-existing RGB panel
tracker; the harness must record that actual source/configuration and not claim
native A3-only recognition equivalence when this option is active. This change
does not modify those detector settings or score thresholds.

The added regression uses real monotonic elapsed time, an already latched red,
a new green vote window and accepted frames with 0.45 s capture age. At the tick
where green voting matures after the last frame has expired, traffic must remain
wait; only a subsequent genuinely fresh green frame may release it.
