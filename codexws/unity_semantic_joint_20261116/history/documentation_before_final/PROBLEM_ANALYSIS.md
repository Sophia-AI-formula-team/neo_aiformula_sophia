# Adapter-only corrections

## Replace the trial signal policy with calibrated RGB panel-only

Both v2 outer-start trials still allowed model red on low-confidence objects
without enough physical identity evidence. Changing cone scores/horizon gates
again would not establish that an object is the installed signal panel. The
independent five-run analysis preserves the failures and supports a narrower,
explicit simulator policy: retain raw model detections/overlay, but allow only
the existing calibrated current RGB-panel evidence and two-frame confirmation
to produce a traffic color. This replaces the active identity guard entirely.

The `signal_panel_only` option defaults false; Joint explicitly enables RGB and
this policy and restores their old values. Disabled RGB or archived guard
conflicts are rejected before prediction; missing/invalid calibration gives
unknown without using the old uncalibrated helper entry point. No signal distance
enters semantic, and model weights, mask/LLP/LYA and semantic source are unchanged.

The staged 13 boundary checks and 3268-frame parity pass preserve complete
pre-v1 output when disabled; enabled results equal the original calibrated
helper with model_state unknown while retaining actual raw model_state as
diagnostic evidence. Inner05/06 saved frames can form native green votes while
stationary; mixed-render inner04 cannot establish that property. This is a
saved-JPEG signal study, not new live driving acceptance. It cannot repair the
separate official-map ambiguity or inner06 short-lane observation gap. See the
new panel-only report and receipt in `evidence/`.

## `outer_joint_01`: broaden the opt-in identity evidence, keep the source planner unchanged

The first outer-start run stopped at 95.72359 m before completing a lap because
two false red model outputs entered the unchanged semantic vote. One had no
overlapping cone; the other cone had a lower score than the light. The v1 guard
was enabled but its deliberately narrow conflict rule did not cover these
cases. This is not evidence of a topology-fusion failure: no reference was built.
The audit's `PERCEPTION_INPUT_ANALYSIS.md` sections 7–8 preserve the original
frames and compare the existing helper on saved JPEGs before further changes.

The reviewed v2 predicate retains the v1 conflict rule and independently rejects
a model light only when enough current same-color pixels all lie below the
calibrated horizon, including a half-pixel uncertainty margin, and no current
same-color RGB panel identity supports it. Missing evidence preserves the model
input. This relies on the stated simulator lamp/camera geometry; replay of JPEGs
is not original PNG inference or live acceptance. Raw detections and overlays
remain evidence, and signal truth is never a controller input.

The C# change here only renames the single default-false, nonpersistent option to
`EnableModelLightIdentityGuard`, sent as `signal_panel_model_identity_guard`.
JointTrial captures the original value, explicitly enables it, records it, and
restores it. There is no old-name alias or second mode. The v1 C# bytes are kept
under `history/perception_identity_v1/`; the original pre-trial rollback files
remain in `before/`. Staging is not a live deployment. The complete v2 Python
payload and its own review must accompany these names before any installation;
no algorithm/threshold/map/LYA/TTL/camera change is part of this rename.

## Opt-in rejection of conflicting below-horizon traffic-light detections

The root's follow-up of `CRITICAL_ISSUES.md` section 6 identified A4 class-wise
NMS retaining a cone and class-10 traffic-light detection on the same box. The
received false red then correctly holds the unchanged semantic planner. This
is a perception-output conflict, not a semantic mapping defect; the original
failed frames/results remain evidence.

The narrow C# wiring adds a nonpersistent, default-false
`RejectConflictingBelowHorizonLights` property and forwards it as
`signal_panel_reject_conflicting_below_horizon_lights` in the detector request.
Only this Joint trial explicitly enables it, records that choice, and restores
the original value during cleanup. Other modes retain their default false state.
The corresponding Python worker/pixel-and-class predicate is reviewed separately
by the root/audit agent; this C# change does not implement or broaden that
predicate. It does not use signal truth, vehicle position, or a hand-authored
route, and does not change model weights, camera calibration, semantic/LYA source,
traffic TTLs, scenes, or render settings. Compile and replay/live acceptance of
the complete adapter remain pending.

## `inner_joint_03`: the progress watchdog incorrectly included cold startup

The third attempt ended at `0.367093235 m` with 0 laps and reason
`No physical progress for 35 simulation seconds`. The first detection was about
30.52 seconds old and was correctly rejected. A later 0.195-second-old frame
started motion; a subsequent 4.236-second inference was stale and correctly
caused native zero-command holding. Another fresh 0.146-second-old frame arrived,
but the harness's 35-second progress budget had counted all cold-start time from
`Initialize`, leaving almost no actual driving time before the evaluator stopped
the run. This is an evaluation-timer defect; stale-frame rejection is correct.

Minimal correction: keep an independent 90-wall-second startup bound. On the
first actual nonzero applied autonomous command, record `motion_control_started`
and initialize the existing no-progress timestamp/distance baseline. Enable the
unchanged 35-simulation-second no-progress check only after that event. If no
command is applied during the startup bound, report a startup timeout. Keep all
physical positions, full traveled distance, lap counters, timestamps, and video
from startup onward unchanged. This adds no warmup sequence, command override,
time-scale change, TTL relaxation, semantic change, or steady-state timeout
extension.

## Joint command consumption must recheck the current lane epoch

Independent review of the new Joint consumer found a short ownership race. The
legacy visual branch checked `value.epoch` before applying a command. The Joint
branch correctly uses the semantic response instead, but `TryGetCommand` did not
compare that response's epoch against the detector's current `LaneEpoch` at the
instant of consumption. If the detector epoch changed, an earlier nonzero
semantic response could remain younger than the existing 0.25-second response
limit while the worker's mismatch error was still in transit.

Minimal correction: in Joint mode only, require an existing detector and exact
`Result.epoch == detector.LaneEpoch` before accepting a command. A missing or
changed epoch returns false, so the Joint follower applies zero input and brake.
Keep worker-side checks, all existing TTLs, fault handling, and legacy/Neo
behavior unchanged. This is a code-review race fix, not a failure observed in a
completed Unity detour. The next trial remains pending.

## `inner_joint_02`: visual follower permanently stops the separate Joint owner

The second live attempt reached `0.6031907 m`, with 0 physical laps and no learned
reference/red schedule. Its terminal semantic output was already zero with
`reason=visual_points_stale`; the original semantic watchdog was doing its job.
The trial ended because `AIFormulaVisionLaneFollower.FixedUpdate` permanently
called `Stop("Invalid or expired lane command: timestamp/sequence")` before
reaching `semantic.TryGetCommand`. That `Stop` shuts down the semantic worker and
disables lane control, so the next genuine fresh image cannot recover the run.

This is an ownership mismatch in the Unity adapter. The Joint package owns the
published command and already enforces its original 0.5-second visual capture/
command watchdog. Its valid detour commands may instead come from the map and
fresh odometry. Requiring the unused raw visual controller command to be fresh
and visually supported before consuming the semantic command also blocks that
detour path. The current run confirms the latency-induced permanent stop; it did
not reach a detour, so the detour restriction is a separate code-review finding.

Minimal correction: add a Joint-only semantic consumption branch after the
existing ownership, focus, driver E-STOP/UI, manual-input, selected-gamepad, and
explicit lane-pipeline-error checks. Keep `driver.AllowInput` and all finite/
fault checks. Call `semantic.TryGetCommand`; when it returns false, clear displayed
command values and apply `drive.SetInput(0,0,true)` without destroying the owner.
Only a later command that passes the unchanged semantic/transport TTL can resume
motion. Never reuse a raw visual command or extend its age allowance. The ROS
angular-sign conversion remains exactly once at the final motor call.

The old Neo/legacy visual-following code remains byte-for-byte unchanged below
the new branch. Raw visual mode is labeled diagnostic in Joint mode. This change
does not modify original LYA, semantic source, model/camera/physics settings, or
the trial's startup/warmup behavior. Real pipeline errors, nonfinite values, lost
ownership and faults still stop the session permanently.

## `inner_joint_01`: missing perception provenance on the Joint branch

The first live attempt stopped on its second fresh perception frame before a
valid map/turn experiment could proceed. The joint worker requires each accepted
vision packet's `vision_commit` to equal the pinned original LYA provenance. The
C# sender had placed both that provenance field and the Neo-only controller
command inside `if (NeoStopWaitMode)`. Joint mode therefore omitted the required
provenance. This is a transport wiring defect, not evidence of a semantic package
planning failure.

Minimal correction: send `lane.commit` as `vision_commit` in both Neo and Joint
modes, while continuing to send the already-computed `controller` command only
in Neo mode. Joint still computes commands from the new package's actual A/B/C
through original LYA; it does not consume the Neo bypass command. Keep the
original run and precise worker replay/error evidence separate from the rerun.

Code review also identified a separate, not-yet-exercised detour wiring defect.
The joint worker marks `corridor_guard.applicable=false` when the package owns a
detour and commands are generated from its map/odometry path. The C# consumer
still applied the last visual corridor's deadline unconditionally, potentially
blocking a valid detour command after the old visual width prior expired. Both
deadline tracking and command consumption must honor this explicit flag only in
Joint mode. Neo and legacy behavior stay unchanged; missing/true applicability
still applies the original corridor checks. Command validity, finite values,
worker/tick identity, response age, and `command_gate.remaining_sec` remain
mandatory for every Joint command. No TTL is increased or invalid command ignored.

The transport before this correction had SHA256
`788ffac4fc69bb6f39b9fa3657bb5c95faa597bd27a0add3fc86072f64fc0fd3`.
The manifest records it as a prior experiment revision, not the original
pre-integration rollback target. The original transport remains archived intact.

## Source inventory and view selection

The initial trial runner recursively hashed every JSON file under `--worker-root`.
During local development that root was `audit/`, which can also contain mutable
smoke receipts and trial reports. A later diagnostic write could therefore report
"source changed" even when executable code was unchanged. This is an evidence
inventory defect, not a semantic planning defect.

The archived runner hashes the single `semantic_joint_worker.py`, the ROS source
tree, and named fixed simulator dependencies (including the original LYA
provenance manifest and its listed source files). Trial output must be outside the
ROS source tree. The archive supplies an isolated `transport/` directory for the
worker. No planning behavior, model weights, physics, camera settings, or LYA
algorithm changes are involved.

The old video harness recorded a chase-camera clone. The joint harness instead
saves the genuine `detector.Input` RGB preview and detector overlay with their
capture timestamps. It does not clone the temporarily repositioned RGB camera or
modify its calibration.

The old Neo worker implements a separate stop/wait experiment. An explicit
`JointExperimentMode` is necessary to execute the new package's actual route
points and traffic gate without silently applying the old traffic policy. The
worker receives actual perception points/light observations and simulated
odometry. Lap gates, physical lamp schedules, and evaluation metrics never enter
the semantic controller.

Simulation outcomes remain pending; these corrections do not demonstrate map
closure, official topology matching, or successful red-triggered turning.
