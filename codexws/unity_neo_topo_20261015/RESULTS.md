# Results — 2026-10-15 JST

The final four Unity cases passed independent log audits on production commit
`3c6fbac2ff9f8dd23b247a187e966648876400b1` (MapAssist 0.3.1). Both lanes
passed red hold/green resume and a full lap with partial-map and closed-reference
semantic-process recovery. **Official-map fusion remained unavailable**; these
results establish a closed odometry reference, not successful official topology
alignment or a left-turn detour.

## Final cases

| Case | Lane option | Red hold | Green resumed path | Observed front-to-light gap | Result |
|---|---|---:|---:|---:|---|
| inner_traffic_03 | component-only | 3.035 s | 5.012 m | 20.732 m | PASS |
| outer_traffic_03 | baseline | 3.001 s | 5.032 m | 22.711 m | PASS |

Each case contains a genuine native filtered-red confirmation, continuous sampled
full-body zero/applied-zero evidence, a genuine filtered-green confirmation, and
at least one second of continuously released positive motion. The final motor
gate is `semantic_go`. Hold samples: 122 inner / 126 outer; maximum body speeds
0.02325/0.01685 m/s, angular speeds 0.000866 / 0.000598 rad/s, both within the declared
0.03 thresholds. Occasional filtered `unknown` observations during the hold are
accepted only while fresh raw red and the native wait latch remain present.

Red is scheduled from the case start. This proves stop/hold and green recovery;
it is not a claimed moving green-to-red braking-distance test. The gaps above are
observations only. **stop_wait does not target 8–9 m.** Distance input, bbox-distance,
signal approach and automatic detour stay disabled throughout all four cases.

| Case | Lane option | Recorded distance at closure | Closed reference | Samples | Exact partial/reference restored prefixes | Travel including resumed motion |
|---|---|---:|---:|---:|---:|---:|
| inner_recovery_03 | component-only | 262.608 m | 264.068 m | 950 | 447 / 950 | 267.466 m |
| outer_recovery_03 | baseline | 285.227 m | 286.589 m | 1027 | 437 / 1027 | 289.939 m |

All ten production closure checks were true. Closure occurred 1.482 m / 3.128° from
the inner start and 1.382 m / 0.655° from the outer start. These remain within the
unchanged 1.5 m / 25° thresholds. Each recovery case drives the actual full lap, with
stationary holds for a partial-map restart and a closed-reference restart; no
teleport, manual route, portal or reconstructed lap substitutes for closure.

Every restart has a clean old-worker checkpoint acknowledgement, a new boot ID
and generation, and a genuine same-boot `restore_event` captured while production
persistence state is `restored`. The partial/reference prefixes compare exactly
against the final checkpoint; schema 1, SHA and fingerprint checks pass. Detector
identity, active lane epoch and time remain continuous. Maximum recorded restart
position drift is below 0.000001 m; final restored LYA motion is 3.049 m inner and
3.024 m outer. After the acceptance event, teardown intentionally increments the
lane epoch; cleanup samples are recorded separately from restart continuity.

Official-map alignment RMSE is 3.200 m inner and3.470 m outer, above the unchanged
3 m quality gate. Both runs therefore remain `odom_only`, with
`map_fusion.available=false` and reason `rmse_exceeds_limit`. Official map nodes
are not used to fabricate a successful alignment or control route.

## Source and configuration evidence

All four cases record the same 97 source hashes, with no before/after differences.
The runtime export identifies fixed production commit 3c6fbac, based on upstream
ed8e397. The necessary production correction permits finite signed progress only
after closed-route creation; it does not alter geometry, closure thresholds,
official matching or traffic decisions. See the
[checkpoint problem and regression archive](../../workspace/src/aiformula/perception/semantic_planning_system/semantic_local_planner_ros/docs/checkpoint_signed_progress_20261015/README.md).

Original LLP/LYA source and parameters, detector frequency 5 Hz, camera calibration,
light pose and plant remain preserved. Two existing runtime options are explicit:
all final cases enable first-frame headroom; inner cases enable
`RecoverProjectedComponents=true` with `UseCurrentCorridor=false` (component-only).
Outer cases retain baseline lane flags. The existing panel-RGB traffic correction
is enabled in all four cases, so signal classification is not described as A3-only.
The harness saves and restores the original runtime options; baseline remains the
default. Source preservation is checked against the recorded original files,
including the existing LLP Python adapter and original LYA path; it is not a new
native C++ numerical-parity claim.

The actual seq 42 mask failure from `inner_closure_02` was independently replayed
before the final live run: baseline reproduced `[0,4,0]`; existing component-only
recovery returned `[8,4,4]` from a real 852-pixel current component spanning 3.5 m.
That offline evidence explains the chosen option; the final live cases supply the
separate closed-loop evidence.

All shutdown receipts within the four final cases report clean exit without forced
termination and `checkpoint_ok=true`. Independent process-monitor receipts bind
observed runner/semantic PIDs to process birth timestamps and confirm their
absence after the cases. A PID value alone is not used as exit/recovery proof.

## Evidence boundaries and retained failures

This is actual Unity Play using genuine RGB model output, existing LLP/LYA,
production semantic state and motor traffic gate through a stdio ROS-message
adapter. It is not a native ROS/DDS, vehicle, sensor-noise, host-reboot or power-loss
durability test. Restart tests hold the body stationary and restart only semantic;
the detector and simulation continue. Motion checks cover logged samples rather
than independently observing every physics step.

Earlier failures remain preserved: the first traffic evaluator incorrectly
required every intermediate filtered color to remain red, and a root live probe
then caused a separate capture watchdog failure; outer_closure_01 accepted a
nearly expired cold frame; inner_closure_02 lost projected lane support; and
outer_closure_02 closed a reference but failed checkpoint persistence on signed
progress. None is relabeled as a final pass. Necessary changes were analyzed before
repair. The separate bbox-distance branch and prior left-turn investigation are
not merged into this acceptance.

## Review files and video status

- [Four-case summary](evidence/results/summary.json)
- [Traffic audit](evidence/results/traffic_audit_03.json)
- [Inner closure/recovery audit](evidence/results/inner_recovery_03_audit.json)
- [Outer closure/recovery audit](evidence/results/outer_recovery_03_audit.json)
- [Exact source identities and runtime options](evidence/results/source_identities.json)

Per-case result and compact process-identity receipts accompany these files.
Original image/video captures and large traces remain in the local run archive;
their hashes identify the audit inputs. Video encoding and visual QA are recorded
separately; this document does not treat a planned video as a verified deliverable.

## Completed video and cleanup review

Five videos are encoded and fully decoded: the four final passing cases plus the original outer checkpoint failure. All are 1280x960 at 15 fps, with the main view occupying 75 percent. Red/green and restart events run at 1x; longer learning intervals are explicitly 4x. See [MEDIA_RESULTS.md](MEDIA_RESULTS.md), [video paths and SHA256](evidence/media/final_media_receipts.json) and [root sampled visual QA](evidence/media/root_visual_qa.json). Original captures remain unchanged.

Unity was restored to the original clean Edit scene. Both scene snapshots preserve all 116 parameters and complete component/body state apart from realtime. Source preservation and a 135-file byte-identical deployment rebuild are recorded in [cleanup evidence](evidence/cleanup/deployment_rebuild.json). Shared Editor/CPU resources were explicitly returned at 01:05 JST.
