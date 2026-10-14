# Unity neo stop_wait verification harness: problem analysis before changes

Scope: a new editor-only AIFormulaNeoTopoTrial harness in staging. No live install, Unity MCP call, model run or tests are authorized for this subtask. Existing traffic/left harness, LYA, LLP, camera calibration, rigidbodies, map and bbox experiment stay untouched.

## Problems in the old harness

1. The old traffic case assumes signal-approach mode and a target 8-9 m stop. Latest neo uses stop_wait: red confirmation latches a stop, green confirmation releases it. Its acceptance must not require a distance target. The old left harness requests detour and accepts a learned reference without persistence verification. Neither matches the new request.
2. AIFormulaVisionLaneFollower.Stop() disables lane control and stops semantic. A Stop/Begin cycle is not a semantic-only restart; it may reset the detector epoch. For restart evidence the follower and detector must remain running, the body and both clocks must remain continuous, and only the semantic process may change. A default-off runtime-only semantic DiagnosticHold is the minimum test wiring to brake before restart while preserving those owners.
3. A changed PID is not proof of map restoration. The harness must observe a fresh worker generation and boot ID, production persistence.state=restored, no persistence.error, and preserved sample count/distance or closed reference. It also archives the actual checkpoint envelope before restart and compares its recorded sample prefix after restoration. The checkpoint is isolated inside the unique case directory.
4. closure.phase=odom_only/guarding alone does not prove official map alignment. Acceptance will require production closure.state=complete plus a positive finite reference_length_m. map_fusion is reported separately and is not silently promoted to successful alignment.
5. Prior inner-lane LYA lost visual lane support around 65 m. This is a known risk for full-lap runs. The harness will preserve that failure and stop; it will not alter LYA, inject a path, teleport after initialization, enlarge freshness gates or synthesize missing ABC points.
6. Perception UI textures are decoded/re-encoded previews. They are labeled as such. trace retains genuine native candidate/LLP/LYA and semantic payloads, and same-payload encoded lane PNG when available. No RGB preview is relabeled as exact model input bytes.

## Proposed cases and evidence

For each inner/outer start (x=68.76/72.53 m, z=-10 m, north-facing):

- traffic: real physical panel red, require genuine red detection + fresh semantic stop_wait/wait + zero applied and actual speed continuously for three seconds. Change only the physical panel to green, require genuine fresh green confirmation, released traffic latch, native positive command and actual forward movement; finish after 5 m resumed motion. Stop gap is descriptive only, never a control input or 8-9 m criterion.
- closure: real panel green, record one continuous LYA lap until production closure complete and finite reference. This separate case preserves an uninterrupted baseline.
- recovery: real panel green, after 120 m recorded distance brake using DiagnosticHold. Wait for actual stop and saved production checkpoint, restart semantic only, require restored partial samples and pose/time/detector identity continuity. Continue on original LYA until closure; hold and restart again, require restored closed reference; release and observe at least 3 m forward motion with new worker. The 120 m mark schedules a test interruption; it never commands route geometry.

Bounds: traffic <=180 wall seconds; full cases <=420 wall seconds; 260 moving simulation seconds; 600 m; startup/each restart <=35 wall seconds; no-progress <=15 sim seconds outside intentional hold. Failure and interrupted runs remain separate unique directories. No automatic retry.

Capture: observer clone writes the existing 1280x720 main view at 15 Hz, motion at 20 Hz, detector frames on actual sequence changes, semantic responses on actual response identity changes, events on transitions. Video composition should place genuine overlay, LLP/LYA, semantic/closure/persistence information in a separate footer; at least 75 percent of the output remains the main view. Capture FPS is a target, actual timestamps determine the eventual video timeline.

## Dependencies that must be verified before install

Transport must provide CheckpointRoot, CheckpointPath, DiagnosticHold, WorkerGeneration, WorkerBootId and RestartWorkerPreservingOdom() while preserving Requested across restart. It must reject EnableSignalApproach=true and omit range input. Result must preserve production status.closure/persistence and genuine gate state. This harness is not compile/runtime-verified while staged. Root must review final API names, hashguard install, compile via MCP and inspect Console before live trials.

## Static review corrections before live installation

The inherited red-stop check used only forward speed and yaw rate; that could count a laterally sliding body as stopped. The trial assertion must instead use full Rigidbody linear/angular magnitudes, without changing the controller. Green-resumed is a latched observation; final acceptance must additionally require the current released/light/command/motion condition so later motion cannot be assumed from the latch. Starting the case already red demonstrates zero-command hold and green release, not a measured moving-to-red braking distance.

Production persistence.state=restored can legitimately become saved before a sampled status. The transport now captures restore_event at the actual successful _on_odom restore, with worker boot identity and production persistence snapshot. The harness consumes that same-boot event and still checks sample prefix, recorded distance, reference closure/length, Body continuity and detector epoch. It never treats saved alone as evidence of restoration.

The transport Stop API returns void and reports an unclean checkpoint/process shutdown through ShutdownReceipt rather than an exception. The harness must record that receipt and mark cleanup failed whenever a worker was started but its receipt is not clean. A reported WorkerId=0 after disposal is not independent process absence proof; the root runner must still check actual owned process IDs separately.
