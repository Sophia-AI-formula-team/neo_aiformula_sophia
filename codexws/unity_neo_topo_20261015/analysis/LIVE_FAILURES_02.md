# inner_closure_02: current-component projection support failure

Analysis before any new trial-option wiring. This is an independent failed run, not an extension or replacement of the successful traffic02 cases. No image/model replay or live probe has been performed in this analysis.

## Observed failure

`runs/inner_closure_02/result.json` is accepted=false after13.1762089 wall seconds and18.3782959 m travelled. Production topo remains learning; the checkpoint saves69 samples/18.1998852 m, with clean shutdown. This is not a completed lap or closed-reference recovery.

The terminal stop evidence identifies frame42, epoch1, frame capture11.6455929 s, observed wall_age0.3101431 s. The follower stops with `Lost predicted lanes; upstream default-straight rejected`, not a TTL exception. Frame42 reports supported=false, mode=default_straight, selection_reason=no_observed_triplet and counts[0,4,0]. The left selected component has179 pixels, below the existing180-pixel support gate; its bottom row356 also indicates that the selected near component reaches the image bottom. The right component is accepted but projects only4 points and cannot supply the required observed triplet. No fabricated default-straight command should be accepted as a fix.

This is preceded by repeated current-frame projection failures: frame29 left projected_output_insufficient with929 component pixels; frame34 with1427 pixels and bottom359; frame35 with1132 pixels and bottom359; frame40 too_few_component_pixels171; frame41 projected_output_insufficient1147/bottom359; frame42 too_few_component_pixels179. Genuine support returns at frame43 with a1038-pixel left component and9 projected left points, but the follower has already safely stopped. A supported later frame is not evidence that frame42 contained a valid alternative.

## Existing option and its limits

The live existing `LanePublisher.recover_components` is adapter-only and explicit opt-in. It searches additional components from the same current mask in the original bottom-band/score order, uses a saved pre-frame ROI/jump reference, reuses the existing evaluate/projection checks, requires an actual observed5/6/7+ point triplet spanning at least2 m, and keeps the original side-spacing guard. Failed candidate searches do not commit recovered tracking state. It does not add mask pixels, relax the180-pixel threshold, synthesize a route, extend freshness, or modify A3/LYA.

This existing mechanism directly targets the observed failure class: a near first-choice component may fail support/projection while a different current component can supply the native triplet. Static evidence justifies a bounded optional trial; it does not prove that a valid alternative exists in every rejected frame or that a full lap will pass. Preserve the failed run and let the next actual run decide.

## Coupling that must remain explicit

`AIFormulaRoadDetector.RecoverProjectedComponents` defaultsfalse and `UseCurrentCorridor` defaultstrue. In the existing upstream adapter, setting only recovery=true also activates CurrentCorridor when UseCurrentCorridor remains true. That path includes the existing current-boundary identity and width policy; it is broader than the component-projection fallback itself.

Two existing wiring combinations are therefore distinguishable:

- component-only: RecoverProjectedComponents=true and UseCurrentCorridor=false. This isolates current-component recovery and the adapter's observed-point selection without activating current-corridor width inference.
- previous full adapter wiring: both flags true, as used in earlier semantic experiments. This also activates the current-corridor policy and must be described explicitly if selected.

Prepare an explicit optional harness parameter only after root selects the intended combination. Defaults must preserve current baseline. Save and restore original runtime values; record original/active flags in configuration/events/results. Do not change LLP source, A3 weights, LYA, TTL, geometry, camera or physics. A new run gets a new output/checkpoint directory and cannot overwrite this evidence.

Root selected the component-only candidate for the next bounded case. The optional harness argument will be laneRecoveryMode="baseline" by default, with the explicit alternative "component-only" setting RecoverProjectedComponents=true and UseCurrentCorridor=false only for that Play session. Both original values are restored during cleanup. The runner exposes the same two choices; no algorithm source changes or execution occur in this staging step.

## Frozen candidate static review: runner start/cleanup boundary

The harness saves/restores original detector overlay selection, panel-RGB flag, frequency, projected-component and current-corridor flags, first-frame headroom, focus requirement, semantic source/worker/checkpoint paths, neo mode, approach/hold, and diagnostic output. Initial vehicle pose/lamp schedule and hidden video observer are intentional case setup within Play; scene is not saved. Root exits Play between cases. All lane/control algorithm source remains unchanged.

The runner does not hard-code ed8e or another production commit. It resolves caller-provided source/worker/motor-gate paths and hashes their actual contents before/after each run; the harness itself is checked against the explicitly supplied SHA. Version provenance must use those recorded hashes and root's Git manifest, not an assumed branch name.

One definite cleanup boundary defect exists: the execute_code Start call and initial manifest write were before the runner try/finally. If the MCP reply or initial file write failed after Play began, the runner could exit without its owned-case stop path. Move those operations inside the existing try/finally after the clean-Edit preflight; set a start-attempted marker so failure cleanup is scoped to an attempted owned case. This changes only orchestration, not the harness or any trial result.

## Offline replay driver attempt01: launch flag compatibility

The owned replay driver failed before subprocess creation because this Python exposes no `subprocess.CREATE_SUSPENDED` attribute. No child or replay was launched; the Job reported zero active processes before/after cleanup. The original attempt and receipt remain under `analysis/owned_component_replay_42_01`. Parent PID32656 was independently absent after return. This is a driver error, not LLP evidence.

For the separately authorized attempt02, use Windows CREATE_SUSPENDED's documented numeric value 0x00000004 and retain CREATE_NO_WINDOW, assign the still-suspended child to the kill-on-close Job, then resume its original thread. Keep the same script/helper pins, 20-second soft replay limit, 24-second work deadline and 5-second cleanup bound. Write a new output directory; do not alter attempt01 or any recorded Unity run.

Attempt02 launched and cleaned its owned interpreter correctly, but rejected a mistyped expected publisher SHA before NumPy/cv2 imports or frame processing. The expected string accidentally had65 characters (`...b238ebbc...`); the unchanged file's actual64-character SHA is `c8ac3ef0a428928a78a18711160e247445b238ebc93eccdd5a39de27e32df1f5`. Trace and calibration hashes matched. Parent25640, suspended launcher45132 and actual interpreter46864 were independently absent after return. Preserve attempt02; create a separately pinned replay03 script/driver with the corrected publisher hash. No source/capture file changes are involved, and this failed attempt supplies no lane-recovery result.

## Completed fixed-mask replay03 evidence

Root authorized the corrected owned attempt03. It processed exactly the original sequence1..42 in2.046 seconds (whole owned driver2.249 seconds), without A3 re-inference, LYA execution, CurrentCorridor, Unity calls or control application. Original mask bytes were checked against each trace SHA; publisher/trace/calibration hashes remained unchanged. Both alternate publisher instances consumed the same ordered masks with independent persistent state.

At sequence42, baseline counts exactly reproduce the recorded `[0,4,0]`, with no observed triplet. The original left candidate remains179 pixels/too_few_component_pixels. Component-only recovers a different current left component:852 pixels, bottom277, rank1, tracking reference, observed span3.5 m. Counts become `[8,4,4]` and the existing observed-triplet predicate is true. This establishes that the existing option addresses this exact recorded failure; it is not a live trajectory, full-lap, LYA or topo-recovery acceptance result.

Receipts are in `analysis/owned_component_replay_42_03`: replay/result.json, replay/frames.jsonl, receipt.json, actual_identity.json and caller_absence.json. Kill-on-close Job finished with zero active processes; launcher6756, actual interpreter10120 and parent30132 were independently absent. Earlier driver attempts remain preserved. Frozen harness SHA stays `c7dfc35da6bb058da575915c5a67d7d55b76ee9042b298eb7735b303067ae005`; runner with the reported cleanup-boundary correction is `1f6919cc67aabc4befe53fa2849c26116c90254bc95547cedbeaf429a05a81e7`.
