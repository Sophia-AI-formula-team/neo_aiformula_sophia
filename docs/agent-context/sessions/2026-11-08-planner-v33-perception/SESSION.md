# Agent session: 2026-11-08-planner-v33-perception

Input handoff: 2026-11-08-planner-v3-acceptance

Result: **partial / training admission denied**. This is a simulator experiment record, not vehicle acceptance. No new long training, vehicle output or deployment was started.

## Objective and scope

Repair unreliable near-camera road interpretation before spending more training compute. The user requested new acceptance gates and Unity trials, then training only after a feasible solution. Existing workers should be reused; this turn explicitly authorized freeing stale worker quota. The user also retains authorization to upload review videos to Owen Drive.

Scope is the isolated learned-racing laboratory and this session. Preserve the prior V3.0/V3.2 acceptance, original Unity project, real-car controller packages, motor backups and all unrelated work. No complete map, GNSS, world pose or obstacle truth enters the policy. Referee geometry is allowed only offline or in the simulator's separate scoring process.

The preregistered requirements are in [evidence/PROTOCOL.md](evidence/PROTOCOL.md): same-input near-field residual/coverage/false-free checks; 8/8 new full laps with mean progress speed >=2 m/s; 4/4 lane-mask-perturbed laps; 16/16 new obstacle segments; independent execution-prediction verification. All gates must pass before >=10,000 procedural training episodes.

## Input handoff

Acknowledge 2026-11-08-planner-v3-acceptance at incoming commit 5e8c8e73a1e8b5f9cc5cb652834e8efeb4dcd76f. Read AGENTS.md, AGENT_CONTEXT.md, STATUS.md, PROTOCOL.md, EXPERIMENT_RUNBOOK.md, laboratory README, and the prior acceptance/session. Used the agent-handoff skill to recover ownership and isolated worktree state.

The previous 30,000 surrogate episodes did not establish a usable Unity baseline. That checkpoint is retained; it was neither reused as an accepted initializer nor retrained this round.

## Source and working tree

Task branch: experiment/driving-v3-3-perception, separate worktree from the previous acceptance. Incoming HEAD: 5e8c8e73a1e8b5f9cc5cb652834e8efeb4dcd76f. The branch was clean before this task; new laboratory files plus UnityRace.cs/run.py and scoped evidence changes belong to this task. Exact final publication commit is supplied by the later receipt, not guessed here.

Each trial manifest names exact source SHA256 and arguments; 24 distinct source snapshots were verified and deduplicated into codexws/learned_racing/unity_laps/v33_evidence/sources/<sha>.source. Sources for earlier prototypes remain available even though the current files moved on. Large raw results and frames remain local under each experiment's runs/ and .work/; Git intentionally excludes them.

Installed A4 SHA256: b4d3556ded29ada232682b607a11296d723ffe74814a85cc4f2bcbf573e5b1e3. Its interface provides lane/object results, not a drivable-area segmentation output. No weights were changed. Existing CUDA Python, OpenCV, NumPy, SciPy and PyAV were used without installation. Only one physical GPU is available; no multi-GPU or training-throughput claim is made.

## Work performed

Work occurred on 2026-11-08 JST. Detailed per-run arguments, versions, outcomes and source hashes are in [evidence/all_trials.json](evidence/all_trials.json). Local events.jsonl retains UTC start/completion events. The private Unity copy was the only editor writer; original simulator assets and installed dynamics were not edited.

1. Registered gates before qualification. Inspected original road processing: independently selecting the closest grey strip per image row and using FOV clipping as a physical road direction creates large false heading changes.
2. V3.3.0 separated physical edges from clipping; V3.3.1 added causal 0.8 s wheel/gyro-transformed history. An initial speed-confounded comparison was explicitly corrected; V3.0 was rerun at the same 2.8 m/s parameter. Its two full-lap attempts failed at 18.43 m and 14.60 m, whereas V3.3.1 completed two development laps. Higher-speed V3.3.1 and larger-yaw-limit V3.3.2 failed and were retained.
3. First fresh-start V3.3.1 qualification failed at 134.96 m. V3.3.3 added camera-measured paint spacing with the user-provided two-lane class prior; V3.3.4 rejected conflicting physical-edge tangents. This is a disclosed prior, not a learned arbitrary-road topology or preloaded metric map.
4. V3.3.4 completed two development laps in 112.74/115.30 s, with minimum body-road gaps +0.104/+0.268 m. Eight obstacle development segments passed only 2/8. Its next frozen qualification completed three laps, but the third took 144.50 s, averaging 1.94 m/s, so the batch stopped before the remaining five starts. A completed slow lap is still a gate failure.
5. V3.4 introduced feedback-predicted passage candidates, a zero-speed option, per-constraint uncertainty and longer retention of missing nearby obstacles. V3.4.1 removed the dark-seam brightness exclusion; V3.4.3 also removed the white-paint exclusion. V3.4.2 explicitly tested uncertainty weight 1.0 instead of 1.8, with geometric margin 0.18 m unchanged. This is a manual risk-parameter comparison, not a calibrated probability or trained result.
6. V3.4.4 bridges a cone silhouette only when pavement pixels exist on both sides of its detected box; the obstacle remains occupied. V3.4.5 corrected the incompatible tangent/endpoint construction that could first steer away from the chosen passage. It passed the A/B/C development segments in 14.09/17.02/17.35 s, with actual minimum cone gaps 0.296/0.582/0.213 m. It departed during the consecutive-obstacle segment and stalled on both empty-course full-lap attempts. Therefore reject the overall V3.4.5 candidate despite local improvement.
7. Generated nine reviewed 1920x1080 comparison/replay films, totalling 396.1 s. All streams fully decoded successfully. Films preserve initialization, failures, real camera/chase frames, candidate decisions, geometric margins, scores and actual speed. A same-input near-field film isolates the original, spatial and causal temporal processing. No synthetic driving images were generated.
8. Reused the existing selected third-party workers after the user-authorized stale-activation reset. One failed with HTTP 413/context size; another hit HTTP 429 and automatic retries consumed the shared 8-request activation. No independent review result returned. No paid overflow, selector change or further reset was used. Requested Astra was not in the checked provider roster, so work continued locally.
9. Computer Use list_apps timed out twice and again after the prescribed JavaScript reset. No new Drive folder or upload was completed. Previous V3.2 Drive delivery remains untouched; this session's external_artifacts is deliberately empty.

Representative reproducible commands from the laboratory directory (private Unity must already be ready):

```
python -B v33_qualify.py run --name V3_3_4 --stage clear
python -B v34_trials.py V3_4_5_tangent --cases 05,06,07,12 --pace 2.8 --parameters v33_evidence/V3_4_2_parameters.json --version V3.4.5
python -B v34_trials.py V3_4_5_clear --cases 01,02 --pace 2.8 --parameters v33_evidence/V3_4_2_parameters.json --version V3.4.5
python -B -m unittest test_v34 -v
python -B v34_perception_media.py V3_4_4_occlusion --run 05
python -B v33_report.py
```

All listed trial processes completed normally; a collision/departure/stall is data failure, not a Python crash. Eight behavioral unit checks passed (exit 0). The first report-generation attempt raised KeyError for an older manifest without an explicit version; fixed the version provenance handling, rerun exited 0. No new execution-step calibration, disturbed full-lap batch, final 16-layout obstacle qualification, V3.2-parameter comparison or long training was run.

## Evidence and result

- [Training admission](evidence/training_admission.json): allowed=false; new_training_episodes=0.
- [All 49 Unity trials](evidence/all_trials.json): includes failed and successful prototypes, controlled same-speed baseline, and stopped qualification.
- [Frozen qualification results](evidence/V3_3_4_clear_qualification.json): 3 completed laps / planned 8; third violates speed gate. Five unrun cases do not count as passes.
- [Visual overview](evidence/验收总览.png), [near-field comparison](evidence/V3_4_4_occlusion_05_perception_0040.jpg), [full-lap regression](evidence/V3_4_5_clear_01_end.jpg).
- [Nine film hashes and full-decode checks](evidence/video_manifest.json). Local review entry point: codexws/learned_racing/unity_laps/v33_evidence/delivery/index.html. Files are not yet on Drive.

The architecture remains hybrid: pretrained A4 vision, an inherited data-fitted vehicle-response model, causal local geometry, and explicit trajectory/footprint/risk scoring. New driving parameters are not trained. Geometry uncertainty is not a calibrated success probability. Policy sensor fields remain exactly t/leftRpm/rightRpm/gyro/wheelRadius/source plus camera pixels, with no world pose or map.

The first replay statistic had unequal sampling support; it remains archived and is not acceptance. The corrected replay uses seven common 1.2–3 m depths and reports coverage/counts, but equal counts do not prove same physical-edge identity. The added offline referee diagnostic uses world truth only in scoring and explicitly excludes it from policy imports; its planar geometry and missing occlusion/pitch handling make it diagnostic only. Near-field acceptance has not passed.

Unity uses the installed three-wheel model, all-wheel-center departure approximation and the same body/cone contact rule. Empty-course means no additional test props; native background objects remain visible. Body-edge excursions are reported separately. Simulation results do not establish real-tire contact, real-car braking or physical acceptance.

## Risks and blockers

Main technical blockers are road-boundary ambiguity, conservative constraints that make recovery impossible once current state violates preferred clearance, consecutive-obstacle planning, and unvalidated long-horizon execution predictions. V3.4.5 must not replace a baseline or be advertised as faster overall.

Process defect retained: the original V3.4 development runner continued after three consecutive stalls in V3.4.3. An automatic three-identical-terminal-failures stop was added afterward; it does not retroactively convert those development results into registered acceptance.

Current A4-derived road pipeline still uses a grey-pavement color prior and two-lane class prior, and can misread inner drivable regions. Occlusion bridging is an explicit inference. Do not silently describe inferred or clipped space as newly observed road.

External blockers: optional worker service failures and unavailable Computer Use prevent worker review and Drive upload. These do not justify training anyway. Public bundle includes only code, aggregate results and reviewed simulator stills; no raw bags, credentials, account details or raw external telemetry are included.

## Next agent actions

1. Preserve all evidence and rejected versions. Inspect the full-lap regression and near-field film before modifying coefficients again. Establish independent visible-road edge correspondence and false-free labels; do not optimize the current self-residual alone.
2. Reconcile road inference with current footprint support and design a measured recovery condition. Do not simply remove stopping or resume the old positive-speed least-harm fallback. Verify fixed-command response and several-second prediction error before extending lookahead or speed.
3. Re-run the failed development full laps and consecutive layout. Only after development feasibility freeze another candidate and create fresh held-out seeds; previously inspected failures are development data now.
4. Complete every gate. Only then compare retained V3.2 initialization with a fresh parameter search and launch >=10,000 independent procedural episodes with truthful GPU/episode accounting. User allows ending constant training supervision after real progress and no errors.
5. Restore the approved Owen Drive upload path and upload the nine curated films, overview and index without changing sharing permissions or overwriting previous acceptance. Verify upload and playback separately.
6. Current work is partial. No human permission is needed for these authorized reversible local experiments; additional physical driving is not authorized by this record.

