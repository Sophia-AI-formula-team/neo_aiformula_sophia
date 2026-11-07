# Agent session: 2026-11-08-planner-v3-acceptance

Input handoff: 2026-11-07-planner-driving-v3

Independent simulation acceptance completed; the trained model fails. External delivery is separately tracked by the reviewed upload receipt.

## Objective and scope

User requested acceptance, clear comparison videos showing vehicle criteria and decisions, and upload to Owen Google Drive. Scope is frozen-model simulation evaluation and presentation. No vehicle, deployment, paid provider overflow or model retuning. Eight external-worker requests had already been used; parent handled this task locally.

## Input handoff

Read prior driving-v3 session, laboratory README, AGENTS, AGENT_CONTEXT, STATUS, PROTOCOL and EXPERIMENT_RUNBOOK. Shared agent-context status pointed to the root bag-consolidation note; the experiment's own handoff is authoritative here. Training completion was verified against status and the actual 30,000-row journal. Generation 118 was frozen before new evaluation.

## Source and working tree

Clean incoming baseline bf63f2044af1addebf5acf65972a4f1164f6283c. Original experiment/driving-v3 preserved. Created isolated worktree lap_v3_acceptance_20261108_wt, branch experiment/v3-unity-acceptance-20261108. Changes are limited to the learned-racing Unity laboratory and this record. Checkout CRLF initially differed from training hashes; normalized content was identical, and exact original LF planner bytes were restored before evaluation. No planner algorithm change occurred.

## Work performed

Registered 16 new paired layouts before seeing outcomes. Frozen V3.0 defaults and V3.2 final parameters. Added post-return profile instrumentation recording all candidates, margins, scores and selected action without altering the original calculation; fixed only the model-version label in the A4 server.

Commands: python -B acceptance_run.py prepare; python -B acceptance_run.py run; then separately declared python -B acceptance_race.py, all exit 0. Existing private Unity 2022.3.22f1 copy restarted under owned PID 25764. Original 32 body-edge attempts remain intact. After identifying the old gate as stricter than the user's rule, a separate 32-run cohort used all actual three wheel centers outside. It is a rule-sensitivity check on the same cases, not another blind sample. Camera/A4, physics and coefficients stayed fixed; default old referee gate remains unchanged.

Rendered 16 paired full-attempt films, captioned overview, referee-only fixed-coordinate movie, training plot and report. acceptance_media.py, acceptance_diagnostics.py, acceptance_verify.py exit 0. Corrected no-feasible-score captions before final re-render; no control/data change. Computer Use timed out, recovered after the user's explicit retry instruction. Native file picker rejected two long typed filename lists before upload. A staged directory containing only the verified 22 delivery files enabled exact native selection. No credentials or unrelated browser content entered evidence.

## Evidence and result

Actual Unity acceptance fails: V3.0 2/16 completed, 0/3 whole laps, 14 departures; V3.2 2/16 completed, 0/3 whole laps, 12 departures and 2 cone contacts. V3.2 completes sequence case 12 but regresses on moving case 15. Only case 16 completes for both: 9.89 versus 9.49 seconds. No valid paired lap-time improvement.

512 original/instrumented command comparisons are exact. Across 2,379 shared time samples the referee variants have identical physical traces before the stricter stop. All 64 sensor streams/image counts checked; 2,544 candidate-bearing decisions agree with transmitted commands and margin/score calculations. Six V3 contract tests pass. All frames of 18 H.264 1080p movies decode successfully; total 537.7 seconds. The 90-file evidence ZIP is CRC/SHA256 verified. Full image archive remains local. Reviewed small evidence is in evidence/; complete records and delivery in the laboratory v3_acceptance directory.

## Risks and blockers

Training used a procedural surrogate, not 30,000 Unity runs. Road association, prediction/execution mismatch and least-harm action with no feasible path remain unresolved. Uncertainty margins are not calibrated passing probabilities. Installed simulation has three wheels; wheel-center departure approximates the competition rule and excludes tire contact-patch width. RPM is a target/plant proxy, not independent encoder feedback. No real vehicle validation is implied. All failed and interrupted tooling attempts are retained in this record and upload manifest.

## Next agent actions

Owen Drive upload is complete: 22 completed uploads confirmed in the UI, all 22 files listed after refresh, and the processed overview played successfully. See evidence/upload_manifest.json for local file hashes and UI verification; server-side checksums were not retrieved. No sharing permissions changed. Keep this candidate rejected. Any perception/prediction/recovery repair requires a new numbered version and new held-out scenarios. Do not change these frozen results. Owned Unity editor was stopped after all 64 runs; no vehicle output or new training remains running from this task. Source publication state is recorded in the final receipt.

