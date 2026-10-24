# inner_traffic_01: observed failures and minimum harness correction

This analysis is written before revising the staged harness. No production semantic, traffic voter, native MotorGate, LLP, LYA, detector frequency, recovery flag, freshness limit or bbox mode is changed.

## Actual evidence

The preserved run `runs/inner_traffic_01/result.json` is rejected. It remained in `red_wait`, recorded 252 detection frames / 960 chase frames, travelled 2.12632418 m, and ended with follower `Invalid or expired lane command: timestamp/sequence`. It did not test green release, topo closure or recovery. Cleanup receipt says clean=true, exited=true, forced=false, exit_code=0. This run must not be promoted to a pass.

The semantic responses around wall 68.99-71.29 s demonstrate the validator mismatch: raw detector remains red, production traffic_mode=wait and traffic_stop_required=true, applied_v=0, while voted `traffic_light` alternates red/unknown as a 5 Hz capture ages between updates. Examples: sequence1351 at69.025 s votedred;1352 at69.086 s votedunknown;1353 at69.162 s votedred. All three remain wait/stop=true and appliedzero. The evaluator required votedred on every editor tick for three seconds, repeatedly restarting its timer despite the correct native latched stop. Repeated red_zero_started events are preserved in events.jsonl.

The analogous green assertion also requires votedgreen every intermediate tick for one second. The same sampled-vote expiration could prevent acceptance even after a real green confirmation correctly releases the production stop latch. Green is not yet observed in this run, so this second issue is a source-derived prediction, not a claimed experimental green failure.

## Separate root-induced end-of-run interruption

Root reports issuing an MCP execute_code introspection around wall71.4 s. The probe result `traffic_state_01.json` reports lane wall_age1.0019227 s while sim_age0.0786872 s. The next terminal evidence records sequence247, wall_age1.0112906 s, beyond the unchanged native0.75 s command age limit. The visible gap from semantic samplewall71.2914 to72.3126 is consistent with the approximately1 s Editor execution block. This is a test-operation interference, not evidence that normal continuous inference cannot meet its deadline. Preserve the rejection and original watchdog; future cases only poll written files during Play, without mid-run execute_code introspection.

## Minimum correction

1. Red: require one actual fresh filtered-red confirmation while native wait/stop is latched. Archive its actual semantic response/detection identity and event. Then require continuous full linear/angular body zero, zero applied command, fresh raw detectorred, and native wait/stop latch for three seconds. Intermediate votedunknown does not erase a valid native latch. Reset the hold timer on motion, stale/missing/red-changed input or latch release; invalidate current confirmation on input/latch discontinuity. Never accept a stop without an actual filteredred confirmation.
2. Green: after the real panel switch, require one actual fresh filteredgreen confirmation. Archive it. Then require continuous fresh raw detectorgreen, native mode=normal and traffic_stop_required=false, positive applied and actual forward motion for one second. The final five-metre criterion still requires the current released condition. A prior red confirmation or unknown alone never authorizes acceptance.
3. Add actual full world linear/angular vectors plus their magnitudes to every Sample, so offline evidence can verify stationary intervals. Existing directional actual_v/w stay for plotting.
4. Store the initial red/green confirmation evidence in result.json. These are evaluator observations, never inputs to semantic or the motor.

Required validation after install: root runs a new unique case02 through MCP, retains case01, and checks true red hold and green release with the unchanged native controller. This subtask performs static edits only and starts no tests, model, MCP or media job.

## outer_closure_01: cold first-frame headroom (before option change)

This separate run had no mid-run MCP introspection. Its first captured result at wall3.7849449 s has frame1 wall_age0.7177206 s and inference567.5662 ms. The existing follower accepted that nearly expired first result under the0.75 s active-command TTL; before the next result arrived it stopped at frame1 wall_age0.7533019 s. Frame2 was then available atwall3.8840051 s with age0.1505127 s and inference35.0317 ms, but the follower was already stopped. The run is rejected after4.8792746 s with only0.001502037 m movement. This is an actual startup handoff issue, distinct from the root-induced interruption in inner_traffic_01.

The minimum wiring correction is to enable the existing runtime-only RequireFirstFrameHeadroom=true in the new harness. Its existing first-accept gate waits for a result no older than0.5 s; active-command TTL remains0.75 s. This is not a follower algorithm edit, timeout relaxation, LYA tune or perception change. Save the original value, explicitly record the temporary option in configuration and events, and restore it during cleanup. The earlier statement that all flags stay as found is superseded for this one named startup option; frequency, recovery and native controller parameters remain as found. Retain outer_closure_01 as rejected evidence and use a new case02.

The outer Evaluate gate returns early when the semantic result is absent or older than0.25 s. Continuous traffic-hold/release timing must not bridge that unobserved interval. Before the next case, clear both traffic timers and current red/green confirmation flags in that branch. Preserve first-confirmation evidence for audit; reacquire a real native confirmation after freshness resumes. This changes the evaluator only and leaves all native freshness thresholds intact.
