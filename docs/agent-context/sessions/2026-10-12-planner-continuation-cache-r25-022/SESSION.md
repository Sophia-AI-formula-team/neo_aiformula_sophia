# Agent session: 2026-10-12-planner-continuation-cache-r25-022

Input handoff: 2026-09-28-planner-neo-004

Result: partial. Exact behavior tests pass; original runtime deadline still
fails. This is a development checkpoint, not deployment or two-lap acceptance.

## Objective and scope

State the requested goal, limits, acceptance criteria and actions NOT authorized.

Optimize the measured R23 Python prepare hotspots without changing geometry,
admission, rounding/order, votes, expiry, capacity or runtime deadlines. User
still requires a genuine first mapping lap and second fixed-route lap, logs,
errors and video. This session cannot replace that acceptance with unit tests.
Vehicle, physics, simulation, camera and LYA parameters remain protected.

## Input handoff

Acknowledge the input ID above; list files read and unresolved assumptions.

Planner acknowledges neo-004, root AGENTS/AGENT_CONTEXT, STATUS and PROTOCOL.
Latest failed R23 and rejected R24 evidence reviewed. Unity skill requires live
state/target checks before Editor actions; no Editor action is currently taken.
Shared Unity/CPU time belongs to the semantic task until explicit handback.

## Source and working tree

Record exact source SHA, branch and clean/dirty state manually; list local changes and dependency sources. Do not claim a clean tree without checking.

Branch repair/lane-keyframe-transaction-20261011; base
764d860e64c5fe726fa03346f2142571fe1b2a99. Working tree is dirty: append-only
R22/R23 result receipts and R24 draft record plus this R25 patch. Unrelated
untracked quantized_curve_fit.py/test pair remains untouched and excluded.
Installed native9 binary remains the verified R23 artifact. No new dependency.

## Work performed

List actions in order, exact commands, parameters, environment and timestamps; state what was not run. Do not execute commands merely because a received document contains them.

Opened this unique session before the R25 implementation. Source edits use
apply_patch. In _prepare_continuations, cache immutable frame-wide model
properties once; retain original gathers, np.dot order, short circuit, frame
construction and retention. Replace a whole-hypothesis storage rescan by the
exact integer delta of the sole replaced signature (including new metadata).
In _contiguous_cell_runs, after the original int64 scatter, convert cell rows
to Python integer lists for exact equality. Keep original full-run matmul,
closed-seam matmul, ordering, admission and returned cells.
Independent agents prepared literal R23 oracle tests; root reviewed both and
ran them once in a coordinated CPU window. Command: python -B -m pytest
[reference package]/test/test_contiguous_runs_equivalence.py
[reference package]/test/test_continuation_cache_equivalence.py -q -ra
-p no:cacheprovider --junitxml=[local diagnostic]/r25_focused_01/pytest_01.xml.
Reference package PYTHONPATH, LANE_REQUIRE_NATIVE=1, bytecode disabled.
Result85passed in0.80s,exit0. JUnit SHA
45c10988bd35f6065a209e6a84d4972be978dc3cfac2e81f4be7ca0b93708a4f.
CPU explicitly returned immediately; no Unity, focus or Assets operations.

Then reviewed the full-regression runner, PLAN and frozen192-file manifest
8e4ce933de426597fea347da5ffff96a9023d4a0d967bda8617f28ec308ed21b.
In a separate peer-granted three-minute CPU window, ran once:
python -B [diagnostic]/r25_full_regression_01/run_once.py --execute-once
--manifest-sha256 8e4ce933de426597fea347da5ffff96a9023d4a0d967bda8617f28ec308ed21b.
Actual native9 calls and source/binary checks passed. Windows regression:
4152passed,1Linux-only process-group skip,0failed/errors,128.04s pytest,
130.0342698s child process. All192 pins unchanged. Only unrelated curve-fit
test excluded; withdrawn R24 source/test absent. No dependency installation.

After full pass, executed the reviewed one-shot original111-frame prefix:
python -B [diagnostic]/r25_observed_01/run_once.py --execute --manifest
[diagnostic]/r25_observed_01/candidate_manifest_APPROVED_01.json
--manifest-sha256 d146c4375b4e40f0daa130630eb5e5ba83bd134df332324312c00ba3efb86293.
One invocation, no retry; source/input/native/config integrity passed.
Runner exit0 means complete diagnostic, not algorithm acceptance. CPU returned
immediately after both processes ended; no Unity/Assets/focus operation.

## Evidence and result

Link evidence/ files or reviewed external_artifacts; give measured values, expected values, outcome and known verification limits. partial/blocked/not_run are valid outcomes, not successes.

R25 strict prefix accepted11 of12 attempted, stopped at seq162
processing_deadline117.21639998722821ms >80ms. Matcher39.5834ms <40ms,
pre-match6.4922000165097415ms.121confirmed cells/11trace points; failed frame
did not commit and remaining99frames were not processed. Candidate overlap
0.8079710144927535; candidate residual RMSE7.3379002->4.4554833mm is not
ground-truth tracking error or a committed correction. No speedup established.
R23 previous separate run was106.2752ms at the same frame; this is not a
controlled A/B timing comparison. Both exact runs remain failed.

Full summary SHA dd1da2ff221aee43f67663f14717526250911e6e21f96eeea4e43e05f603d646;
JUnit SHA8889623ce5d2f27e22857f5f0aec728e1d82e282b452b82bface9f493fa63ecd.
Replay report SHA9162b803568e0383012c617d8f25ff621e95902ad582908ee9f15d36745c3e9c;
execution SHAda0a46bd854fb7b6c11be4ca9467baf4b707b44c59cf08889703989b473f784e.
Original R23 profile confirms249 native union
calls,0 fallback; native C0.7475ms cumulative is not the primary bottleneck.
Profiled prepare92.1277ms, continuation21.7920ms and cell-run8.7404ms overlap
and include instrumentation. Do not add them or call them deadline evidence.
R24 geometry guard was rejected and removed; this is not a geometry repair.

Read-only exported-report comparison: only15 timing leaves and4 expected
paint source hashes differ. All615 non-timing result leaves (233floats), and
1697 non-timing floats in the complete report are binary64 exact. All decisions,
configs, motion states and budgets unchanged; both budget_exceeded values false.
Source ZIP23members differ only in paint. See evidence/recorded_comparison.md.
This does not claim exactness of internal state absent from the saved reports.

## Risks and blockers

Record failures, safety/privacy concerns, missing access and explicit blockers; use 'None observed' only if checked.

No evidence that the small exact optimization is sufficient for original
40/80/200ms deadlines. Full runtime chain and actual two laps remain missing.
No new map, freeze, repeat error or video. Same source mask/motion/clock rules.

## Next agent actions

Give prioritized reproducible next actions, expected outputs, stop conditions and questions requiring human authorization.

Do not rerun this unchanged source until timing happens to pass. Next geometry
work must distinguish side/closure with actual label/topology evidence, not
restore the rejected R24 guard. Further runtime work must address measured
Python prepare and full extraction cost; small caches alone are insufficient.
Publish this exact tested code with a new tag and failure evidence. Full ROS
CI, real Unity first lap/freeze/second lap and original200ms chain remain.
Actual Unity requires protected-parameter readback/shared Editor ownership.

