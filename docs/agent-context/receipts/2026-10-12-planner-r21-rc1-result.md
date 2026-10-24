# R21 RC1 CI result — append-only reviewed receipt

- Type: RESULT_READY (evidence only; not acceptance of code or permission to drive).
- Scope: exact commit `79ad3438c4d90d5436b73151796fff1febb4a908`, tag `lane-mapping-20261011-r21-rc1`, canonical repository ID `1303517209`.
- Input handoff: `2026-09-28-planner-neo-004`. Corresponding sealed session: `2026-10-11-planner-model-consistency-r21-018`; it was not modified.
- Commit: https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/79ad3438c4d90d5436b73151796fff1febb4a908
- Previously verified session ZIP: 12663 bytes, SHA256 `72bbd526511f33c74e7657f59c8f215fb57e6bf82fccfdbc5d535ad778c36b9c`.
- Raw diagnostic ZIPs remain local; this text omits endpoint IDs, coordinates, private paths and raw messages. Sealed session018 is unchanged.

## Terminal CI state

All eight automatic workflows are completed: **6 success / 2 failure**. Their 16 jobs are **14 success / 2 failure**. Each link identifies the exact run, not a retry:

| Ref | Lane Foxy | GNSS Foxy | Native Windows/Linux | Handoff matrix |
|---|---|---|---|---|
| branch | [success](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38149701280) | [failure](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38149701237) | [success](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38149701249) | [success](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38149701214) |
| tag | [success](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38149701093) | [failure](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38149701071) | [success](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38149701076) | [success](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38149701092) |

| Run | Job ID | Job | Conclusion |
|---|---|---|---|
| 38149701237 | 114500398300 | foxy-build-and-dds | failure |
| 38149701071 | 114500397877 | foxy-build-and-dds | failure |
| 38149701280 | 114500398765 | foxy-build-and-dds | success |
| 38149701093 | 114500397971 | foxy-build-and-dds | success |
| 38149701249 | 114500398485 | native (windows-latest) | success |
| 38149701249 | 114500398595 | native (ubuntu-latest) | success |
| 38149701076 | 114500397927 | native (ubuntu-latest) | success |
| 38149701076 | 114500398030 | native (windows-latest) | success |
| 38149701214 | 114500398387 | context-and-offline-tools (ubuntu-latest, 3.13) | success |
| 38149701214 | 114500398570 | context-and-offline-tools (windows-latest, 3.13) | success |
| 38149701214 | 114500398649 | context-and-offline-tools (windows-latest, 3.8) | success |
| 38149701214 | 114500398684 | context-and-offline-tools (ubuntu-latest, 3.8) | success |
| 38149701092 | 114500398028 | context-and-offline-tools (ubuntu-latest, 3.8) | success |
| 38149701092 | 114500398129 | context-and-offline-tools (windows-latest, 3.13) | success |
| 38149701092 | 114500398176 | context-and-offline-tools (ubuntu-latest, 3.13) | success |
| 38149701092 | 114500398188 | context-and-offline-tools (windows-latest, 3.8) | success |

Only the failed GNSS jobs' logs were inspected in this follow-up; successful job states are API evidence, not an invented new count of their tests.

## Actual GNSS failure and counts

Both jobs built **5 packages**, then completed **3598 Linux unit tests**: 2768 reference and 830 GNSS, **0 errors / 0 failures / 0 skipped**. The separate Header check passed **4 tests**. The combined build/regression/DDS step subsequently exited 1 when `runtime.json` reported failure. Warnings in test stderr are not unit failures.

Each actual-DDS synthetic runtime report has **2/9 passing scenarios** (`default_guard`, `startup_gap`). The remaining seven fail identically at `test/runtime_smoke.py:300`, waiting for `engine.counts["accepted"] >= 5`:
`SmokeFailure: timeout: five actual stationary camera commits`.
The wait's default is **6 seconds** (`runtime_smoke.py:190-195`). **121 named checks were reached and passed per run**, but later checks were not executed; do not call this a full DDS pass or reuse a historic larger check count.

The seven failed scenarios' command JSONL evidence:

| Scenario | Branch attempts/accepted/deadline/guard/bootstrap-failure | Tag attempts/accepted/deadline/guard/bootstrap-failure |
|---|---|---|
| normal | 20/0/15/0/5 | 21/0/14/0/7 |
| no_gnss_build | 21/1/14/0/6 | 21/0/15/0/6 |
| future_gps | 20/0/11/3/6 | 21/0/15/0/6 |
| missing_bits | 19/1/8/4/6 | 20/0/15/0/5 |
| uncertainty | 20/1/12/2/5 | 21/0/15/0/6 |
| future_wheel | 21/1/14/0/6 | 21/0/15/0/6 |
| future_gyro | 20/1/14/0/5 | 21/0/15/0/6 |

Branch totals: 141 attempts, 5 accepted `bootstrap_odometry`, 88 processing-deadline rejections, 9 commit-guard rejections, 39 bootstrap-anchor failures.
Tag totals: 146 attempts, 0 accepted, 104 processing-deadline rejections, 42 bootstrap-anchor failures.
Every scenario's first mapping result was `processing_deadline`; all fourteen failed scenarios finish with `bootstrap_anchor_not_established`, invalid engine phase, zero confirmed cells and 0 or 1 trace point.

Across the fourteen failed scenarios:

- Deadline-rejected engine processing: **81.304508–154.611296 ms**, exceeding the unchanged 80 ms engine budget.
- Logged worker compute: **227.049540–439.427878 ms**.
- Mask source-to-command-log-event age: **270.367427–595.553312 ms**, calculated as `(wall_time_ns-source_ns)/1e6`. This is not receipt-only latency, queue-only delay, or worker compute.

These observations establish repeated deadline/guard rejection and subsequent bootstrap failure. They do not attribute all cost to a specific function; these CI artifacts contain no stage profile.

## Safety boundary and R20 comparison

The timeout report is **TEACH**, not already HOLD: no visual source, streak 0, no bundle and no validated endpoint proximity. Subsequent shutdown records in **all 18 scenario command logs** invalidate localization into **HOLD** and end with explicit **speed=0, yaw_rate=0**, GNSS subscription inactive. The startup config has vehicle output disabled; commands use only the private DDS topic. The test explicitly reports `physical_stop_tested=false` and `production_speed_default_tested=false` (synthetic teacher fixture 0.2 m/s). These logs prove software cleanup in the isolated smoke test, not physical stopping, a completed lap, or Unity's current state. The `future_wheel`/`future_gyro` scenarios fail before their later future-motion injection checks, which are not claimed as passed.

The reviewed R20 receipt (`docs/agent-context/receipts/2026-10-11-planner-r20-rc1-result.md`, SHA256 `8a64bc6dfded9fb6c9aa35c13303a9ff35847568671388d801c273c27006cdf1`) reports the same 6/8 workflows, same seven-scenario five-commit timeout and deadline-to-bootstrap failure chain. Its unit count was 3418, versus 3598 here. Different CI runs are not a controlled speed benchmark; the overlapping timing ranges do not prove a robust runtime improvement.

## Local evidence and review

- `jobs_snapshot_02.json`: all job identities, steps and terminal states.
- `gnss_job_excerpts_01.json`: exact selected failed-step log lines, raw cached-member hashes; runner evidence path redacted.
- `gnss_artifacts_01.json` and `gnss_download_identity_01.json`: exact run/head/artifact identity and verified ZIP digest.
- `gnss_runtime_summary_01.json`: redacted per-scenario counts, latency ranges and software-shutdown state.
- `collection_notes_02.json`: retained local collector issues; no CI rerun or source modification.
- Private ZIPs: branch artifact `11706246993`, 411368 bytes, SHA256 `0cd96d26adfccf8db3e6b31e02eb48518678ccb0236b5975f4a59f040ea2b4d8`; tag artifact `11706542661`, 413632 bytes, SHA256 `3b1a14bd3c277be39be282f0fd45e9e262d79fa57992d9bffde252b2f05e20b5`.
- Only text was inspected. No native binary was downloaded, extracted, installed or executed. No workflow dispatch/rerun, Unity calls or tests occurred.

Next owner: planner/root. This append-only receipt records the inspected results; diagnose the existing runtime failure without relaxing gates. No code-acceptance or driving authorization is implied.
