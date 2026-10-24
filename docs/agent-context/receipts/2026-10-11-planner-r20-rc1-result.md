# R20 RC1 publication and CI result

- Role: planner subagent `sim_interface_audit`; read-only remote audit on
  2026-10-11. `RESULT_READY` for analysis, **partial/failed checkpoint**, not
  deployment or driving approval.
- Input handoff: `2026-09-28-planner-neo-004`. Exact checkpoint:
  [51890caaac3be996b6ffef53eee7c64cd8043859](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/51890caaac3be996b6ffef53eee7c64cd8043859),
  branch `repair/lane-keyframe-transaction-20261011`, tag
  `lane-mapping-20261011-r20-rc1`.
- GitHub API independently verified canonical repository ID **1303517209** and
  `Sophia-AI-formula-team/neo_aiformula_sophia`; actual origin push URL agrees.
  Remote branch and annotated tag object
  `d651bacda51860c37adf77001dcd6512bcd7bb27` peel to that exact commit.
  Local HEAD also agreed at readback. Older untracked curve-fit prototype files
  were preserved, not included or tested by this audit.
- [Sealed session017](../sessions/2026-10-11-planner-pixel-footprint-r20-017/SESSION.md)
  and [ZIP](../bundles/2026-10-11-planner-pixel-footprint-r20-017.zip) were not
  modified. ZIP content read from the exact remote commit, local ZIP and sidecar
  agree: **230542 bytes**, SHA256
  `623a3f9e0188a961f9aa32277f07c69dad167efa003b2f9adf771671043a5e04`.
  Remote Git blob is `05b96321f721c6c9b462ee0fe0ed22502370b4b3`.

## Automatic CI: 6/8 successful, all terminal

All eight runs are `push`, attempt 1, on the exact SHA. Final readback completed
at 2026-10-11 14:39 UTC: **14 successful / 16 jobs**, two failed GNSS jobs.
There was no dispatch, rerun, cancellation, commit, push or binary installation
by this audit. Repeated branch/tag and platform runs are not unique test counts.

| Workflow/ref | Run | Result | Job IDs | Artifact IDs |
| --- | --- | --- | --- | --- |
| Lane Foxy / branch | [38147454792](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38147454792) | success | 114493693906 | 11705093840 |
| Lane Foxy / tag | [38147454789](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38147454789) | success | 114493693876 | 11704594425 |
| GNSS Foxy / branch | [38147454759](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38147454759) | failure | 114493693817 | 11705486720 |
| GNSS Foxy / tag | [38147454790](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38147454790) | failure | 114493694123 | 11704634450 |
| Native / branch | [38147454737](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38147454737) | success | Windows 114493693902; Linux 114493694148 | Windows 11705103646; Linux 11704544123 |
| Native / tag | [38147454772](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38147454772) | success | Windows 114493694265; Linux 114493694011 | Windows 11704354515; Linux 11704514114 |
| Handoff / branch | [38147454736](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38147454736) | success | 114493693763, 114493694075, 114493694085, 114493694129 | none |
| Handoff / tag | [38147454794](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38147454794) | success | 114493694128, 114493694211, 114493694277, 114493694319 | none |

- Each Lane Foxy run built four packages and passed **2588 unit tests**, plus
  **4 Header tests**, **63 synthetic DDS checks**, and **26 real upstream
  LYA/Header checks**. Its control-demo step also completed. These are not a
  full-lap vehicle/Unity trial and do not validate the GNSS raw-motion path.
- Each GNSS Foxy run built five packages and passed **3418 unit tests**
  (2588 reference + 830 GNSS), with zero errors, failures or skips. Header
  **4 tests** passed before the subsequent DDS failure. Fixed has no independent
  unit tests. The old R19 exact-margin failure did not recur on R20.
- Every native Windows/Linux job reported **487 passed + 954 subtests passed**;
  this is not 1441 independent top-level tests. Source/prefix checks and real
  calls to all eight APIs passed, each API version 1. C source SHA256:
  `0c54086e66554c8e9c96b48893e4b19c076c8a747b8f2cf08d6ad216fdeca12d`.
  The four Foxy native reports independently verify the installed Python 3.8
  module and real calls; each records binary SHA256
  `e220d6615422a2b310a544b871ac13ab417b3a133818eca66624911c9b627cf3`.
- Each of eight handoff matrix jobs ran **33 tests, OK**, on Windows/Linux
  and Python 3.8/3.13. No local tests, solver, build or Unity operation were run
  as part of this publication audit.

## GNSS DDS failure: actual engine records, not merely a timeout inference

Both GNSS workflows fail the step
`Build all three packages, regression and real DDS isolation checks`, after
build and unit success. Each `runtime.json` reports only **2/9 scenarios passed**:
`default_guard` and `startup_gap`. The other seven reach this exact failure:

```text
runtime_smoke.py:300, run_case
wait(lambda: node.worker.engine.counts["accepted"] >= 5,
     "five actual stationary camera commits")
runtime_smoke.py:195, wait
SmokeFailure: timeout: five actual stationary camera commits
```

All 121 named checks reached before those waits passed in each run, but seven
scenarios aborted before later checks; this is not the historical full DDS
151-check pass. At timeout the safety snapshot still reports TEACH, no visual
source, visual streak 0, no bundle and no endpoint proximity approval. Existing
shutdown records subsequently invalidate localization and output zero.

The original command JSONL records below establish the actual map outcome.
`PD` = `processing_deadline`, `CG` = `commit_guard_rejected`, `BA` =
`bootstrap_anchor_not_established`. All accepted results are `bootstrap_odometry`.

| Scenario | Branch attempts / accepted / PD / CG / BA | Tag attempts / accepted / PD / CG / BA |
| --- | --- | --- |
| normal | 20 / 1 / 12 / 1 / 6 | 20 / 0 / 14 / 0 / 6 |
| no_gnss_build | 20 / 1 / 13 / 0 / 6 | 20 / 0 / 14 / 0 / 6 |
| future_gps | 21 / 1 / 13 / 1 / 6 | 20 / 0 / 14 / 0 / 6 |
| missing_bits | 20 / 0 / 14 / 0 / 6 | 20 / 0 / 14 / 0 / 6 |
| uncertainty | 20 / 0 / 15 / 0 / 5 | 20 / 0 / 14 / 0 / 6 |
| future_wheel | 20 / 0 / 14 / 0 / 6 | 20 / 0 / 14 / 0 / 6 |
| future_gyro | 20 / 1 / 14 / 0 / 5 | 20 / 0 / 14 / 0 / 6 |

All fourteen failed scenarios have zero confirmed cells and final engine result
reason `bootstrap_anchor_not_established`, phase `invalid`. A scenario with one
accepted bootstrap has one trace point; the others have zero. The first result
in every failed scenario is `processing_deadline`. Exact per-scenario counters,
all timing ranges and original log hashes are in the local summary.

- Across the fourteen scenarios, deadline-rejected engine times range from
  **80.814830 to 166.789930 ms**, above the unchanged 80 ms gate.
- Recorded worker `compute_ms` ranges from **229.365831 to 448.273455 ms**.
  This is worker elapsed computation, not only the engine budget or an exact
  end-to-end admission sample.
- Logged image-source-to-result-event age ranges from **262.372096 to
  540.526838 ms**, computed as recorded `wall_time_ns - source_ns`. It is not
  sensor receipt age, queue-only latency or the exact commit-guard time.
- Artifacts do **not** record a `worker.last_error` field or an independently
  named `last_reason` status field. The final reason above is the last actual
  `teach_mask.result.reason`; absence of a worker error field is not evidence
  of an empty error. No unsupported causal attribution is made for the two
  guard rejections or individual processing stages.

These records establish repeated deadline rejection followed by failure to
establish a bootstrap anchor. They do not identify which extraction/packing
stage dominates, nor prove how a future optimization will perform. Do not
relax publication cadence, freshness, bootstrap, compute or geometry gates to
turn this checkpoint green.

## Artifact integrity and local evidence

All eight runs' full logs and complete run/job/step/artifact metadata were read.
Four Foxy evidence archives were read and their downloaded bytes verified
against GitHub artifact digests. Only bounded text members were retained;
no CI native binary was downloaded, imported or installed by this audit.

| Artifact | SHA256 of downloaded archive |
| --- | --- |
| 11705093840 | `2bd546b5e284720efd6fa942317aa3181d551526ec66f973241a27c50877f917` |
| 11704594425 | `c4ff3ef0814574579e3f9aa7d0a7738e3e8f1eeda8a6c627a94dac4beb7e3bd0` |
| 11705486720 | `221eab90715bf1a108938bdfb257dbf2017965e869491cb0419cdc300ea8f836` |
| 11704634450 | `b3898606cf1d62460b45953b154c69c5abe78a1c870f4cb5204b7ed8da3972cd` |

Evidence remains local under `r20_publication_01`; this receipt contains no
personal filesystem paths, raw images, GNSS coordinates, credentials or binary
payloads. Native artifact metadata/digests are preserved, not mistaken for
locally verified installed binaries.

- Initial `ci_readback_01.json` SHA256
  `af052de7487005fef4fa22db52a8621282f08fe409ae4a14b80b7b2184aa587b`.
- Terminal `ci_readback_02.json` SHA256
  `95d0179d68e69c554bce3a0f71f327020088e09fac2bedbe8fa3692788cfc68e`.
- `artifact_texts_01.json` SHA256
  `90dab78f6cf169999e168aa906c13291a994ec5c189d6eb720fd7cd4b6d828e5`.
- Derived `summary_01.json` SHA256
  `bbcbc3fec9ab2101dd27ca5467fb7ff28b48277b3a92c36c19c9ec0c1f171363`.
  It binds each original log and preserves exact counts/timing definitions.
- `command_logs_readback_01.json` lists all 18 preserved synthetic DDS command
  logs and their hashes. Two local PowerShell display commands had parser
  errors; corrected read-only display commands made no change to evidence or CI.

## Lap outcome and next owner

Sealed017's Windows core **3417 passed / 1 Linux-only skip** does not supersede
the two actual Foxy DDS failures above. Four unchanged development cases remain
**0/4**, with accepted counts **69/435/13/413**; two pitched cases reached the
finite-domain capacity gate and their corrected lateral RMSE was worse than raw.
The strict recorded prefix remains **11/12 attempted frames accepted**, all
eleven bootstrap steps; seq162 exceeded the original engine gate at
**81.3592 ms**. Those are fixed recorded inputs, not a new trajectory.

No new Unity TEACH lap, frozen route or REPEAT lap occurred. Latest actual
driving evidence remains failed trial28. Vehicle, physics, simulator, camera,
LYA speed and original safety gates were unchanged by this audit. Root retains
exclusive ownership of any future Unity operation and real-vehicle permission
remains absent.

Next owner: planner. Preserve this failed checkpoint; measure actual worker
stage latency and repair the geometric/representation issues under unchanged
contracts, then validate a new candidate separately. CI/build success alone
cannot authorize driving. This append-only receipt was prepared locally for
review; the auditor did not commit or publish it.
