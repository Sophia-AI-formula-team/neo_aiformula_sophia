# R26 exact-commit CI readback — FAILED

Canonical repository: `Sophia-AI-formula-team/neo_aiformula_sophia`, immutable ID
`1303517209`. This receipt uses only commit
`251eaebd9cfe902e703e3714af7d61515eb8e90a`; no R25 result is inherited.

Verified branch: `repair/lane-keyframe-transaction-20261011`.
Verified tag: `lane-mapping-20261012-r26-rc1`.
Annotated tag object: `6fcee085016e23821c44e876ca537e37c54b4c60`, peeled to the
same commit. Branch and tag runs are retained separately, all attempt 1.

## Terminal result

All 8 workflow runs (4 distinct workflow definitions × branch/tag) and all 16
jobs are terminal: **6 runs / 14 jobs succeeded; 2 runs / 2 jobs failed**.

| Workflow | Ref | Run | Jobs | Conclusion |
| --- | --- | --- | ---: | --- |
| Lane exact native graph | branch | [38164709067](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38164709067) | 2 | success |
| Lane exact native graph | tag | [38164709196](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38164709196) | 2 | success |
| Agent handoff integrity | branch | [38164709113](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38164709113) | 4 | success |
| Agent handoff integrity | tag | [38164709142](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38164709142) | 4 | success |
| Lane teach-repeat (ROS 2 Foxy) | branch | [38164709133](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38164709133) | 1 | success |
| Lane teach-repeat (ROS 2 Foxy) | tag | [38164709122](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38164709122) | 1 | success |
| Endpoint GNSS (ROS 2 Foxy) | branch | [38164709085](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38164709085) | 1 | **failure** |
| Endpoint GNSS (ROS 2 Foxy) | tag | [38164709165](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38164709165) | 1 | **failure** |

All 16 full job logs and exact job/step identities are saved under `final_01`.
`summary_01.json` contains per-job evidence line numbers and native identities.

## Build / unit / DDS separation

- Native Windows and Ubuntu: each of four jobs reports **646 passed + 954
  subtests passed**, nine native APIs and real calls passed. C source digest is
  `9dae4a7ab517d872af29d3c2d0a27d1cdfa7e8446c8f138be206df5ba4c4438f`.
- Handoff Windows/Ubuntu × Python 3.8/3.13: each of eight jobs reports **33 tests,
  OK**.
- Teach-repeat Foxy, independently on branch and tag: build summary **4 packages
  finished**; **3311 tests, 0 errors, 0 failures, 0 skipped**; additional **4
  pytest tests passed**. DDS runtime **63 checks passed** and neo-upstream runtime
  **26 checks passed**. Installed Foxy native nine-API verification reports true.
- Endpoint GNSS Foxy, independently on branch and tag: build summary **5 packages
  finished** (not the workflow step title's “three”); test phase **3311 + 932 =
  4243 tests, 0 errors, 0 failures, 0 skipped**, plus **4 pytest tests passed**.
  The later DDS runtime report has `passed=false`, causing process exit 1.

Thus the two failures are **DDS camera-commit checkpoint failures**, not build or
unit-test failures. The failed jobs are branch `114544752546` and tag
`114544752654`.

## Exact Endpoint DDS failure

Each run attempted the same nine synthetic stationary DDS cases:

- Passed: `default_guard`, `startup_gap`.
- Failed: `normal`, `no_gnss_build`, `future_gps`, `missing_bits`, `uncertainty`,
  `future_wheel`, `future_gyro`.

All seven failed cases stopped at:

```
runtime_smoke.py:300
wait(lambda: node.worker.engine.counts["accepted"] >= 5,
     "five actual stationary camera commits")
SmokeFailure: timeout: five actual stationary camera commits
```

Each final failed-case safety snapshot was `state=TEACH`, `phase=TEACH`, reason
`teacher command selected`; no bundle or visual source had been established.
Each run records 121 checks reached before case termination and all those checks
are true, **but seven cases time out before later checks**. This is not 121/121
complete DDS acceptance. The saved evidence identifies the checkpoint; it does
not determine whether insufficient commits arose from timing, geometry or another
worker condition. Warning text alone is not treated as a root-cause proof.

The runtime reports explicitly state `synthetic=true`, `actual_ros_messages=true`,
`physical_stop_tested=false`, `production_speed_default_tested=false`; fixture
teacher speed is 0.2 m/s. These are CI test facts, not user LYA speed settings or
physical vehicle facts. No full-lap/repeat or physical emergency-stop proof.

## Evidence identity and preservation

| Evidence | SHA-256 |
| --- | --- |
| `final_01/snapshot.json` | `e736cf22f84775a2dc9d01327cbd277abf606d59b8db0a2b445cfa055cdd85a8` |
| `summary_01.json` | `b2095d2238469acbfd8defb6817a6f12e0e49e8a20690ff799d8572882a63268` |
| branch validation ZIP | `e9a84bbf5d34883548c96b2ec6f26844f112d747b4e75073af6d014a10346ed1` |
| branch `runtime.json` (`reports_branch_01/report_122.json`) | `577e1030cfa6c6e39fea7ac00b4e1d681dc574d232fb6ba532e0332da63cd53f` |
| tag validation ZIP | `b40a26bc55cc2b55e77ff86dcbfe0c63dc11e8dc258d9a56baf9ccb932d6cb40` |
| tag `runtime.json` (`reports_tag_01/report_122.json`) | `e01b7e290a9732f1812bf3428fc71416bd871f7ddfec911383ebf14b9d8a10c3` |

Both ZIP digests exactly match the GitHub artifact metadata. ZIPs contain CI logs,
test results and evidence, not selected native binary artifacts. Their member
names, lengths and hashes are recorded separately. Only report JSONs are extracted.

Initial and partial reads remain intact. `initial_01/setup_failure.json` preserves
the first local readback script's trailing-slash repository GET 404; it occurred
before that script fetched CI data and is not a CI failure. The corrected GET
used new output directories. A read-only PowerShell summary command also required
parenthesizing its foreach pipeline; it changed no evidence or remote state.

All remote operations were GET requests. No rerun, dispatch, cancellation,
binary installation, repository/adapter write, local test/engine or Unity action
was performed. This receipt is local audit evidence, not a published deployment
approval and not a successful runtime checkpoint.
