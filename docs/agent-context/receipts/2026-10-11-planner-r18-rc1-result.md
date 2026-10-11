# R18 RC1 development result

- Final remote readback: 2026-10-11 12:51:09--12:51:25 UTC; role: planner
  subagent `sim_interface_audit`. RESULT_READY for analysis, partial result;
  not code acceptance or vehicle release.
- Input: `2026-09-28-planner-neo-004`. Published checkpoint:
  [5f483e2c](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/5f483e2c6764851ef81c4b854f8626f63287c0c6),
  tag `lane-mapping-20261011-r18-rc1`, branch
  `repair/lane-keyframe-transaction-20261011`.
- Canonical repository ID `1303517209`, push destination, branch SHA, annotated
  tag object `e30dd19f34afc0273867c6777591ad0b8e8e8f69` and its peeled commit
  were independently checked. The tag and branch both target the checkpoint.
  Local HEAD also matched; subsequent dirty R19 work was not included or changed.
- Session: [015](../sessions/2026-10-11-planner-incident-domain-r18-015/SESSION.md).
  [ZIP](../bundles/2026-10-11-planner-incident-domain-r18-015.zip), 28724 bytes,
  independently verified SHA256
  `db1f94a6ec4b3ac02f5c654198e8769d7f65d09387c5ccd4b9e9c813b96e1a2c`.
  The sealed session and ZIP were not changed.

## Remote CI: 4/8 successful, not all green

All eight automatic push runs completed on the exact checkpoint, attempt 1.
Twelve jobs succeeded and four failed out of sixteen. No run was retried,
dispatched or cancelled by this audit.

| Workflow | Branch | Tag |
| --- | --- | --- |
| Lane Foxy | [38140437133](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38140437133) failure | [38140437045](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38140437045) failure |
| GNSS Foxy | [38140437119](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38140437119) failure | [38140437024](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38140437024) failure |
| Native Windows/Linux | [38140437102](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38140437102) success | [38140437054](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38140437054) success |
| Handoff integrity | [38140437143](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38140437143) success | [38140437029](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38140437029) success |

All four Foxy jobs built successfully: four packages in each Lane job and five
in each GNSS job. Their installed Python 3.8 native module verified the source
SHA, installed prefix and real calls to all eight APIs. They subsequently failed
the same reference-package unit test:

```text
test/test_paint_geometry.py:731
test_graph_capacity_rejection_leaves_node_edge_and_frontier_unchanged
  consensus.commit(consensus.prepare(graph_observation(points, 6, order=[0, 1, 2, 3])))
lane_mapping_lya_reference/paint_geometry.py:830 in prepare
  raise ValueError("combined boundary finite-domain capacity")
```

- Each Lane reference-package run: **2292 passed, 1 failed**, 964 warnings;
  branch 99.81 s, tag 100.88 s.
- Each GNSS reference-package run: **2292 passed, 1 failed**, 964 warnings,
  100.53 s. The GNSS package separately passed **830 tests**, 170 warnings;
  branch 36.58 s, tag 36.66 s.
- Colcon testing exited 1. The subsequent Header/DDS tests, and Lane's later
  real-LYA/control demonstration, were not reached. This failure is not the
  earlier R17 stationary DDS timeout and does not establish its repair.

These counts and the failure stack were read from complete original CI stdout,
not inferred from artifact contents. The planner's later R19 test-contract
repair is outside the R18 commit and these runs; it cannot turn this result green.

## Downloaded native evidence

The tag's Windows and Ubuntu Python 3.13 artifacts were downloaded once. Each
ZIP SHA matched its GitHub API digest; embedded C source SHA matched the exact
published Git blob. Each artifact's JUnit file reports **1441 tests, zero
failures/errors/skips**. Each `native.json` reports all eight API versions as 1
and `real_calls_passed=true`, including `FINITE_PROJECTION_BOUNDS_API_VERSION`.

Published C source SHA256:
`0c54086e66554c8e9c96b48893e4b19c076c8a747b8f2cf08d6ad216fdeca12d`.

| Tag artifact | ZIP bytes / SHA256 | Binary bytes / SHA256 |
| --- | --- | --- |
| Windows, ID 11702901363 | 27762 / `7a98cd38594afdbc2871f65c7e3f911870bda14bc966ef9908b93e0d845a033f` | 37376 / `690e7b900844f44f0d21cf1ba41dd903c437a36ac8053a41ea0f9f44a87a7f83` |
| Ubuntu, ID 11703265975 | 88681 / `0406c26e2cfa14fe71fabea6a029ceb1f083400b310eb14d0d6ed80e993aefdb` | 177024 / `d872074ca39fd152d5c83589de74d926acaf2d55a7258b6ada3c02a1efd76a99` |

The local Windows file is retained under the diagnostic root at
`r18_publication_01/lane-native-windows-latest-py313/_lane_mapping_lya_native.cp313-win_amd64.pyd`.
The Ubuntu file is in the analogous `lane-native-ubuntu-latest-py313` directory.
This audit downloaded and inspected bytes only: it did not install/import a
binary, execute local tests, compile, or modify dependencies. The native artifact
receipt has `installed_prefix_verified=false` because it is the standalone
native workflow; the separate Foxy installed-prefix verification above is true.
Any subsequent local installation/verification is a different task and record.

Read-only `gh api` / `gh run view --log-failed` evidence is retained locally:

- `r18_publication_01/ci_readback_02.json`, SHA256
  `516ff439f84436fd63d11ebaf60302ed467740d95d3dbab490868a4048465143`:
  repository/ref identities, all runs/jobs/steps, eight artifact metadata records.
- `r18_publication_01/native_artifact_verification_01.json`, SHA256
  `bcb79607dc2888589b9c74c8b7002396346b0ac8a1fe72ef2ffbaf2e4a4c3e14`:
  archive and binary digests, source identity, parsed native receipts and JUnit.
- `r18_publication_01/failed_run_<run-id>.txt` and corresponding summary JSON:
  complete four failed-run logs and extracted original stack/counts.

The compact facts and hashes are preserved here; raw logs and binaries remain
local. Temporary CI artifacts are not promised as permanent archival storage.

## Mandatory local failures and untested scope

Sealed015 retains the overlapping 110/125/22/19 focused test groups; their counts
must not be added together. Its pre-CI local missing-MSVC build failure and
explicit missing-eighth-API skip/rejection are retained even though CI has now
compiled and exercised that API successfully.

The four development geometry cases remain **0/4**. Recorded-input attempt01
accepted 11/12 before sequence162 exceeded 80 ms at 81.0084 ms. Attempt02 accepted
12/13 before sequence163 exceeded 80 ms at 100.4721 ms. The separate instrumented
sequence163 profile is diagnostic timing, not a runtime pass. All are fixed
previously recorded inputs, not a new closed-loop driving run.

The observed 3-by-1 white-pixel islands explain insufficient direction support
in the examined synthetic masks. A local full-label LP establishes only the
existence of a label-consistent band, not physical continuity, unique geometry,
six-face admission, confirmed support, accurate localization or driving success.
Subsequent R19 ribbon experiments do not belong to this checkpoint.

No new Unity drive, complete TEACH lap, frozen driving route or REPEAT lap was
performed by this audit. Last actual trial remains failed trial28. No current
Editor or vehicle state is inferred from CI. Vehicle, simulation, camera, LYA
speed, motion/safety gates and GNSS mapping policy were not changed here.

Next owner: planner. Preserve R18 and sealed015 as an immutable failed
development checkpoint; validate the test-contract repair and any R19 algorithm
changes on their own exact source. Native build success does not remove the
whole-frame runtime or geometry failures, and does not authorize driving.
