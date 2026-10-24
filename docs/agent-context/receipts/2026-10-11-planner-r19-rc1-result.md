# R19 RC1 publication and CI result

- Role: planner subagent `sim_lane_backend`, read-only remote audit on 2026-10-11.
  `RESULT_READY` for analysis; partial result, not deployment or driving approval.
- Input handoff: `2026-09-28-planner-neo-004`. Exact checkpoint:
  [01442bbf75cb68213dff3c05b88f3bef8639a877](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/01442bbf75cb68213dff3c05b88f3bef8639a877),
  branch `repair/lane-keyframe-transaction-20261011`, tag
  `lane-mapping-20261011-r19-rc1`.
- Independently read canonical push URL and GitHub repository ID **1303517209**.
  Remote branch and annotated tag object `5bd49fbd6a0803238f8f646c76bfff34ee4ae536`
  both resolve to the exact checkpoint. No remote write, dispatch, rerun or
  cancellation was performed by this audit.
- [Sealed session016](../sessions/2026-10-11-planner-quantized-ribbon-r19-016/SESSION.md)
  and [ZIP](../bundles/2026-10-11-planner-quantized-ribbon-r19-016.zip) were not
  changed. The ZIP downloaded as content from this exact Git commit, the local
  ZIP and its sidecar all agree: **221996 bytes**, SHA256
  `efe860a415fd46f495cd1f0421aa36afc204bb66ccb9a0cd595f402bb5df50ec`.

## Automatic CI: 4/8 successful

All eight runs completed on the exact SHA, attempt 1: 12 successful jobs and four
failed jobs out of 16. Workflow success is not runtime or full-lap acceptance.

| Workflow | Branch | Tag |
| --- | --- | --- |
| Lane Foxy | [38143633687](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38143633687) failure | [38143633662](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38143633662) failure |
| GNSS Foxy | [38143633784](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38143633784) failure | [38143633723](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38143633723) failure |
| Native Windows/Linux | [38143633691](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38143633691) success | [38143633689](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38143633689) success |
| Handoff integrity | [38143633764](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38143633764) success | [38143633672](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38143633672) success |

Every Foxy job built successfully: four packages per Lane job and five per GNSS
job. Each installed Python 3.8 native module passed source/prefix verification and
real calls to all eight APIs. Each reference-package unit run then failed the
same independent exact-arithmetic assertion:

```text
test_quantized_ribbon.py::test_feasible_constants_lines_and_curves_keep_every_hard_label[coefficients0-0.25]
assert Fraction.from_float(result.minimum_margin) <= min(exact_margins)
reported minimum_margin: 0.24999999999999992
```

The reported lower bound exceeds the exact polynomial-label margin by
`1.6049936164862886e-18`. Both round to the same displayed binary64 number;
the Fraction oracle correctly detects the overstatement. The test must not be
relaxed to hide it. Source inspection identifies the final margin subtraction as
round-to-nearest after the outward polynomial enclosure, without another outward
rounding step. This is a bound-contract defect exposed by the Foxy result, not
evidence of a native build failure or a false geometric label acceptance. A later
repair belongs to a new candidate, not this immutable checkpoint.

- Each reference-package run: **2447 passed, 1 failed**, 1128 warnings.
  Lane branch/tag: 99.87/98.37 s; GNSS branch/tag: 73.94/100.03 s.
- Each GNSS package separately passed **830 tests**, 170 warnings;
  branch/tag: 24.05/33.74 s. Fixed has no independent tests.
- Colcon test exited 1. Subsequent Header/DDS, upstream LYA and control-demo
  stages were not reached. This does not establish repair of prior DDS failures.
- Each native Windows/Linux job reported **487 passed plus 954 subtests passed**.
  These are not 1441 independent top-level tests. All eight API versions were 1
  and real calls passed. C source SHA256:
  `0c54086e66554c8e9c96b48893e4b19c076c8a747b8f2cf08d6ad216fdeca12d`.
- Every handoff matrix job ran **33 tests** successfully across Windows/Linux
  and Python 3.8/3.13. Repeated platform and branch/tag counts are not added as
  unique test coverage.

The audit saved artifact metadata/digests but did not download, import or install
CI native binaries, and did not run local tests or compile. Original complete
stdout was read for all eight runs; aggregate counts are not inferred from
artifact object counts.

## Local failures and remaining scope

Sealed016 retains the native Windows core result **3277 passed, 1 Linux-only
skip**, eight API identity verification and 175 unchanged source hashes. It does
not supersede the exact Foxy failure above. Loader-focused testing retained
43 passes and two historical-native identity rejections; those were not skipped
or converted to success.

The four unchanged 509-frame development cases remain **0/4**: accepted counts
14, 11, 13 and 11. No claim of full-circle quality follows from these short
prefixes. Fixed trial28 replay accepted **13/14 attempted frames**, out of 111
available; sequence164 matched visually at 26.6116 ms but exceeded the unchanged
80 ms engine gate at **86.2260 ms**, so it was not committed. There was no retry
or budget change. Later profiling or R20 work is separate from this checkpoint.

No new Unity drive, complete TEACH lap, frozen route or REPEAT lap occurred.
Latest actual driving evidence remains failed trial28. Vehicle, simulation,
camera, LYA reference and original safety gates were not changed by this audit.

## Evidence and next owner

Raw files remain local under `r19_publication_01`; no personal paths, image data,
truth coordinates, GNSS data or native binaries are copied into this receipt.

- `ci_readback_01.json`, SHA256
  `ed98351b323f28749ef4975b83a41c3dc73427177b40363849f34b426153bf5a`:
  repository/refs and independent remote/local/sidecar ZIP check.
- `ci_readback_02.json`, SHA256
  `855ed200c84e023bd25df5963f909b49c4023c8d3ec38dbbe533a8bc09806365`:
  final eight runs, all jobs/steps and artifact metadata.
- `run_<id>_log_01.txt` and `run_<id>_log_summary_01.json` retain all original
  stdout, hashes and selected count/failure lines for each listed run.

Next owner: planner. Preserve this failed checkpoint and exact assertion;
repair the bound without label slack or relaxed tests, then validate the new
source separately. Geometry coverage and whole-frame runtime remain independent
unmet requirements; CI/build success cannot authorize driving.
