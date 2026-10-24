# R22 local test ledger

All results below are local offline Windows tests or bounded API checks. They
are not Foxy CI, ROS/DDS operation, Unity driving, runtime-budget acceptance or
completed-lap evidence. Counts are taken from preserved execution/JUnit files;
they must not be summed because suites overlap. Failed attempts were retained.

## Failure-to-fix history

| Evidence | Preserved initial result | Correction / later evidence |
| --- | --- | --- |
| Continuation helper, `r22_support_tests_01` | 71 passed / 2 failed of 73; NumPy interval rows reached ambiguous array comparison in sorting | `_union` scalar ordering fixed; unchanged 73 passed, then 90 passed including 17 new projection-bound tests |
| Public prepare/snapshot capacity review | An exact-capacity case committed and only later failed at snapshot export | Permission capacity is now checked transactionally during prepare. Original 38 integration tests passed; expanded 40 passed, including just-below-cap rejection with unchanged state and exact-cap success |
| Corner helper, `r22_corner_direction_tests_01` | 54 passed / 1 failed of 55; identical signed cones unnecessarily returned indeterminate | Exact collinearity and positive-dot identity handles the logically identical cones; unchanged 55 passed. No epsilon or relaxed label condition |
| Adapter startup baseline, `r22_continuation_adapter_01` | 1 failed: current boundary v4 not admitted | Explicit v4 representation and recursive source dependencies wired |
| Adapter first focused run | 117 passed / 3 failed | All three failures were historical R8/R12 native-C/binary identity mismatches; exact pre-edit replay bytes failed the same three tests. Guard was not bypassed and binary was not replaced |
| Adapter follow-up, `r22_continuation_adapter_02` | Historical failures above remain preserved | 128 passed; the same 3 historical tests explicitly deselected, not counted as passing. Current helper dependency/archive tests added |
| Full local regression01 | **3907 passed / 1 failed / 1 skipped**, exit 1 | One old explicit-legacy fixture cleared all position sidecars except `position_footprints`. Fixture now clears that sidecar too and retains a negative assertion for malformed dangling footprints. Full02:3908 passed/1 target-Linux skip/0 failures/0 errors; source inventory unchanged; see final regression section |

The capacity defect was discovered by an independent public-API reproduction
after the first 38-test pass, not reported as an initial pytest failure.

## Other verified coverage

- V4 bundle/persistence: **249 passed** (102 new +147 existing), source unchanged;
  `r22_continuation_bundle_01/focused_01.xml`. Five permission arrays and explicit
  representation/hash/schema checks; old schemas do not invent permissions.
- Corner integration: all **24** cases passed within full regression01. Includes
  unchanged prerequisites, generic original support count, true arc distance,
  immutable raw evidence, no-cut scope and post-proposal veto ordering.
- Full regression01 checked the three ROS-package test roots with native mode
  required. JUnit: 3909 total, 1 failure, 1 skip, 0 errors; 109.939 s JUnit /
  111.19502990000183 s process. Source inventories before/after were identical.
  The one skip is the target-Linux process-group test on Windows.
- Eight installed native API real-call checks passed before full01. This is
  local API identity, not installed deployment-prefix verification or Foxy CI.
  C SHA256 `0c54086e66554c8e9c96b48893e4b19c076c8a747b8f2cf08d6ad216fdeca12d`;
  binary SHA256 `690e7b900844f44f0d21cf1ba41dd903c437a36ac8053a41ea0f9f44a87a7f83`.

The unrelated unintegrated `test_quantized_curve_fit.py` was explicitly excluded
from the full command; no claim is made about that user's prototype. Historical
native-identity failures and this deliberate scope exclusion must remain visible.

Full01 summary SHA256:
`ce45895fecc4d7c4933cda8c4b67fc102074cb7f502c68e1b1933983e14feed5`.
Full01 JUnit SHA256:
`52e513ea85c0e7bd785424de7cc211f80aa20ab0b6c14ae08d57fd2883fa0ba1`.

The source manifest is a selected relative-path identity inventory captured
after the fixture correction, not a claim that full01 tested that correction.
Earlier geometry and focused artifacts retain their separately pinned source
versions. No tests, extractor, engine or Unity operation were run to prepare
these three draft publication files. No commit, upload or sealing is implied.

## Final full02 regression

Exact same complete test selection, only the explicit-legacy fixture correction:
3909 tests,3908 passed,1 target-Linux process-group skip,0 failures/errors.
Pytest exit0; JUnit108.328s, process109.4956547000329s. All190 source/config/binary
identities and Git status unchanged. Native eight API/source/binary real-call
verification passed. No new ignore, skip, retry or parameter change.
The new runner differed from full01 only by a trailing blank line; initial
byte-equality preparation assertion failed, AST equality then confirmed. That
preparation failure is retained. This is not a production algorithm change.

Full02 summary SHA256:
`d71a6baeaa7d02fd580f6c594036eee2e89834962d75f611c8e1c83597743f5e`.
Full02 JUnit SHA256:
`9b3935ce0dd3111f252ef598e43f4adf5d63fba5376b22b56ab2826fbd2f08eb`.

