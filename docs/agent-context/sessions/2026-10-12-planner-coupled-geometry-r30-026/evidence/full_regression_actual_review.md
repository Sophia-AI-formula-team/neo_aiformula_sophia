# R30 actual full regression — independent read-only result review

Result: **4,403 passed, 1 documented Linux-only skip, 0 failures/errors**.
This is a completed local offline regression, NOT runtime/geometry/lap/driving
acceptance. Review performed 2026-10-12 by `indexed_bounds_review`; no test,
Python/AST/model/native/Unity execution or production/old-artifact edit.

## Actual evidence checked

Read the actual APPROVED, root receipt, child summary, native JSON/stdout,
phase launch/exit receipts and all four source snapshots. Independently parsed
JUnit XML rather than relying only on the summary's pass flag. JSON digest
verification used the same raw bytes for hash and parsing. All recorded root
and child output hashes matched; no missing or changed evidence was found.

| Actual scope | Result |
| --- | --- |
| Pytest stdout | 4,403 passed, 1 skipped in 90.86 s |
| JUnit | 4,404 testcase elements; tests=4,404, failures=0, errors=0, skipped=1; 90.848 s |
| New named methods | Exact 58 fit + 30 joint + 11 roundoff; none missing, duplicated, skipped or failed |
| Old indexed production port | All 25 testcase elements present, none skipped/failed |
| Native verifier subprocess | Exit 0; 0.1712018 s |
| Pytest subprocess | Exit 0; 92.4446779 s |
| Owned orchestration run | Exit 0; 94.8930391 s, below the shared 150-s primary |

The sole skip is `test_owned_process_shutdown_leaves_unrelated_process_alive`,
explicitly marked `Target Linux process-group test`. It is retained, not counted
as a pass or silently removed. JUnit counts and subprocess/outer timing scopes
are different measurements; they must not be mixed into a performance claim.

## Binding and ownership

- APPROVED authenticated the original PENDING digest; its review/CPU/execute
  flags are true, while the frozen PENDING flags remain false/null. Stripped
  APPROVED and PENDING contents are equal. Only allowed approval fields changed.
- Actual native argv, 111-file pytest argv and original environment match the
  approved values. All original 108 test entries retain their order; only the
  three fit/joint/roundoff files follow the last original GNSS worker entry.
  Native arguments are unchanged except the exclusive new output path.
- Executed raw inner-source marker and actual bootstrap launch prefix match
  the approved runner identity. No directory/glob test collection was added.
- Each of the four outer/inner before/after snapshots has exactly 202 files,
  no hash/header mismatch versus APPROVED, and identical snapshot SHA below.
  HEAD remains the reviewed `9b8636fc3c1aa9e9712142b3924a722211df19bd` during
  this run. The original 197 pins remain unchanged; only the five declared
  additions are present. Historical R29 `source_count=196` error is preserved
  in its original artifact; this run correctly records 202.
- Native JSON reports backend=native, all nine API versions=1, real_calls_passed
  and finite_current_union_real_call_passed=true. C and binary hashes equal the
  frozen originals. **installed_prefix_verified=false**: this is local cp313
  Windows native validation, not a Foxy/colcon installed-prefix proof.
- Both phase receipts bind PID/creation identity and actual command to their
  successful waited exit. Completion receipt reports orchestration/native/
  pytest processes inactive, native/pytest PIDs no longer present and known
  owned processes confirmed stopped. No stop attempt or retry occurred.
  Cleanup/timeout branches were NOT exercised; no strict WCET is inferred.

## Compact identity ledger

- APPROVED: `c0a78b1d865483873fcf95bcc0f6a2bfd3ca6d7ee00af011eb51946308fbb159`
- PENDING: `a9d2c43e5cb9ff89b7a991574c814edaf0717c637e89a098e25feb0ecbc56f05`
- Root receipt: `a39c0aad53216338b41c2c13d9da1b29495ba73eab240701836254cb23fda9bd`
- Child summary: `793ee70ae9ae99294f5527f01ddae111302a42fa63eaa0793f89cf48571891b8`
- All four snapshots: `2028d7767eabf4c4736eab73bc4b348ae6bcbf6964ea6894b85f7348affd592b`
- JUnit: `4baf71d5f320be167ae2e2d7de978eadc4f50675956480ef3e98077fbba5ae60`
- Native JSON: `286231e8f872fd271b9fbe15b18e1e616814d19c72bf5c9bf27c735d5368bd1c`
- Native C: `9dae4a7ab517d872af29d3c2d0a27d1cdfa7e8446c8f138be206df5ba4c4438f`
- Native binary: `f2f9712754f8cecaae379af5355c8aa82b3a28855fa2da2d0ce949ddf339ef45`

No additional protocol gate or rerun is proposed by this review. The evidence
supports moving on to the serial actual capture and real core repair. It does
not erase the existing strict 13/14 / seq164 engine-80-ms failure, prove a
geometry improvement, enable the diagnostic helpers, loosen matcher40/engine80/
full-adapter200 limits, complete a new-scene lap/repeat or authorize deployment.
Private workstation paths/hostname are deliberately omitted from this summary.


