# 2026-10-11-planner-geometry-r3

Planner receipt: RESULT_READY for an auditable partial code checkpoint, not for
deployment or the requested complete first/second lap.

- Exact code/evidence commit:
  [d49f3c3352f2f7bfb670ea66bcc70cf4e18aae0e](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/d49f3c3352f2f7bfb670ea66bcc70cf4e18aae0e).
- Branch: repair/lane-keyframe-transaction-20261011; immutable annotated tag
  [lane-mapping-20261011-r3](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/lane-mapping-20261011-r3).
  Actual push URL and repository ID 1303517209 were checked; remote branch and
  peeled tag both read back at that exact commit. No main merge or old-repo write.
- Sealed [session007](../sessions/2026-10-11-planner-observability-007/SESSION.md)
  and [ZIP](../bundles/2026-10-11-planner-observability-007.zip): 3,591,547 bytes,
  SHA256 09e5b117234cbad99b3b284c7a0dbe67ff3f5bf8ddada82ab67bcf2d7d635d30.
  All 14 reviewed files match staged Git objects. Recorded CRLF evidence and an
  inherited SESSION blank EOF were preserved; full-record whitespace checking
  is not claimed clean. Runtime/test diff checks passed.
- Local: 878 core passes / 1 target-Linux skip on Windows. After sealing,
  33 handoff tests completed in 7.951 s, exit 0, with one platform skip.
  The draft/unpacked pre-check failure is retained in the session.

## Evidence wording correction

Session007's phrase "held-out scoring" means simulator truth was never supplied
to the estimator. It does NOT mean an independent held-out dataset: this same
trial16 development prefix was used for diagnosis, ablation and repair. Its
mixed accuracy numbers are development-prefix evaluation, not generalization
proof. The sealed original is retained; STATUS and this receipt clarify it.

## R3 remote CI (failed Foxy tests, not a deployment pass)

All runs below use d49f3c3. Branch/tag duplicates are not independent algorithm
evidence. No run was manually rerun or deleted to hide failure.

| Workflow | Branch / tag runs | Actual outcome |
| --- | --- | --- |
| Lane Foxy | [38072857986](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38072857986) / [38072858045](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38072858045) | 4 packages built; 519 unit passes, 1 failure; later DDS not run |
| GNSS Foxy | [38072857957](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38072857957) / [38072858043](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38072858043) | 5 packages built; 878 unit passes, 1 failure; later DDS not run |
| Handoff | [38072857949](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38072857949) / [38072858006](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38072858006) | each: Windows/Linux, Python 3.8/3.13, 33 tests per environment passed |

The sole Foxy failure is the empty-array oracle in test_cell_grouping.py:
NumPy 1.17.4's np.unique(empty Nx2, axis=0) raises while reshaping zero elements.
Production _unique_cells already explicitly handles empty input. The compatibility
repair keeps the empty test with explicit expected shapes/dtypes and keeps the
other NumPy oracles; local 27/27 pass. It is not part of R3 and requires a NEW tag
and new Foxy CI. R3 is never silently moved to the repair.

Four CI artifact manifests were read, not their archive contents downloaded:
IDs 11677681209, 11677835398, 11677451657, 11677576462; total 633,832 bytes.

## Actual runtime follow-up: trial18 failed, safely ended

The adapter's Mono Stopwatch/native-QPC clock-domain bug was corrected outside
the ROS packages. Compiled Unity directly brackets the bridge counter with
kernel32 QPC: 35/35 checks; 197 offline adapter tests pass. The first offline
invocation incorrectly named an evidence subdirectory and had 8 missing-file
errors; it is retained, followed by the correct-root run. Original preservation
still reports the same 12 peer-attributed changes, not a false clean pass.

Scene bytes and all 113 current parameter values match the prior Edit snapshot.
Teacher reference remains 2 m/s from the current LYA source with no override.
Trial18 accepted independent wheel/gyro input, passed stationary warmup and the
explicit Start, then taught 106 frames. The 107th image (sequence410) was rejected
because matching took 40.7357 ms against the unchanged 40 ms limit. Engine work
was 75.4226 ms; end-to-end backend processing was 97.4254 ms. These are different
timing boundaries, not interchangeable performance numbers.

Sampled travel: 28.76994 m; 1186 confirmed cells. No completed lap, frozen route
or repeat. HOLD sent zero; the saved post-halt snapshot confirms E-stop and
released ownership but still showed 0.103 m/s deceleration, so it does not prove
full physical settling. The Unity scene then returned to Edit and the backend
process exited. All logs, masks and failure video remain local under trial18;
the fixed 60x60 m replay displays only consensus available at each logged reply.

Next: exact current-mask geometry memoization to reduce runtime work, causal
replay of the newly captured prefix, a separately tagged compatibility/performance
repair, Foxy CI, and another complete new-scene run. No gate, vehicle parameter,
future input or truth input may be changed to manufacture success.
