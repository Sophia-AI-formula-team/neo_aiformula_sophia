# R29 production indexed-gather regression — reviewed public summary

RESULT_READY for planner review. This is a new, redacted summary of the root-owned
one-shot offline run. It does not overwrite the frozen manifests or original
run evidence, and it is not runtime, geometry-effect, two-lap or vehicle approval.
The delegated publication-preparation reviewer used PowerShell JSON/XML reads,
SHA256 checks and existing independent reviews only; it did not execute Python,
tests, a native verifier, Unity, a build, an install or a replay. It did not change
the repository, simulator adapters, staged files, commits, tags or remote state.

## Exact baseline and source scope

Canonical repository: `Sophia-AI-formula-team/neo_aiformula_sophia`, immutable ID
`1303517209`. Observed branch: `repair/lane-keyframe-transaction-20261011`.
Document HEAD at execution: `f8a2732a2763e97382f430fe2998808198e8b864`.
Inherited code checkpoint: `8fcd4c0533a13d95400b9f377959733ed798f744`.
The run used a dirty, hash-pinned R29 source snapshot, not a new committed R29 SHA.

There were 197 actual source/config/test/native pins and 108 literal pytest file
arguments. All original 107 selected test files were retained, one reviewed
portable indexed test was added, and no selected file was duplicated or omitted.
Against the 196 inherited pins: 193 unchanged, three specified changes and one
addition, with no removal. The independent static review verified all 197 live
hashes matched. The original excluded prototype pair remained outside source
reads, imports, hashes, tests and staging.

The four source/test paths below are repository-relative. They are the complete
production indexed-gather change, not the complete eventual publication scope.

| Path | Executed R29 SHA256 |
| --- | --- |
| `workspace/src/aiformula/control/lane_mapping_lya_reference/lane_mapping_lya_reference/paint_geometry.py` | `c57c1d53da9f66844b5477d4cd9848cb724ed0632e35054a1d93f252655251d3` |
| `workspace/src/aiformula/control/lane_mapping_lya_reference/test/test_boundary_fit_domain_equivalence.py` | `9ae0b69f5297ca9652c194cfacc14fd691b37ec842862e32b931fb8229354e8d` |
| `workspace/src/aiformula/control/lane_mapping_lya_reference/test/test_boundary_fit_domains.py` | `b818474692e6538c59f4af142daa93e42db1224c2366616300e55f4f8580cdf1` |
| `workspace/src/aiformula/control/lane_mapping_lya_reference/test/test_boundary_fit_indexed.py` | `d27915f353ef40c43fd612f3a9e1089fe4ee2ec40001ad868376825f0721a6c5` |

The paint source SHA exactly equals the candidate actually used in the earlier
22-mask comparison. That separate run records all 29 observation fields and five
derived arrays byte/read-only exact in all 22 cases, with 248 actual native moments
calls and 120 indexed bounds calls. Its result SHA is
`b75360b6fec6954018a4fdb0d89cc3503fa2dd96d4c3e928275c8bb644c8da3e`.
Neither execution measures improved geometric truth or an end-to-end speedup.

## Actual completed execution

Run ID: `r29_full_regression_01/run_01`. UTC start/finish:
`2026-10-11T20:43:30.410181+00:00` /
`2026-10-11T20:45:02.300091+00:00` (12 October JST).
Exactly one native verification and one pytest invocation; retries 0, exit 0,
exception null and Unity calls 0. The reviewer inspected the existing outputs,
not a new execution.

| Evidence | Actual result |
| --- | --- |
| JUnit | 4305 total: 4304 passed, 0 failures, 0 errors, 1 skipped |
| New `IndexedBoundsTest` | 25 actual cases, 25 passed, 0 failed/errored/skipped |
| Only skip | `test_owned_process_shutdown_leaves_unrelated_process_alive`; original Linux process-group test |
| Native verifier | exit 0; native backend; all nine API values 1; actual calls passed |
| Source snapshots | 197 pins; before/after bytes and hash identical; source_changes empty |

The native C SHA is
`9dae4a7ab517d872af29d3c2d0a27d1cdfa7e8446c8f138be206df5ba4c4438f`;
the Windows binary SHA is
`f2f9712754f8cecaae379af5355c8aa82b3a28855fa2da2d0ce949ddf339ef45`.
Both retain their original identities. `installed_prefix_verified=false` remains
visible: this is not a new native installation, ROS/colcon prefix check or Foxy
runtime acceptance. Python version was 3.13.13. Bytecode writes were disabled,
native was required, and the original four package roots and real-frame input
selection were retained; personal machine paths are deliberately not published.

Timing values have different scopes: JUnit suite 89.802 s; pytest process
91.37270060001174 s; native verification 0.18546930002048612 s; root bounded-command
wall 92.01783020002767 s. These are one diagnostic execution, not repeated
measurements, a speed ratio, frame latency or a worst-case bound.

## Retained metadata and reproducibility limitations

1. Both frozen R29 manifests retain the incorrect top-level `source_count: 196`.
   Their actual `snapshot.files` dictionaries contain 197 pins, all iterated by
   the driver; no pin or test was truncated. The inherited R28 count 196 is
   correct. This new summary records the correction without editing originals.
2. The original 107-file set is complete, but its argv order is not unchanged.
   Four adjacent pairs swapped. A culture-dependent sort is only a plausible
   cause, not a proved cause. Do not claim order-byte equivalence with R28.
3. The shared inner deadline was 105 s; the outer primary wait was 110 s; the
   requested CPU lease was 120 s. Outer cleanup has no independent timeout, so
   110/120 s is not a strict future wall-clock/WCET guarantee. The actual run was
   within the primary limits. Root reported its own children stopped and CPU
   returned; this static reviewer did not recheck that process state.
4. The manifest binds the inner driver and plan, not the outer wrapper. The
   wrapper's separately observed SHA is retained below; do not call it a
   snapshot-pinned artifact.
5. `driver_AST_valid=null` is the honest preparation state: no Python syntax/AST
   check was executed during static preparation. The actual driver subsequently
   executed. The approval note's wording `staged-scope diff` must not substitute
   for the fresh, complete cached-diff review required before publication.

The following zero-based positions exclude the newly added file. All four pairs
are under `workspace/src/aiformula/control/lane_mapping_lya_reference/test/`.

| Positions | R28 order | Actual R29 retained-file order |
| --- | --- | --- |
| 30, 31 | `test_corner_direction.py`, `test_corner_direction_integration.py` | integration, base |
| 50, 51 | `test_mask_localization.py`, `test_mask_localization_replay.py` | replay, base |
| 56, 57 | `test_native_current_union.py`, `test_native_current_union_dispatch.py` | dispatch, base |
| 70, 71 | `test_quantized_ribbon.py`, `test_quantized_ribbon_margin.py` | margin, base |

The original static preparation refusal, indexed-driver syntax failure, loader
50-pass/2-error result, prior strict replay and mixed R28 CI remain retained.
They are not overwritten, added to this run's pytest count or concealed by its
successful unit result. The preparation refusal occurred before manifest
creation/Python imports/tests and is not an estimator retry. Existing session025
records their separate outcomes and corrected follow-up IDs.

## Provenance hashes

Original artifacts remain local. Their raw content includes machine paths or
host details and is not approved for direct public upload by this summary.
The names below identify artifacts within the named diagnostic run; no absolute
user path, precise GNSS coordinate, raw bag, image or credential is reproduced.
All nine output hashes recorded in the original summary were recomputed and
matched by this reviewer.

| Artifact | SHA256 |
| --- | --- |
| Inherited R28 APPROVED | `b351224c1e666e3b7ba04045baa260f2456fe3685c5e28728f1530bc9aaaf545` |
| R29 PENDING | `f5c6b065bf00c43d91ab940c2cc249a4de4e845a953c3881a4dbc9e1e2e025f4` |
| R29 APPROVED | `2b201436ea79a97c6241a027eae4170d8b4971ea1deb6d165b4866c00a1f48e9` |
| Inner driver | `c89c03bde5d8c9012224b1e42d9766b12a09cad97e5968045f70694330796546` |
| Outer wrapper, separately observed | `e97fadbf506836054c76ca87e196d9b174a9056242337e9b43c998b34cbbe3e1` |
| PLAN | `a6669266595dfc7e38235d0f88442cfb3ddebc747420d1a94033e601c6def28a` |
| `root_execution_01.json` | `5b59448636dac5ed42ee9b7e62405646cb3b0da08ddf1009e5f562821d315de5` |
| `summary_01.json` | `d0379e3b8cc3e606e8c562ed76d854db1971511c5a0a51626f8c416b9ec22054` |
| `native_actual_01.json` | `286231e8f872fd271b9fbe15b18e1e616814d19c72bf5c9bf27c735d5368bd1c` |
| `pytest_01.xml` | `c87e3e34245f4a0b43614590eb5936aece4332d73d26df712c5d52945060d30b` |
| `sources_before_01.json`, `sources_after_01.json` | `6bc7afe4de4f0cf69996c4759b604237102feed548f258c9a5b3a0aabecc2782` |
| Independent production-test port REVIEW | `366e4a6909c411776068fba5cf32403bac651d1ec637467365ab4eeb7992d712` |
| Independent full-regression REVIEW | `dc04c18a3f5481fb2f0a80b378515856e4ef0db550000ee6109697c7d060ac9e` |

## What remains unproved

This narrowly supports integrating the reviewed indexed allocation change and
its literal oracle tests. It does not repair unresolved corner/side attribution.
The original 111-frame strict engine replay, full 200 ms runtime chain, stable
DDS timing, geometry truth, valid first-lap map/closure, frozen route, second-lap
fixed-route localization/control, driving-error metrics and a new video are not
established by these units. Vehicle, simulator, camera, LYA reference, motion
sources, causal order and original 40/80/200 ms plus all geometry/safety gates
remain unchanged. There is no vehicle release.

## Planner checklist before session025 sealing and publication

- Append this reviewed summary and its JSON to session025 evidence; update its
  stale NOT-run text with actual counts, four order swaps, count correction and
  limitations. Keep result `partial`; do not mark two-lap acceptance.
- Preserve original failures and frozen manifests. Record the count correction
  in a new append-only receipt, not an edited original. Keep raw private-path
  evidence local; a hash does not authorize its upload.
- Review the complete intended four-file code/test diff, scoped adapter evidence,
  all session/receipt documents and each evidence file for privacy/secrets. Do not
  read or stage the excluded pair or unrelated simulator/peer edits.
- Only root may set both publication-review flags after real review, update
  status/receipt, then validate/pack into a new non-clobber bundle and inspect
  its member list and hashes. Confirm the small-package limits.
- Stage explicit allowed paths, inspect the complete cached diff and exact
  source hashes, commit on the authorized repair branch and create a new unique
  annotated development-checkpoint tag. Do not alter old tags or force-push.
- Immediately before push, verify the actual origin push URL and immutable
  canonical repository ID 1303517209. After push, read back branch/tag SHA and
  run/read exact-new-SHA CI without treating units as runtime acceptance. Publish
  a separate receipt that cites the actual uploaded commit/bundle; never invent
  a commit SHA before it exists.
- Continue actual-model provenance/geometry repair and bounded strict runtime
  verification. Obtain fresh CPU/Unity ownership before a new run; successful
  units do not authorize Refresh, Play, driving or sensor/gate modifications.
