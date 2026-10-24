# R28 development checkpoint published — PARTIAL

Role: planner. Input and consumed handoff: `2026-09-28-planner-neo-004`.
Disposition: `RESULT_READY`, evidence available for analysis, not runtime/driving acceptance.
Remote readback: 2026-10-11 UTC; local session date 2026-10-12 JST.

## Exact publication

- Push URL checked: `https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git`.
- GitHub immutable repository ID checked: `1303517209`.
- Code/session commit: `8fcd4c0533a13d95400b9f377959733ed798f744`.
- Authorized branch: `repair/lane-keyframe-transaction-20261011`.
- Annotated tag: `lane-mapping-20261012-r28-rc1`.
- Tag object: `87ae9243514dc2395038b5b755774119df9c20fa`.
- Successful branch and tag push; API commit and `ls-remote` branch/peeled tag both read back the exact code SHA. No main, legacy repository or force push.
- Sealed [session024](../sessions/2026-10-12-planner-pca-moments-r28-024/SESSION.md) and its ZIP remain unchanged.
- Bundle53519 bytes, SHA256 `319fceec9d1d00d1eddd593ac220ce4dff05821bb602fda4d717094bf69e67cc`.
- Exact19-file staging; complete root and independent static source/test/document/privacy review, whitespace check and archive/member/hash consistency checks passed. Excluded untracked prototype pair remains untouched.

## Actual evidence and limits

Full local Windows production regression:4279 passed, one original Linux-only skip, zero failures/errors; all107 explicit test files and196 source pins match before/after. Native moments used in22 complete original/candidate observations, all29 fields and five derived arrays byte-exact. Diagnostic mocks8/8 pass; they are not DDS proof.

One causal recorded seq163 extraction196.799100ms, one separately instrumented extraction236.311300ms, outputs byte-exact. This is neither a comparable speedup measurement nor full200ms adapter acceptance. Original40/80/200ms and geometric gates retained. R26 EndpointGNSS DDS failed and rejected closure holdout remains rejected. No new Unity lap, frozen route, repeat lap, driving error or video.

## Exact-SHA CI dispatch readback

Eight runs exist for the exact code SHA; first readback is partial, not a final CI verdict:

| Workflow | Branch run | Tag run | First readback |
| --- | --- | --- | --- |
| Lane exact native graph | 38171022750 | 38171022711 | both completed/success |
| Agent handoff integrity | 38171022778 | 38171022718 | branch success, tag in progress |
| Lane teach-repeat Foxy | 38171022747 | 38171022764 | both in progress |
| Endpoint GNSS Foxy | 38171022797 | 38171022715 | both in progress |

CI must be read to terminal per exact SHA and its failed stages recorded in a new append-only receipt. This upload does not certify the two-lap requirement or enable vehicle output. Next session025 prepares selected-only geometry gathering and independent model-provenance repair; no vehicle/simulation/LYA parameter change is authorized.
