# R30 development checkpoint: next geometry/runtime work

This is a failed v5 development checkpoint, not a deployable map or driving
baseline. Keep TeachMapEngine/control on their existing default path. Do not
enable v5 because local units pass. Car/simulator/camera/LYA settings and all
original numerical safety gates stay unchanged.

## Findings that change the plan

The current original-input prefix commits151..162, then rejects163 during
prepare at the development capacity gate. Last committed total193188 is
base15423 + curve170651 + export7114. Failed163's prospective total is not
available; do not invent it. This was a normal31.826s diagnostic exit, not a
timeout. No can_commit/commit was called for163;164 was not started.

The v5 `support_records` label is not yet a proved unit mapping to the old
quad/Interval budget. Existing code is more conservative, not evidence of a
physical-memory limit. Do not lower coefficients and call that compression.
Old continuation counts a complete quad with its normal/reliability as one
record, plus one frame header; a finite Interval also counts once despite
multiple fields. V5 still carries20/state,4/frame,2/affine,8/piece and
10/packet+2/occurrence charges inherited from scalar/header accounting.

For162 the executed curve170651 decomposes exactly as follows:

| Category | Executed charge |
| --- | ---: |
| Complete source tables/faces |21418|
| Packet metadata |20130|
| Directed occurrences |20854|
| States |15960|
| Retained-frame metadata |8052|
| Affines |19766|
| Consensus pieces |29480|
| Contributor references |34991|

These are not suggested deletions. The raw+canonical payload4379192bytes is
not Python heap/RSS and excludes object/allocator costs. Twelve source tables,
798keys,2013packets and3685pieces do not prove whole-lap feasibility.

## Required sequence

1. Define and version a consistent per-class semantic record contract before
   changing the ledger: key/chart, frame header, full face/quad, directed
   occurrence+role, affine, consensus/export piece, contributor reference,
   permission and base map. Keep separate scalar, canonical payload and actual
   storage metrics. Preserve this failed run and both old ledger corrections.
2. Implement actual lossless representation/lifecycle improvements against a
   decoded semantic oracle. Keep ALL current5+1 evidence, own valid roots/quads
   not participating in pairs, repeated incident endpoints/roles, alternatives,
   UNKNOWN, creation charts, timestamps and failure-without-commit behavior.
   A small ndarray view that still owns the full source table is not release.
   Per-key5+1 does not bound the global key count or stale-signature lifetime.
3. Explicitly decide the state-pinned-unowned lifetime. Current builder scans
   ALL sources reachable from states OR orphan history. A source may still
   export standalone unowned boxes after leaving the global orphan5+1 tuple.
   Dropping those rows changes the rejection domain; it is not silent parity.
   Do not replace an owned unknown-affine envelope with a smaller quad hull.
4. Only after that contract, accelerate actual envelope work using the package's
   existing C extension if justified. The uncompiled diagnostic draft is NOT
   adopted: its conservative MSVC path declines, so it cannot yet solve the
   Windows demo. Audit binary64 operations/signedzero/FMA/FP policy and source
   closure for mainC+include, preserve old11callables/9capability versions and
   scalar first-error fallback. No missing local compiler installation is
   authorized; existing CI can build the existing extension dependency chain.
5. Compare complete original inputs and required semantic outputs, then run the
   original causal Engine chain. Historical accepted-A poses in inventory/pair
   diagnostics are NOT v5 localization. Test effect and bias on independent
   geometry inputs, not merely matching bytes to a known bad model. Keep the
   original40ms matcher/80ms Engine/200ms complete adapter timing boundaries.
6. Before Unity Play, obtain a fresh editor/scene/parameter/protected-file
   readback. The peer returned Unity9828 in clean LaneTeachRepeatValidation
   Edit state, but that receipt is not a fresh live snapshot. Protect its
   semantic-planner/UI files. Do not refresh/switch Play without sole ownership.
7. Then perform actual new-scene first LYA lap, stopped freeze and second fixed
   route lap. Inherit LYA's current speed parameter, no slow override or vehicle/
   physics modification. Mapping remains full mask+causal raw motion, no GNSS/
   future/teacher-command pose input. Save every failure, raw logs, matched-frame
   offline-only error evaluation and synchronized fixed-scale causal-consensus
   video. A failed prefix or old video does not satisfy the two-lap request.

No new lap/freeze/REPEAT/video was produced in this continuation. The current
local4612PASS/one Linux skip checks code compatibility, not these unmet criteria.
