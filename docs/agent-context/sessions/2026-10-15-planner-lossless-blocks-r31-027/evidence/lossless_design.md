# R31 static design review: lossless retained-evidence blocks

Status: design only. No R edits, implementation, Python/AST/tests/native/Unity execution, or runtime acceptance. The current v5 development failure remains preserved. Root owns all representation, lifecycle and budget-contract decisions.

## 1. Recommendation and measured boundary

The smallest credible storage change is **independently owned immutable complete-face blocks plus explicit per-key frame leases**, not smaller counter coefficients and not ndarray slices of a full source table. Keep the numerical curve/chart/overlay/envelope algorithms unchanged. Implement this behind a new, explicit development representation version; do not silently weaken SourceFrame's current full-table contract.

FULL-read: session026/evidence/NEXT_GEOMETRY_RUNTIME.md (the file moved into evidence during this review; the current location was rediscovered and FULL-read again). Relevant current code reviewed: curve_evidence, curve_observation_adapter, curve_consensus_compact, curve_consensus, curve_geometry_builder, continuation_support and finite_support. This report adopts the checkpoint's required order: contract first, real representation/lifecycle change second, decoded oracle before acceleration/runtime.

The actual inventory run committed151..162, rejected163 in prepare at the existing combined capacity gate, never called can_commit/commit for163, and never started164. Its last total193188=base15423+curve170651+export7114 is an executed ledger count, not physical memory. There is no prospective163 total. At162, complete source tables/faces charge21418 of193188; eliminating every one of those charges would not be a valid implementation or a measured saving. Active-face coverage is not available from the inventory-only output, so no amount of achievable reclamation or full-lap fit is claimed.

This change targets avoidable **whole-frame pinning**. It does not bound the global key count, expire old signatures, reduce envelope conservatism, or prove200000 suffices. The unchanged current weighted gate can still reject163 after a physical storage improvement.

## 2. Why a sparse view alone is unsafe

Current SourceFrame validates contiguous original face IDs, full original reciprocal neighbour/contour/owner relationships, independent quad validity and complete immutable field bytes (curve_evidence.py:24-35,61-119). PairPacket retains ordered directed occurrences/roles and references the whole source (:138-197). The adapter produces that source once and authenticates owners from the real prepare output; it does not infer owners spatially.

The builder consumes more than packet endpoints:

- _state_box and _remaining_boxes directly scan ALL own valid model roots and own valid quads in every retained source, including faces absent from every pair.
- Both endpoints of ALL incident occurrences matter, including the other owner, repeated occurrences and original order.
- build_curve_geometry scans ALL distinct sources reachable from ALL state frames OR the orphan tuple. Every valid unowned quad creates its own standalone UNKNOWN box, including a source pinned only by a stale state after it leaves orphan5+1.
- Unknown/pending/alternative-signature competitors also enclose rounded original-chart affines and possible coefficient averages. A footprint hull cannot replace those bounds.

Consequently, keeping only pair endpoints, current owners, ready geometry, recent global frames or a spatial window is lossy. A small ndarray whose base still owns the original full buffer is not reclamation.

## 3. Proposed concrete interfaces

Names are proposed interfaces, not new production code.

| Type | Required immutable content / ownership |
| --- | --- |
| FrameIdentity | Representation/schema/source_kind, exact stamp, original face count, original producer/frame identity and a private validated-source seal. It holds NO SourceFrame, observation, full-array buffer, complete-face roster or persistent global-store reference. A digest identifies bytes but does not itself authenticate transport/pose. |
| FaceBlock | FrameIdentity + original face ID + ALL ten current SourceFrame field payloads for that row: face_ids, previous_next, contour_ids, endpoint_owners, model_roots, normals, quads, quad_valid, reliable, valid. Exact native dtype/shape/binary bytes, including signedzero and invalid placeholders. Owns independent bounded bytes, not a row view into a full source. |
| UnownedCapsule | FrameIdentity + every source row satisfying original quad_valid AND endpoint_owner<0, in original increasing face-ID order. Shares complete FaceBlocks. Carries the separately authenticated orphan reliable-quad flag when needed; do not reconstruct this from nominal-root reliable. |
| KeyFrameLease | Exact owner/side/signature + FrameIdentity; all own valid-root OR valid-quad FaceBlocks, even without pairs; every endpoint FaceBlock required by the packet; a strong reference to that source's UnownedCapsule, including an empty capsule. |
| BlockPairPacket | KeyFrameLease + original roles:uint8 and original face-ID pairs:int64, preserving every ordinal/repetition/order. Existing stamp/owner/side/signature/_has_reliable semantics. No new association or nominal source is created. |
| OrphanLease | UnownedCapsule + original stamp and reliable-quad retention classification. Retain exactly current latest5 plus latest reliable1 history. It does not retain unrelated owned geometry. |

A frame is distinguished by the authenticated original producer event, not merely equal coordinates, a stamp or a content hash. Two independently supplied sources must not be accidentally merged. All leases created from the SAME original source share its exact FrameIdentity and FaceBlocks. Do not introduce a process-wide strong registry that keeps dead frames alive.

Suggested narrow read interface:

- packet.gather_original(field): resolve original face IDs to complete FaceBlocks; return the exact ordered pair-shaped readonly native-C array expected by the existing _PacketView/_chart_frame. Preserve duplicate endpoints and role order.
- packet.occurrence_normals(ordinal): resolve BOTH original endpoint normals for that exact occurrence, in original order. This replaces _finite_piece's direct source.normals[face_indices[ordinal]] lookup without changing its direction test.
- lease.own_rows(): original increasing face-ID order for this owner, not packet order or arbitrary dict/set order.
- lease.own_roots()/own_quads(): apply the original independent validity masks; do not use root validity to discard a valid quad.
- reachable_unowned_capsules(all_states, all_orphans): deduplicate by original source identity and preserve the current builder's first-source traversal order; each original unowned face is still one independent box.
- retained_inventory(...): report the full staged map and separate legacy weighted charge, actual retained blocks, scalar entries, canonical payload and storage/allocator measurements. No query-local credit.

Builder adaptation replaces its own-row scans, standalone-unowned scan AND _finite_piece's contributor-normal lookup with these ordered accessors. The arithmetic/order inside _state_box, _remaining_boxes, _unknown_affine, _world_envelope, _finite_piece and _permission_arrays stays unchanged; only storage resolution is replaced. compact._chart_frame still feeds the same complete transient _PacketView to existing arithmetic. This is not a vectorization, refit, envelope shrink or permission change.

New packet/lease types need explicit versioned private seals and strict factory validation; do not replace current exact-type guards with permissive duck typing. Old SourceFrame remains a full-table legacy type. New storage must not silently serialize as that old type or accept a new sparse object through an old bundle/constructor path. Emitted reference/geometry/permission validation, including root's fixed v5-only5/1 guards, remains intact.

## 4. Factory and lifecycle: release only truly unretained rows

Proposed first implementation:

1. Receive ALL original29+5 observation fields and authenticate original groups/final-owner records exactly as today. This task does not authorize changing input extraction or cropping observations.
2. Validate the complete original source with the existing full-source rules BEFORE dropping any row. Invalid out-of-range/reciprocal/owner/quad data must still fail, even if the row would be unretained. Keep original per-frame200000 limits before projection.
3. Derive per-owner own-row manifests, exact packet occurrence manifests and the complete unowned capsule from that validated source. No spatial/nearness/ready-status selection.
4. Materialize each required FaceBlock once for this input event and share it among leases. For an initial implementation, the current full SourceFrame can exist transiently for validation, then be released after block creation. Its temporary peak allocation and work belong inside prepare and must be measured later.
5. Every new state frame owns its lease; advancing an immutable state retains exactly the original per-key latest5 plus latest reliable1 frame set. Unchanged old states/signatures keep their previous leases. Empty packets/UNKNOWN frames still carry all their own evidence and their source capsule.
6. Prospective map preparation builds a complete new immutable lease graph without changing the committed graph. can_commit/commit authority, generation/frontier rules, TTL and confirmed-owner lifetime remain unchanged. No in-place migration of old state/snapshot objects.
7. A block is released only when no relevant state/orphan/prepared-token/snapshot lease references it. Old tokens and snapshots remain readable. Removing current-map references does not prove process memory dropped while another live snapshot holds them.

The header must not keep all FaceBlocks. Shared auxiliary lookup structures must be lease-local or bounded to the actual retained block set; a hidden full-frame map recreates the pinning bug. FaceBlock bytes must not retain the original structured array or column via .base/memoryview. A first prototype may use one packed owned row per face; Python object/reference overhead may erase nominal payload gains. Fixed-size immutable pages are a later option: every physically pinned page/slack row must appear in physical metrics, and pages must never be presented as exact per-face release.

Original neighbour IDs are **identity facts**, not automatic strong geometry references. Do not recursively retain every neighbour's neighbour and thereby pin the whole contour. Every actual SAME_OWNER_MODEL_NEXT/CROSS_OWNER_INCIDENT occurrence still owns BOTH complete endpoints. The full-source reciprocal validation receipt covers references to unretained, nonincident neighbours. Such a reference must not be rewritten to -1, renumbered to a local slot, or treated as evidence that an original neighbour was absent. If any downstream consumer needs that neighbour's geometry, it must be explicitly included in the lease; a missing required row is a hard error, not a default placeholder.

### Unowned lifetime is deliberately NOT changed

To preserve current builder semantics, each state lease retains its source's entire UnownedCapsule for as long as that source is state-reachable; orphan history retains it independently. The union of those references determines standalone unknown competition. Releasing a source from orphan5+1 while an old state remains does NOT release its unowned faces. No old signature or UNKNOWN state is deleted to reclaim storage.

A policy that keeps unowned faces only through global orphan5+1 may be a separately proposed contract change, but is not this lossless fix. Current actual151..162 happened to report zero unowned quads; that is not permission to omit this path.

## 5. Semantic budget: an explicit mapping, not a coefficient edit

The old contract supplies an anchor, not a ready-made bijection for new curve classes:

- ContinuationFrame.storage_units is one frame header plus one COMPLETE quad record; normal/reliability are fields of that record, not extra records (continuation_support.py:35-58).
- A complete five-field Interval is one interval record, with capacity checked by interval count (finite_support.py:90-108,664-672).
- Current compact metadata weights20/state,4/frame,2/affine,8/piece plus contributor refs and PairPacket10+2*occurrences are not a proved mapping to those units. SourceFrame already counts one full face row despite its many fields.

Root should version a **decoded semantic record schema** before any gate rewrite. Each record has a stable kind+identity and its full payload; a mapping manifest must show exactly which retained evidence produces each record. A proposed classification is:

| Current logical material | Complete semantic record / old-unit relationship |
| --- | --- |
| Base segments, directed topology, finite-domain Interval and continuation frames/quads | Keep the existing base calculation and its exact record identities unchanged; do not subtract mirrored evidence without an explicit alias/budget rule. |
| Retained source face/quad | One complete face-evidence record keyed by source identity+original face ID, including root/quad/normal/flags/neighbours. Quad geometry plus all fields is one unit, not a coordinate-field count. Shared use refers to this record rather than copying its payload. |
| Per-key retained observation frame, including empty/unknown | One complete observation/header record keyed by key+source event, analogous to ContinuationFrame's header. Source-wide identity metadata is not20 duplicated fields per key. Source-header vs per-key-header aliasing must be specified, not guessed from physical sharing. |
| Directed occurrence | One complete ordered occurrence record keyed by key+source+ordinal, carrying both endpoint IDs and role. Repeated occurrences/cross-owner copies remain distinct. It is a NEW semantic class; no false claim it is literally an old quad. |
| Key/creation chart, affine, consensus piece, exported primitive, contributor reference, permission | Specify complete per-class records and stable identities. An affine/piece is treated like a full Interval-like record, not8 coordinate/header units; contributor membership/order must still be represented. These are explicit extensions where no old one-to-one class exists. Root must decide embedded fields vs independently charged relations and avoid double charging the same record through aliases. |

The mapping must be a bijection between the **chosen decoded record schema** and ledger entries: no missing UNKNOWN/alternative/header/reference records, no multiplication merely because a dataclass has many fields, and explicit sharing/dedup rules. There is NOT a provable literal bijection from every new curve/occurrence/contributor class to old quad/Interval classes already established by the code. Pretending otherwise would simply hide a contract change.

For the minimum storage implementation, keep the existing acceptance gate/200000 cap and its historical full-source charge available from FrameIdentity.original_face_count; report actual retained-block metrics alongside it. This decouples a demonstrable physical release from a later root-reviewed unit-contract change. If a revised versioned ledger is adopted, preserve both old and new results and explain exactly why acceptance boundaries differ. Do not call passing163 after weight changes a storage optimization.

Separate metrics must include: decoded semantic counts by class; legacy weighted charge; scalar entries; actual live array/packed-byte storage including backing buffers/page slack; canonical payload; independently measured heap/RSS/allocation/time where authorized. Canonical raw+derived bytes are not heap, and a smaller ledger is not evidence of fewer bytes.

## 6. Independent decoded semantic oracle

Do not use candidate accessors, candidate retention manifests or its inventory routine to decide what the oracle expects.

A reference oracle starts with the existing complete immutable SourceFrames and ALL current/staged states+orphans. Independently enumerate:

1. Every key, original creation chart, status/reason/frontier and retained-frame order selected by original5+1.
2. For each retained key frame, all full source rows whose owner equals that key and whose root is valid OR quad is valid, including isolated/no-pair faces.
3. Every original ordered occurrence: ordinal, role, both original endpoint IDs and both complete endpoint row bytes, including repeats, incident copies, invalid placeholders and signedzero.
4. For every source reachable through ANY state OR orphan, ALL valid unowned-quad rows and their exact source/face order; orphan classification/history comes from the original independent reliable-quad rule.
5. ALL chart affines, pieces, coefficients/domains, contributor and reliable-contributor stamp order/membership, frame votes, stamps, owner alternatives and original base/permission data.

Normalize both reference and candidate to this same complete keyed semantic stream. Compare dtype/shape/exact bytes for full row/occurrence payloads, and typed scalar/tuple data for charts and contributors. Use streaming canonical hashing plus first-difference diagnostics, not huge JSON object graphs. Normalize process-local seals/addresses only; do not normalize signedzero, order, source identity, unknown status, missing rows or record multiplicity. Candidate decoding resolves its own stored blocks; it must not recover missing geometry from the gold source table.

Then compare actual forward behaviour: complete staged/committed state stream, all reference/map metadata and arrays, nine curve arrays, five permission arrays, geometry_complete/reasons, all unknown boxes, status, token acceptance and first-error class/message. For invalid input, verify the same rejection before any public mutation. Snapshot/token validity and before/after complete content digests must be checked; an identity-only marker is not this oracle.

Required future tests (proposal only; none executed here):

- Own valid root/quad absent from all pairs; empty packet; invalid-root but valid-unowned quad; invalid placeholder bytes and strict dtype/native/shape cases.
- Repeated pair rows, both incident endpoints, opposite owner copies, same coordinates with distinct source/face/role identities and ordinal order.
- Original latest5 plus an older latest-reliable1; alternative signatures and nonupdated/UNKNOWN states; confirmed owners surviving TTL; clean frames that do not refresh an orphan; state-pinned source after orphan eviction.
- All-source standalone unowned boxes including coordinate-limit/subnormal failclosed cases; owned unknown-affine and partially-ready complements unchanged, not replaced by quad hulls.
- A far unrelated row expires while a retained neighbour ID remains; reciprocal validation still rejects malformed full inputs before projection. No recursive contour pinning, renumbering or fabricated -1.
- Rejected prepare and unused tokens preserve the old full semantic stream; commit authority/generation/frontier and old snapshots stay unchanged/readable. Allocation failure/missing required block fails closed before commit.
- Independent physical reachability checks: required blocks stay live through each real lease; unretained blocks and original full buffers become collectible when all owners are released; no .base/registry/cache pin. Payload counters alone do not prove release.
- Hand-calculated tiny fixtures exercise every budget class and cross-alias case, global exact-capacity and one-over rejection. Test old/new unit manifests separately; no threshold increase.
- Original ALL14 input corpus: preserve the current12commit/163capacity failure boundary under the legacy gate; do not claim164 parity without starting it or assume a successful prospective163 token exists.

Byte parity to a possibly over-conservative model is necessary for a storage-only change, not sufficient evidence of useful geometry or runtime. Independent geometric inputs must still test effect/bias, and the real causal Engine chain, original40/80/200ms boundaries and two new LYA laps remain later root-owned work.

## 7. Separate option: transactional computation memoization

Root's cache direction is plausible because _prepare_curves reuses unchanged immutable state/domain objects. It is a DIFFERENT optimization from source-block lifecycle compaction. The key suggested by root is necessary but not sufficient on its own.

| Property | Source-block lifecycle change | Per-key computation cache |
| --- | --- | --- |
| Removes dead full-table backing storage | Yes, if independently verified | No; it may retain extra output tuples |
| Changes which retained evidence is decoded | Must not | Must not |
| Avoids repeated _finite_piece/_state_box/_remaining_boxes arithmetic | Not inherently | Potentially, for identical reachable immutable inputs |
| Repairs capacity-unit contract or proves200000 fits | No | No |
| Acceptance evidence | Semantic oracle + actual release/physical accounting | Semantic/FP/failure parity + measured hit/work/time accounting |

### Narrow cache interface and dependencies

Prefer one cached **owner-slot-independent local key contribution** over a cache of every piece/interval pair, which can multiply entries by a large cross-product. A local value contains the exact finite exports/covered spans and selected state-box or remaining-box tuples, plus normal branch metadata. It never contains a current global owner slot or permission primitive index. Assembly remaps to the current ordered owner slots and recomputes the full global permission arrays/inventory/capacity gate every prepare.

A proposed cache key holds strong references and explicit identity checks for state and DomainState (not naked id() integers susceptible to reuse), confirmed-owner bool, exact current key-count/alternative branch, cosine binary64 bits, storage/algorithm version, and the accepted numerical-environment token. Use an identity wrapper with is-based equality and retained object references; do not invoke dataclass equality/hash over ndarray fields. State identity covers pieces, contributor occurrence order and source leases; domain identity covers all interval bounds/votes/reliability/stamps. The local builder's only use of the current record is its confirmed-owner condition, but that assumption must stay an audited dependency. Reordering owners must not change a cached value's ownership assignment. Key count matters because one key can use remaining-box logic while alternatives require full state boxes.

If functions are cached separately, _remaining_boxes also requires the EXACT ordered covered tuple, including signedzero/bounds; it cannot be keyed by state alone. A cached _finite_piece requires piece+interval identities (or the containing state/domain contribution) and cosine. _state_box needs state plus the numerical/helper environment. Missing/unknown domains and any other existing structural checks still execute in their original order before accepting a hit.

### Transaction boundary and bounded lifetime

- The committed cache is immutable/read-only during prepare. Hits must not mutate LRU order, counters stored in mapper state or shared values. Build a new prospective cache containing only keys/variants actually reachable and applicable in this prospective map; do not copy stale historical variants merely because the old state still exists.
- Attach the prospective immutable cache to the genuine prepared token. Only a successful real commit may replace the mapper cache; a rejected/unused token or any prepare error leaves the old cache identical. Do not construct a second cross-map transaction or pretend unexpected partial commit failure rolls back.
- Prune entries with expired states, changed domains/branches/environments and unused variants at successful publication. No global strong memo registry or unbounded history. Old retained prepared tokens may keep their own old caches alive, so physical metrics must count those separately from the current-map semantic budget.
- Cache values are immutable copies/tuples, not writable arrays later handed out for mutation. Count retained output/cache bytes and peak construction storage. Duplicating all output geometry can consume more memory even if arithmetic becomes faster.
- Initially cache only normal successful computations. Do not negative-cache ArithmeticError/FloatingPointError, caught key rejection, unbounded boxes or incomplete-reason paths. Re-run those paths to preserve current first-error/failclosed behaviour. A successful numerical None for disjoint finite intervals is distinct from a cached failure, but is not a reason to skip structural/input validation.

### FP and helper-rebinding guard is mandatory

An old success must not hide an exception that the current scalar code would now raise. At minimum inspect/guard the exact current np.geterr() mapping, np.geterrcall(), relevant warning/filter/hook state, and the supported rounding/FTZ/DAZ environment where it can be reliably verified. Never set the caller's FP policy to obtain a hit. A policy change from ignored underflow to raise must force original execution. A callback/log/warn mode or changed warning filter can have observable side effects even when the final numeric result is equal; policy equality alone does not prove those effects can be skipped. Decline caching for unsupported/effectful contexts unless their equivalence has an explicit proof. np.geterr is not a complete hardware-FP environment query. If that environment cannot be authenticated on a platform, use the original computation rather than claim a portable cache hit.

Also authenticate the complete transitive helper binding set, not only the three entry functions. It includes builder projection/world/unknown-affine/envelope/complement/step/bounded/add/neg/mul/div helpers, arithmetic endpoint/normal/dot and their reachable numerical helpers/constants, and relevant NumPy/math bindings. Record both callable identity and original code/defaults where mutable Python function code is in scope; helper rebinding or unsupported numerical implementation causes a miss/fallback. The check itself must not trust a rebound geterr/query helper. Do not wrap these helpers merely to measure cache hits: the wrapper should deliberately invalidate the guard.

This is a conformance guard in a controlled module environment, not a security proof against arbitrary Python object mutation. Its supported environment and fallback policy must be explicit. Caching successfully computed values in a strict but identical policy may be safe for error outcomes, yet warning/callback/sticky-FP behaviour still needs its own contract; do not assume successful-return parity is the entire FP requirement.

Required cache-specific oracle cases: unchanged objects/hit; state/domain replacement/miss; confirmed false-to-true; one-to-many signatures; changed owner-slot ordering; covered-span order/signedzero; strict np.seterr and error callback changes; warning-as-error/filter changes; helper rebinding and same-object code replacement; unsupported hardware FP state; original first-error path; prepare failure after some cacheable keys; rejected/unused tokens; and cache pruning without pinning dead source leases. Compare complete outputs, reasons, can_commit/commit behaviour and before/after committed-cache contents. Preserve the original capacity ledger, so a speedup is never reported as a capacity repair.

Recommendation: establish the storage lease/decoded contract first as the requested physical fix. The memoization prototype can be evaluated separately against unchanged storage once the guard is designed; the two changes should have separate parity/physical/timing evidence and can each be rejected independently. No useful hit ratio or speedup is measured by this review.

## 8. Implementation scope and explicit risks

A bounded first implementation would add a new evidence-block module/factories and decoder; adapt only the observation adapter's storage factory, compact's explicit packet-version acceptance/gather, builder's ordered own-row/unowned accessors, and dedicated oracle/lifecycle tests. Keep existing arithmetic,5+1, masks, thresholds, TTL, owner association, unknown envelopes, all global evidence, curve/reference permissions and default Engine path unchanged. No compiler/native acceleration is necessary to establish the representation's correctness.

Main risks are hidden required own faces, reliable-quad vs nominal-reliable confusion, accidental source/occurrence deduplication, changed reduction order/signedzero, legacy full-table guards accidentally bypassed, strong references through headers/caches/views, Python per-block overhead/transient gathers, and mixing new ledger units into old acceptance claims. The first reportable success would be decoded-semantic parity plus verified physical-buffer release under the unchanged gate, not a new runtime PASS or full-lap fit.

No new runtime/lap/freeze/repeat/video is produced by this review.

Reviewed-current byte identities (SHA256):

- NEXT_GEOMETRY_RUNTIME.md:615507dacd582810320faaa74b3644710d520196bb3332a13e504122895d10a6
- curve_evidence.py:7c500480134bd1f70c7a1348a50d8e56efc5fdb2722f304791ecf86fd4b8c665
- curve_consensus_compact.py:7e63e67d7546afdc474f49e4ef456bc2b37e72c401320fac220d8ec3a4e914f7
- curve_geometry_builder.py:7bc6ebb983f6c5f9c787e19cb71995b20e355cd8fde26aff5a4fdc2ea14dc1da
- curve_observation_adapter.py:f67f3afbbdc633a240a23b0eb0992fb8edaa847aaeb71ccd4c664ca5036d7e2a
- paint_geometry.py:3bb5426cba95c34d3e04ea7a337249e41fde193f36d040d024cb31cc4b38ca73

