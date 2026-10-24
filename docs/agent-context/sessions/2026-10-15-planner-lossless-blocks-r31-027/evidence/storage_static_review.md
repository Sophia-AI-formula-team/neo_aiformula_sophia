# R31 retained-block candidate: independent static review

Date: 2026-10-15. Scope: D-only `r31_lossless_blocks_candidate_01`, not a production implementation or runtime acceptance.

The reviewer full-read PLAN, both initial source files, the revised candidate, every final test revision through the final18 source, the unchanged initial RESULT, the red/green record and the later separate final18 record. A read-only subreview independently read the initial candidate/tests and reference evidence schema and confirmed the findings. No Python, tests, native routine, compiler, installation or Unity operation was executed by this reviewer; no candidate/R/index file was modified. Only this report was written. Excluded prototypes and aliases were not accessed.

## Result

One actual byte-loss bug was found in the original candidate, despite its initial12 PASS. Root reproduced it with new independent counterexamples and fixed it. The final candidate's row-slice copy removes that defect, and the final18 test source addresses the identified own/incident/unowned raw-flag and row-selection gaps. No remaining blocking static implementation defect was found for this explicitly isolated projection contract.

This is not approval of a full-source byte-for-byte representation replacement. It retains the complete fields of the rows needed by selected leases and complete valid-unowned capsules; unused other-owner rows may intentionally be absent. There is no full builder arithmetic, global5+1 lifecycle, transaction, actual14, capacity-gate, heap/RSS or80ms proof here.

## F1 — original candidate lost noncanonical bool bytes; fixed

Original `retained_blocks.py:141` joined `getattr(source, name)[face].tobytes(...)` for each field. Three fields are scalar booleans: `quad_valid`, `reliable`, `valid`. Scalar extraction maps a nonzero underlying boolean byte to the True scalar, so legal raw bytes128/2/255 become01. This is not merely a different buffer owner: it changes the retained original flag bytes.

The production evidence schema accepts exact native bool arrays and freezes their whole raw contents; it does not require true bytes to be01. Existing `test_curve_evidence.py:453-462` explicitly retains noncanonical backing bytes. NumPy's bool-scalar route is visible in [PyArray_Scalar, version2.2.6](https://github.com/numpy/numpy/blob/v2.2.6/numpy/_core/src/multiarray/scalarapi.c#L483-L485); its [bool singleton macros](https://github.com/numpy/numpy/blob/v2.2.6/numpy/_core/include/numpy/arrayscalars.h#L162-L168) select a scalar by nonzero truth rather than preserving the original byte. These official sources were inspected read-only; NumPy was not executed by this reviewer.

The initial fixture only supplied canonical bool bytes. Worse, two original block-field oracles also used scalar indexing on the expected source row, potentially normalizing the gold value in the same way. Thus original12 PASS did not establish the claimed complete retained-field byte preservation.

Root's corrected candidate at lines141-144 uses `source_field[face:face+1].tobytes(order="C")` for every field, retaining a one-row ndarray until the independent bytes copy. No arithmetic, source-number remap, role/order change or gate change is involved. `FaceBlock.get` returns an ndarray view on that row's new bytes, not a bool scalar. The revised expected block-field bytes likewise use source ndarray slices, while pair-gather oracles use independent original advanced indexing.

Final coverage includes raw flags for an own endpoint and the opposite-owner incident endpoint, an unowned quad flag surviving through a state-only reference, raw flag decoding after original object release, and corrected older own/unowned full-field oracles. The found defect is closed in candidate `bae3...7512`; the original12 result remains separately preserved, not relabelled.

## Actual ownership / absence of an old full-table pin

For objects produced by `project_frame` from valid original inputs, the present static dataflow has no old SourceFrame, original ndarray, original buffer view or persistent full-frame block registry in its returned graph:

- `FrameIdentity` lines25-30 stores only stamp, original face count, exact source-kind string and a scalar seal. The original validator normalizes integer key values and requires exact str source-kind; no custom metadata carrier is copied through those fields.
- `get_block` lines137-148 owns only a temporary per-call dictionary. Its source-capturing closure is not stored in any returned object. Each retained face receives a fresh join of all ten row-field byte strings, with the complete139-byte row length checked.
- `FaceBlock.get` lines40-46 can keep that independently owned139-byte payload alive, not a view of the original full source field. `gather_original` lines93-106 joins decoded row bytes in the original occurrence/endpoint order and returns a new immutable byte-backed ndarray.
- Roles and original face indices are separately copied into immutable bytes at lines109-111 and161-162. The original packet is not retained. No module-global frame/block registry is created.
- Own blocks, required blocks and unowned blocks share the same local FaceBlock object where appropriate. Releasing another lease can therefore release its unrelated blocks without dropping a face still required by the retained lease.

This conclusion is a static review of the current constructors, not a security claim against deliberate in-process object forgery or a guarantee that exception tracebacks/unrelated callers release their own references.

## Semantic row coverage and order

`project_frame:149-160` preserves:

1. Every source row with `quad_valid` and owner<0 in a shared unowned capsule, in original face order, independent of which owner packets survive.
2. Every own row whose original valid root OR valid quad is required, including isolated own roots absent from directed pairs.
3. Both original endpoints of every packet occurrence, including the opposite owner of incident support.
4. All ten fields of every retained row, including invalid placeholders, neighbor/contour/owner IDs, NaN payloads, signed zero and raw flags.

Sorting/deduplication at lines156-157 affects only the physical required-block table. It does not deduplicate or sort `face_indices` or `roles`; repeated incident events and their endpoint order remain in the copied arrays and decoding loop.

The final test source has18 explicit methods and272 lines. New tests49-67 use two nonadjacent unowned faces4/7 and compare every complete row after original references are dropped. Tests69-81 retain an isolated own valid root with `quad_valid=False` and a NaN quad placeholder. Tests83-130 cover noncanonical own/incident/unowned flags and post-release decoding. The original full pair dtype/shape/byte, duplicate-event, source-isolation, invalid-placeholder, empty-packet, immutable-output and zero-packet assertions remain.

These are small independent dense-source indexing/slice oracles, not a new builder reader or a decoded-equivalence comparison of all real arithmetic/box/permission outputs. The field layout is taken from the current original schema; this review does not claim independence from all possible schema drift.

## Weakref evidence: useful but bounded

The tests at183-195 show collectibility of the observed original SourceFrame and ten ndarray objects while a retained packet decodes. Tests197-210 observe one unrelated FaceBlock (face5) becoming collectible. They do not independently prove that every original immutable backing bytes object, every dropped face or Python allocator/RSS allocation was released. For example, retaining an original array's ultimate bytes backing through a hidden header could allow its ndarray weakref to die while still pinning the old buffer.

No such hidden backing/header/registry retention path was found in the current implementation: its absence is supported by the separate static field/closure/copy review above, not by weakrefs alone. `red_green.json:48` now explicitly states this limitation. This is not an unresolved implementation blocker for the D candidate and does not authorize a physical-memory savings claim.

## Source-event and representation boundaries

- Each `project_frame` call mints a fresh FrameIdentity at135-136. Re-projecting the SAME SourceFrame twice produces two frame identities and duplicates the full legacy source charge in inventory. The intended integration contract must be one complete projection per original event, with its projected packets/capsule reused; this is an unenforced caller obligation explicitly recorded in `red_green.json:48`. It must not be confused with stable old-table identity across arbitrary calls.
- Conversely, different original SourceFrame objects sharing a timestamp must remain distinct. The explicit same-stamp distinct-source test checks that case. Do not "fix" the previous boundary by globally merging timestamps.
- BlockPacket/KeyFrameLease/UnownedCapsule have their own diagnostic representation and seal. They are not original PairPacket/SourceFrame instances; original type/seal checks are not bypassed. No automatic legacy-builder substitution or R import wiring was added by this candidate.
- This is not exact legacy API/exception/writeability parity either: diagnostic gathers are immutable, whereas the original PairPacket gather is a transient independently mutable array. Content preservation in this new reader must not be promoted to an unreviewed drop-in replacement contract.
- The capsule intentionally preserves the complete valid-unowned set reachable from a retained source event. This candidate does not resolve when that event should cease being retained in global5+1/orphan/state history.

## Units were not relaxed

`inventory:190-192` retains the existing source/packet weighted formula separately: original full face count plus one per source event, and10+2*occurrences per retained packet. It does not reduce coefficients in response to a smaller projected payload. Other state/affine/piece/contributor/base/permission charges and the actual production capacity gate are outside this prototype.

The8-face/one-key fixture's face payload decreases from1112 to695 bytes while packet payload stays51 and the old source+packet charge remains25. That is a count of these specific byte buffers, not measured Python heap/RSS, a whole-map compression ratio or a capacity/runtime improvement. `heap_or_runtime_acceptance` remainsFalse.

## Execution chronology — root-owned, not reviewer execution

The reviewer statically read `RESULT.json`, `red_green.json` and `final18.json`; figures below are root's recorded executions, not tests rerun here or a new audit of their raw process output:

| Stage | Candidate | Test source | Root record |
| --- | --- | --- | --- |
| Original fixture batch |906d...f5a1|a745...546d|12PASS,0.013s, exit0; insufficient raw-bool coverage; RESULT preserved |
| New raw-bool counterexamples |906d...f5a1|e322...3652|3methods /7 failure entries including subtests,0error/skip,0.008s, exit1 |
| Same new test source after row-slice fix |bae3...7512|e322...3652|15PASS,0.018s, exit0 |
| Opposite-incident and old-oracle strengthening |bae3...7512|8048...9b36|16PASS,0.021s, exit0; distinct test hash |
| Final unowned/isolated-root strengthening |bae3...7512|e4fd...5e63|Separate final18 record:18PASS,0fail/error/skip,0.023s, exit0, PID16504 recorded independently absent; distinct final test source |

The red subtests stop at their first failed assertion;7 failure entries are not7 independent methods and do not establish execution of later assertions. Neither the original12 nor the15/16 records are retroactively called18 or assigned the final test hash. Ordinary imports plus a few source hashes are not a217-source closure/raw-loader attestation. PID/birth absence claims are root-owned; missing birth observations are not fabricated.

## Final reviewed source identities

| File | Bytes / lines | SHA256 |
| --- | --- | --- |
| PLAN.md |1107 /17|`453f3b38e3b4d1f8bc427227e6bcb71be7f51ff9a6fe669edf8b3230c0154bc7`|
| retained_blocks.py |7770 /194|`bae3efa9d89a600bf076c92c73f272ca6081e55918b2dab697e876cd5b407512`|
| test_retained_blocks.py |14192 /272|`e4fdb64854af393ef01ee235436a13a39c4a9dd4de79bbe9879741d455475e63`|
| RESULT.json (initial12, unchanged) |2747 /45|`29033fbbf6b22284d05cb2771481527b353c34edeb079a904a4157bcdabd1643`|
| red_green.json (through16, unchanged) |2426 /49|`b82dacb2378d3990d627878a9ed85f693cc6beeb18d9b91dfe05367f0660e12f`|
| final18.json (separate later record) |1637 /22|`ffe303508bf35d13204cfa4d96865c917087c29aa28183ecdd1feacea8f1f89b`|

Original reviewed candidate SHA256 was `906d9506bfd07d6b2d53d4bfb3595d2d873c89b5d43ccf13505f96501f88f5a1`; initial test SHA256 was `a745c78ef202294eaac25fd6498560bb5184454ebec4edda767715f7f211546d`. The reference evidence module hash recorded by root is `7c500480134bd1f70c7a1348a50d8e56efc5fdb2722f304791ecf86fd4b8c665`. This review did not alter or import that module.

Final scope remains a D-only storage projection and bounded synthetic review. Copying these sources into a session as `.txt` evidence is not production integration. Full builder parity, all global evidence/lifecycle rules, transactions, original real inputs, capacity-unit correctness and the original40/80/200ms acceptance chain remain explicitly unimplemented/unverified here.

