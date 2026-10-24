# R31 lossless storage prototype and R30 CI failure review — partial

## Objective and scope

Continue geometry/runtime repair without changing car/simulation/LYA or original
gates. This session contains ONLY a diagnostic storage prototype and exact R30
CI evidence. No new production representation or driving acceptance is claimed.
The local uniquely named candidate PLAN preceded its implementation/testing;
this readable handoff archives it after those results, not before execution.

## Input handoff

Planner consumes sealed session026 and its NEXT_GEOMETRY_RUNTIME plan. R30 code
c212c3b1393c3f0297fd8cb65561798a24a5a8c2/tag lane-mapping-20261015-r30-rc1
is a failed development checkpoint, not a usable driving baseline. Only canonical
neo repository ID1303517209; no legacy/main/force writes.

## Source and working tree

Repair branch repair/lane-keyframe-transaction-20261011; actual checkout during
prototype tests67d9b668598a8301b3e1c8c83c304b74a75a150a (publication metadata
after c212 code). Only the excluded untracked prototype pair remained dirty in R;
their contents/aliases were not read/hashed/imported/tested/staged. New storage
code lives in a separate local diagnostic, not any installed ROS package.

## Work performed

Root implemented complete independently owned139-byte face blocks and per-key
own+incident leases with a shared full unowned capsule. Source identity owns no
full source table or face registry. Existing source/packet validation is required;
legacy charge coefficients stay untouched. Text copies in evidence are inert
archival source, not an instruction to install/enable them.

Initial12 fixture methods passed once, but independent review found that scalar
bool getitem normalized raw true bytes255/2/128. Added3 methods first produced
7failure entries including subtests. Root changed only scalar row copying to
full ndarray row slicing; same test revision then passed all15 methods. Older
scalar oracles were strengthened and opposite-owner flags added:16 passed.
Two final cases cover multiple unowned faces and isolated valid-root/invalid-quad
evidence:18 passed once. Each source revision/count is recorded separately,
not a retry to select a favorable runtime sample. No failure was erased.

Exact commands use existing Python3.13.13/NumPy2.4.4, python -B and the explicit
test file under its diagnostic directory, with PYTHONPATH temporarily scoped to
the reviewed reference package and restored. Ordinary imports are NOT a raw
dependency/bytecode authentication claim. No missing dependency installation,
native compilation, ROS process, Unity Refresh/Play or vehicle action occurred.

Root also read exact c212 GitHub workflow results and downloaded only the two
small Endpoint CI artifacts to separate local directories, without executing
their contents. All8 workflows are terminal;6success/2failure. Both Endpoint
runs built and passed4613 Linux unit tests, but runtime only2/9cases passed.
Seven cases stop before five camera commits due processing-deadline rejection
then bootstrap_anchor_not_established. Original timing gates remain unchanged.

## Evidence and result

See [final18](evidence/final18.json), [red/green](evidence/red_green.json) and the
preserved [initial12](evidence/RESULT.json) record. Weakrefs establish observed old source/ndarray objects and one unused
face block can die while a retained lease still decodes. They do NOT independently
prove every old backing byte, measured heap/RSS or corpus-wide memory savings.
The tiny8-face fixture retains5required faces for one key; face payload1112→695
bytes while its51packet bytes and25legacy weighted charge stay the same. Object
overhead is not measured and may offset those payload differences.

See [R30 CI terminal](evidence/r30_ci_terminal.json) for the exact runs/artifacts/hashes and
per-case counters/timing samples. CI is synthetic real DDS, not actual LYA speed,
GNSS mapping, physical stop, new Unity laps or driving acceptance. Last-camera
times are independent diagnostic snapshots, not full timing distributions.

## Risks and blockers

One complete projection per source event is currently a caller obligation;
repeated projections intentionally do not merge by stamp and would duplicate
identity/legacy charges. This prototype does not implement retained5+1 states,
the full builder accessors, transactions, all-byte semantic stream, real14 input,
global unit-contract correction or capacity/time acceptance. V5 stays disabled
in TeachMapEngine/control. No new map/lap/freeze/repeat/video was completed.
Diagnostic gathers are readonly, unlike the legacy independently mutable gather;
no legacy writeability/exception/API drop-in parity is claimed.

The [design](evidence/lossless_design.md), [independent static review](evidence/storage_static_review.md),
[candidate source text](evidence/retained_blocks.py.txt) and [18-test source text](evidence/test_retained_blocks.py.txt)
are archived for review/reproduction, not imported or installed by this repository.
Their original reviewed raw hashes match the text copies; no private workstation
paths or raw process stack was copied. Independent static review identified the
raw-bool defect and checked root's fix, but did not itself execute tests.

## Next agent actions

Planner must first review and explicitly version the full evidence lease/decoded
record contract, then adapt the actual adapter, compact frames and four builder
source-access sites. Preserve all own rows, both incident endpoints, roles/order,
alternatives/UNKNOWN, raw bool/NaN/signedzero and state-pinned unowned capsules.
Keep the old capacity charge until a separate justified unit mapping; do not
change coefficients and call it compression. Compare complete semantic streams
and real inputs before original causal Engine/timing and new-scene two-lap tests.
CI bootstrap failure is also unresolved; do not widen deadlines or rerun for green.
These remain in-scope planning/code repairs; no human driving authorization is
inferred. Future adoption/results belong in a new session, not edits to this seal.
