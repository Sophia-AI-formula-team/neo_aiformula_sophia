# Agent session: 2026-10-11-planner-finite-domain-r17-014

Reviewed partial-result checkpoint, not deployment acceptance. Root reviewed
source differences, the adapter archive and all 68 indexed hashes. This session summarizes
the final frozen `4d5dbea1…` finite-domain candidate and retained earlier
attempts. Full native03 and recorded04 are complete. The checkpoint remains
an honest failure of recorded-runtime and geometric-quality acceptance.

## Objective and scope

Represent repeatedly observed within-record finite support causally, without
inventing graph nodes, padding cells, inserting future data, discarding original
foreground mass or lowering the existing 5-frame/1-reliable, geometric and
40/80 ms runtime gates. Full-mask consensus/display remains separate.
Vehicle, simulator, control/speed and sensor-source parameters were not changed
for this work. A complete teaching/repeat lap is still not demonstrated.

## Input handoff

Consumed handoff: `2026-09-28-planner-neo-004`. Read AGENTS, AGENT_CONTEXT,
current STATUS, PROTOCOL and the incoming task. Base:
`c7e316a8b9e52afb87f7d00db564cc670295b7b4`
(`lane-mapping-20261011-r16-rc1`). Canonical destination remains
`Sophia-AI-formula-team/neo_aiformula_sophia`, repository ID `1303517209`.
Private drafts01/02 remain unchanged historical summaries, not publication
receipts. The earlier R15 6/8 CI result, including two GNSS DDS failures,
is not retroactively made green by this local work.

## Source and working tree

Branch: `repair/lane-keyframe-transaction-20261011`; working tree is dirty
with coordinated production, tests and documentation. No clean-tree claim.
Root owns integration/publication. Unrelated unintegrated QP prototype files
are not part of this candidate.

Pinned tested finite-support SHA256:
`4d5dbea1be3e83bf7640b372d578911275e3832675edcef92bcfff515766dce4`;
mapping:
`a2171210d68c1d3d55e67cf2e826914f109f382280a7dbdc67a05571da574b88`;
paint:
`3b75c68ec9072a9a2a4889381e499df3e7e9871d28cbf1cbe9aae47223a21f35`.
The selected manifest and exact evidence hashes are in
[evidence/reviewed_summary.json](evidence/reviewed_summary.json).
These identify tested snapshots, not future source modifications.

The final full native03 run checked 166 source identities, zero changed contents and
seven real native API calls. Native C SHA:
`a39fc90d8373056bc98dc45f936e60cc5d2ef6639aea2417e0e35fde35be6705`;
Windows binary SHA:
`0674d769ef5f70bfd4c3ef054fefbe96778e74d0a375f32630a8c7a023451165`.
Installed-prefix verification is false: this is Windows source-tree evidence,
not Foxy build/DDS acceptance.

## Work performed

1. Added immutable fixed-creation-chart support domains from actual adjacent
   same-frame model segments. Each frame earns at most one vote per location;
   unreliable bridges cannot create reliable coverage. Original confirmation,
   TTL, capacity and transaction requirements remain.
2. Boundary v3 persists five hash-covered certificate arrays (segments, owners,
   frame/reliable votes, source stamps), with explicit representation
   `confirmed_creation_chart_domains_v1`. Explicit graph arrays are required.
   All-None certificates preserve old v1/v2 hashes; outer schema/16 MiB cap
   did not change. Loader/provenance updates preserve selected historical
   dependencies rather than silently adding current helpers.
3. Removed repeated scalar NumPy nextafter overhead with independently checked
   IEEE behavior; active coverage rejection can try a strictly past fresh
   candidate through the same guarded commit path.
4. An exact-endpoint continuity fix replaced separate inward rounding of every
   segment by independently connected base/trusted segment components. Only
   completely equal physical endpoints connect; no epsilon, near-neighbor
   merge or filling of real gaps is permitted. Projection is still conservative.
5. Coalesced equal interval tuples before constructing the final validated
   immutable Interval objects. Thirty strict equivalence tests compare old
   eager construction with the final path, including byte-identical state and
   export buffers, errors, counters, stamps, TTL, capacity and immutability.
   These tests are included in full native03; no separate new run is claimed.
6. Retained failures and diagnostic limitations, and archived the changed
   Simulator adapter afterimages plus dependencies at
   [evidence/simulator_adapter](evidence/simulator_adapter/README.md).

For immutable creation chart `(o0,t0)`, parameter `s=t0·(q-o0)` is mapped
to current line `(p,t)` using
`F(s)=p+((s-t0·(p-o0))/(t0·t))*t`, with the original normal gate and finite
representability checks. **This is a finite fitted model, not ground truth
and not a claim that all past raw points lie exactly on the fitted line.**
Incident-domain support belongs to future R18 research; it is not implemented
or accepted into this R17 version.

Commands and environment are retained in local indexed originals; public
summary uses relative paths. Core runs used Python 3.13, native required,
four control-package roots on PYTHONPATH, `-B -m pytest -q -ra -p no:cacheprovider`,
the reference and GNSS test directories, and excluded only the separate
unintegrated `test_quantized_curve_fit.py`. No tests or Unity were rerun
while preparing this documentation.

## Evidence and result

| Attempt | Recorded result | Evidence level |
| --- | --- | --- |
| Standalone domain prototype | 47 passed | Isolated correctness |
| Initial v3 contract | 28 passed / 19 failed | Real combined-array IndexError; preserved |
| Repaired contract + legacy focus | 117 passed | Scoped contract |
| Initial integrated domains | 70 passed / 3 failed | Real world-ULP export errors; preserved |
| Repaired integrated domains | 85 passed | Export/TTL coverage |
| Bundle / replay / additional adapter focus | 115 / 93 / 9 passed | Separate overlapping focused runs |
| Initial full native01 | 2878 passed / 1 skipped / 1 failed | Failed attempt retained |
| Narrow legacy-fixture correction | 1 passed | Explicitly clears all five v3 fields to make v2; old assertions retained |
| Nextafter focus | 17 passed | Exact scalar/pipeline coverage |
| Active-refresh focus | 136 passed: 42 new + 94 old | Strict-past/ownership regression |
| Connected-projection focus | 128 passed: 26 new + 102 old; 0.92 s | Fraction subset oracle, 48 random subcases |
| Full native02 | 2964 passed / 1 Linux-only skip; pytest 98.41 s, process 99.384 s | Exit0, zero source changes; local core only |
| Final full native03 | 2994 passed / 1 Linux-only skip; pytest 102.54 s, JUnit 102.523 s, process 103.8424893 s | Exit0, 166 sources unchanged; includes 30 coalescing-equivalence tests |
| Recorded01 | 0/1 accepted; seq151 deadline 81.6843 ms | Failed |
| Recorded02 | 5/6 accepted; seq156 deadline 86.5323 ms | Failed |
| Recorded03 | 11/12 accepted of111 available; seq162 deadline 84.7509 ms | Failed despite integrity checks passing |
| Final recorded04 | 12/13 accepted of111 available; seq163 deadline 90.3375 ms | Failed despite integrity checks passing |
| Four development cases01 | 0/4; left fails frame7, right frame9 | Failed |
| Four development cases02 | 0/4; left frame8, right frame10 | Failed |
| Four development cases03 | 0/4; left frame8, right frame10 | Failed, report SHA fe454455…7c5d21 |
| Development cases04 | Not run; runner prepared only | No new measured quality result or pass claimed |

The sole full-native01 failure was an old fixture removing graph arrays while
leaving new v3 certificates; mixed-representation rejection was correct.
The repair explicitly constructed legacy v2 and retained its assertions.
The Windows skip is the existing Linux process-group supervisor test.
An earlier adapter invocation failed collection through cross-drive pytest
root discovery; it ran zero cases. Correcting cwd/rootdir only gave 93 passes.
No failed run was replaced, and overlapping counts are not a unique total.

The isolated first-frame prepare measurement changed 75.4462 →31.9094 ms,
with plain/profile tokens exact apart from owner identity and no commit.
This is a single-input measurement, not an end-to-end latency distribution.

A separate seq156 diagnostic completed six extractions, five accepted-past
prepare/commits, one target plain prepare and one target profiled prepare.
Post-profile fragmentation arithmetic then failed. Read-only recovery used
saved artifacts with zero extraction/prepare/commit/engine calls. Unpersisted
ordinary timing and target token fingerprints were **not** reconstructed or
guessed. Possible peer Unity activity also prevents runtime comparison.

The synthetic support audit used exactly nine original engine calls and eight
commits; all non-timing rows reproduced development02. Full denominator was
60.5, fresh covered mass32.25 (0.5330578512). Its disjoint rejected groups were:
22 terminal-extrapolation queries/mass10.5; 3 over-distance/mass1.5; and12 with
no nonzero primitive in the original hash-group AABB/mass5.5. For the22 terminal
queries,18 were outside all selected-owner domains and4 underconfirmed; none
was in that owner's confirmed interval or gap. Nearby latest raw faces are
not replacement matching authority.

The same audit found 8263 true shared endpoint projections excluded by old
independent inward rounding (8249 fully reliable); stored domains had412 gaps,
409 within64 endpoint ULPs, one at66 ULPs and two real gaps of1.859503/5.911443mm.
None of the gaps had fully confirmed support on both sides. This proves a
numerical continuity defect, **not** that closing it cures the actual failed
queries. Development03 after the fix still failed all four cases.

Recorded03's visual candidate alone passed in22.02 ms with overlap0.79710145;
the whole engine exceeded80 ms, so seq162 was not committed and trace count
remained11. Process exit0 means report generation/integrity succeeded, not
algorithm success. Original gates and111-frame input availability remain.
Synthetic cases retain their predeclared2000 ms offline geometry budget; they
are not runtime acceptance or untouched holdouts.

Final recorded04 used the frozen final source with all input/configuration/
source/native integrity checks true. Its visual active-reference match passed
in30.58 ms with overlap0.6448275862, but engine time was90.337499976 ms;
seq163 was rejected without commit and trace count remained12. This is
captured-input replay, not actual driving. Full native03 also passed all30
coalescing-equivalence tests, source SHA
`c72f14d9cca5240eb9de774f131590b6917cb7541cf81239a0a0aefd0b4f33a7`.
They support unchanged finite-domain state/export semantics for the latest
optimization; they do not count as running quality04. The last measured
four-case quality result remains quality03, **0/4**, and no improvement in
that result is claimed. All earlier failed attempts remain indexed.

## Risks and blockers

- Final native03 covers the frozen coalescing source, but recorded04 still
  exceeds the unchanged engine deadline. All measured quality cases and
  recorded prefixes remain failed. Quality04 was not run. No complete lap or
  useful long-run quality claim follows from these unit results.
- Per-owner repeated support/identity remains a geometric limitation. Do not
  rescue overlap by dropping denominator mass, promoting latest raw faces,
  padding endpoints or importing future observations.
- ROS source-age, queue-to-command/DDS and live end-to-end acceptance remain
  separate from Windows core tests and isolated preparation timing.
- No new actual29 occurred. The Editor was loaned to the peer semantic task;
  it reported clean Edit restoration and return at 2026-10-11T11:38:36Z.
  This report is coordination evidence, not our own fresh runtime preflight.
- Raw originals may contain private paths/observations and stay local. Public
  material contains only sanitized summaries/hashes and reviewed code text.
  Publication review covers this sanitized record and inert source archive only.

## Next agent actions

1. Preserve this final source-specific R17 failure checkpoint and all earlier
   attempts. Any later algorithm or runtime optimization requires a new
   candidate and separately recorded evidence, not overwriting these results.
2. Continue geometry and whole-engine timing diagnosis under unchanged gates;
   treat incident-domain ideas as separately scoped R18 work, outside R17.
3. Review the 12-file adapter archive and its dependency/restore limitations.
   It is byte-exact afterimages, not a fabricated pre-edit patch or full Unity
   distribution. No binary, scene, recording or private absolute path is bundled.
4. Verify the companion bundle hash and publication receipt. Editor ownership
   has returned; still perform our own live protected-state preflight before
   any separately recorded actual trial. Do not treat the peer's trials as ours.

