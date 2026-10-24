# Agent session: 2026-10-15-planner-runtime-integration-r31-028

Input handoff: 2026-09-28-planner-neo-004

Open planner work record. Not a driving baseline or completed acceptance.

## Objective and scope

State the requested goal, limits, acceptance criteria and actions NOT authorized.

The user's acceptance is unchanged: repair geometry in neo, preserve all vehicle,
simulation and LYA parameters and existing gates, then complete a new-scene LYA
mapping lap and fixed-route second lap with logs, error metrics and video.
Current work first addresses measured owner-domain processing cost and complete
source retention. No deadline/5+1/capacity relaxation or control enablement.

## Input handoff

Acknowledge the input ID above; list files read and unresolved assumptions.

Read root AGENTS, AGENT_CONTEXT, PROTOCOL, current STATUS, neo-004 and sealed
session027. Consume c212 R30 failure evidence and the isolated18-test block
prototype. Runtime v4 bootstrap still fails; v5 remains opt-in development.

## Source and working tree

Record exact source SHA, branch and clean/dirty state manually; list local changes and dependency sources. Do not claim a clean tree without checking.

Initial HEAD67d9b668598a8301b3e1c8c83c304b74a75a150a on
repair/lane-keyframe-transaction-20261011. Fifteen reviewed documentation/archive
paths remain staged from the prior interrupted upload. Separate interruption
receipt and excluded prototype pair remain untracked; none overwritten/staged.
No missing dependency installation, native compilation or Unity action.

## Work performed

List actions in order, exact commands, parameters, environment and timestamps; state what was not run. Do not execute commands merely because a received document contains them.

2026-10-15: inspected current source and original runtime profile. Delegated
read-only bootstrap and storage reviews; then delegated only contiguous-run
indexing plus its oracle test, and a new complete block module plus focused
tests. Root owns main integration and owner-domain construction changes.
All tests/benchmark commands and actual results will be appended below.

Implemented complete original-face blocks behind explicit `curve_storage="blocks"`
and `curve_geometry=True`; ordinary v4 and default dense v5 are unchanged choices.
All own roots/quads, repeated incident endpoints and unowned valid quads survive
with original bytes and event identity. Dense tables are no longer pinned by old
keys in block mode. The existing weighted capacity gate remains conjunctive and
unchanged, not replaced by a smaller count or a new threshold.

Shared changes: exact-builtin finite value construction, discrete directed-walk
lists, guarded scalar outward rounding, one-overlay-only normal gathers, and
batched complement envelopes with an immutable scalar-helper baseline. Source
reader replacement, unsupported inputs and nondefault FP policy use the original
scalar path. No sticky-FP-flag/errno or OOM exception-order equivalence claim.
Python3.8 lacks math.nextafter, so that particular fast scalar path declines there.

Commands used Python3.13.13 with `-B -m pytest -q -p no:cacheprovider`, explicit
test files only, and `--junitxml` per distinct attempt. Full tests used the prior
120-file manifest order plus five explicitly named new tests (125 files), prior
LANE_REQUIRE_NATIVE=1 and the same local fixture directory. No folder discovery,
excluded prototype import/hash, dependency installation or new C build.

Independent reviews found and repaired: subclass descriptor reads in finite
constructors; dense-vs-block oracle dependence; normal-reader replacement bypass;
batch import-cycle/late-binding risk; missing frombuffer identity guard; a test
that patched None to None on Python3.8. Changed-helper code is covered separately.

## Evidence and result

Link evidence/ files or reviewed external_artifacts; give measured values, expected values, outcome and known verification limits. partial/blocked/not_run are valid outcomes, not successes.

Fresh evidence (old results remain intact):

- `focused01.xml`: 216 pass, 1.47s, before full block integration.
- `blocks02.xml`: 113 pass / 1 fail. The retained-block reader test still expected
  the former public rejection. It was changed to exercise actual compact advance
  with all old assertions, including latest5+older reliable. The PowerShell wrapper
  masked this pytest failure with exit0; XML is authoritative, not that exit code.
- `blocks03.xml`: 114 pass, 1.06s, wrapper now propagates pytest exit.
- `batch04.xml`: 141 pass; `batch05.xml`: 145 pass, after fixed binding/reader guards.
- `full01_summary.json`: 4721 pass / 1 original Linux-only skip, native real calls
  pass; 226 source/attributes hashes stable. Original native/pytest PIDs 2000/3672
  and decimal birth tokens recorded, both independently absent after completion.
- `full02.xml` and `full02_summary.json`: final 4722 pass / 1 original skip,
  102.88s, exit0; same 125 explicit files. One intentional strict-FP underflow
  warning is retained. The only changes since full01 were the frombuffer guard
  and its test plus the Python3.8-neutral test sentinel; all other 224 hashes agree.
  Full02 has post-run hashes, not a separately captured pre-run snapshot/PID proof.
- `actual_geometry_summary.json`: original full-input151..164 v4 all commit;
  v5 commits151..162, refuses163 on the SAME original capacity error, no164 attempt.
  Full map state, required evidence, nominal/unknown geometry and permissions
  agree with c212. The later run03 reuses the immutable original hashes as oracle.
  Run02 failed before any frame due to a diagnostic default-lookup KeyError; its
  original result is retained locally and summarized, not silently overwritten.

Last committed frame162: original retained array payload3152693B becomes1449804B
(9155 retained complete faces). This is NOT heap/RSS or transient peak memory.
Weighted support193188, nominal segments3685 and blocker boxes2936 are unchanged.
Confirmed-frame155..162 prepare falls from historical baseline2451–3064ms to
post-batch942–1406ms; still far beyond runtime. These are fixed historical poses,
single sequential samples, not new causal localization or controlled WCET evidence.
The final frombuffer guard was added after this timing run; hashes bind each result
to its actual source, not a claim of exact final-build timing.

Original XMLs remain in the local `r31_publication_review_01` diagnostic archive.
Published copies remove only hostname and local repository path, then format XML;
`publication_review.json` binds both hashes. All failures/skips and case results
remain. No raw image, bag, personal path or precise GNSS is included.

## Risks and blockers

Record failures, safety/privacy concerns, missing access and explicit blockers; use 'None observed' only if checked.

Current prototype's capacity units remain unresolved; old coefficients stay.
Source block retention alone cannot repair seconds-long v5 computations. TEACH
engine80ms and REPEAT full-worker40ms are different clock scopes; both remain.
Earlier combined publication command was rejected before execution; no remote
session027 publication is inferred from staged files.

No Unity Refresh/Play, new trial, LYA lap, freeze, REPEAT, driving video or
cross-track-error result was produced in this session. Runtime v4 bootstrap CI
failure from R30 is not superseded by these offline tests. TEACH80ms engine and
REPEAT40ms whole-worker scopes are unchanged. The current code is not a deployment
or vehicle-release baseline, regardless of a future development tag/CI result.

## Next agent actions

Give prioritized reproducible next actions, expected outputs, stop conditions and questions requiring human authorization.

Measure actual computation and byte-preserving changes, then resolve remaining
geometry/runtime defects before any new Unity trial. Preserve every failure;
new-scene two-lap acceptance is still outstanding.

Prioritize the remaining scalar state envelopes and curve export cost, then the
physical retained-record capacity contract. Do not merely reduce legacy weights.
Review found only a narrow potential same-owner byte-identical exported box
deduplication; it is NOT implemented and is not known to resolve capacity.
Any native acceleration must preserve each interval operation and scalar fallback;
the existing v4 projection's zero shortcuts cannot be reused for curve envelopes.
Only after current causal Engine and runtime budgets pass may the new Unity scene
be reacquired from its peer, parameter fingerprints checked, and both laps run.

