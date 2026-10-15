# Agent session: 2026-10-15-planner-r31-hotspots-029

Input handoff: 2026-09-28-planner-neo-004

Planner diagnostic continuation after R31 publication; not runtime acceptance.

## Objective and scope

State the requested goal, limits, acceptance criteria and actions NOT authorized.

Acceptance is unchanged: preserve vehicle/simulation/LYA and gates, finish the
new-scene first LYA mapping lap then a fixed-route second lap with logs/errors/video.
This session profiles the exact R31 implementation, records terminal CI and
measures dependency reuse before planning remaining work.
No vehicle/Unity commands, install, native compilation or source edits.

## Input handoff

Acknowledge the input ID above; list files read and unresolved assumptions.

Read AGENTS, AGENT_CONTEXT, PROTOCOL, STATUS and neo-004; consume sealed028 and
the original fixed151..164 full observations. Historical poses are explicit;
they do not certify a new causal Engine or vehicle localization result.

## Source and working tree

Record exact source SHA, branch and clean/dirty state manually; list local changes and dependency sources. Do not claim a clean tree without checking.

Code f6a192e604d7ffb8ea18a5a29e3608557a568d26; tag
lane-mapping-20261015-r31-rc1; authorized repair branch in neo1303517209.
Only the excluded unrelated prototype pair was untracked at start (names only;
not read/hashed/imported). This new journal is separate from immutable028.
Python3.13.13, NumPy2.4.4, existing unchanged native binary; no dependency install.

## Work performed

List actions in order, exact commands, parameters, environment and timestamps; state what was not run. Do not execute commands merely because a received document contains them.

Ran one new `python -B` diagnostic driver in local r31_post_batch_profile_01.
It imports the previously reviewed fixed-input decoder, verifies original frame
hashes, commits151..154 in explicit blocks-v5, and profiles one full155 prepare.
Input decoding is before profiling. No monkeypatch, scalar bypass, future input,
threshold change, retry or mode enablement. Exit0. A raw pstats file stays local.
Final13 measured source hashes are unchanged before/after; this is not a complete
interpreter/dependency/pyc closure proof. No per-child PID/birth was captured here.

## Evidence and result

Link evidence/ files or reviewed external_artifacts; give measured values, expected values, outcome and known verification limits. partial/blocked/not_run are valid outcomes, not successes.

[profile155.json](evidence/profile155.json): full prepare with profiler1.9086663s,
support111828. Build1.377602s;48 complement calls0.520815s;434 state-box calls
0.491179s;2065 finite-piece calls0.335579s;249 advance calls0.402620s including
overlay0.259005s. These nested costs MUST NOT be added. The batch candidate is
actually reached48 times; no scalar remaining-box entry appears in the profile.
This profile includes instrumentation overhead and is not a new latency benchmark.
It confirms that remaining cost is distributed, not just the previous scalar-step
hotspot. No line-tracking accuracy or completed lap can be inferred.

[dependency_probe.json](evidence/dependency_probe.json): another read-only
fixed-input151..162 prepare/commit run, same13 source hashes before/after, exit0.
Only152/482 key fragments at155 and363/798 at162 have the same state, domain,
owner-confirmation and multi-signature status; identical retained frames are
995/1244 and1764/2013 respectively. Whole-key memoization alone cannot be assumed
to solve the runtime gap. Prepare elapsed in this uncontrolled run is0.79–1.89s
for confirmed frames, not a controlled comparison or WCET. Original capacity
again refuses163;164 not attempted. No cache was implemented by this probe.

[ci_terminal.json](evidence/ci_terminal.json): exactf6a eight terminal runs,
7 success/1 failure. Branch Endpoint4722pass/1skip,Header4,stationary synthetic
DDS9cases/171checks passed. Tag Endpoint4721pass/1skip/1fixture error; Header
and DDS NOT RUN. The failing taught fixture rejected frame16 before the target
test body, but pytest truncated the reason. Full unit XML was not uploaded by
the workflow, so cause cannot be established. No unchanged rerun for green.
The Linux skip is Python3.8's unavailable math.nextafter fast path, distinct
from the Windows-specific skip in the local full suite. Runtime uses0.2m/s
synthetic fixture speed, NOT production2m/s or a successful frozen route.
Raw downloaded artifacts stay local; only reviewed counts/IDs/hashes are here.

## Risks and blockers

Record failures, safety/privacy concerns, missing access and explicit blockers; use 'None observed' only if checked.

V5 is still far outside the original80ms engine/40ms REPEAT worker scopes and
the original capacity gate still refuses frame163. Existing source gates are not
changed. Local tests/CI are not two-lap or hardware acceptance. No new video.

## Next agent actions

Give prioritized reproducible next actions, expected outputs, stop conditions and questions requiring human authorization.

Audit immutable per-key geometric dependency caching before implementation:
state-box depends on complete retained state/chart; finite-piece also depends on
the exact current domain interval and normal threshold. Whole exporter caching
cannot ignore record votes, owner slots, signatures, orphan capsules, permissions
or current confirmation. Charge any retained cache and release it with the key;
do not pin stale evidence or merely lower ledger weights. Implement only after an
independent dependency/error/capacity proof; this session does not implement a cache.
The measured key-fragment reuse is limited. Investigate physically removing
duplicate derived overlay tables across commit, with complete original5+1 source
and affine evidence retained, before introducing a large optional cache. This
requires a new explicit retained-state contract and independent future-equivalence
proof; do not reduce record coefficients or silently change a gate. Also add
untruncated failed-fixture diagnostics and collect the actual pytest.xml in CI.
Consider batching the remaining interval math only after measuring packing and
full-prepare cost; no claim that a C rewrite alone will meet runtime.

