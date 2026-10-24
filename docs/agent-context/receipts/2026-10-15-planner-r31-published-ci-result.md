# RESULT_READY — R31 published; CI7/8, driving acceptance unmet

Planner,2026-10-15. Incoming neo-004. This append-only receipt supersedes the
publication-pending wording in the R31 local-result receipt, not its evidence.

Canonical repository1303517209 was verified through GitHub and the exact push
URL before a normal atomic branch/tag push. Published code:
`f6a192e604d7ffb8ea18a5a29e3608557a568d26`, branch
`repair/lane-keyframe-transaction-20261011`, annotated tag
`lane-mapping-20261015-r31-rc1` (tag object
`b7b01c9ff9ce0a3910ecdbcd3fca45d5b9fb2fe6`). ls-remote readback matched the
branch, tag object and peeled code SHA. No main, force or legacy-repository write.
The earlier interrupted publication attempt remains an immutable historical
receipt; the later successful ordinary push is distinct.

Sessions027 and028 and both sealed bundles are now in that exact commit.
GitHub contents readback confirmed028 ZIP1267109bytes, Git blob
`3d3af5a6bab56478f5baf3cc7f1e287d0a8710c9`; its SHA256 is
`d495aae7095fb1eaa522921bc75f2570f5f4384e7809d32be91d5ae435804623`.
027 ZIP92444bytes, SHA256
`454a9479665743f75e99a2dd130fc7df7931ef6a624ca240eb0f2045d1b83f0d`.
Local handoff-tool regression after sealing:32pass/1Windows privilege skip.
Five known EOF whitespace warnings remain recorded; no clean-style claim.

Exactf6a eight CI runs are terminal:7success/1failure. All handoff, native graph,
and teach-repeat workflow runs passed. Endpoint results differ:

- [Branch38688191457](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38688191457):
  4722unit pass/1skip,4Header tests,9stationary synthetic DDS cases/171checks.
- [Tag38688191261](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38688191261):
  4721unit pass/1skip/1fixture error. Header and DDS **not run**. Frame16 was
  rejected during setup, before the target repeat-state test body. The reason
  is truncated and cannot be reconstructed; neither engine nor matcher timeout
  is established. Workflow did not capture the actual unit pytest.xml.

No unchanged rerun was requested to obtain green. Synthetic DDS uses0.2m/s
fixture commands, not the actual2m/s LYA reference; its stationary route remains
insufficient for REPEAT. This is not a passed lap or physical-stop test.

[Sealed session029](../sessions/2026-10-15-planner-r31-hotspots-029/SESSION.md)
contains reviewed terminalCI counts, exact-code profile and dependency-reuse
census. ZIP SHA256
`023d99a517af089bcb90e1d4f0cbbf95048033548a23c93c976ae49c711d1d7d`.
It establishes limited whole-key reuse, not a runtime speedup. R31 still exceeds
the original clock scopes and refuses163 at the unchanged capacity gate.

Acceptance remains: unchanged vehicle/simulation/LYA and gates, actual first
mapping lap, freeze, fixed-route second lap, logs/errors/video. None completed.
No Unity Refresh/Play was performed. V5 remains explicit development-only,
not enabled in TeachMapEngine/control. R32 is a separate unsealed continuation,
not a deployment baseline. No physical driving permission is implied.
