# Agent session: 2026-10-11-planner-repeat-handoff-006

Input handoff: 2026-09-28-planner-neo-004

Reviewed partial planner checkpoint. Not a successful-lap or vehicle-release
certificate. Prior session005 and its R1 tag/archive remain immutable.

## Objective and scope

Continue the user's authorized repair of the actual neo mapping packages and
simulation teacher/repeat adapter. Retain version tags, logs and videos. Do not
change vehicle/simulation/reference-speed parameters, use future observations,
feed teacher ROI/fitted points to mapping, use GNSS for mapping, widen gates or
call a partial trial a completed first-map/second-repeat lap.

## Input handoff

Read AGENTS, AGENT_CONTEXT, STATUS, PROTOCOL and the existing neo-004 handoff.
The prior field handoff is historical; R1 repair is an explicit simulation
checkpoint, not an automatic field baseline. Publication remains only
Sophia-AI-formula-team/neo_aiformula_sophia, immutable repository ID1303517209.

## Source and working tree

Initial clean branch repair/lane-keyframe-transaction-20261011, HEAD
e28c9dc5d591ad3ba9cf0e68be7d18431e2fb8a4 (documentation after R1 code422cb4b).
No previous package edits left unstaged. This session initialized before the
new repeat-handoff production edits. Separate simulator Tools are not a Git
checkout; their source hashes/snapshots and original-file manifests are retained
locally. Shared Unity/Assets are currently leased to a separately authorized
physics diagnosis; this planner makes no Editor or Assets calls during that lease.

## Work performed

1. Actual trial14 used R1 in the new LaneTeachRepeatValidation scene with the
   same111 live physical parameters. 101/101 mapping frames accepted; sampled
   travel26.627889m,1011 consensus cells. seq200 failed the simplified teacher's
   longitudinal-support check, before mapping. Safe E-stop/Edit snapshot taken;
   no frozen route/repeat lap. Exact paired evidence/video remains local.
2. Adapted the simulator teacher to the complete pinned current neo frontend,
   retaining the current feature LYA speed profile and target/omega only. Full
   original mask projection remains the map input. 143 adapter tests passed;
   original200 image sequence is supported by upstream, all right_only (the
   original2.6m assumed width is explicit). This is not new closed-loop evidence.
3. Independent review found a second actual core error: repeat_localizer() and
   worker bundle preparation discard the committed teach correction while using
   the same continuous raw odometry. A real-core synthetic17-frame test accrued
   y=-0.6m correction; new zero matcher had122/122 inliers and zero RMS but failed
   initial_innovation_gate. Same-map fork preserving the committed correction
   accepts without gate changes. This is a code-path reproduction, not an observed
   failure in trial14, which never reached repeat.
4. Implemented same-session frozen-map handoff through existing public
   repeat_localizer(), plus snapshot ownership and exact loaded geometry/hash/
   frontier verification. New immutable CommittedLocalizationState carries only
   configuration, committed correction and accepted/seen source times. Pending
   proposals, candidate, counters and refinement do not cross the boundary.
5. The simulator captures only that small state before its worker starts. Full
   index construction stays in the worker, not in the command-heartbeat callback.
   Returning state/config/normalized anchor must match before route/controller
   publication. Three NEW stationary matches remain mandatory before attachment.
   No pose reset, arbitrary relocalization seed or gate/parameter edit.
6. Full two-package regression:691 passed,1 Windows Linux-only skip,17.76s,
   exit0. New state/handoff34 cases plus R1 transaction42:76 passed,2.34s.
   Simulator160/160 passed; independently captured report13.770s,exit0 for tests.
   That report's combined command exits1 because original-file preservation
   correctly flags12 concurrent, separately authorized simulation edits; the
   immutable878-file baseline was not replaced. This is not an overall clean-
   plant preflight claim. Full modified source/diff reviewed before packaging.

## Evidence and result

Partial; production handoff edit/test complete, next actual trial pending. Do not
interpret offline target support, numerical unit tests or Foxy CI as physical
two-lap completion. The public R1 receipt contains the already-passed CI results;
this new change requires its own tag/CI result before publication claims.
See [reviewed verification](evidence/verification.md) for exact commands, failures,
metrics and artifact hashes. Raw simulation logs/images/video remain local;
they are not uploaded or included in this small public checkpoint.

## Risks and blockers

No completed map lap, route feasibility, stopped attachment or repeat lap yet.
Current upstream single-side width/asymmetric offsets are inherited assumptions,
not road measurements. Shared-editor ownership must be released explicitly.
The other physics task promises old plant behavior via disabled opt-in settings;
verify exact baseline and replay equivalence before driving this scene again.
Only state capture is constant-size. Snapshot copy/journaling and source-anchor
normalization still scale with map size;100000-point normalization p95=61.62ms
in an isolated7-run check is not a complete200ms callback/deadline guarantee.

## Next agent actions

Publish the reviewed same-session handoff checkpoint with a new R2 tag; do not
move R1/baseline tags. Run and read back Foxy/handoff CI. Once Editor is released, preflight the
unchanged physical baseline, run a fresh trial with current mask-only LYA target
frontend and R2 mapping code. Save every failure and render causal consensus.
HOLD is terminal, never a completed lap; do not relax gates to manufacture pass.
Before publication, validate/package and verify exact authorized push URL/
repository ID. Use a new append-only receipt for actual publication/CI results.
Do not edit sealed session005 or this session after packaging.

