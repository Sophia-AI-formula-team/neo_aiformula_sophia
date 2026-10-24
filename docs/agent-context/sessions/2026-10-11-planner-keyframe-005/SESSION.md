# Agent session: 2026-10-11-planner-keyframe-005

Input handoff: 2026-09-28-planner-neo-004

Sealed core-repair checkpoint. Not a hardware or completed-lap acceptance.

## Objective and scope

User explicitly authorized repair of the neo lane-mapping core, tag-based version control,
and a new-scene simulation of a complete teach lap followed by fixed-route repeat.
No vehicle, physics, camera calibration, motion gates or reference speed changes.
Writable scope expands to the neo lane packages/tests/handoff plus the existing isolated
Simulator adapter and its evidence. No real vehicle output, old repository, raw bag upload
or replacement of historical evidence is authorized.

## Input handoff

Planner acknowledges neo-004 at checkout 85ddb70209e6c346c422f9ae8b2a88cb49da72e0.
Read root AGENTS, AGENT_CONTEXT, STATUS, PROTOCOL and the referenced handoff.
Prior simulation trial13 failed after 6.497 m, not a completed lap. Its exact current
mask passes unchanged tracking gates against the actual previous active keyframe,
while a new candidate fails. Production replaced the active matcher before validating
that candidate. Baseline trace/PNG/motion evidence is retained locally, not rewritten.

## Source and working tree

Started clean on feat/causal-lane-teach-repeat at 85ddb702, origin the authorized
Sophia-AI-formula-team/neo_aiformula_sophia repository. Created dedicated branch
repair/lane-keyframe-transaction-20261011, not main. Annotated baseline tag
lane-mapping-20261011-baseline points at that exact pre-repair commit and explicitly
does not claim vehicle acceptance. Git identity matches the authenticated user.
Runtime source changes: mapping.py and shared mask_localization.py; tests, README and this
new session are task-owned. The tested source is retained by the repair commit/tag;
the checkout was dirty with these reviewed changes during offline validation.
Simulator is not a Git checkout; parameter/source
preservation is audited separately, retaining authorized peer-task UI/lamp differences.

## Work performed

- Baseline: python -B -m pytest workspace/src/aiformula/control/lane_mapping_fixed_gnss/test/test_mapping.py -q -p no:cacheprovider -> 17 passed.
- Prepared candidate reference separately from active reference; preserved committed
  tracking correction and time fences; candidate admission cannot reset tracking gates.
- Geometric candidate rejection permits at most the real current active reference;
  no history search, odometry-only fusion, current/future map evidence or gate relaxation.
- Final guard/deadline, one match commit, one map vote and trace append precede
  promotion. Discard uncommitted proposals on known rejection paths. Existing prepared
  index cache survives deadlines only while its source and committed tracking seed match.
- First focused regression: mapping plus shared mask localization -> 57 passed.
- New transaction tests initially had11 failures and10 passes: failure injection
  returned a fabricated rejected result without the real localizer's pending
  revocation. Corrected the test injection, not production admission;21 passed,
  then35 after more boundaries. Independent actual-diff review ran92 core checks.
- Review found one logging error: a rejected candidate transaction reported the
  old active frontier as its matching frontier. Fixed actual/active fields and
  preserved attempt metrics through capacity rejection. Seven more regression
  scenarios give42 passing transaction cases.
- First combined package pytest invocation lacked the colcon overlay/PYTHONPATH
  and failed collection on4 imports. Retried with the4 actual source-package
  directories on PYTHONPATH:647 passed,3 failed,1 skipped. The3 failures were
  old worker-test assertions assuming a first candidate had already become active.
  Strengthened them to require active/frontier None, inactive candidate uncommitted,
  pending revoked, and all original map/trace/vote invariants unchanged.
- Final full lane command: python -B -m pytest
  workspace/src/aiformula/control/lane_mapping_lya_reference/test
  workspace/src/aiformula/control/lane_mapping_fixed_gnss/test -q -p no:cacheprovider
  with PYTHONPATH containing lane_mapping_lya_reference, lane_mapping_fixed,
  lane_mapping_fixed_gnss and trajectory_follower source roots ->657 passed,
  1 skipped,16.31s,exit0. The only skip is the target-Linux process-group test
  on Windows. fixed has no separate test directory; its contracts are in the
  shared suite, not an invented third test run.
- Simulator Tools/LaneTeachRepeat unittest discovery ->115/115 passed,
  10.593s,exit0, using current repaired neo source and prior real-frame evidence.
- Portable evidence helper tests ->5/5 passed,exit0. Initial pack rejected Python
  source inside evidence (`unapproved file`). Kept the archive rule unchanged;
  moved the independently runnable source/tests to repository tools/ instead.
- tools/tests initially ->29 passed,3 failed,1 skipped while this session was
  still draft/unreviewed/unpackaged. Those are archive-readiness checks, not
  mapping failures. Seal/package first, then rerun without changing their rules.
- Unity is currently leased to the separate perception/complete-LYA task. No Unity,
  Assets edits or Refresh until explicit release; independent offline work only.

## Evidence and result

Result: partial against the user's complete goal; the diagnosed core defect is
repaired and independently reviewed. [Portable evidence](evidence/README.md)
contains the exact32 masks and566 sensor samples, baseline reproduction and
repair replay. Baseline accepts31/32; repaired source accepts32/32. At sequence52
the new candidate still fails the unchanged geometry gate, while the actual old
active passes strict tracking:92/92 inliers, RMSE0.050170574m, rank2. Its source
frontier6439999878 is strictly earlier than current stamp6659999873.
No future samples, input world poses, odometry reset or relaxed defaults are used.
Report03 includes the final logging fix and source hashes; earlier local outputs
are retained. Each included file was reviewed for privacy/secrets; fixture images
are ideal semantic masks, not personal photographs. No raw bag/GNSS/private path
or world-truth log is included. New physical trials and any further code repair
will use a new record, not edits to this sealed checkpoint. No ROS, hardware or
complete two-lap claim is made by these local tests.

## Risks and blockers

Full first-lap mapping and second-lap repeat still unverified. Also retain known
stopping-distance/start-attachment and dashed-lane coverage risks. Current physical
model remains provisional but must not be retuned to make this test pass. Candidate
transaction repair covers known rejection/cancellation boundaries, not arbitrary
MemoryError or rollback of all possible exceptions in map.update.

## Next agent actions

Commit/tag this reviewed core checkpoint as lane-mapping-20261011-r1; verify remote
repository ID1303517209 and exact push URL before publication. The package code is
on the repair branch/tag, not origin/main; do not imply main already contains it.
Run ROS Foxy CI for the exact commit and log the outcome separately. Receive Unity
lease and verify real state/parameters before a new non-overwriting trial.
Run new non-overwriting trials through TEACH, STOPPING, PREPARE, ACQUIRE, REPEAT,
COMPLETE. Any HOLD remains failed; save its reason and safely stop before editing.
Record their results/video in a new simulation session; keep this checkpoint immutable.

