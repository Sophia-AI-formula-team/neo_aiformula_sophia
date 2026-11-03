# Selected experimental snapshot — 2026-11-04

The user explicitly selected the latest experiment package and authorized merging
it first, with rollback if needed. This import preserves the supplied implementation,
including its known regressions; it is not a claim that existing acceptance gates pass.

## Source and preserved baseline

- Source: `1027simtest/perception-upload-20261102.tar.gz`, `source/` overlay (442 files),
  plus the four `simtest_speed/stop/turn/mixed` source packages (24 files).
- The archive's `lane_0927oa.py` equals the loose `lane_1027.py`; the older expanded
  perception copy is intentionally not used for that file.
- Pre-import main: `bef4e78415ce04e87351000dfed6a4a08fe9ea78`.
- Permanent rollback reference: `backup/pre-1027simtest-20261104` at that commit.
- Existing main-only files, model weights, launchers, motor 0.4.1 and both complete
  0.2.1/0.4.1 motor backups remain. PR #18's experiment archive is unchanged.
- No raw bags, database sidecars, caches, image captures or local backup copies
  are imported. No vehicle/Unity node was started or deployed by this task.

## Known differences deliberately retained

- The supplied semantic adapter is 0.2.3 and uses RAM learning. It no longer calls
  main's TopoState checkpoint/recovery and closure diagnostics. Their main-only
  source/test/history files are preserved; they are not automatically active in
  this selected adapter. Historical claims remain tied to their old commits.
- Legacy road_detector outputs keep timestamps but lose complete Header/frame_id
  copying. The retained main Header contract has 3 failures and 1 pass. It has not
  been weakened or deleted to make this experimental import appear green.
- `first_lap_wait_then_detour` is rejected by motor 0.4.1 when its semantic session
  is active. Existing default A3/allnodes/motor wiring remains unchanged. The
  imported RACE_MAP guide describes a different explicit remap path, not an
  automatically commissioned production chain.
- race_map.yaml uses an 89-degree closure heading while its guide still says
  25 degrees; other experimental closure/fusion thresholds also differ.
- The four simtest completion flags/terminal zero sends are commented out. The
  turn script's positive angular commands disagree with its negative/right-turn
  description. The source is retained without silently correcting the experiment.
- Runtime/helper source includes reviewed original whitespace and line endings.
  Exact staged source bytes match all 466 supplied files.

## Checks and rollback

The imported 228-test semantic subset passes. The known Header contract regression
is recorded separately. Syntax/source hashes and protected trees were checked;
no C++ build, native ROS/DDS, inference or physical acceptance is claimed. Full CI
may fail on these known compatibility differences and is not bypassed or rewritten.

If the experiment must be undone, create a rollback branch from current main and
use `git revert -m 1 <this PR's merge commit>` through a new PR. The named backup
reference identifies the exact prior baseline; do not reset/force-push main or
discard later unrelated commits. This task prepares rollback but does not invoke it.
