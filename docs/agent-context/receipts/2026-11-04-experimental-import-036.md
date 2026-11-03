# RESULT_READY — selected experimental snapshot

The user explicitly authorized merging this experimental version first and reverting
if needed after reviewing the known issues. Baseline: `bef4e78415ce04e87351000dfed6a4a08fe9ea78`;
rollback reference: `backup/pre-1027simtest-20261104`.

See [source, known differences and rollback](../sessions/2026-11-04-planner-experimental-import-036/evidence/selection_and_rollback.md)
and [reviewed session](../sessions/2026-11-04-planner-experimental-import-036/SESSION.md).
All 466 staged source files match the supplied archive/packages; no source fixes,
ROS-node startup or vehicle deployment are included. Motor 0.4.1, both 0.2.1/0.4.1
backups, latest launchers and PR #18 archive are unchanged.
The imported semantic subset passes 228 tests; 3 retained Header tests fail as
expected. CI failures remain visible and are not presented as acceptance.
