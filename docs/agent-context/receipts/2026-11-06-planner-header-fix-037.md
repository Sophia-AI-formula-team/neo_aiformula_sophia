# RESULT_READY — road_detector Header repair

- Date / role: 2026-11-06 JST, planner.
- Authorization: repair Header, manage the version and merge into main; organize
  local source already retained in Git. Exclude YOLOP and fixed-trajectory code.
- Current base: `94e1797faaf511ed4cb9593c9646508dae17491b`; branch:
  `fix/road-detector-header-20261106`. The historical input handoff is
  `2026-09-28-planner-neo-004`; current user scope and current base take precedence.
- [Reviewed session](../sessions/2026-11-06-planner-header-fix-037/SESSION.md),
  [checks](../sessions/2026-11-06-planner-header-fix-037/evidence/checks.json),
  [bundle](../bundles/2026-11-06-planner-header-fix-037.zip) and
  [checksum](../bundles/2026-11-06-planner-header-fix-037.zip.sha256).
  Bundle SHA256: `ba130f3d006831b56277d3212c14a0f773c5e589bc2ae115aa2356cd5a1a8672`.
- road_detector `0.0.0` → `0.0.1`: both image outputs independently retain the full
  input Header. Callback, image pixels and existing regression tests are preserved.
- Offline test command exits: before 1 (1 passed, 3 failed), after 0 (4 passed).
  Python 3.8 syntax, package-version consistency and diff whitespace checks pass.
- GitHub CI and merge are pending at preparation time; consult this branch's PR
  for their final outcomes. Local cleanup receipts are retained locally.
- No ROS/DDS, inference, target build, deployment or physical action was run.
  Other imported experiment issues remain. Hardware state was not inspected.
- Next: review/publish the scoped change, verify relevant CI and merge the exact
  reviewed head, then verify main before hash-based local cleanup.
