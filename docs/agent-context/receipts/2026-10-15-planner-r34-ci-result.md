# R34 terminal CI result —6/8, NOT accepted

Exact functional commit010c089e03d8952677f688656639e215b816984d,
tag lane-mapping-20261015-r34-rc1, canonical neo repository ID1303517209.

Both native Windows/Linux, teach-repeat Foxy and handoff runs passed. Both
Endpoint runs failed; no unchanged rerun and no gate/clock/scope/executor change.

- Tag38703128393:4812unit passes/1skip/1fixture setup error at frame16.
  Complete diagnostic is match_runtime_budget_exceeded:45.175277ms >40ms.
  Engine47.537878ms/pre-match2.314020ms. DDS NOT executed.
- Branch38703128321:4813unit passes/1skip, then DDS2/9passes. Seven cases fail
  bootstrap after14–16 processing-deadline rejects. Sample rejected Engine
  98.656..158.155ms >80ms; whole-worker312.166..385.434ms. Final latched-invalid
  result's0.095ms must NOT be substituted for those rejected-frame costs.

No CPU/thread/GC/backend/cgroup data establish the lower-level cause. This is
not diagnosed as host load/BLAS/GC and is not a reason to loosen gates. The new
explicit geometry mode remains default-off; its historical geometry improvement
does not establish default-runtime or full-foreground weighted-overlap success.

[Reviewed session034](../sessions/2026-10-15-planner-r34-ci-result-034/SESSION.md)
preserves exact run IDs, complete scalar diagnostics and raw-log hashes without
raw journals, epochs, hostnames, paths or GNSS coordinates. Two simulator laps,
stopped freeze, trajectory errors and video remain unperformed. No vehicle,
simulator or LYA parameter changed by this task.
