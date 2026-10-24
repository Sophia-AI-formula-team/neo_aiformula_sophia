# semantic_local_planner_ros changelog

## 0.3.1 — 2026-10-15

- Fix checkpoint rejection of legitimate negative, unwrapped route progress at
  a closed lap's start seam. Keep finite-number validation, nonnegative recorded
  distance and nonnegative progress before closure.
- Preserve geometry, closure/fusion thresholds, official map, command/traffic
  behavior and checkpoint schema 1. Do not normalize or clamp saved progress.
- Add disk-format and production-callback regressions for signed seam save and
  restart. 118 targeted methods passed with documented Windows test boundaries.
- Preserve the original genuine Unity failure and source-specific receipts in
  [the issue archive](docs/checkpoint_signed_progress_20261015/README.md).

## 0.3.0 — 2026-10-13

Added closure diagnostics, checkpoint persistence and continuous-odom semantic
restart recovery. The 0.3.1 entry fixes a persistence contract defect in this
release; it does not retroactively change the original validation claims.
