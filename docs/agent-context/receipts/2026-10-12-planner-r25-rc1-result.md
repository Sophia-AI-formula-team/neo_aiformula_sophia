# R25 final CI result — PARTIAL

Planner; input/consumed handoff `2026-09-28-planner-neo-004`.
Exact source `40852db8392f3b66a6f37c92d1b2d649df8adb1d` in canonical
repository ID1303517209. Branch/tag initially resolved to this same source.
Final result:7/8 workflows,15/16 jobs succeeded; not a stable driving release.

| Workflow | Branch run | Tag run |
| --- | --- | --- |
| Handoff | 38161466721 success | 38161466772 success |
| Native | 38161466769 success | 38161466867 success |
| Lane Foxy | 38161466895 success | 38161466715 success |
| GNSS Foxy | 38161466741 failure | 38161466798 success |

Both GNSS jobs built5 packages, reported4153 tests/0 errors/0 failures/0 skips,
and passed4 Header tests. The branch failed during synthetic DDS, not unit tests.
Seven of9 scenarios timed out waiting for five actual stationary camera commits:
normal, no_gnss_build, future_gps, missing_bits, uncertainty, future_wheel and
future_gyro. All121 checks actually reached passed, but later checks were not
reached: this must not be called121/121 runtime success. Seven timeout snapshots
show TEACH, teacher command selected, visual_streak0; they are not HOLD proof.
The same-source tag DDS run passed all9 scenarios and151 checks.

Branch command logs contain221 literal processing_deadline occurrences across
failing cases, tag logs14. These are text occurrences, not deduplicated frames
or a proved cause. Branch reference/GNSS unit durations109.62/39.05s versus
tag59.54/20.14s also differ. Timing variability remains unresolved; do not
declare infrastructure-only failure or choose the successful run as acceptance.

Both Lane jobs passed3221 tests,4 Header tests,63 DDS and26 neo-upstream checks.
Each of four standalone native jobs passed646 top-level tests plus954 subtests
and exercised all9 native APIs. Counts are kept separate.

Exact GNSS artifact ZIP digests matched GitHub metadata, read only in memory:

- Branch artifact11709959104:
  `c58750cd6cf0ea768f2ed17b313d84885d129e497c781e5d23cfb6a537958ad6`.
- Tag artifact11710068413:
  `fa1dd6b2e75bdfa0c4d7226ccbfcb23c6424cc41edddcc9477c3f0661cc9a677`.
- Local detailed final snapshot SHA256:
  `30107eebe7008f6ac952cc1a0e5072c350976881fc059ddc2d1c27ea51225601`.

The detailed snapshot hash was checked from the actual saved file. Full
downloaded diagnostic data remain local; no raw inputs or private paths are
published. No workflow was rerun, cancelled or dispatched for a better result.
No binary was installed from these CI runs. Physical stops, real-speed input,
Unity first/second laps and video remain unverified.
