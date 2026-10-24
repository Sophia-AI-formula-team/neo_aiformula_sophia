# R22 development checkpoint published — PARTIAL

Role: planner. Input/consumed handoff: `2026-09-28-planner-neo-004`.
Disposition: `RESULT_READY`, evidence accepted for analysis; not driving acceptance.

- Canonical repository ID read before push: `1303517209`.
- Commit: `baba879468b8674e7044dadebe2363baae378238`.
- Branch: `repair/lane-keyframe-transaction-20261011`.
- Annotated tag: `lane-mapping-20261012-r22-rc1`.
- Atomic branch/tag push completed successfully. No main/legacy/force push.
- Readable session: `sessions/2026-10-12-planner-corner-side-r22-019`.
- Bundle: `bundles/2026-10-12-planner-corner-side-r22-019.zip`, 44343 bytes,
  SHA256 `f8e175a53e4ae61723d595e69d95cebdc15b806609569f2169127951ee86affb`.

## Evidence limits

Final local full regression: 3908 passed, one Windows/Linux-process-group skip,
zero failures/errors. Source inventory unchanged across the run. The earlier
3907-pass/one-failure run is retained; only its inconsistent legacy test fixture
was corrected. Handoff-tool regression: 32 passed, one Windows symlink-privilege
skip, 18.248 s. Structural validation and packing succeeded.

Fixed offline 22-mask audit: normal errors over 30 degrees changed 8 to 3;
distance RMSE changed 8.2071 to 8.2154 mm, so distance accuracy did not improve.
Three remaining coarse-arc failures are unresolved.

Strict captured-input replay: 11 accepted of 12 attempted; frame 162 failed
processing deadline (100.9906 ms > unchanged 80 ms). Matcher 28.6062 ms,
overlap 0.807971; these do not make the whole engine pass. All 25 integrity
checks passed. No new Unity run, completed teach lap, frozen route, repeat lap,
tracking-ground-truth metric or new video. CI is pending this checkpoint;
no CI success is inferred from local testing or successful upload.

## Staging review

Explicit 26-file staging and complete source/test/document/privacy review were
performed. The unrelated quantized-curve-fit prototype pair was not staged.
The ordinary whitespace check flags pre-existing final blank lines in three
tested new test files and sealed local_tests.md, plus context-only lines in the
sealed simulator patch. No sealed evidence was rewritten to silence these.
`git -c core.whitespace=-blank-at-eof diff --cached --check -- .
':!docs/agent-context/sessions/2026-10-12-planner-corner-side-r22-019/evidence/simulator_v4_adapter.patch'`
exited zero before the commit. These are formatting exceptions, not ignored
test failures. No vehicle/simulation/LYA parameters or acceptance gates changed.

Next: source-identical finite-union acceleration and original-window geometry
consistency checks, followed by strict replay and only then fresh live validation.
