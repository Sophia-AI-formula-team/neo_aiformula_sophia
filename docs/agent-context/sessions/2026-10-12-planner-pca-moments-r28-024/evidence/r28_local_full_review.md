# R28 local full-regression read-only review

Result: **4,279 passed, 1 Linux-only skipped**, 0 failures/errors; exit 0.
This is offline Windows regression, not runtime, ROS/DDS, full-lap or deployment acceptance.

## Independent checks

- Parsed all 4,280 JUnit testcase elements, including 29 production local-moments tests and 8 pure-mock smoke-diagnostics tests.
- Only skip: `test_supervisor.test_owned_process_shutdown_leaves_unrelated_process_alive`, reason “Target Linux process-group test”.
- All 107 explicit pytest file arguments exactly match the approved ordered allowlist. No test-directory invocation or excluded user prototype is included.
- Recorded 196-file source hash map exactly matches approval; before/after snapshot bytes match SHA-256 `44e4e07f20f113d24a05643342591bbb796ff85f0869592c50b15e1f484586fd`.
- All nine recorded output hashes independently matched files. Both stderr files are empty; summary exception is null; no retry was recorded.
- Native nine API versions are 1 and real calls passed. C `9dae4a7ab517d872af29d3c2d0a27d1cdfa7e8446c8f138be206df5ba4c4438f`; binary `f2f9712754f8cecaae379af5355c8aa82b3a28855fa2da2d0ce949ddf339ef45`.
- **`installed_prefix_verified=false`** in this particular native invocation. It must not be relabeled installed-prefix or Foxy validation.

Recorded HEAD: `e43792a9ce119384d7a6bcfaa502b8980df65f06`.
Approval: `b351224c1e666e3b7ba04045baa260f2456fe3685c5e28728f1530bc9aaaf545`.
Summary: `ff3d0cf7291cd07308852fb4760670b90a9e1b210dc2e29e1d9ba20f3ff3b688`.
JUnit: `768cb10642ef2eb474d4b2c1b6fef7de79bb63381a13aaca4747212bc46ca6d0`.

Timing scopes: pytest child 99.4351229 s; JUnit suite 97.823 s; pytest text 97.84 s.
Parent separately reported outer wall 99.982971 s. These are test durations, not runtime frame latency.

## Evidence boundary and publication

The new eight smoke checks use mocks; they do not repair or supersede R26's failed Endpoint DDS checkpoint by themselves.
The rejected closure holdout remains rejected (9/12 normal cases meet 90% coverage); the native PCA reuse retained original geometry.
Neither observation equivalence nor this full regression proves 40/80 ms engine timing, the full 200 ms boundary, a first lap, a repeat lap or a physical emergency stop.

Only new D reports were written. No sealed output, repository, session or STATUS was changed.
Original JUnit contains a machine hostname; original manifests/summary contain local absolute paths.
Use the sanitized source manifest and review below, not those originals unchanged, for a public evidence packet.
Full output digests are in `REVIEW_01.json`. No Python, model/test execution, encoding or Unity action was performed for this review.
