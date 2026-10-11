# R23 RC1 native build result and test-path correction

Planner; input handoff neo-004. RESULT_READY, partial; no driving acceptance.
Exact code: a327a1ba50451fd6319065c87a936a07f3e5b105.
Tag lane-mapping-20261012-r23-rc1, annotated object
3066ad429dc56fd4c4bc2074db774f1f4a40f239, remote peel matches code.
Canonical repository ID1303517209; branch/tag atomic push succeeded.

Session020 ZIP is16870bytes, SHA
be8e42c32d83acb481eb83a2f385a8601cdd2d5fde7314e52e7c7dfd1384c29f.
One initial metadata validation rejected missing exact required section names;
headers were corrected BEFORE sealing, then validation/pack passed.
Staged review covered all15 files. Sole whitespace exception was the sealed
SESSION.md final blank line; check with -blank-at-eof disabled exited0.

## Native CI failure, retained

Exact-SHA automatic runs38156827382 and38156827393, both Windows/Ubuntu:
compilation succeeded and verify_native genuinely exercised all nine APIs,
including the exact SOURCE_SHA256
9dae4a7ab517d872af29d3c2d0a27d1cdfa7e8446c8f138be206df5ba4c4438f.
Tests reported645 passed,1 failed,954 subtests passed. No rerun was requested.
Workflow failure is NOT called success. Binary copying occurs after pytest,
so a metadata/JUnit artifact alone is not an installable binary.

The failure is test_native_finite_projection.py:
test_capacity_rejection_has_same_message_and_preserves_previous_state.
Exception type/message and unchanged-state assertions passed; its final API8
spy expected calls, but native9 handled the union and bypassed that API8 entry.
The production failure/transaction behavior was not relaxed.

## Correction

Isolate the entire API8 projection test module with _NATIVE_CURRENT_UNION=None
so its independent Python-vs-API8 whole-state comparisons actually exercise
their intended paths. Retain every old assertion. The separate native9 module
continues to require actual ninth calls and tests whole-state/export/capacity.
No numerical source, gate, time budget, compiler flag or physics change.
Local isolated projection module: python -B -m pytest [package test module]
-q -p no:cacheprovider with LANE_REQUIRE_NATIVE=1, exit0,64 passed in0.19s.
This used the unchanged installed native8, not native9; no performance claim.
Publish a NEW RC2 tag; never move the failed RC1 tag. CI result remains pending
until a fresh exact-commit run. Raw logs and failed artifacts remain retained.
