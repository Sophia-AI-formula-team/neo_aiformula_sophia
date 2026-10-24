# R23 native CI follow-up; failed runs preserved

Planner, neo-004; no driving acceptance.
R23 RC2 published code65870e9c6a0251b41207f28d62aafbc6b31ae839,
tag lane-mapping-20261012-r23-rc2; changed only API8 test isolation and receipt.

Later RC1 Foxy logs exposed a second old white-box assertion:
test_finite_union_hotloop::test_first_seen_projection_order_and_signed_zero_bits.
The R19 Python oracle had four Python projection calls while native9 had zero,
because the C kernel now owns them. Other output/state comparisons passed.
All four RC1 Foxy workflows built and verified installed native9, then stopped
before DDS on two reference-test failures (3134 passed/2 failed; GNSS932 passed).
The two failures are this observer and the API8 observer already corrected.
RC1 final2/8 workflows successful,6/8 failed; no cancelled/retried run.
Native CI new tests:120 current-union cases +39 dispatcher cases all passed,
zero skips. They include source-only and real-native tests; this is not a claim
that all159 individually called native9. Four artifacts contained metadata/JUnit,
no copied binaries because the test step failed.

Additional correction: target only this Python invocation-order assertion at
_current_union_python. Retain exact four calls and first signed-zero assertions.
All other R19 black-box/state comparisons keep the default native9 dispatcher.
New native9 tests independently test output bytes, first-seen signed-zero graphs,
actual dispatch, state and failures. No production source/gate/budget change.
Local old-native8 run of both corrected old modules:87 passed in0.27s,exit0;
LANE_REQUIRE_NATIVE=1. New-native full CI must still independently pass.
Publish a fresh RC3, preserving RC1/RC2 history. CI remains separate evidence.
