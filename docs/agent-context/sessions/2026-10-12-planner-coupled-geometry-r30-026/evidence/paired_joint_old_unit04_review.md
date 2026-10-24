# Post-execution readback: independent nominal joint units04

Separate append-only evidence note. The frozen mathematical REVIEW.md remains
SHA256 `51bb71ef3bc656501bb2f205335bfffe9e0fb82d03b6ca72aa398ba3f9600c2c`.
Its prepared/static wording is not rewritten or treated as an execution gate.
Root reported one actual bounded units04 execution after reading the source and
test. This child subsequently read/verified its records only; no test rerun,
Python import/AST/compile, solver, extractor/native payload, Unity or source edit.

## Actual receipts independently read and hashed

- units_run_04/result.json:
  `9ea5712d958e8243341394a6f1614497954f751513743f47d454eb0dc1cafa9e`.
- units_run_04/unit_result.json:
  `07e909c34e460bc4731d2eb7ef5fdc3c53c1e6ded7dd01a7a2b724cce01b01e7`.
- run_units_04.py (185 lines fully read statically):
  `d4e5ec0f6a9ffdf80646bee037c04e634c779a245070df04c2792b9af51d0898`.

Both JSON files were read in full with PowerShell UTF-8. Parent's linked child
digest matches the current child receipt. All four before/after binding maps
(parent-before, parent-after, child-before, child-after) agree: three complete
map comparisons against parent-before, five literal binding entries each.
All FIVE current source/driver files were independently hashed and match:

| Source | SHA256 |
| --- | --- |
| footprint_geometry.py | eb03d1ceb011c2419c4c8a25fb1cf8e40f7cb3c084d714d9d39b7a065e181d17 |
| paired_curve_fit.py | d367d48e4b0e9afbd5c6fe7e18f349211ff7906dcc74b0a8d4706c2ea05a8679 |
| paired_curve_joint.py | bd2b0edef2d34a252dc079cc35cc6eb594fada49264d763a683f21ec5bf259eb |
| test_paired_curve_joint.py | f9e1ee7974147f56438ff2299304e00a05bf0bd0ea22d5ffe5a4e60fba0c4366 |
| run_units_04.py | d4e5ec0f6a9ffdf80646bee037c04e634c779a245070df04c2792b9af51d0898 |

The 30 literal test method names were independently extracted using PowerShell
regex and compared with the complete actual ID list. All 30 exact IDs match,
with 30 unique executed IDs and no extra/missing method. testsRun=30,
failures=0, errors=0, skipped=0, wasSuccessful/evidence_valid=true. Thus the
records support 30/30 artificial nominal-checker checks, not only a counter
copied from root's message.

## Source-consumption and resource scope

The statically read driver checks source SHA before executing the SAME raw
bytes. It creates a fresh synthetic package shell with no production __init__,
then consumes the three actual modules under their ordinary names in dependency
order (footprint, fit, joint), followed by this portable test source. It checks
the module object/path/source marker and tests.joint identity after execution;
unexpected other package modules are prohibited. The actual receipt records
three production module sources executed, mapping/extraction imports=0,
custom_native_calls=0, map_writes=0 and unity_actions=0.

This confirms the tested helper source/dependencies in that isolated unit
process, not normal installed-package/__init__/colcon/Foxy/ROS loading. NumPy
library algebra is used by the checker, but no custom native extractor/fitter
is part of this unit scope. No actual provenance packets or observations enter
the artificial tests.

Parent receipt: failure=null, exit_code=0, retries=0, own_child_pid=30964,
own_child_stopped=true, primary_wait_s=30, single_stop_wait_s=5.
Parent elapsed 0.3669510000036098 s; child receipt elapsed
0.24336840002797544 s. These are recorded unit-process diagnostics, not runtime
mapping latency, kernel timing or a platform worst-case execution guarantee.
The driver uses an own-child-only stop_once guard marked before kill/wait, so
an exception path does not issue a second stop wait. Normal completion required
no timeout branch; live OS process state was not independently queried here.

## Remaining limits

No evidence inconsistency found. Actual unit evidence does not prove authentic
primary ownership/all-WB input identity, actual original22 coefficient effects,
physical lane/joint truth, generalization, runtime gates, ROS package build,
two laps, tracking/localization error, vehicle safety or a completed video.
The math predicates remain disabled in the inspected extraction path and all
original downstream gates remain required. No original evidence/source was
modified and the excluded prototype pair was not read, hashed, imported or
staged. Root owns any later actual-packet evaluation, integration, version
control, publishing and the separately scoped runtime/scene validation.
