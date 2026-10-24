# Independent unit06, portable-test packaging and routing review

Status: static/read-only review plus actual unit06 receipt readback.
No Python, AST, imports, tests, solver, NPZ, native build or Unity was executed
by this reviewer. No R source/config/CI, prior evidence or root-session026 edit
was made. This is not ROS/Foxy CI execution, map adoption or scene/lap acceptance.

## Unit06 actual evidence

The complete unit06 driver and both complete actual receipts were read.
All30 actual unique test IDs match the30 statically declared methods in
test_paired_curve_joint.py. Their full recorded sequence also equals unit04's
original30-ID sequence, not just its count. There are zero failures/errors/skips,
testsRun30, wasSuccessful=true and evidence_valid=true.

Authenticated identities:

| Artifact | SHA256 |
| --- | --- |
| run_units_06.py | f34b0392a163226f1263e876449ac3e51945652231299cf060314e275a0365d9 |
| units_run_06/result.json | e486af68e551df0e566ff3287a8d9d30da3ec35514b08df14b8a14dc4889ac96 |
| units_run_06/unit_result.json | cf1dda34b444c8420473d901c01a0b5926e5f996498d8d6be5351f631c21d871 |
| footprint_geometry.py | eb03d1ceb011c2419c4c8a25fb1cf8e40f7cb3c084d714d9d39b7a065e181d17 |
| paired_curve_fit.py | 597703b0a408b462807df57ece9e17787f322a167632ea44558c1765b3daa47c |
| paired_curve_joint.py | bd2b0edef2d34a252dc079cc35cc6eb594fada49264d763a683f21ec5bf259eb |
| test_paired_curve_joint.py | f9e1ee7974147f56438ff2299304e00a05bf0bd0ea22d5ffe5a4e60fba0c4366 |

All five current file hashes match the parent-before map; parent-after and child
before/after maps equal it. The parent authenticates the child receipt digest.
Parent reports exit0, failure=null, source_unchanged=true,
unit_evidence_valid=true, retries0 and own child34712 stopped.
Observed parent0.3399054s / child receipt0.2064075s are diagnostics only.

The unit06 driver uses the same closed package and authenticated raw compile
scheme as unit04: no production __init__, mapping or extraction imports.
Its actual three source modules and test.joint object identities are checked.
Unexpected additional production-package imports are rejected. The exact whole
driver is the same as unit04 after replacing the fit pin, output-folder and
driver-binding names and disregarding one final blank line; no extra solver,
checker or stop path is introduced.

The parent has one30s wait and at most one scoped5s stop wait; stop_attempted is
set before kill. No retries or all-Python kill. This does not certify total
preflight/build/finalization wall time or a runtime deadline.

## Mathematical closure remains unchanged

The entire current joint145-line source was read again; bd2 is exactly the
same mathematical checker and f9e exactly the same independent tests already
reviewed in r30_runtime_joint_review_01/REVIEW.md. The closure changed only from
fitd367 to fit597. Exact text comparison of the fit prefix through PrimaryPatch
and _array (before _chart_exact) passes: these are the only fit definitions
imported by the joint source. The footprint interval source remains eb03.

No generator-only _definite_linear_violations call enters joint certification.
The joint checker still verifies full outward numerical box membership in
original reciprocal-axis P and both original finite charts, certifies saved
binary64 preconditioner nonsingularity, and requires the Krawczyk image strictly
inside inward bounds of the original exact center-plus-radius box.
Small residual, a nominal shared floating q, tangent/root coincidence or an
uncertain denominator does not produce yes. Failure remains unknown, not no.

The earlier mathematical limitations remain: nominal root existence, not
uniqueness/topology/physical truth; authenticated caller ownership/support and
old angle/J/root gates are separate; ordinary IEEE binary64 without FTZ or
unsafe fast-math is assumed. Unit06 is artificial one-box-checker evidence,
not actual captured-packet checker replacement, runtime route or driving proof.
It does not replace actual04's separate exact D joint evidence.

## Three new portable test sources

| Source | TestCase class | Static methods |
| --- | --- | ---: |
| test/test_paired_curve_fit.py | PairedCurveProposalTests | 58 |
| test/test_paired_curve_roundoff.py | PairedCurveRoundoffTests | 11 |
| test/test_paired_curve_joint.py | PairedCurveJointTests | 30 |

All99 names match pytest's default test_*.py discovery and unittest TestCase
collection. No skip decorator, xfail mechanism, D/private absolute path,
Windows staging path or pinned offline-source import is required by these
three files. They use ordinary package imports. Fit and joint tests add their
source package root via their own __file__ relative parents; roundoff relies on
the normally available source/installed package, as in the authenticated unit05
driver and a sourced colcon workspace.

The complete current new11 source and its portable imports were inspected in
the preceding actual04 review, and the unchanged58/30 sources were already
fully inspected. Current source hashes and regex method counts were checked
again. The new roundoff module's special registry name in the diagnostic
driver is not a pytest discovery requirement; standard pytest can collect the
same class under its normal module name.

Manual source/syntax inspection plus targeted patterns found no structural
Python3.9/3.10-only syntax, built-in generic annotation dependency, match/case,
math.nextafter call, dataclass slots, str.removeprefix/removesuffix, or new
NumPy strict-testing argument in these three tests. Helpers use NumPy nextafter,
not the post-Python3.8 math API. This is a static Python3.8 compatibility finding,
not compilation/import/collection under an actual3.8 interpreter.

New nonstdlib numerical dependency is NumPy; dataclasses, Fraction, unittest,
mock, inspect, ast, pathlib and itertools are standard Python3.8 modules.
No new ROS, native, plotting, SciPy or offline D dependency is introduced by the
three tests or the two helper modules.

## Setup and ROS/Foxy CI collection path

Read in full:

- setup.py:
  5c629a6b0755842eb6a95e22bef5cb585ac2c7c5959ab216471b75e465bd6b9e
- setup.cfg:
  aae0d0f4b42eee7f658a57a4fa4b2fee09aa97a716332384197416ddada5a782
- package.xml:
  ee671163f9e0a1b07b1ef80e76db737fb2b2d1e827f1f56db25c2286b457efac
- lane_mapping_lya_reference/__init__.py:
  29bd7278966cfae838211b18769e8349e49c3a3518c71939ba8cba79b4256d5c
- scripts/ci_verify.sh:
  526c816fc1aeb7a3bc418811231150c2041a1dad3ade0c41c824b240e4c2affd
- lane_mapping_fixed_gnss/scripts/ci_verify.sh:
  a9b401008451b250deefe3e929782b731b2e005f615056f35f72dccd43a67688

setup.py uses find_packages(exclude=["test"]); helper .py modules inside the
package are included normally. Tests remain source-checkout tests, not installed
runtime data. setup.cfg has [tool:pytest] testpaths=test and no filter excluding
the three new filenames. setup.py tests_require=["pytest"] and package.xml
test_depend python3-pytest are present. package.xml already declares
exec_depend python3-numpy.

Both lane-teach-repeat.yml and lane-endpoint-gnss.yml sparse-check out the entire
reference package, install python3-numpy/python3-pytest and native build tools in
the disposable ROS Foxy Focal container, build this ament_python package and then
colcon test the whole reference package. These paths structurally include all
three new tests once those files are present in the selected published revision.
No new setup test registration, test glob or dependency edit appears necessary
for normal source-based ROS CI collection.

Workflow identities:

- lane-teach-repeat.yml:
  42ccaf07131599cb856397852d45d0835f20c78ffb8292aa6fbe9e6684aafa66
- lane-endpoint-gnss.yml:
  8ef2a6cc2e95c2902d5b5341edbaf8056ba9328b176f0e22a9efc5443b0338cc
- lane-native.yml:
  30dcd7a4bc02adad70aa849125bfbf03319825fddc4ad39cf1e6d51b9c29afaa

The native-only workflow explicitly selects11 native test filenames and does
not run these three new portable tests. Its Python3.13 job must not be called
Foxy/Python3.8 coverage of them. Canonical repository guards remain
Sophia-AI-formula-team/neo_aiformula_sophia / repository ID1303517209.

The Foxy scripts' existing aggregate minimum JUnit counts (325 and552) reject
zero-test success but do not individually prove these99 new methods executed.
Future real CI readback should verify their named result IDs; an explicit
three-file pytest/JUnit evidence step is a possible scoped enhancement, but was
not edited or executed here. A green old aggregate floor alone is insufficient
for the new99 claim.

ROS dependency provisioning is via package.xml and explicit CI apt installation.
setup.py install_requires lists only setuptools, so standalone pip installation
without ROS/rosdep dependency provisioning must not be advertised as NumPy
dependency-complete. This is existing packaging behavior, not a new helper
regression and not a blocker for the inspected ROS CI installation path.

## Local excluded-file collection boundary

Filename-only guards confirm both protected experimental excluded filenames
are present in this dirty local checkout. Neither contents nor hashes were read:

- lane_mapping_lya_reference/quantized_curve_fit.py
- test/test_quantized_curve_fit.py

Default whole testpaths discovery in this local tree would include the excluded
test filename. Therefore unrestricted local pytest/colcon test collection is
not authorized under the current exclusion boundary. The separate111-file
regression harness must maintain a literal reviewed allowlist and exclude it
before reads, imports, collection or hash operations. This is a scope blocker
for unqualified local whole-directory collection, not a failure of the new
three tests.

This reviewer did not check the Git tracked/publication state of excluded files.
A canonical GitHub checkout that omits them will not contain them, but that
selection must be verified by the publishing agent; do not assume the dirty
local working tree equals the cloud checkout. No excluded file was staged,
removed or changed by this review.

## Main route remains inactive

The package __init__.py is a docstring only. Setup console entry points remain
lap_recorder, reference_follower and teacher_supervisor; none targets a helper.
A recursive source-reference search in the main package explicitly excluded
both protected filenames. paired_curve references occur only in the two helper
definitions and the joint-to-fit import. No ordinary main-package import/call
to propose_pair or certify_pair was found outside those files.

This agrees with the source docstrings' not-enabled contract. Package inclusion
or test imports must not be mislabeled runtime activation. No current
extractor/recorder/follower route, map, car, simulator parameter or runtime gate
was enabled/modified/tested here.

## Readback limitations and disposition

One initially overstrict whole-driver comparison failed because unit04 has one
extra EOF blank line; comparison limited to the declared fit pin/IO identity
changes plus final whitespace then passed. A broad combined rg output was
truncated; the selected scripts were reread completely and compact targeted
syntax/method checks were repeated. These are readback issues, not test failures
or batch reruns. The receipt validation used explicitly parenthesized equality
checks for the old/new30-ID sequence.

Disposition: unit06 receipt identity and unchanged mathematical closure pass
this independent static/readback audit. The three portable tests are structurally
eligible for normal ROS CI discovery; actual Foxy/3.8 compilation, colcon build/
collection/results and the full allowlisted regression remain unverified by
this worker. The local excluded-file boundary and named-test CI readback
requirements must remain explicit. No runtime/WCET/scene/two-lap/video claim.

