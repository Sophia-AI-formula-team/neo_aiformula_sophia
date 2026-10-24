# Planner sub-review: runtime nominal paired-curve joint predicate

Unique child record for /root/capture04_review; parent planner owns session026,
integration, handoff validation/packaging, tags and publication. This child does
not edit the parent's single-writer session or execute the handoff tool.

Acknowledged incoming root task, AGENTS.md, AGENT_CONTEXT.md, complete STATUS.md,
PROTOCOL.md and active handoff 2026-09-28-planner-neo-004. Live checkout inspected:
repair/lane-keyframe-transaction-20261011, HEAD
9b8636fc3c1aa9e9712142b3924a722211df19bd. Historical handoff vehicle baseline is
not adopted as permission for this unexecuted development source.

Reviewed source: paired_curve_joint.py bd2b0edef2d34a252dc079cc35cc6eb594fada49264d763a683f21ec5bf259eb;
Initially read PrimaryPatch/_array dependency paired_curve_fit.py c315dc102585265c413359fef1761528a06f39c55e74aa92b1ed45a74d10736c;
footprint_geometry.py eb03d1ceb011c2419c4c8a25fb1cf8e40f7cb3c084d714d9d39b7a065e181d17.
Each entire file was read with PowerShell UTF-8 and its SHA256 matched.

Initial state: static mathematical review and portable test preparation pending.
No main-module edits, execution, imports/AST/solver/tests/native/Unity/encoding,
vehicle/simulator/LYA parameter changes or source/admission enablement. Existing
dirty and excluded prototype files remain untouched; their filenames only were
visible in Git inventory.

## Final static review and current source binding

Prepared and frozen, NOT executed. No mathematical false-positive defect was
identified under the module's ordinary IEEE binary64/no-FTZ/no-unsafe-fast-math
precondition and the authenticated-original-chart caller contract. This is not
approval to adopt coefficients, enable extraction, expand a domain or drive.

Current exact files read completely with PowerShell UTF-8:

- paired_curve_joint.py SHA256
  `bd2b0edef2d34a252dc079cc35cc6eb594fada49264d763a683f21ec5bf259eb`,
  145 lines; unchanged from the incoming root task.
- footprint_geometry.py SHA256
  `eb03d1ceb011c2419c4c8a25fb1cf8e40f7cb3c084d714d9d39b7a065e181d17`,
  316 lines; existing outward primitive source unchanged.
- CURRENT paired_curve_fit.py SHA256
  `d367d48e4b0e9afbd5c6fe7e18f349211ff7906dcc74b0a8d4706c2ea05a8679`,
  330 lines; entirely reread after root's compatibility repair.
- New portable test/test_paired_curve_joint.py SHA256
  `f9e1ee7974147f56438ff2299304e00a05bf0bd0ea22d5ffe5a4e60fba0c4366`,
  21,022 bytes / 459 lines / STATIC 30 test methods, not 30 passes.

The reviewed incoming fit version used math.nextafter, unavailable in Foxy's
default Python3.8. This would have turned proposal execution into AttributeError
and UNKNOWN in that environment; joint's own entry point did not call it.
The reviewer promptly reported it without modifying the dependency. Root changed
it to float(np.nextafter(...)) using the existing NumPy dependency, resulting in
the current d367 source. No Foxy build/run or Python3.8 execution occurred here.
The unchanged portable test uses NumPy nextafter and math.ldexp, not math.nextafter
or newer Python-only syntax. Any later unit manifest must bind d367, not c315.

## Why an approximate floating inverse does not create a false proof

Write x0 for the supplied exact binary64 center, r>0 for the prescribed exact
binary64 radius, and X=x0+[-r,r]^2 in exact real arithmetic. The two supplied
binary64 polynomials define a continuously differentiable F on this box because
each copied chart scale is finite and strictly positive.

The NumPy inverse generates only a candidate finite binary64 matrix C. There is
no assumption that this C equals the inverse of an exact or interval Jacobian.
Its determinant is independently enclosed with outward primitive operations;
the interval must be finite and exclude zero, so the exact real matrix of the
saved binary64 entries is nonsingular. If it cannot be certified, return UNKNOWN.

For T(x)=x-CF(x), the segment from x0 to x lies in X. Each component's mean
derivative is contained in the outward Jacobian enclosure J evaluated over an
OUTER enclosure of X. Thus

`T(X) subset x0 - C F(x0) + (I - C J) [-r,r]^2 = K`.

All arithmetic producing the interval F(x0), C*F, I-C*J, correction and K is
outward. Strict K containment in an INWARD enclosure of X implies T maps the
original compact convex X into itself. Continuity gives a fixed point in X;
C nonsingular makes C*F(x)=0 imply F(x)=0. This is an existence proof, not a
uniqueness theorem, a physical lane truth claim or a global no/feasibility test.
The independent test deliberately supplies C=0.875 I to prove the API does not
silently require an exact inverse; its expected result is checked with exact
rational K algebra. Another test supplies singular C and requires UNKNOWN.

## Boxes, domains, derivatives and rounding

Lines 83-88 of joint distinguish three objects without changing r:

- ORIGINAL X uses exact input center +/- exact input radius.
- OUTER uses outward addition and contains X. It is only a numerical enclosure.
- INNER uses nextafter(center-radius,+inf) and nextafter(center+radius,-inf),
  so its closed coordinates lie inside X. Empty/reversed interior is UNKNOWN.

No rounded-outward X is substituted as the actual existence box. The correction
uses ORIGINAL [-r,r], not the numerically wider outer offsets. F(x0) is evaluated
at the original center point; J is evaluated on OUTER, which also contains all
segments needed by the mean-derivative argument. Strict image inclusion is
tested inside INNER, so rounding cannot certify a root merely on the boundary
or just outside X. No residual epsilon, q snapping or post hoc radius search
is present.

The reciprocal original-axis coordinates u/v are evaluated over the ENTIRE
OUTER box. Determinant intervals including zero are rejected before division;
positive and negative sign-certified intervals both work, including reflected
axes. Finite ordered u/v intervals must be contained in original [-1/2,1/2].
This retains original P; it does not construct a hull, normalize axes, repair
a bad domain or clip away a failing part of the box.

Each original chart x is evaluated over ENTIRE OUTER and must fit its unchanged
closed finite bounds. Scale is a constant copied original positive binary64
scalar, not a spatially varying denominator. Exact domain contact may conservatively
fail because OUTER is larger than X; the algorithm never grows a chart to pass.
All original finite charts/normals/scales are copied without normalization.

The value model is y-a-b*x-c*x*x in the saved original chart. Its gradient is
the saved normal minus the chart derivative times the saved tangent/scale.
The inspected formula has the correct scale and both x/y signs. Multiplying
an interval x by itself can be wider than a dedicated squared interval when
it straddles zero, but it still contains all exact x^2 values; this is a possible
false-UNKNOWN cost, not a false-positive omission.

## Singularities, large values, subnormals and schema

The inherited _point/_add/_sub/_mul helpers interpret copied binary64 scalars
as exact data and expand rounded arithmetic endpoints by nextafter. Division
by a strictly positive exact scale is monotone; reciprocal-axis division uses
all four endpoint quotients after sign certification. With ordinary binary64
rounding and no FTZ, one outward adjacent step contains an underflowed rounded
operation too. An overflow/NaN/reversed enclosure cannot pass the finite gates.

Near-singular true axes/J/C can legitimately be UNKNOWN even if a true joint
exists. C's determinant overflow/underflow or ambiguous interval sign is not
replaced by a residual test. NumPy inverse errors/nonfinite entries are UNKNOWN.
No assertion that exact J is nonsingular everywhere is required for the
existence proof; the candidate C's certified nonsingularity is sufficient.

Latest _array requires exact requested shape and finite binary64-compatible
real numerical data, rejects bool/complex/object/string and floating dtypes
wider than binary64, and rejects integer conversions that would round values.
It copies data. PrimaryPatch.from_arrays validates origin/normal/bounds lengths,
scalar scale, nonzero normal, positive scale and ordered bounds, with no owner
authentication or label/source substitution. Joint does not accept a scalar
radius array with shape(1,) or broadcast a malformed coefficient shape.

Memory/process-resource failures are not a global mathematical NO. The caller
must still own a bounded external execution/timing guard. Ordinary IEEE/no-FTZ
is an explicitly required platform precondition, not something these NumPy
calls certify for every target machine. Subnormal oracle fixtures are useful
checks but are not a universal hardware/rounding-mode audit.

## Independent portable test source (prepared only)

The tests import the ordinary neo package API and require no private D path,
existing solver/test helpers, ROS/native extractor, actual observations or
vehicle parameters. The portable source intentionally does not hard-code an
immutable production-source digest into normal package regression. Root's
planned fresh bounded unit driver must separately pin raw consumed source bytes,
module identities, dependencies and test source before/after. Such a synthetic
module-loading check is not installed-package/colcon/Foxy/ROS runtime acceptance.

Own ManualPatch data supplies fixed synthetic charts. Own Fraction saved-world
polynomial supplies values. Each exact gradient is computed by symmetric
central difference with unit step; for a quadratic this is exact, so no copy of
the module's analytic gradient is used as an expected value. The derivative is
affine, so its exact extrema on the ORIGINAL box are independently obtained
at the four exact corners. Own rational interval addition/subtraction/product
then constructs K using the REPORTED saved C, checked for exact nonsingularity.

EVERY returned YES in the tests is checked for all of the following:

- Reported OUTER contains the exact original box; INNER lies within it.
- The reported floating K interval contains the independent exact original-box
  K interval, and both are strictly within the original box/INNER as appropriate.
- Exact det(C) lies in the reported sign-certified nonzero enclosure.
- All four OUTER corners satisfy independent reciprocal original P inequalities
  and both independent original finite-chart bounds. Because these coordinate
  maps are affine, corner bounds cover the whole box.

Thirty STATIC test methods cover literal origin/translated affine roots,
nonzero center residuals with an existing joint, one curved rational-root pair,
two curved models checked with the exact K oracle, tiny inconsistent parallel
lines, tangency/coincidence, a wrong/root-boundary box, moderate/extreme
near-parallel models, tiny well-conditioned normals, subnormal inverse/radius,
huge finite translated roots, determinant overflow/preconditioner underflow,
whole-box P/chart clipping prohibition, uncertain reciprocal denominator sign,
reflected/skew axes, nonunit normals/unequal scales, exact scalar/array shapes,
invalid/nonreal/nonfinite/integer-loss input, independent point F/gradient
interval enclosure, singular/arbitrary nonsingular C, signedzero/readonly/
noncontiguous input immutability, and exchanged side order.

Scoped inverse spies only inject a candidate C and restore it afterward. They
do not supply the test's expected F/J/K values. Tests exercise module _value/
_gradient for enclosure comparisons, but their expected values come solely
from own direct polynomial/Fraction central differences. No tests were executed
by this reviewer, no actual new unit receipt exists in this child record, and
synthetic test count is not a pass count.

## Integration/authority limits and next handoff

Static rg found no paired_curve_joint/certify_pair enablement in the inspected
paint_geometry.py or paired_curve_fit.py. No production observation generator
was enabled and no support/model/gate/radius/parameter was changed by this child.
Only one new portable test source and this private child review record were
written with apply_patch; unrelated dirty/root-owned source remains untouched.

PrimaryPatch is numerical copying, not authentic producer ownership. A later
caller must bind these fields and coefficient packet to the SAME actual primary
support, require all original white/black labels, and retain original root,
angle, J, source/cut/finite-domain and runtime gates. This nominal existence
predicate cannot supply those missing contracts, choose a physical branch,
recover UNKNOWN by declaring NO, or authorize map adoption on its own.

Root is to read this complete record, freeze the d367/bd2/eb03/test source pins,
run at most its separately authorized bounded unit batch, and record actual
counts/failures/skip/source identities in a NEW execution receipt. A passing
synthetic batch would still leave real original22 consumption/parity, runtime
performance, causal ROS/Unity integration, first-lap mapping/second-lap fixed
trajectory, logs/physical error metrics/video and neo tag/publication work.
The child did not run handoff validation/packaging/commit/push; parent planner
owns consolidation into session026 and all subsequent authority checks.
