# Actual first-match readback, capture02

Scope: returned arrays and receipts only; not a two-lap, scene, vehicle, trajectory,
GT, WCET, or geometry-adoption result. No thresholds, samples, runtime core,
car/simulator parameters, or old artifacts were changed by this work.

## Actual provenance and one-shot execution

The first capture01 launch failed before output creation, child start, core import,
or replay because its manifest decoder used the invalid codec `utf8-sig`.
The source and approval01 were retained unchanged. Capture02 corrected that codec
to `utf-8-sig`, plus literal driver/output/manifest/approval suffixes and trailing
blank lines; its other 22 source pins and policy were unchanged. This is a
preflight harness failure, not an estimator failure. The short approval02 review
was completed while the root launched it, not an invented pre-execution gate.

Actual capture02 completed once, exit0, failure null, owned child17972 stopped
normally, no stop/retry. Its 23 before/after bindings equal approved02. The four
original method identities and one loader call were checked. Hooks were inside
the original relevant engine/matcher clocks, with no instrumentation subtraction;
copy/archive output occurred after the first engine return. JSON/NPZ final names
use separate exclusive temporary files and rename, not one whole-run transaction;
append JSONL is explicitly non-atomic. This normal run does not establish WCET.

One subsequently authorized lightweight offline readback was executed with `-B`,
exit0, tool wall0.3520259s and internal diagnostic wall0.03279919998021796s.
It imported no runtime core, custom native graph, replay, solver, or GT and did not
recompute an H/optimizer step. Input and driver SHA256 values were unchanged.

- Readback source: `readback_01.py`, SHA256
  `674798cb1e081efd706641821b2ccfedc8a5cb6635127ca4f9dac0cf129f32e2`.
- Result: `run_01/analysis.json`, SHA256
  `31cf4b99bbcc6be0fa8ad79fd8d31a9c223dcb8203bfd3d09b07a8bbf2a0bbfb`.
- Capture parent: SHA256
  `67f761e36dafc60085ed0d324d4202845d58a2fe877f285f324cc9272542a909`.
- Captured NPZ: SHA256
  `a3115cec8e31195e34c00b010ad08011902e7ca67c18150528ccda57215a52de`.

## All107 descriptors and actual returned arrays

All107 unique archive entries have the declared dtype, shape, byte count and
uncompressed C-byte SHA256; no object array or additional entry was accepted.
Four actual query/correspondence groups each retained all84 prepared rows and77
selected rows. Query world points, selected indices, normals and directly
reconstructed signed residuals were bitwise equal to their returned counterparts.
The first two groups are identical; no unrecorded stage/basis was inferred.

Seq162 uses mask18779999603, strictly past reference18669999605 and motion18759999603.
Its sole candidate attempt was accepted; the reported selected frontier matches
the capture frontier. This does not supply a general multi-attempt object identity.
2778 reliable projected inputs,2452 engine-eligible points,111 prepared foreground
rows and84 boundary representatives were preserved rather than display-cropped.

## Actual numbers, preserving the complete denominator

| Quantity | First returned group | Final returned group |
|---|---:|---:|
| Signed-residual RMS, mm | 7.337900 | 4.455483 |
| Absolute-residual p95, mm | 13.797057 | 8.824453 |
| Original feature-weighted RMS, mm | 7.233787 | 4.400967 |
| Yaw geometric lever RMS, m/rad | 2.642952 | 2.642004 |
| Top5 owners' absolute weighted lever contribution | 24.1116% | 26.2823% |
| 3-5m absolute weighted lever contribution | 85.0537% | 82.8244% |

The original foreground mass denominator is46.00000000000001. Usable assigned
mass40.66666666666667 is88.4058% of that D; selected mass37.16666666666667 is80.7971%.
Seven nearest-but-uncovered rows retain mass3.5 (7.6087% of D); zero rows are unmatched.
Original missing/uncertified/ambiguous buckets are1.3333333333333333/4/0, giving
unassigned5.333333333333343 (11.5942% of D). Face counts are not a replacement for D.

All selected body-range bins:0-1m=0rows/0mass;1-2m=3/1.5;2-3m=28/13.666666666666666;
3-5m=46/22;5-10m=0/0;10m+=0/0. This concentration is not authority to clip at3m,
drop far rows, change the radius, reassign owners, or relax a gate. Nominal `w*r`
and `w*r*lever` sums are diagnostics, not the original robust gradient/H/pose cause.
Owners are node aliases of incident primitives, not physical left/right lanes;
vote/reliable-vote counts are not calibrated variances. No single owner dominates.

Seq162 normal diversity is0.0000501373280263818, observable rank2 and longitudinal
observable=false. Longitudinal motion is pinned to raw VectorNav, not recovered
from these almost-parallel visible boundaries. Innovation is2.865351mm translation
and0.0006893554rad yaw. Residual reduction is internal consistency, not truth error.

1257 explicitly valid observation raw/model pairs have displacement median4.964075mm,
p9551.011543mm, maximum214.461657mm. The107 fused nodes,96 graph chords and106
certified finite primitives provide202 actual competing primitives. Among105 owners
with latest raw support, node/latest-support-centroid displacement median6.015375mm,
p9528.768504mm, maximum42.604836mm. Neither displacement is GT error or proof of a
mean/chord bias: past pose histories, full model support and primitive selection
were not captured. The tiny node-target normal offset does not exclude chord bias.

## Current blocker and what this does not establish

The actual prefix stopped at15/111 masks:14 accepted, then seq165 rejected by the
unchanged engine80ms gate. Engine84.96970002306625ms; matcher35.2584ms within its
unchanged40ms gate. Only seq162 has matcher arrays. It cannot explain the later
deadline or establish overshoot magnitude; no exclusive stage-time breakdown or
closed-loop motion truth was measured. Root must retain this failure, optimize
effect-preservingly where appropriate, and rerun only under a new explicit scope.

Current boundary matching bypasses the old FilledIndex reference PCA target;
foreground/observation fitting remains in the path. Missing captured evidence:
selected primitive/alpha/permission IDs; per-iteration estimate/basis/stage;
original inline robust weights/H/gradient; past per-frame model/pose/full-support
histories; foreground half-mass assignment IDs; observation model coefficients
and full-WB provenance; GT and closed-loop/two-lap runtime delivery. A geometry
cause or cure cannot be asserted from these returns alone.

## Actual fixed-scale diagnostic figure

The separately authorized display used existing Matplotlib3.10.9, with one run,
exit0, tool wall1.2063829s and internal display wall0.4555566000053659s. It reloaded
only the already captured arrays and pinned analysis; all107 descriptors and three
input hashes were unchanged. No runtime core, custom native graph, replay, new
fitting, H or GT was imported. PNG was visually inspected. Literal world
x[0,6]m/y[-2,2]m is an equal-meter display viewport, so upper geometry may be clipped
on screen; all202 past primitives and all84 first/final query rows were submitted.
All77 first/final residuals,107 owner bars (including zero mass), original D and
count statistics remain complete. It does not zoom to enlarge the millimeter
correction or classify node owners as physical lane sides.

- Display source: `plot_01.py`, SHA256
  `91b3434fce875136f685dc48c1637c2b739bfd7ab993b4fe77ff4e0e0ec3f534`.
- PNG: `plot_run_01/seq162_diagnostic.png`, SHA256
  `5794b8a7ad4ee176bfd13bc82fa97861803f7f9ea8a82e3ec46e9f6c7ca38683`.
- SVG: `plot_run_01/seq162_diagnostic.svg`, SHA256
  `1fc83877eac69192d066a6efd9fa1e3d4f766f1461891f378db8120a347017df`.
- Display receipt: `plot_run_01/receipt.json`, SHA256
  `0c1d69ee2eacbb029943303a94653e94740b0f78c97b5d3ac71889bec3812b96`.
