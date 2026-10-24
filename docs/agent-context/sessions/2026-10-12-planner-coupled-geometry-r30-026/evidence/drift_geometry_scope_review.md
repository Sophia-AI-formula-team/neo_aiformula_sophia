# Drift cause, current boundary pipeline, and paired-cut repair scope

Static diagnostic and next-work proposal only. No R/Simulator source edit,
parameter change, Python/AST/import/test/solver/native/Unity execution, raw
image/NPZ read, or large-JSON scan. This is NOT a new lap or replay result.

## 1. Answer first

The paired-cut proposal does **not directly repair the demonstrated main
geometry error at trial28 seq160**. That first jump was dominated by right-side
filled-strip reference representation and lateral/yaw coupling, not a proved
failure of two actual corner quadratics to meet. A paired proposal changes
at most two selected primary seed roots/gradients/fragments; it does not
rebuild an existing frozen reference or persist a quadratic map model.

Current R29-style source already addresses important OLD trial28 mechanisms:
the production scene uses signed physical mask boundaries, bypasses the filled
anchor's second voxel/PCA-centroid representation, and carries a guarded
bootstrap motion prior into the first teach visual match. Thus trial28's exact
old causal explanation cannot simply be asserted to remain unchanged now.

However, source routing is not numerical/driving validation. Current code still
reduces boundary geometry to fused mean nodes, chords and finite tangent lines.
Those reductions have a demonstrable curvature/visibility-bias possibility.
Whether they dominate any CURRENT actual yaw/lateral drift remains unmeasured.
No corner-candidate rate, source unit count or toy example establishes that.

Priority: first capture the actual current pipeline's FIRST visual match and
its reference/residual contribution, then fix a measured representation error
at its producer-to-reference transition. Keep paired repair a separately
scoped corner/domain-consistency change; do not use it as the drift diagnosis.

## 2. Evidence boundary and source pins

Read FULL existing reports:

- `trial28_analysis/first_match_audit_01/FINDINGS.md`, SHA
  `ca58894659a4cba8e65c43e987f15fe367ccd68772cde8737d6c7282194c314c`.
- `trial28_analysis/DELIVERY.md`, SHA
  `0a04a46138c077095b626db055467f607323369bb5351052fde9cbdf8f1102a3`.

Those reports concern immutable OLD source
`023ccc0b18003dffdfb7262e87539077e53fb4c7`. They retain a failed trial and its
one original-source reconstruction. Their numbers below are report evidence,
not measurements recomputed here or results for current R.

Current sources read in full (paint c57 was fully read in the immediately
preceding wiring task and its hash rechecked unchanged here):

| Source | SHA256 |
| --- | --- |
| `mask_localization.py` | `11f33a2ddf239c668dd5ecc4a233f12fa9f085e14469863636bf9a623abe8935` |
| `mapping_core.py` | `ef2cc93dcdcb07c861a9f21d9ae0d7fb061f33f5772a48044671e1875ef09128` |
| `paint_geometry.py` | `c57c1d53da9f66844b5477d4cd9848cb724ed0632e35054a1d93f252655251d3` |
| `boundary_localization.py` | `762cddfb84daded733aba561399bb0cf90fb60cab93aa4a602f0a20f12616db5` |
| `finite_support.py` | `9e71c177b72e1fef2b145f325f7234b5b7c7a9c2f7d90c4610485d13ea9065ee` |
| `_gnss/mapping.py` | `a2171210d68c1d3d55e67cf2e826914f109f382280a7dbdc67a05571da574b88` |
| `Simulator/Tools/LaneTeachRepeat/lane_backend.py` | `bb92fc04fe0be61412f96139ec8b703cd71684c0edea8d75f2305eefa11c424b` |

The first five files are in neo
`workspace/src/aiformula/control/lane_mapping_lya_reference/lane_mapping_lya_reference/`;
`_gnss/mapping.py` means
`workspace/src/aiformula/control/lane_mapping_fixed_gnss/lane_mapping_fixed_gnss/mapping.py`.
The Simulator adapter was read as TEXT only, not run or imported. No excluded
quantized prototype pair was read/imported/hashed. Existing schemas/data and
old 01–03 artifacts are untouched.

## 3. What trial28 actually established

Seq151–159 committed bootstrap geometry under raw-prior poses. Seq160 was the
first visual match, before any visual pose-map feedback could contaminate its
past reference. The audit reproduced all 105 confirmed points byte-exactly.

- Correction: -0.04720319 m lateral, +0.01572953 rad yaw; RMSE only changed
  0.04729885 to 0.04572030 m. The first iteration supplied nearly all the step.
- Lateral/yaw Hessian-column correlation was 0.96033. Visual rank passed, but
  the finite forward-only view permitted a strong trade-off of these modes.
- Right paint supplied 99.20% of first-iteration yaw gradient attribution;
  right near-range points below 2 m supplied +0.01145668 rad under the SAME
  original Hessian. This was attribution, not an alternate fit or truth input.
- Over the SAME overlapping observed x interval, headings changed from current
  reliable pixels -0.00010499 to past dense pixels +0.00543831, and to one
  measured mean per confirmed filled cell +0.01736488 rad. Selection and
  compression were therefore real contributors, not just projection error.
- The right painted strip occupied three lateral rows. The lower row had only
  19 confirmed cells and ended at x=3.3030 m; the other rows had 31 each and
  reached about 4.274 m. Eight lower-edge cells had only 1–4 ORIGINAL votes.
  The sparse cloud acquired a positive slope by underrepresenting the lower
  edge farther ahead. Lowering five votes or dropping current unmatched mass
  would disguise, not repair, this representation problem.
- Residual decomposition under the original Hessian gave +0.00805260 rad from
  current-to-actual-anchor and +0.00687377 from actual-anchor-to-PCA-centroid.
  Therefore replacing only the PCA centroid with its seed could not remove
  the entire old bias.
- All three initial iterations had `motion_precision is None`. This was an
  amplifier of the biased visual direction, not a geometric explanation.

The delivery retained 111 non-timing mapping results (110 accepted, terminal
seq261 rejected) and a failed HOLD video. It did NOT establish a completed lap
or the numerical cause of every later yaw/lateral deviation. The old audit also
explicitly lacked continuous capture-time heave/tilt evidence and found a
smaller difference already in historical dense projections. Do not assign
all remaining error to a centroid, corner, pitch, heave or rasterization.

## 4. Current actual scene path: boundary and motion are wired

### 4.1 Adapter and source/phase routing

`lane_backend.ProductionCore:166-277` imports the real neo modules from the
requested checkout and checks module paths. Lines219–231 require boundary
version2/3/4 and declared topology/domain contracts; current paint is version4
(`paint_geometry:29`). Production provenance is `paint_boundary_v1` at271.
The loader does not silently select legacy XY when this contract is missing.

`LaneSession.step:1283-1292` builds a SAME-mask, SAME-stamp BoundaryObservation
from the decoded PNG and selected raw attitude. `_boundary_kwargs:919-925`
rejects a missing/wrong-stamp observation for production boundary provenance.
The empty-kwargs branch is explicitly for historical isolated XY fixtures,
not this ProductionCore route.

`_map_frame:903-909` passes that observation AND the acknowledged raw sample's
`motion_source_stamp_ns=self.sample.stamp_ns` into engine.process. Teacher
frontend targets/ROI clouds do not enter these map inputs (`_mask:853-857`,
`_teach:938-955`). Raw prior uses one teach-start-relative frame (`_prior:894-901`),
not GNSS or simulator world pose.

Engine `_process_observation:374-387` binds representation and stamps, and
`475-514` prepares/commits the actual boundary map together with the filled
map under its guard. `_prepare_matcher:224-235` uses a boundary snapshot and
BoundaryLaneLocalizer whenever boundary map exists. No missing-kwargs bug was
found on this current static production path.

### 4.2 Initial acquisition is not the old visual-only special case now

Engine `430-454` prepares the same-session bootstrap MotionInformationState;
`519-522` publishes it only with the accepted guarded bootstrap map commit.
`_prepare_matcher:188-191,234-237` binds that immutable state to a prepared
candidate, and `_match_reference:268-272` forwards it on first teach match.

`mask_localization.match:1494-1540` validates it, requires a strictly newer
causal motion source, propagates covariance, and computes precision. The
visual rank test still precedes prior regularization (`1615-1642`): a prior
does not manufacture visual observability. This addresses the OLD first-jump
amplifier in source; it does not correct biased reference geometry or establish
a current numerical improvement without a replay.

### 4.3 Repeat really freezes its reference, and refinement is separate

Adapter `_prepare_repeat:1025-1060` freezes the complete now-past map and copies
only committed localization state. `_build_repeat:1076-1105` constructs a
BoundaryLocalizer from snapshot.boundary_reference, not consensus_xy, for the
current boundary production route. `_finish_prepare:1111-1130` checks frozen
source equality and handoff state. `_repeat:1141-1151` passes SAME-current
BoundaryObservation, original full support and the actual prior source stamp.

`mask_localization.commit:1268-1304` updates a separate inlier refinement layer
and correction state. That refinement does NOT participate in matching or
alter the frozen route/reference. Existing anchor geometry is immutable.

Timing scopes are also separate: scene `_commit_guard:717-731` really checks
independent-motion receipt/capture age <=200 ms before commits; `_repeat:1153-1162`
also requires match <=40 ms; engine retains internal80 ms. ROS RuntimeSafety's
different full-repeat40 ms contract is not replaced by adapter200 ms. No gates
or simulation/vehicle parameters are changed by this diagnosis.

## 5. The exact position/direction/centroid chain now

```text
same PNG → original projected directed W/B faces
    → each target's primary or ribbon model
    → observed W/B root + analytic gradient + own J/quad-clipped tangent
    → transformed by current admitted pose
    → contiguous signed boundary run in a grid bucket
    → one point/normal frame mean → equal-frame fused record
    → confirmed nodes + confirmed adjacency chords + finite mean-normal lines
    → globally nearest signed finite primitive → unchanged finite permission
    → point-to-primitive residual → bounded lateral/yaw solve
```

Specific current functions:

- `paint_geometry.extract_boundary_observation:2106-2172`: primary quadratic
  coefficients affect `_quantized_curve_positions:1946-1984`; final gradients
  and roots survive ribbon, competing, own-root-J and footprint gates.
  The BoundaryObservation returned at2165 stores roots/gradients/fragments,
  **not origin/normal/scale/abc primary curve packets**.
- `BoundaryObservation.model_points/model_normals:196-215` expose those roots
  and gradients; `transformed:220-239` transports them with this admitted pose.
- `BoundaryConsensus.prepare:730-748` groups eligible signed model geometry
  using model points and grid buckets. `_contiguous_cell_runs:500-548` refuses
  broken runs, opposite normals, disconnected reentries and artificial seam
  merging. Opposite sides are not generic filled XY voxel means.
- `_group_frame_means:551-570` averages point and normal within each current
  directed run. `prepare:839-841` then equally averages those means across
  committed frames and renormalizes the normal. `_BoundaryRecord:458-467`
  stores mean point/normal, votes and latest raw face segments/neighbors;
  **it stores no quadratic coefficient or original primary support model**.
- `prepare:980-990` maintains a fixed creation chart and pointwise finite
  support votes. Confirmation remains5+1 (`714-716`); topology edges have
  independent repeated directed transition votes (`884-914`).
- `_certified_model:1258-1284` calls `finite_support.certified_segments:579-629`.
  The exported line passes through CURRENT fused point with CURRENT fused
  normal; the original creation chart certifies finite extent, not its exact
  curvature/position. It is not an export of the quadratic fit.
- `boundary_localization._FrozenBoundaryGraph:146-213` builds node-to-node
  chords and those finite model segments. Chord normals derive from the NODE
  position difference (`160-175`); observed normals only qualify signed angle
  agreement. At smooth vertices the measured node normal is used (`281-286`),
  but at chord interiors the direction is the chord normal (`257-260`).
- `FrozenBoundaryIndex:304-310` uses reference.points_xy directly, without
  `_voxel_means(anchor,.08)` or a new PCA centroid. Its graph query first finds
  the nearest signed finite primitive and only THEN checks permission; failed
  support cannot expose a farther convenient line (`query:215-299`).
- `BoundaryLaneLocalizer._correspondences:480-505` evaluates residual relative
  to the nearest chord/model projected target. Legacy `_index.line_points`
  PCA centers at `mask_localization:574-580,1306-1328` are NOT its position
  residual target. With no graph, boundary residual uses the actual owner mean.

Current scan foreground PCA still exists, but its role differs: parent
`_prepare_scan_geometry:1330-1373` determines current-only structure and the
COMPLETE cross-section mass. Boundary override `445-478` assigns half masses
to the first actual W/B crossing on each side (`assign_foreground_mass:43-107`),
then chooses actual model representatives instead of across-width means
(`_scan_representatives:110-135`). Missing/uncertified mass stays in the original
denominator. Do not confuse these current feature normals with a PCA fit to
historical confirmed filled cells.

## 6. What R29 structurally avoids, and what remains unknown

| Old mechanism | Current source status | Not established here |
| --- | --- | --- |
| Different strip-interior rows become a sloped cloud of confirmed anchors | Boundary identity/direction keeps actual thin signed sides distinct | Numerical first-match yaw on the same recorded input |
| Additional .08 m anchor voxelization changes that cloud | BoundaryLocalizer uses reference directly | Remaining boundary selection/visibility/projection bias |
| PCA-centroid offset changes reference position | Boundary graph/finite mean-line residual bypasses old PCA line_points | Accuracy of fused nodes/chord shape/tangent-line approximation |
| First teach visual match has no motion precision | Guarded causal bootstrap state now supplied | Degree of improvement, calibrated covariance or absence of drift |
| Late corrected poses influence later teach anchors | Still possible through admitted pose→map→later keyframe | Attribution of current actual large drift; no current trace analyzed |

For an IDEAL noiseless straight physical boundary, arbitrary subsets of its
points/means remain on that same line, and measured normals remain consistent.
The old three-filled-row slope mechanism is therefore not inevitable on this
new boundary path. That is a structural statement, NOT proof that actual
quantized/projected/past-pose boundary observations are noiseless or unbiased.

Two current representation risks are clear from source, not measured frequency:

1. Curved-point averaging need not remain on the measured curve. Hand example:
   exact roots of `y=k*x²` at x=-h,+h have mean `(0,k*h²)` and their symmetric
   outward-gradient mean normalizes to `(0,1)`. Exported mean-normal line is
   `y=k*h²`, not the curve at x=0. For arbitrary local x distribution,
   `mean(y)-k*mean(x)² = k*Var(x)`. All roots could be correct before aggregation.
   Choose h and k sufficiently small to satisfy one cell/old angular gates;
   adding five identical source-frame observations changes votes, not this
   algebra. This is an independent mathematical example, NOT actual stats or
   a tested production mask fixture.
2. Adjacent mean nodes can have different visibility/curvature offsets. For
   two quadratic runs with mean abscissae u1,u2 and variances v1,v2, the chord
   slope contains extra `k*(v2-v1)/(u2-u1)` beyond the chord through points on
   the curve. Small normal-direction errors can therefore become chord yaw
   errors, especially over a short node baseline. Current30-degree agreement
   is a coarse qualification, not proof that chord slope equals measured
   gradient. Finite-domain/quad certificates constrain extent, not curvature.

Projection/pose-associated errors can also remain, and actual node association
is conditional. These examples do not identify the main cause of any later
CURRENT scene drift. The old audit's smaller dense-history slope difference
and missing heave/tilt evidence remain reasons not to invent that attribution.

## 7. Why paired coefficients cannot by themselves resolve this chain

The proposed paired helper is restricted to two actual cut seed primary
packets, with unchanged charts/bounds/full supports. If eventually admitted,
new beta changes those two roots, analytic gradients and tangent fragments.
Those rows may change assigned foreground mass and future owner means/domains.
This can matter at a short cap/corner, recover observed support, or remove a
local inconsistent join. Its participating solver mass/leverage must be measured.

It does NOT change all other target-specific primary fits, the geometry of an
already frozen anchor, or the same-frame filled projection. It does NOT retain
or fuse quadratic curves in `_BoundaryRecord`, replace node/chord reduction,
change the solver's forward-only lever-arm coupling, correct tilt/heave, or
replace admitted pose-map feedback with a joint pose/map optimization.

At fixed incoming pose, a modified current scan is matched against the SAME
past frozen reference; new coefficients cannot retroactively move that anchor
centroid. Only subsequent guarded map updates/keyframe refreshes can have an
indirect effect. Initial limited retention/adoption policy also deliberately
leaves healthy/ribbon/shared pairs alone. Do not call this all-lane geometry.

The scene's visible `map_points` is another layer: `LaneSession.step:1311-1314`
uses engine.consensus_points_and_votes, alias of its FILLED SparseConsensusMap
(`mapping:540-545`), not boundary_reference geometry. Freeze also preserves
that filled layer for route corridor construction (`mapping:547-560`, adapter
`1084`). At fixed pose paired beta does not directly modify those yellow
points at all; any reduction of their apparent overshoot must come through a
demonstrated improved pose, not a cosmetic renderer or corner-curve overlay.

## 8. Minimal next real geometry check, then the right repair layer

### A. First authorized bounded CURRENT-source replay/capture, not a new lap

No run is performed by this document. Root can next authorize one fresh,
hash-bound replay/capture of the old raw prefix through its first current-source
visual match (include seq160 and a bounded later horizon such as163). Keep every
original mask, raw IMU/wheel/source/receipt/acknowledged sample/calibration and
unchanged40/80/200 timing fences. First-match sequence may move because boundary
confirmation differs; never force a visual match at160 or bypass bootstrap.
No GT/future/GNSS input or known track is supplied to the estimator.

Retain compact source-bound arrays for actual current scan model roots/gradient,
run-to-owner membership, frame means, confirmed boundary nodes/normals, graph
chords, finite model segments/domains/permissions, selected primitive owners,
targets/normals/residuals, exact original feature mass denominator, and original
Hessian/gradient before motion fusion. Log whether bootstrap prior was really
used and the actual module/reference frontier. Existing old-source numeric
equalities are not acceptance expectations for this intentionally changed core.

Independent posthoc attribution uses ONLY that run's ORIGINAL correspondences,
weights and Hessian (no alternate pose fits, search/gate tuning):

- compare current and past boundary geometry over the SAME observed tangent
  interval and same signed side, not differing extents;
- separate projected boundary input discrepancy, run-mean displacement,
  repeated-frame fused geometry, chord-vs-observed normal, and residual target;
- attribute lateral/yaw gradient by side/range/primitive class;
- intersect actual paired cut seed IDs with selected scan representatives and
  solver masses/levers; this measures whether paired work covers the problem;
- retain all missing/subthreshold faces/mass and original5+1 histories. A
  reduction of denominator or deleting nearest competitors cannot be scored
  as improved geometry. Keep original invalid/unknown inventories.

If the current boundary path removes the original first-jump signature, do not
keep repairing its already-bypassed filled/PCA code. If a later error persists,
capture the FIRST onset on current sources rather than extrapolate trial28's
first-match explanation to the whole trajectory.

### B. If aggregation/chord bias is actually demonstrated

Minimum principled change is at `BoundaryConsensus.prepare`→reference export,
NOT another two-seed coefficient tweak. Separate owner identity/votes from
position/direction geometry. Fuse boundary position at a FIXED observed
creation-chart abscissa, with normals evaluated at that SAME position; do not
average points at changing tangential coordinates and attach an unrelated
averaged normal. For a local curve, retain its authenticated nominal packet
and map its actual point/gradient to the owner's common chart before fusion.

This requires real source/support and domain work; not every old model packet
is currently returned. New position constraints must be valid inside its
original finite chart AND actual measured footprint. A polynomial evaluated
inside center bounds is not automatically a measured curve fragment. Current
`clip_local_lines` certifies a tangent line, so it cannot be relabeled as a
whole-quadratic permission. Any curve trace needs its own conservative
inward-domain/quad and normal/J proof, or returns unknown.

Preserve independently confirmed pointwise5+1 extent votes, original topology
identities, branch/continuation alternatives and source frontiers. If the fixed
abscissa was not observed in a frame, do not extrapolate it or fabricate a vote.
At first a bounded affine/straight common-chart model can be studied alongside
the unchanged baseline; curved canonical transport is a separate tested step.

Reference primitives must carry coherent position AND analytic direction from
that one finite geometry. Do not merely replace chord normals with a nicer
average while keeping the old chord zero set; do not remove an inconvenient
chord from nearest competition to expose a farther line. A representation
change must explicitly inventory/replace the same owner/edge hypotheses and
retain globally-nearest FIRST, finite/branch permission SECOND, full denominator,
and no support/bounds growth. A new canonical model whose finite permission is
unknown remains unknown; it cannot silently create supported geometry.

This proposal needs independent straight/curved changing-visibility fixtures,
actual-source attribution, full budget/no-regression tests and a named tag.
It is not a claim that a complete safe production patch has been proved here.

### C. If the measured discrepancy arises BEFORE aggregation

Repair the demonstrated input layer instead: full per-target source/chart,
W/B root/gradient/footprint model or causal projection transport, keeping
calibration and vehicle/simulation parameters fixed. Do not move map means
or constrain pose to an authored track to hide projection/pose errors. Use
only actually arrived permitted motion information; no future attitude, GT,
GNSS or imagined heave measurement. Current evidence does not choose that
layer conclusively, so no numerical offset or relaxed gate is proposed.

## 9. Delivery boundary

The known old cause is real; the current source already contains substantial
structural fixes; the paired-cut method is a limited local consistency tool;
remaining global drift cause/coverage and current scene success are UNPROVED.
This static plan changes no code or outputs and promises no lap. The next
evidence must connect an actual current-source geometry discrepancy to the
unchanged solver's gradient before choosing the smallest production fix.
