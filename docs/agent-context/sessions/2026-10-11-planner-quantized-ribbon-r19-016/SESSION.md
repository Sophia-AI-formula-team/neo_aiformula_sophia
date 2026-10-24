# Agent session: 2026-10-11-planner-quantized-ribbon-r19-016

Input handoff: 2026-09-28-planner-neo-004

RESULT_READY for planner analysis; failed development checkpoint, not a deployment release.

## Objective and scope

Continue the actual one-lap mask/raw-motion map and fixed-route second lap.
No vehicle/simulation/camera/LYA parameter or safety-gate changes. Repair the
observed quantization geometry problem, not the coverage denominator.

## Input handoff

Planner consumes neo-004 and sealed015; root AGENTS, AGENT_CONTEXT, STATUS,
PROTOCOL and current source were read. Local draft is deliberately outside
published sessions until reviewed and sealed. Root owns primary code.

## Source and working tree

Base 5f483e2c6764851ef81c4b854f8626f63287c0c6, tag
lane-mapping-20261011-r18-rc1, repair/lane-keyframe-transaction-20261011.
Canonical ID1303517209 and refs verified. R18 CI finished: 4/8 workflows and
12/16 jobs passed. All four Foxy jobs built but failed the same capacity test;
the later DDS checks did not execute. Append-only R18 receipt preserves this.
The two older unintegrated quantized_curve_fit prototype files are preserved
and excluded. New quantized_ribbon.py and ribbon_observation.py are root-owned
and integrated into extraction, without replacing actual raw topology.

## Work performed

First fixed four-region experiment used original radii and actual same-side
faces, fitting shape by unconstrained least squares and solving only an
intercept interval against all pixel labels. All four failed at all three
scales. Records are r18_ribbon_direction_probe_01; none overwritten or reused
as successful evidence. No direction, graph edge or finite vote was published.

Added bounded four-variable log-barrier phase-I curve proposal in
quantized_ribbon.py. All white/black labels constrain both shape and offset;
there is no SciPy dependency, slack label, subset selection or negative
feasibility tolerance. Returns only an outward-Horner-certified positive
margin, not an exact optimum claim. No existing contour topology is changed.
Independent arithmetic tests passed 67/67. The fixed four-region probe found
strict label solutions at .45/.9 m (8/12 patch scales); all 1.35 m attempts hit
the bounded iteration limit. This is local feasibility, not mapping quality.

Whole-frame integration audit exposed three issues: target supports could lie
almost 2r away, chart normals replaced side-PCA normals, and old-valid faces
could be overwritten. Corrected all three: reject each target unless its full
side support is within its radius, check the actual signed side-PCA direction,
and propose only for original direction-invalid faces. All 31 extraction and
admission tests pass, including the three new regressions. No gate increased.

One instrumented frame found repeated all-pixel chart scans and label solving
expensive (460.9 ms profiled extraction, not runtime acceptance). A conservative
sorted-axis candidate index now avoids scanning all pixels per seed; unchanged
disk tests still retain all original labels. All 24 observation fields matched
bit-for-bit on four inputs (two distinct images). A separate finite-support
batch validates immutable arrays once per frame; 56 independent tests preserve
public error/vote semantics and full consensus state/export/transaction bytes.

R18 CI fixture repaired without changing capacity: keep the old horizontal
case as an explicit combined-domain-capacity rejection; isolate the graph-only
capacity case with zero projected finite extent. Both focused tests pass.

## Evidence and result

Full R19 core regression: 3277 passed, one Windows skip of a Linux process-group
test, exit 0; 175 source identities unchanged. Eight native APIs were actually
called. The older unintegrated prototype is the only explicit test exclusion.
This is Windows source/native testing, not colcon or DDS proof.

Simulator source loading and runtime provenance now recursively include the
two actual new imports. The focused adapter suite was 43 pass, two failures:
historical R8/R12 loads correctly reject the currently installed R18 native
binary's different source identity. No identity guard was bypassed. Nine new
dependency tests pass. Simulator parameters/C# assets/scenes were not edited.

R19 has not passed the full mapping/runtime chain. Four original development
cases are 0/4: accepted 14, 11, 13, 11 of 509 planned each, then coverage failure.
These retain the previously declared 2000 ms geometry-diagnostic budget, not
the production time budget. The separate fixed trial28 prefix accepts 13/14,
then seq164 exceeds 80 ms at 86.226 ms (matcher 26.612 ms, overlap .82833). That
frame is not committed. Full inputs, thresholds and source identities match.
R18 source-only results and
all geometry/runtime failures remain as sealed015. New R18 recorded prefix03
with the CI-built eighth API still failed at seq162, 80.0552 ms versus the
unchanged 80 ms budget (11/12 commits). No new Unity drive, first lap, frozen route,
second lap or new successful video. Original actual trial28 is still failed.

## Risks and blockers

A label-feasible local ribbon is inferred shape, not proof of paint continuity.
Caps, nearby stripes/U arms, raster gaps and sensitivity gaps need independent
controls. Only original real face adjacency may observe graph/finite support.
The verified R18 CI binary is installed locally; all eight actual API calls
passed and the new required-native suite passed 64/64 without skips. Original
binary was backed up. No compiler/dependency was installed. This does not make
the rejected prefix or failed R18 CI pass.

The source still discards an individually valid face's finite positional
extent unless it has a valid next face. Investigate explicit finite support
strictly inside each original W/B pixel footprint; this would be a NEW model
assumption, not an equivalence optimization or permission to bridge a gap.
One diagnostic bug remains: the post-raw-prior-anchor overlap rejection logs
the previous query's ratio. The missing final numerator must not be invented.

## Next agent actions

Retain all R19 failures and the append-only R18 CI receipt. Inspect the current
seq164 instrumented profile separately from runtime evidence. Repair finite
footprint representation and the stale diagnostic ratio; independently test
shape/caps/gaps, whole-footprint J4, causal voting and original deadlines.
Repeat the original development cases and strict prefix without gate changes.
Fresh protected Unity preflight is mandatory
before a continuous actual TEACH/REPEAT, with all failures/logs/metrics/video.

