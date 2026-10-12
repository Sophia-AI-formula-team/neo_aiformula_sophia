# Anonymous observed-junction association — isolated candidate v0

Status: **disabled, not installed, not imported or tested at authoring**. This
directory is independent of the active semantic core. No Python, fitting,
model, Unity, or test execution was performed for this deliverable. Synthetic
tests are written, not a test receipt. A future run must record actual results.

The controlling repair plan is
`../../analysis/OFFICIAL_AUTO_MATCH_PROBLEM_AND_PLAN.md`. The user selected the
original official OSM plus automatic matching. The declined simulator portal
calibration, its coordinates, evaluator CAD, and Unity scene coordinates are
not inputs to this candidate.

## Problem and bounded scope

The existing exact outer first-lap result remains rejected: whole-route RMSE
3.404463140 m is above its 3.0 m gate. Its two-direction, full cyclic phase search
already ran. Comparing junction chords to a rounded driven contour and asking
for body curvature at each official junction is a different measurement model
from seeing a branch while driving straight through a T-junction.

This candidate scores **new, independent correspondences between actually
observed junction anchors and official node positions**. It does not reinterpret
the old whole-route score, relax its gate, deform the measured reference, infer
an unseen internal road, or set `FusionResult.available`. Its result always says
`detour_authorized=False` and `internal_connectivity_observed=False`, including
when `available=True` for this association hypothesis alone.

An empty/insufficient/ambiguous observation ledger stays unavailable. The five
outer official nodes all have degree three: their number and order do not prove
their IDs. The candidate never searches for road bends as a substitute for
junction observations and never reads an official ID from an observation.

## API and ownership

`junction_association.py` is standard-library-only and duck-types the current
`OfficialRoute` and `RouteReference` public data. No active-core import, ROS node,
global state, clock, file I/O, or installation hook is present.

```python
result = associate_junctions(
    observations, official_route, measured_reference,
    context=ObservationContext(
        epoch=epoch, calibration_id=calibration_hash, frame_id=odom_frame,
        lap_start_time_s=start, lap_end_time_s=end, completed_lap=True,
    ),
    official_branch_bearings=branch_bearings_from_graph(official_route, graph),
    original_whole_lap_status=original_result.status,
    # enabled omitted => disabled; only literal bool True enables the candidate.
    enabled=True,
)
```

The example is a proposed interface, not an integration instruction or an
acceptance receipt. `official_route` and `graph` must be loaded from the same
unchanged, hash-verified OSM. This module checks coordinate/outer adjacency
agreement in its prior helper; it cannot authenticate a caller's in-memory
objects or establish official travel permission.

Result fields include the unchanged copied `original_whole_lap_status`, every
candidate's IDs/direction/real fit/residuals/bearing errors/rejection reasons,
best and runner-up scores, required ambiguity gap, and optional `match`.
`match.nodes` separates `observed_anchor_xy` from `reference_anchor_xy`, retaining
actual projection distance and measured-reference progress. No point array,
trust bit, route length, original alignment, or caller object is edited.

## Required observation meaning and evidence boundary

A `JunctionObservation` has an anonymous track ID, measured odom anchor,
directed outward branch bearing, bounded position/angular uncertainty, and
three or more `CaptureSupport` records. The anchor must identify the junction's
connection station on the driven main corridor. A road's outer lip, a body-turn
location, an arbitrary portal, and an official intersection centre are not
automatically the same physical landmark. The image observer must establish
which station is measured and justify its uncertainty. If it cannot, association
must remain unavailable; projection must not disguise that semantic mismatch.

Each support carries source sequence, capture time, original ingress time,
capture-time pose time, epoch, calibration identity, odom frame, an independently
estimated anchor/bearing, counts for both **observed side-branch boundaries**,
and a crop/truncation flag. The module checks metadata consistency, positive
counts on both boundaries, no truncation, and support geometry agreeing within
the observation's stated uncertainty. These checks **cannot tell what pixels
depict**. Counts do not prove a T-junction, a two-sided branch, branch width, or
road clearance. The upstream observer must retain actual pixel IDs/geometry and
reject gaps, crosswalks, dash patterns, service openings, occlusions, and cropped
components. It must not label every paired component a verified junction.

The newly staged raw-mask observer currently emits anonymous geometric
candidates with `junction_recognized=False`; those candidates therefore do not
yet satisfy this API's positive junction observation contract. No adapter is
provided to relabel them automatically. Real-mask positive observability remains
a prerequisite before any enablement or active-core integration.

Historical first-lap anchors are not live steering measurements. Their original
capture-to-ingress age must be <=0.5 s, pose/capture skew <=1e-6 s, and all captures
must belong to the declared completed lap/epoch/calibration/frame. Support
sequences and times are strictly increasing and globally unique. Three frames
must fit within 1.5 s. New matching does not refresh ABC, command, RGB, odom, or
traffic-light freshness. A later source reset requires a new ledger and reference.
The caller is responsible for independently proving lap completion and original
capture/pose binding; this pure function does not reconstruct either claim.

V0 deliberately accepts exactly one nonoverlapping track per outer node, with
no reused capture across tracks. Missing or extra candidates are rejected rather
than choosing a convenient subset. Simultaneously visible real junctions or
fragmented tracking may cause a conservative false negative; merging such tracks
requires a separate evidence-backed observer change, not silent association.

## Projection, correspondence, and failure gates

1. Validate finite numeric data, exact cycle IDs, distinct official positions,
   measured segment lengths/cumulative lengths, and actual bool trust flags.
2. Project each measured anchor onto the unchanged reference. Distance plus its
   uncertainty must be <=1.0 m by default, with a hard configurable ceiling of
   2.0 m. A nearby competing part of the loop is rejected instead of silently
   selecting one branch. Adjacent samples of one connected segment are not
   separate junctions. The whole +/-0.5 m neighbourhood must be trusted, widened
   to the positional uncertainty when needed. Nearest untrusted geometry is
   not skipped in favour of a convenient trusted segment.
3. Require distinct projected stations (>=2 m), one chronological traversal of
   the reference, and exactly N observations. Reference progress orders the
   independent observations; it is not a full-lap shape-correspondence prior.
4. Enumerate all N cyclic assignments in both directions. There is no fixed
   initial ID, sampled phase grid, bend seed, candidate pruning, or reflection.
5. Fit each assignment with an orientation-preserving similarity from official
   XY to **measured odom XY**. Least squares and residuals are directly in measured
   metres; scaling the official sketch uniformly does not change the geometric
   score. This is a separate actual node-correspondence residual, not the old
   contour residual with different units. Neither reference metres nor trust is
   transformed. Collinear/degenerate observations are rejected.
6. Keep the measured RMSE cap <=3 m, also require every node residual <=3 m, and
   add each relevant observation uncertainty conservatively. These preserve the
   old numerical maximum but do not establish statistical equivalence to the
   old full-route model. No real-junction calibration of these candidate limits
   is claimed. Scale must be inside explicit bounds.
7. Keep every finite hypothesis when checking the runner-up gap, including those
   failing scale or branch prior gates. Require gap >0.3 m plus twice the largest
   position uncertainty. Exact/near symmetry remains unavailable. A chord prior
   cannot discard a geometric competitor and manufacture uniqueness.
8. The best geometric correspondence must also pass each directed branch-bearing
   gate. Observed uncertainty is added to angular error, with a maximum 30-degree
   candidate gate. Wrong-way entry is not made valid by count/order matching.

Apart from the explicitly bounded projection-distance setting (<=2 m), limit
configuration cannot loosen these defaults: ingress/pose/support-age, ambiguity,
trust neighbourhood, anchor separation, scale, residual, uncertainty, and bearing
checks have validated upper/lower bounds. Inconsistent settings fail closed.

`branch_bearings_from_graph` uses the single non-outer adjacent OSM edge as a
direction **hypothesis**. It does not use the unseen internal node/edge network
as observed evidence. Crucially, a long sparse chord need not have the local
road's entry tangent. This can reject a correct real association and prevents
claiming that a successful bearing test proves a left-turn road. Replacing this
prior with an explicitly justified local-bearing model, or proving it is adequate
from real observations, is a separate necessary design decision. No relaxed
angle gate or manual simulator bearing is a fallback here.

## Counterexamples written; no execution yet

`test_junction_association.py` uses the real core's reference/route data classes
and wholly synthetic coordinates unrelated to Unity or the evaluation road.
It covers five straight junctions on a four-corner reference, reversed traversal,
all 2N candidates, uniform official scaling, unchanged reference/old failure,
same outline with moved openings, equal-degree exact and near symmetry, missing
or extra events, no positive branch evidence, repeated frames/IDs, stale capture,
capture-time pose mismatch, epoch/calibration/frame mismatch, incomplete lap,
nonfinite data, uncertain/inconsistent geometry, untrusted or ambiguous projection,
duplicate stations, impossible observation order, wrong branch direction, a
bearing filter hiding a competitor, and individual-node error vs low average.

The tests intentionally do not claim to detect a real junction from an image.
No test may be cited as passing until it is actually executed in an authorized
CPU window. A later portable command, with the existing core package parent and
this directory on `PYTHONPATH`, is:

```text
python -m unittest discover -s <official_association-directory> -p test_junction_association.py -v
```

## Remaining integration and acceptance work

First verify that real saved full masks, original K/R/t and correctly paired
capture-time odom contain positive branch geometry at enough distinct stations.
Do not promote the current observer's unrecognized geometric candidates.
Then review event tracking, uncertainty, duplicate suppression, and evidence
transport; inspect every ambiguity rejection instead of changing thresholds to
force the five expected nodes. The baseline whole-route failure stays visible.

Even a unique association does not establish which official node corresponds
to a signal, whether an entry is a legal/observed left branch, whether internal
chord geometry follows the road, or whether the original DetourTracker can
traverse a right-angle corner. Those require separate measured geometry,
continuous preview, command ownership/freshness, and real inner/outer closed-loop
entry/traverse/exit evidence. Until then, the candidate stays isolated and the
existing red stop, unknown hold, green restart, and emergency guards remain
unchanged.
