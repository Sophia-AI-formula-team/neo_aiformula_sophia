# Agent session: 2026-10-12-planner-corner-side-r22-019

Input handoff: 2026-09-28-planner-neo-004

Reviewed development checkpoint. Result PARTIAL: unit regression passed; runtime, geometry and two-lap acceptance remain unmet. This record is not permission to drive.

## Objective and scope

Continue the actual new-scene TEACH lap then frozen-route REPEAT lap, preserving
vehicle/physics/sim/camera/LYA speed and all original runtime/geometry gates.
No actual new drive has occurred. This is an incomplete task outcome.

## Input handoff

Planner retains neo-004, current root instructions and same GNSS/causal rules.
Read R21 session018 and reviewed independent reports; physical line identity
is not established merely by close angles or non-cooccurrence of graph edges.

## Source and working tree

R21 published79ad3438c4d90d5436b73151796fff1febb4a908 on
repair/lane-keyframe-transaction-20261011, annotated lane-mapping-20261011-r21-rc1.
Push URL and neo ID1303517209 verified; atomic push succeeded. Session018 ZIP
12663 bytes, SHA72bbd526511f33c74e7657f59c8f215fb57e6bf82fccfdbc5d535ad778c36b9c.
Only the two preexisting untracked curve-fit prototype files remained after
commit. Do not edit/stage them. Root owns main source changes.

## Work performed

R21 one-shot target profile at seq163 retained original40/80 gates and exact
first12 non-timing results/priors. Root read the complete driver/PLAN before
authorizing check-only then execute-once; no retry. Target reached but matching
crossed40ms, so no prepare/current_union data were collected at that target.
Completed D-only probes investigated complete corner feasible regions and
independently counted continuation support. Root read continuation script and
requested correction of omitted raw/position reliable flags before execution.
Corner fixed-normal side-region guard rejected only two of three known bad
endpoints: face351 still passed. It was not installed as a repair.
Original query29/primitive208 was independently covered by both actual
continuations in six frames/six reliable frames each; full primitive shared
owned-quad support was only72.0931%, so no whole-owner branch waiver is allowed.

REPAIR decision: preserve original primitive coordinates/order/global nearest;
carry per-continuation actual own-face 2D footprint witnesses and export only
independently confirmed common alpha permission intervals. Every signature,
including missing/FOV neighbours, remains inventoried; missing or under-voted
hypotheses veto. Store latest k frames union latest r reliable-containing frames
(original5+1, at most6 retained), selected independently of query/model. This
is a subset of real past evidence at the SAME geometry, not a temporally
monotone certificate. Reclip for current model, do not reuse old tangent spans.
Quad/frame/signature/permission storage all counts toward unchanged200000.

Root implements continuation_support.py, observation footprint transport,
immutable consensus deltas, v4 reference and post-nearest alpha test. Agents
own only new independent tests and v4 bundle serialization. No vehicle/sim,
LYA, time/overlap/normal/vote gates changed. Existing unrelated untracked
curve-fit prototypes remain excluded.

## Evidence and result

R21 profile13 attempted/12 accepted, target match40.1929ms failed; its engine
40.5554ms must NOT be compared as an improvement over strict87.1215ms, because
prepare never ran. First12 unprofiled extraction144.5056-159.4021ms,
engine30.5524-69.1949ms. All12 integrity checks true. RESULT_01.md SHA
b1c6be267fd4b60960a527cfcc4a882221e62e2e29987e1cfcdb48c9212900af.
R21 strict/full evidence remains in sealed018, unmodified.
R21 CI terminal6/8 workflows14/16 jobs; both GNSS DDS failed7/9 scenarios after
processing deadlines81.304508-154.611296ms, worker227.049540-439.427878ms.
All18 scenarios ended shutdown HOLD with zero private output; no physical test.
Reviewed R21 CI receipt is published separately; raw diagnostic artifacts remain local.

Root targeted regression command: python -B -m pytest -q -p no:cacheprovider
test_boundary_certified_domains.py test_boundary_local_snapshot.py
test_boundary_footprint_domains.py test_boundary_association.py (full paths under
reference package test directory). Exit0,97 passed in0.71s. This is an
intermediate source state, not final/full verification or a timing benchmark.

## Risks and blockers

### R22 execution addendum

Implementation now includes immutable original W/B footprint transport,
per-hypothesis past-frame witnesses, v4 optional finite alpha permissions,
strict bundle roundtrip, and an exact competing straight-direction witness
for endpoints of actual selected cuts. The latter runs after every proposal;
it changes admission only, not the proposed root/normal or raw mass. It does
not prove a unique physical side or classify unsplit coarse-pixel corners.

An independent capacity review found a real prospective-publication defect:
59 stored units could commit and only fail at snapshot after one permission.
The fix accounts for a cached conservative permission bound transactionally;
when it touches the unchanged cap, an unpublished prospective view counts
actual exported intervals. A loose bound cannot reject a valid update. Tests
retain cap59 rejection before commit and cap60 exact success, without eviction.

Known failures retained: support union initially attempted ambiguous NumPy-row
ordering (71 pass/2 fail, then 73 pass and later90 pass); original four-file
regression first had import setup errors, then43 fail/141 pass from legacy
empty sidecar/stats compatibility, then184 pass after correction. Exact
corner helper initially54 pass/1 fail for identical signed cones; exact
collinearity/positive-dot exclusion fixed it, then55 pass. None was hidden.

Current full first run:3907 pass/1 fail/1 Windows-only Linux-process skip.
Failure was a legacy test clearing model arrays without the new footprint
sidecar; production correctly rejected that inconsistent object. Test fixture
now explicitly clears the new sidecar and separately asserts the original
invalid combination still raises. Full second run:3908 passed/1 target-Linux skip/0 failed/0 errors; pytest exit0, JUnit108.328s, process109.4956547s. Native8 real calls pass; all190 source/config/binary identities unchanged.
Original first JUnit/logs remain retained.

Strict trial28 recorded prefix:12 attempted/11 accepted, first failure seq162,
engine100.990600ms>80ms. Candidate matcher28.6062ms<40ms, overlap.807971/rank2,
but deadline-rejected frame was not committed. All25 source/causal/config
checks passed; process exit0 means a report was generated, not111-frame pass.
No performance improvement follows from a different single historical run.

Fixed22-mask geometry audit: original raw fields and proposed roots/normals
unchanged. Uniquely scored roots9987->9940;47 removed, including5 of8 errors
over30deg and42 at/below30deg. Remaining3 all on coarse arc8.0 phase.35.
Distance RMSE8.2070765->8.2153984mm (not improved), max43.7946934mm unchanged.
Normal max89.8361165->60.0233468deg; P95 effectively3.8583928deg unchanged.
Three specified bad endpoints all vetoed; their exact alternative-direction
witnesses independently satisfy original7 labels/strictseed/two signed cones.
Residual arc has no actual selected cut. Its face0 original quadratic violates
four original W/B support labels despite passing its own seed bracket. The
other two have not yet had that whole-window claim independently established.

Saved synthetic continuation differential: only first12 past commits through
4.11s, no future query vote. New coverage2/112, lost0; globally nearest target,
owner and normal unchanged. Original owner120 query29 is certified by BOTH
actual hypotheses with5 distinct past/reliable frames each. Primitive common
alpha measure.718758818974641 in12 pieces, not a whole-owner waiver. Synthetic
query evidence is not real-time matching or a lap.

Adapter provenance loads14/16 actual modules (normal/raw-IMU), supports v4 with
both markers, and excludes unrelated curve-fit prototypes. Final adapter
suite128 pass/3 named historical-native identity tests deselected; those same
3 fail against frozen pre-change adapter bytes as well. Preserve initial
117 pass/3 fail and historical3-fail control. No missing binary was replaced.

Live read-only Unity snapshot confirmed original new validation scene clean
Edit, disarmed, requested V/W0, pending0, original scene SHA unchanged. No
Initialize, Play, commands, scene save or Asset refresh was used. The other
UI task reports Unity handed back; the semantic task's successful videos are
unrelated and are not adopted as this task's result. Vehicle/simulation/
camera/LYA config and all original gates remain unchanged.

Native ninth-API work is D-only reviewed preparation, not applied, compiled
or installed. Existing native8 C/binary identity remains. Its intended exact
current-frame union acceleration is not a measured end-to-end improvement.

No complete TEACH/frozen route/REPEAT or new video. Full200ms request is not
validated. Pending geometry ideas can be falsified; do not substitute reduced
admission for improved accuracy. The R21 staged check had one blank-at-EOF
warning in the corner test; exact tested source bytes were preserved. Check
with only blank-at-EOF excluded passed. Initial session018 packaging validation
failed combined headings; corrected before sealing, no source/test alteration.

## Next agent actions

Repair whole-selected-window W/B model consistency and measure coverage, including
all original circle/coarse-arc negative controls. Continue exact hot-path runtime
work: native9 is preparation only; full200ms needs extraction/engine/transport
measurement, not isolated kernel subtraction. Preserve all failures and tags.
Fresh Unity ownership/protected-state readback required before any live run.
