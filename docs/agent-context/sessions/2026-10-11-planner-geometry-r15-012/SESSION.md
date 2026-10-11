# R15 finite observed-face geometry checkpoint

Role: planner. Consumed handoff: `2026-09-28-planner-neo-004`.
Base commit: `cde9adbb598174904f84dd9829c51453d1bd0d66`.
Predecessor session011 is sealed and unchanged. This is **partial failed
development**, not a new field baseline or permission to drive.

## Objective and scope

Repair observed boundary geometry without changing vehicle, simulation or LYA
parameters, future-data rules, confirmation counts or runtime safety gates.
The requested two-lap closed-loop result remains unmet at this checkpoint.

## Input handoff

Consumed `2026-09-28-planner-neo-004`; read the root agent instructions,
handoff protocol, current status and sealed predecessor session011. Retain
failed evidence and keep post-hoc ground truth outside estimator inputs.

## Source and working tree

Branch `repair/lane-keyframe-transaction-20261011`, base commit above.
The tree is intentionally dirty with the extractor, two geometry test files,
this session, STATUS and the exact evidence text attribute. The two unused QP
files remain unrelated untracked experiments and are excluded from publication.
No dependencies were installed or changed.

## Work performed

Only the shared boundary extractor and its tests change. A fitted quadratic was
previously restricted to the centres of its selected pixel faces. Its legitimate
observation domain is the original finite faces, clipped to the SAME signed arc
radius. R15 derives that domain without adding fit samples, changing coefficients,
using a W/B endpoint hull, connecting gaps or extending a contour.

This explicitly changes geometric admission: evaluation outside the fit-sample
centre hull is now possible INSIDE the observed quantized-face domain. It is not
a claim of unchanged acceptance, recovered subpixel truth, or pure interpolation.
Disconnected or invalid inputs retain the old domain. Original PCA, unique
in-bracket root, J4, 5+1 confirmation, .55 overlap and 40/80 ms limits remain.

The old arithmetic-equivalence test now shares this new domain with its literal
old numerical oracle. Independent new scalar domain tests validate the changed
field; the equivalence wrapper alone is not a domain oracle. The unused
quantized-curve QP files remain untracked and are NOT part of this checkpoint.

## Evidence and result

- Focused native checks: 363 passed. Full reference/GNSS core: 2617 passed,
  one Linux-only test skipped on Windows, zero failures/errors. 155 source
  identities unchanged; seven native APIs were actually exercised.
- The one actual frame256 extraction adds 33 model-valid faces and loses none.
  Raw geometry, fitted normals/coefficients/scales/counts and old valid outputs
  remain byte-exact. Assigned mass rises 42.0833 to 44.0833; denominator50.0833
  is unchanged. A prior pin-mismatch stop did no extraction and is retained.
- Original-budget recorded-input candidate: 105/106 accepted, seq256 still
  `match_insufficient_overlap`. Fresh overlap .5207986689, active .4209650582;
  measured engine57.7192 ms for that failed attempt. Rejection did not commit.
- Four fixed full-mask development cases: 0/4 pass, each seven accepted out of
  509 planned, first failure at frame7. The explicit 2000 ms geometry-test
  budget is NOT runtime evidence. No held-out acceptance is claimed.
- One separate low-overhead diagnostic replay reproduces all 106 non-timing
  results and retains ten unchanged source identities. No timing-success retry.
- R14's completed adapter suite is independently recounted as 339 pass, 196
  sources unchanged. Its ORIGINAL 878-file protection check remains failed:
  16 changed peer UI paths. Reconciliation attributes those changes; it does
  not reset the baseline or convert aggregate false into true.

Reproducible core command from repository root: set PYTHONPATH to the four
control package roots, require the native extension, then run
`python -B -m pytest workspace/src/aiformula/control/lane_mapping_lya_reference/test workspace/src/aiformula/control/lane_mapping_fixed_gnss/test -q -ra -p no:cacheprovider --ignore=workspace/src/aiformula/control/lane_mapping_lya_reference/test/test_quantized_curve_fit.py`.
Exact local commands, source manifests, retained failures and output hashes are
referenced in the reviewed evidence index. Private paths and raw data are not
published.

## Accuracy: post-hoc only

Exactly 105 common accepted frames151-255, fixed original first-TEACH origin,
no spatial/temporal alignment, interpolation, extrapolation or sensor-lag
correction. GT is used only AFTER saved estimator-output/input checks.

| Model | Lateral RMSE m | Yaw RMSE rad | Longitudinal RMSE m |
| --- | ---: | ---: | ---: |
| raw motion | .004622761876 | .000621861705 | .303231163957 |
| current R14 a0e34451 | .010140481993 | .001099477226 | .303231037310 |
| R15 a42f4fbb | .009606833510 | .001062790028 | .303231066596 |

R15 improves modestly over this R14, but lateral/yaw remain WORSE than raw.
Older eef-source R14 numbers are not substituted. This reused 11.45 s recording
is not a new closed loop, full lap or independent validation.

## Unity ownership, parameters and safe-state evidence

User authorized coordinating the peer Unity window. Peer handed it back; root
made fresh read-only get-state/Snapshot calls at 2026-10-11T09:14:31Z. Against R10
and trial29 preflight, all non-clock fields are exact: 116 parameters,
175 components, three bodies, globals and spawn data. Scene SHA remains
`542b69e811b5eca13456a7d39f32c8b47290a2f7211865337497242bd5d3aa98`.

Observed clean Edit/idle, zero requested commands and unarmed. **EmergencyStop
and brake are false**, so this is not a claimed latched-stop check. No Initialize,
scene save, Play transition, backend start or driving took place in R15.
No vehicle/physics/camera/LYA parameter was changed. Later driving requires a
fresh runtime preflight; saved sequential snapshots are not future guarantees.

## Risks and blockers

Per-cell scan representative mass relocation is a genuine finite-end geometry
error (independent tests show false credit and false loss), but correcting it
alone gives actual fresh .522795 and synthetic .491736: still below .55.
Do not inflate unweighted point counts/RMS statistics by simply deaggregating.

Saved cross-queries instead identify one .5-mass loss in the PAST reference,
independent of current-pose changes. Investigate grid-cell identity fragmentation
and actual historical adjacency before changing association. Do not grant votes
retroactively, infer unseen support, merge separate contours or lower limits.

Still missing: successful new-scene first TEACH lap, frozen validated route,
second REPEAT lap, their error metrics and requested two-lap video. Last actual
drive remains failed trial28. This checkpoint is not the requested completion.

## Next agent actions

First retain this failed R15 checkpoint. Then independently validate a causal,
topology-witnessed adjacent-cell identity association: finite raw-face overlap,
an independently matched neighbour, unique compatible historical directed edge,
one current vote only and atomic cell-index migration. Do not merge historical
votes or turn identity witnesses into localization support. Re-run adversarial
tests and the unchanged development cases before a fresh Unity driving trial.

Canonical destination only: Sophia-AI-formula-team/neo_aiformula_sophia,
immutable repository ID1303517209, existing repair branch. Intended version
`lane-mapping-20261011-r15-rc1`; no main merge, old-repo write or force push.
R14's eight exact-commit CI runs succeeded and are recorded in the evidence.
They do not cover dirty R15; R15 CI must be checked after its actual publication.

The small evidence index contains sanitized summaries/hashes, not masks, bags,
GNSS coordinates, binaries, private scene arrays or secrets. Original diagnostic
data and rejected attempts remain local. Session012 is sealed only after review
and packing; later corrections must use a new record.

