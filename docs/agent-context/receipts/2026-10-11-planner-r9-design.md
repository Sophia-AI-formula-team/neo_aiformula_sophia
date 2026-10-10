# R9 predeclared development plan — no runtime acceptance yet

Base code is R8 RC1 `ae7e27b909319a49285d50b3e4b3b7aa4789ddaf`.
Role: planner, same ongoing session009. User authorizes fixing the neo mapping
packages and simulator adaptation, not changing vehicle/physics/camera/LYA parameters.

Verdict: REPAIR. The measured-geometry data model is the first bounded correction.
It is not permission to weaken a failed localization gate or skip failed frames.

## R9a: physical geometry separate from confirmation bins

The `_gnss` teach engine explicitly uses measured geometry. Existing key-only
SparseConsensusMap callers keep their existing semantics; this is not a silent
change to old VectorNav packages. Every committed frame contributes one cell
mean and one vote per occupied cell, irrespective of pixel count. The cell
representative is the equally weighted mean of those frame means, not the
rounded bin center. No extra points are introduced for rank/PCA support.

The existing votes, reliable-vote qualification, TTL, capacity limits, bootstrap
100-cell requirement, scan/anchor voxel sizes, finite-support test and all
quality/time/stop gates remain unchanged. The 100-cell test is before the
existing 0.08 m anchor voxelization; fewer post-voxel points are not by themselves
a violation of that pre-voxel gate.

Prepare finite immutable per-frame geometry before the commit guard. Rejected
matches, expired jobs, capacity rejection and failed matcher commits cannot
change the persistent geometry. TTL removes unconfirmed measurement state.
Candidate/confirmed count remains bounded by the existing limits; two float64
means per candidate add at most 3.2 MB numerical payload at 200000 entries,
plus explicitly bounded Python container overhead. Pixel clouds are not kept
across frames. Snapshot/frozen route/repeat use the same measured consensus.

Do not add eight-point extrema, expand endpoints, reassign a rejected point to
a different stripe, or change bundle schema in this step. Such changes would
need a separate justified design and evidence.

## R9b: startup gravity/gyro model consistency

The raw-IMU opt-in path gains a stricter engineering direction-consistency
budget of 2.5e-5 rad. This is NOT measured vehicle noise or proof of absolute
tilt accuracy. Its predeclared design rationale is an ideal level-plane envelope
with camera height at least 0.5 m and range at most the existing 30 m limit:
`(h*h + R*R) * tan(theta) / (h - R*tan(theta))` is about 0.04508 m
at h=0.5, R=30, theta=2.5e-5, below half the existing 0.1 m grid.
The module does not know camera height; outside that design envelope no such
metric bound is claimed. No camera height or range is changed to meet it.

Keep the existing 1 s / 20 minimum / 512 maximum sample window and all existing
clock, norm and dispersion gates. Divide qualified samples into four disjoint
source-time blocks, with at least two samples each. Estimate a fixed gyro bias
from the first three blocks; reserve the fourth for validation, not refitting.
Transport measured specific-force directions to the actual final source time
using this candidate bias and past-held gyro rates. Check block-center gravity
directions and held-out residual against the declared angular budget, and check
training/held-out transverse bias discrepancy times actual duration against
the same budget. Reject an inconsistent candidate and continue waiting; do not
backfill earlier poses. Once ready, keep the original gyro-only propagation
and never correct moving attitude with acceleration.

Constant horizontal acceleration/accelerometer bias and a quiet constant yaw
remain unobservable under these inputs. Applied zero commands and zero wheel
readings are evidence, not a physical stillness certificate. The raw-IMU path
remains explicit opt-in for real ROS deployment.

## Validation order

1. Focused independent geometry/transaction and rotation oracle tests.
2. Existing four deployed-camera synthetic circle cases, with identical masks,
   raw motion, timing provenance and quality gates. These are development cases.
3. Predeclared additional geometries, not tuned after seeing their results.
4. Causal recorded-prefix candidate evaluation and full regression.
5. Only then a new actual scene run with the original parameter preservation
   audit, unique logs, past-only consensus video and explicit TEACH/REPEAT result.

A complete two-lap run has not occurred. A passing build or high match acceptance
rate cannot replace positional quality or actual runtime evidence.
