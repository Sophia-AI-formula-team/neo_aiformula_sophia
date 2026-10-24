# R8 RC1 publication and CI readback — still not a lap pass

Published code: `ae7e27b909319a49285d50b3e4b3b7aa4789ddaf`.
Branch: `repair/lane-keyframe-transaction-20261011`.
Annotated tag: `lane-mapping-20261011-r8-rc1`; tag object
`f80b78bd71770af6c9c0272ac626cba5dc2db85b`.
The actual push URL and immutable repository ID 1303517209 were checked before
publication. No main, legacy repository, forced update or tag replacement.

All eight branch/tag runs now report completed/success for this exact commit:

| Workflow | Branch run | Tag run |
| --- | --- | --- |
| Exact native graph | [38089499015](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38089499015) | [38089499066](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38089499066) |
| Endpoint GNSS / Foxy | [38089499002](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38089499002) | [38089499070](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38089499070) |
| Reference/fixed / Foxy | [38089499049](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38089499049) | [38089499105](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38089499105) |
| Handoff integrity | [38089499011](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38089499011) | [38089499064](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38089499064) |

This supersedes only the pending CI/publication status in the immutable
[R8 checkpoint receipt](2026-10-11-planner-r8-rc1-checkpoint.md).
It does not supersede its failures: trial27 stopped near 29 m, no first lap,
no frozen driving route and no second lap. CI is not simulator or vehicle acceptance.

The exact failed prefix was reproduced: 107/107 mapping decisions and all
non-timing fields match. Finite-support exclusions are not simply a PCA-radius
bug: all 83 rejected candidate points exceed their corresponding old connected
component's tangent range. Ten alternative references belong to different
stripes and are 0.441–0.647 m away; reassociation is not justified. Even without
the finite filter, the active reference's weighted overlap is 29/57, below 0.55.

A separate geometry-only audit of the independent deployed-camera circle
reproduced the first visual yaw gradient of approximately +/-2.55 mrad from
committed grid-center geometry. Measured past positions reduced that gradient,
but are not yet a validated matcher replacement. No quality, timing, overlap,
rank, bootstrap or stopping gate was relaxed. Next development must separate
confirmation votes from measured geometry and test remaining source/target
patch bias, while fixing the raw-IMU startup model independently.
