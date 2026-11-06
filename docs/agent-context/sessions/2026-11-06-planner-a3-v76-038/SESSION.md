# A3 V76 Deployment Package

## Objective and scope

User request: select the current model as stable, package it in neo, write deployment
and handoff instructions, and merge. Task: 2026-11-06-a3-v76-release; issue #32.
Allowed scope: A3 deployment directory, scoped CI workflow and these handoff records.
No vehicle motion, launcher switch, controller changes or new training.

## Input handoff

Read root AGENTS, AGENT_CONTEXT, STATUS, PROTOCOL; package README/context/handoff,
manifest and AGENTS; selected V76 source runtime, geometry, model and verification.
The current main includes the newer independent source-Header repair.

## Source and working tree

Base: 2ee51daccf2c8afd1448ea7eae82160cc44f6d75.
Branch: feat/a3-v76-deployment, separate clean clone before this task.
Existing dirty publication checkout was left untouched.
Source checkpoint SHA256: 1fc29a6cd4bb5de3390570ce805adca5167303176cecdd867eddb6d36752f1ca.
No licenses or dependency versions changed. Build dependencies were isolated.

## Work performed

2026-11-06: baseline 26 tests; replace obsolete Faster R-CNN runtime with selected
shared MobileNet/FCOS model and 49-parameter probability gate; export one model-only
checkpoint; preserve raw masks, independent object output and source Headers.
Add explicit predict_frame geometry API and sequence CLI, with reset/status tests.
ROS wrapper remains explicitly image-only; no uncalibrated motion is inferred.
Update package docs/manifest/verification and scoped Linux CPU CI.
Train/resume state, datasets and private source paths are absent from the package.

## Evidence and result

See evidence/release_checks.json.
Package tests: 40 passed. Legacy Header regression: 4 passed / 12 subtests.
900 native 1280x720 held-out frames / 36 clips: exact source/export lane logits,
object boxes/labels/scores and previous raw probability; one encoder per frame.
Isolated wheel CUDA inference and single-image/three-frame sequence CLIs passed.
Export SHA256: b4d3556ded29ada232682b607a11296d723ffe74814a85cc4f2bcbf573e5b1e3.
Weight: 13,916,097 bytes, 341 tensors, 3,403,740 parameters.

## Risks and blockers

V76 gap gain +0.448pp is below its +2pp experiment gate; user stable selection is
recorded independently, not a rewritten quality pass. Objects remain the V73
parent's outputs; no new V14 per-class comparison is claimed.
No calibrated real-camera projection producer is included. ROS remains single-frame
until its owner integrates timestamp-aligned geometry through the explicit API.
No live ROS/DDS, Orin timing, TensorRT or vehicle test was run.
The first baseline command failed to create a missing local temp parent; after
creating it, all 26 tests passed. Initial pip build entry was unavailable in the
shared environment; isolated uv build/install succeeded without changing it.
Delegated read-only audit returned no result and was stopped; supervisor review
and executable parity checks were completed.

## Next agent actions

Review the branch/CI, merge only its scoped changes, verify remote main and tag
a3-v76. Keep a3-v14/a3-v6 immutable. Start deployment in isolated shadow topics.
Validate real calibration and target-device latency before changing live perception.
Rollback the whole tagged package and weight, not only the checkpoint.

