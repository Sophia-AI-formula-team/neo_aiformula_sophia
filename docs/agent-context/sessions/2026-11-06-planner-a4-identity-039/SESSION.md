# A4 Identity And Mode Clarification

## Objective and scope

Name the current perception release A4 and explain single-frame versus temporal
operation in Markdown. Continue the user-authorized publication and merge.
Scope: package docs/metadata/display labels/tests, CI label and handoff records.
No training, model/weight changes, real-camera integration or vehicle action.

## Input handoff

Task 2026-11-06-a4-identity, issue #35.
Read root and package instructions, context/status/protocol and current docs.
The previous release is a3-v76; its source checkpoint identity remains historical.

## Source and working tree

Base 254ba4ef8e6c850ec7602a87eabb7e42361c364e.
Started from clean main in the owned publication clone.
Branch docs/a4-release-identity; unrelated dirty clone and all old tags untouched.

## Work performed

Added MODE_GUIDE_ZH and explicit publication-versus-activation paragraphs in
README/DEPLOYMENT/handoff/TEMPORAL/context. Single-frame processes every image,
like V14 did, but uses new base weights. Unity supplied calibrated camera poses;
the ROS image-only wrapper does not. Real-camera temporal integration is unfinished.
Set release A4, package and ROS metadata 4.0.0, planned tag a4; preserve existing
folder/distribution/import/ROS names. Updated logs and checksum-tool identity.
Added release consistency test; no serialized checkpoint fields were rewritten.

## Evidence and result

See evidence/identity_checks.json and evidence/package_smoke.json.
Baseline40 and final41 tests passed. Weight checksum unchanged; git diff confirms
weight plus eight inference-source files unchanged. Isolated 4.0.0 wheel CUDA
inference, image CLI and calibrated three-frame sequence CLI passed.
No new 900-frame experiment was run: the original source/export comparison
remains valid for the unchanged model and is retained under its original record.

## Risks and blockers

A4 is a release rename, not new training or automatic temporal activation.
Real-camera calibration/synchronization and Orin timing remain issue #34.
Requested documentation reviewer route was unavailable; supervisor reviewed docs.
No live ROS/DDS or vehicle test was run.

## Next agent actions

Review the scoped branch and checks; merge and create a4 without moving old tags.
Use the new Chinese mode guide when handing off. Complete calibrated temporal
inputs separately; do not describe published API as active vehicle temporal fusion.
