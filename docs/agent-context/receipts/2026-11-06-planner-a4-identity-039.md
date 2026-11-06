# A4 Identity Release Review

- Date/role: 2026-11-06 UTC, planner; RESULT_READY for user-authorized merge.
- Input task: [A4 identity task](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/blob/ca476d46755f3cbcf8c7b1a6bf49243a10d88977/docs/agent-context/handoffs/2026-11-06-a4-identity.md).
- Base: 254ba4ef8e6c850ec7602a87eabb7e42361c364e; branch docs/a4-release-identity.
- Reviewed/uploaded evidence commit: [ca476d46755f3cbcf8c7b1a6bf49243a10d88977](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/ca476d46755f3cbcf8c7b1a6bf49243a10d88977), read back from GitHub.
- [Session039](../sessions/2026-11-06-planner-a4-identity-039/SESSION.md),
  [sealed ZIP](../bundles/2026-11-06-planner-a4-identity-039.zip), 5,582 bytes;
  SHA256 d37a001724b41d848f6e6589deb18b078dd4b129b9cba826eb4fbd4ab67daaf3.
- A4 / 4.0.0 / planned tag a4 consistently identify this release. The model,
  weight and eight numerical inference sources are unchanged from a3-v76.
  Existing folder/distribution/Python/ROS names remain compatible.
- MODE_GUIDE_ZH.md and deployment/handoff docs explain single-frame operation,
  the calibrated temporal API, Unity pose inputs and unfinished real-camera
  integration. Naming the release A4 does not enable temporal fusion in ROS.
- Baseline40 and final41 package tests passed; context suite32 passed and one
  Windows symlink-permission skip. pytest and unittest commands exited0.
  Weight verification, isolated4.0.0 wheel CUDA inference and both image and
  calibrated sequence CLIs passed. git diff --check passed; four indexed session
  files match the sealed ZIP byte-for-byte.
- Not rerun: the original900-frame comparison, because weight and numerical
  inference are unchanged. It remains recorded as prior V76 evidence.
- No failed final checks. Writing reviewer route was unavailable; supervisor
  reviewed the complete change. No new training, live ROS/DDS, Orin or vehicle
  test. No hardware command issued; launch/control and historical tags untouched.
- Next owner: planner merges after checks and verifies new immutable a4 tag.
  Follow-up issue #34 handles calibrated real-camera temporal inputs and Orin.
  This receipt does not accept vehicle operation; final merge baseline and CI
  outcome are recorded in the pull request and local project handoff.
