# 2026-10-11-planner-keyframe-r1

- Recorded2026-10-10T15:33Z (2026-10-11 JST), planner; RESULT_READY for the core
  repair checkpoint only, partial against the complete two-lap request.
- Input: neo-004 handoff at [baseline85ddb702](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/85ddb70209e6c346c422f9ae8b2a88cb49da72e0/docs/agent-context/handoffs/2026-09-28-planner-neo-004.md).
- Exact reviewed code/evidence commit:
  [422cb4be820c046a34f916f94759487253ee2004](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/422cb4be820c046a34f916f94759487253ee2004).
- Branch: repair/lane-keyframe-transaction-20261011. Annotated version tag
  [lane-mapping-20261011-r1](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/tree/lane-mapping-20261011-r1)
  points to422cb4b; before tag lane-mapping-20261011-baseline points to85ddb702.
  Both tags and branch were read back from repo ID1303517209. No main merge,
  force-push, old-repository publication or physical vehicle output.
- Session: [2026-10-11-planner-keyframe-005](../sessions/2026-10-11-planner-keyframe-005/SESSION.md).
  [ZIP](../bundles/2026-10-11-planner-keyframe-005.zip),428472 bytes,
  SHA256523e4afb5e517cd5eea960be18bcc4817292c44026ba05b8616469d4405b8cfb.
- Package contains exact observed mask/wheel/gyro inputs, old failure and fixed
  prefix reports. No raw bag, world-truth trajectory, personal path or GNSS.
  Standalone replay tool is in tools/, outside the archive's allowed-data rules.
- Final local lane657 pass /1 target-Linux skip; simulator adapter115/115;
  helper5/5. After sealing, handoff32 pass /1 Windows symlink-privilege skip.
  Git staging initially normalized raw evidence JSON line endings; a narrow
  -text attribute plus re-add preserved the original bytes. All6 sealed files
  match the staged Git blobs. No sealed record was rewritten. One extra blank
  EOF in SESSION.md was retained as cosmetic, not concealed as a clean check.
- Immutable-tag replay also passes32/32, recomputing566 motion samples with no
  reset/input pose. Local report trial13_tag_r1_prefix.json has SHA256
  51d57bd84a9964cc68cf22adf7ec553994c15139fc99193b6fd80602cff292f9.

## Remote verification for exact R1

All six checks succeeded. Branch and tag runs use the same commit and are
duplicates, not six independent algorithm validations.

- [Lane Foxy branch](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38063701540)
  / [tag](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38063701460):
  4 packages built;354 unit tests,0 failures/errors/skips;4 Header tests;
  63 DDS checks;26 real neo LYA/supervisor checks;3 synthetic control demos.
- [Endpoint Foxy branch](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38063701587)
  / [tag](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38063701534):
  5 packages built;658 unit tests,0 failures/errors/skips;4 Header tests;
  9 stationary DDS cases/151 checks passed. This includes fault windows, not
  a driven lap, GNSS map fusion or physical endpoint precision.
- [Handoff branch](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38063701633)
  / [tag](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38063701575):
  Linux/Windows with Python3.8/3.13,33 tests per environment,all OK.

## Unmet criteria and next ownership

The recorded-prefix failure is fixed; **neither a complete teach lap nor a
fixed-route second lap has passed in this task**. ROS transport CI and another
task's successful ordinary YOLOP/LYA lap cannot substitute for that evidence.
Unity is still exclusively leased to that separate task for safe UI cleanup.
This planner has not operated the editor or changed vehicle/physics parameters
during the lease. Original failed-trial videos and large logs remain local.

Next owner: this planner, after explicit clean editor release. Verify scene,
runtime parameters, control ownership and source hashes; run new isolated trials
and save logs/video. Further repairs receive new commits/tags and new records;
never move R1 or overwrite the sealed checkpoint. This receipt is not physical
vehicle permission and does not automatically change the field handoff baseline.
