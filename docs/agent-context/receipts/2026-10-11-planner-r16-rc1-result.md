# R16 RC1 development result

- Role: planner; RESULT_READY for analysis, partial result; not vehicle release.
- Input: `2026-09-28-planner-neo-004`. Code checkpoint:
  [c7e316a8](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/c7e316a8b9e52afb87f7d00db564cc670295b7b4),
  `lane-mapping-20261011-r16-rc1`, branch `repair/lane-keyframe-transaction-20261011`.
- Session: [013](../sessions/2026-10-11-planner-geometry-r16-013/SESSION.md).
  [ZIP](../bundles/2026-10-11-planner-geometry-r16-013.zip), 33630 bytes, SHA256
  `a6e706012ba56a970beae50943377b2f2ee7a043e1fe43940fb766fef2e45be8`.
- Final offline core: 2697 passed / 1 Linux-only skip; 157 source hashes unchanged.
  The unrelated unintegrated curve-fit prototype is explicitly excluded.
- Subsequent remote readback: 8/8 workflows, 16/16 jobs, all steps successful.
  Native runs: 38131922333 / 38131922331; handoff: 38131922309 / 38131922316;
  Lane Foxy: 38131922301 / 38131922270; GNSS Foxy: 38131922302 / 38131922317.
  All have the exact checkpoint SHA. Eight unexpired artifact digests retained;
  this readback did not download XML, so no exact remote test count is inferred.
- CI audit retained locally as `r16_publication_01/ci_readback_01.json`, SHA256
  `613d6c4bfa3b79e02df0506c911d41395c0baaa52019463be28428d6c68aab15`.
- Mandatory failures: the original recorded input remains 105/106 accepted;
  seq256 fresh overlap0.530782 is below0.55. All four development cases fail.
  Profiling and the fixed-prediction support counterfactual do not repair these.
- No new Unity drive was run for R16. Last actual drive remains failed trial28;
  no complete TEACH lap, frozen route or REPEAT lap exists. No vehicle/simulation,
  speed, safety gate or GNSS mapping policy changed. No current live state is
  asserted by these offline and remote evidence reads.
- Next owner: planner, implementing explicit pointwise-confirmed finite domains
  under session014. Retain failures, validate numerics/transactions/bundle identity,
  then run the original quality and recorded candidates before a fresh preflight.
  Do not overwrite sealed013 or treat this checkpoint as deployment acceptance.
