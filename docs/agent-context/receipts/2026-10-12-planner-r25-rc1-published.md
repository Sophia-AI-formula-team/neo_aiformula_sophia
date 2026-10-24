# R25 development checkpoint published — PARTIAL

Planner; input/consumed handoff `2026-09-28-planner-neo-004`.
Disposition: `RESULT_READY` for analysis, not driving acceptance.

- Canonical repository ID checked before push: `1303517209`.
- Source commit: `40852db8392f3b66a6f37c92d1b2d649df8adb1d`.
- Branch: `repair/lane-keyframe-transaction-20261011`.
- Annotated tag: `lane-mapping-20261012-r25-rc1`.
- Tag object: `2d7623a5413b03145e92697a723b04968ed096d9`.
- Branch/tag push exited zero; GitHub API readback confirms branch SHA and
  tag object. No main, legacy, force push or release acceptance.
- Session022 bundle SHA256:
  `e8f735e66fb4b3b0596c6aa2809b1e8fa9a0eeb56494470b88bde2138af5b2be`.
- Rejected R24 session021 bundle SHA256:
  `f3ba50ebbd5a9738eaf62dfa4a6416f87ad2980e0991950c0afee1df7154eeb2`.

## Measured outcome

Windows full regression: 4152 passed, one Linux-only process-group skip.
All 192 source/test/binary pins unchanged. The original recorded prefix still
fails at seq162: 11 accepted of 12 attempted, engine117.2164ms >80ms,
matcher39.5834ms <40ms. Failed frame did not commit. No retry-to-pass.
Exported non-timing R23/R25 results are exact; this is not a speedup or
geometry improvement claim. R24 lost 2327 evaluable roots and was withdrawn.
Detailed evidence is in sessions021/022; native9 remains source-identical.

No new Unity lap, frozen route, second lap, ground-truth tracking metric or
video. Original vehicle/physics/simulation/camera/LYA/gates unchanged.
CI dispatched for this exact source; its final result is not implied here.

## Review and continuity

All22 explicitly staged files reviewed, including source/tests, rejected
candidate evidence, bundle manifests and privacy scan. Unrelated untracked
curve-fit source/test pair remains excluded and untouched. Whitespace check
reports final blank lines in two sealed SESSION files and one tested test,
plus blank context lines in the rejected patch; these preserved evidence
formatting warnings were not rewritten to hide them.

Shared Unity/CPU remains coordinated with the semantic simulation task.
No Unity, focus, Assets, physics or scene action was taken for this checkpoint.
Next work: useful piecewise boundary geometry and substantial batch processing,
not tighter rejection alone or repeated unchanged timing runs.
