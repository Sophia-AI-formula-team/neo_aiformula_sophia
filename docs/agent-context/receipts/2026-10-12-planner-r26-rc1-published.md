# R26 development checkpoint published — PARTIAL

Canonical repository Sophia-AI-formula-team/neo_aiformula_sophia, immutable
ID1303517209. Branch repair/lane-keyframe-transaction-20261011 and annotated tag
lane-mapping-20261012-r26-rc1 were pushed atomically, then read back from GitHub.

- Source/evidence commit:251eaebd9cfe902e703e3714af7d61515eb8e90a.
- Annotated tag object:6fcee085016e23821c44e876ca537e37c54b4c60,
  GitHub tag target is the same source commit.
- Session:2026-10-12-planner-frame-batch-r26-023, result partial/handed_off.
- Bundle40958bytes SHA256
  f032f89d43f0563eed686cd54c30d4f521bad12940791bcd30cb01812b7f66f5.

Published14 explicitly reviewed files:two continuation batch production files,
one independent test, status/R25 final CI receipt, seven session files and the
bundle/checksum. The unrelated untracked curve-fit source/test pair stayed out.
The rejected piecewise geometry was archived as text, never imported by production.

Observed local validation:175 focused tests;4242 full passes/one Linux-only skip,
193 pins unchanged and all nine native APIs exercised. The original111-frame
strict replay accepted14/15, then rejected seq165 at81.9721ms against the unchanged
pre-commit80ms gate. Seq164 accepted returned80.5944ms; no return-WCET guarantee
is inferred. New geometry introduced an80.79degree cap error and was rejected.

No new Unity lap, route freeze, second lap, whole-pipeline runtime success or
video is established. This tag records a development checkpoint. CI was triggered
by the source/tag push; final exact-source results need a separate receipt.
