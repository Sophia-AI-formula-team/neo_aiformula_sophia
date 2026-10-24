# R34 published checkpoint — NOT runtime/two-lap acceptance

Canonical destination checked immediately before atomic push:
Sophia-AI-formula-team/neo_aiformula_sophia, repository ID1303517209,
push URL https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia.git.

- Code commit010c089e03d8952677f688656639e215b816984d.
- Branch repair/lane-keyframe-transaction-20261011.
- Annotated tag lane-mapping-20261015-r34-rc1.
- Tag object2f91aef5c519586e37386be07953111e881ffccd, peeled commit010c089e03d8952677f688656639e215b816984d.
- Normal atomic branch/tag push succeeded; git ls-remote independently returned
  those exact branch, object and peeled values. No main/force/legacy push.

Eight exact-code CI runs were read back IN PROGRESS, not green:

| Workflow | Branch run | Tag run |
| --- | --- | --- |
| Native Windows/Linux matrix | 38703128343 | 38703128466 |
| Teach-repeat Foxy | 38703128352 | 38703128567 |
| Endpoint GNSS Foxy | 38703128321 | 38703128393 |
| Handoff integrity matrix | 38703128372 | 38703128476 |

No unchanged rerun. Later terminal results require a new receipt; do not infer
this development geometry passed ROS deadlines or physical acceptance.

[Local results](2026-10-15-planner-r34-local-result.md) and
[sealed033](../sessions/2026-10-15-planner-oriented-competitors-r34-033/SESSION.md)
contain4812pass/2platformskip, complete evidence parity and measured geometry
improvement with original gates. Prepare is still982..1548ms. Default/Engine/
control not enabled. No new Unity run, first lap, stopped freeze, second lap,
tracking error or video. No car/simulator/LYA parameter edit by this task.

Publication review details/correction: preserving sealed CRLF JSON bytes makes
default git diff --check report their CR as trailing whitespace, in addition to
two sealed markdown EOF-empty-line warnings. They are intentionally not rewritten.
The numerical sources/tests/workflow pass the scoped whitespace check; all24
new staged session/bundle files were independently byte-identical to the reviewed
working files/ZIPs before commit. Tooling33tests passed with one Windows-only
symlink privilege skip. One read-only rev-parse used an unquoted PowerShell
^{ } expression and failed; quoted ^{} readback then confirmed the correct tag.
This did not change source or Git state. Only the two excluded unrelated
prototype names remain untracked; they were not accessed or staged.

Next safe work: original full-foreground scoring and causal Engine budgets;
incremental complete-key geometry reuse with explicit cache capacity, no cropping
or old-map pinning; model/options provenance before any Engine activation.
Mandatory simulator two-lap acceptance remains outstanding.
