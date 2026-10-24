# RESULT_READY — R30 CI is6/8, runtime not accepted

Role:planner. Exact code c212c3b1393c3f0297fd8cb65561798a24a5a8c2,
tag lane-mapping-20261015-r30-rc1. This appends terminal results to the
[publication receipt](2026-10-15-planner-r30-rc1-published.md); it does not
rewrite that receipt's earlier pending snapshot or sealed session026.

| Workflow | Branch run | Tag run | Terminal result |
| --- | --- | --- | --- |
| Native graph |38607072967|38607072916|both success|
| Handoff integrity |38607072971|38607073016|both success|
| Lane Foxy |38607072950|38607072808|both success|
| Endpoint GNSS |[38607072990](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38607072990)|[38607073093](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38607073093)|both FAILURE|

Both Endpoint runs built the packages, passed4613 Linux tests with zero errors,
failures/skips and four road-detector Header checks. But each synthetic real-DDS
runtime report passes only2/9cases:default_guard and startup_gap. The other7
stop before five actual stationary-camera commits; processing_deadline rejects
precede terminal bootstrap_anchor_not_established. For example, branch normal's
last sampled Engine time109.732ms exceeds the original80ms. Some other cases'
last samples are early fault returns, not proof of deadline success. These are
independent diagnostic snapshots, not full timing distributions.

This does not diagnose7GNSS logic regressions: those scenarios did not reach
their intended later checks. It DOES mean the runtime suite is failed and not
accepted. Neither a green build nor unit count cancels it. No retry was requested,
gate enlarged, clock moved or failed log removed. The existing0.2m/s private
synthetic fixture does not validate production inherited LYA speed or hardware.

Raw Endpoint artifacts11916376761(branch) and11915926915(tag) were downloaded
and read locally, never executed. A [sanitized exact summary](../sessions/2026-10-15-planner-lossless-blocks-r31-027/evidence/r30_ci_terminal.json)
retains both runtime hashes, all18 case rows and counters; raw stacks/anchors/
poses/private paths are not republished. Linked runs retain public original CI.

Separate continuation: planner implemented ONLY an isolated retained-face-block
prototype, not a new installed package path. Independent review found a bool
scalar normalization defect;3counterexample methods produced7failure entries,
then the row-slice copy fixed it. Initial12, same-source15, strengthened16 and
final18 results are separately preserved with exact hashes, not merged counts.
Final18 passed once, zero failure/error/skip. It retains all own+incident required
rows and full unowned capsules without holding the old table in the returned
graph, under the stated new projection contract. No full old API/backing-buffer/
builder/global5+1/transaction/corpus/heap/capacity/runtime acceptance is claimed.

[Session027](../sessions/2026-10-15-planner-lossless-blocks-r31-027/SESSION.md)
archives the final candidate/test as inert.txt source, design, independent review,
old failure and result records. Its [ZIP](../bundles/2026-10-15-planner-lossless-blocks-r31-027.zip)
has11 reviewed inputs/12members, SHA256
`454a9479665743f75e99a2dd130fc7df7931ef6a624ca240eb0f2045d1b83f0d`.
The source text matches the reviewed local files exactly; not auto-imported.
ZIP92444bytes; reviewed inputs88799bytes. Post-seal unchanged tools command
`python -B -m unittest discover -s tools/tests -v` completed33tests in18.405s:
32PASS/one Windows symlink-privilege skip, exit0, including exact all-session
readable/ZIP/hashmanifest/repack and local-link checks. This is packaging
validation, not another Engine/runtime pass.
The two verbatim archived Markdown reviews retain an extra EOF blank line;
git diff --check reports those2style warnings (not a clean-whitespace claim).
They remain byte-identical to the reviewed originals and sealed ZIP.

Safe state: no new production code after c212, v5 still disabled in Engine/control;
car/simulator/camera/LYA and original gates unchanged, Unity not refreshed/played,
no new first lap/freeze/second lap/error/video or physical driving acceptance.
Planner retains responsibility for actual core integration and timing repair.
The next evidence must compare the complete semantics/real causal Engine and
then new-scene laps; do not take these storage fixtures as permission to drive.
