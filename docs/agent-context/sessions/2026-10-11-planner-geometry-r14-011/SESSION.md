# R14 geometry development checkpoint — TWO LAPS NOT PASSED

## Objective and scope

Planner continuing geometry repair in neo and the requested actual Unity
LYA/TEACH lap followed by a fixed-route REPEAT lap. Vehicle, physics, camera,
simulation, LYA speed, gates and runtime budgets remain protected. No future
data or GNSS mapping. This is partial development evidence, not a release.

## Input handoff

Acknowledged 2026-09-28-planner-neo-004; read AGENTS, AGENT_CONTEXT, STATUS,
PROTOCOL and the active handoff. R12 session010 is the incoming development
code checkpoint. The field handoff is not automatically superseded by this
failed candidate. The authorized UI thread currently owns Editor/Assets;
explicit handback and original full parameter/pose verification precede driving.

## Source and working tree

Canonical: Sophia-AI-formula-team/neo_aiformula_sophia, repository ID 1303517209.
Branch repair/lane-keyframe-transaction-20261011; incoming commit
37119203ff84f1fcda71222b292d1edd3756008b, tag lane-mapping-20261011-r12-rc1.
Source-specific identities and changes are in the accompanying evidence index
and staged diff. Local source hashes may differ from Git-normalized line endings.
Unused quantized_curve_fit.py and its test are untracked and excluded.
No main/legacy push, PR, history rewriting, dependency installation or vehicle
deployment is part of this checkpoint.

## Work performed

R13 adds a commit-only per-cell index and exact local-snapshot packaging,
including global graph degrees. Metadata scans remain O(map). R14 partitions
smooth fitting domains using independently supported contour directions and
all original binary labels inside the same half-face footprint. The exact
certificate witness is never an observation. Cuts are computational fit domains,
not certified physical corners. Raw faces, links and final model gates remain.

A one-sided-derivative variant was reverted because independent shape checks
showed severe output coverage degradation. Its failed evidence is retained.
The initial real-label test oracle wrongly required no fit cuts for every
smooth circle/short cap. Its original 22-pass/7-fail source and XML are retained.
The corrected 29-test contract checks output in original quantization brackets
and rejects short-cap models. This is a reviewed test-contract correction, NOT
an algorithmic repair of seven failures. Original masks/gates are unchanged.

Simulator loader/archive adaptations include the actual optional helper in
source identity and copy verified core bytes; old manifests stay compatible.
An external repeated-support Fraction prototype is not production integration.

Private diagnostics aliases D (diagnostics), S (Simulator), R (this repository):

```text
python -B D/r14_full_native_01_run.py
python -B S/Tools/LaneTeachRepeat/replay_observed_keyframe.py replay --fixture D/trial28_analysis/observed_prefix.json --repo R --motion-factor --candidate-evaluation --candidate-plane-offset-m .007 --candidate-boundary --output D/r14_boundary_observed_02.json
```

Full command lines, stdout, source identities and failed attempts are preserved
locally in the indexed evidence. Personal paths/raw inputs are not published.
Matcher/engine budgets remain 40/80 ms. The synthetic quality harness uses its
pre-existing diagnostic 2000 ms budget, not runtime acceptance.

## Evidence and result

- Current core: 2577 passed, 1 platform skip; unchanged source before/after.
  Seven native APIs actually invoked.
- Recorded prefix: 105/106 accepted, seq151..255; seq256 insufficient overlap,
  engine57.15ms. Fresh past overlap0.530782 remains below unchanged0.55.
  The rejected frame does not commit.
- Fixed full-mask development: 0/4 pass; all accept only7/509 planned frames.
  Pitch ramps have not begun when these fail, so these are only two distinct
  pre-failure geometries, not four independent pitch results.
- Corrected label tests29/29; loader subset51/51. Prior full adapter318pass/4fail,
  repaired fixture5/5; that is not a complete repaired-adapter pass.
- Intrusive history capture03: original80ms deadline at seq250,114.385ms,
  99commits/0targetqueries; postprocessing also omitted metadata. Preserved.
  Capture04 removes global trace and per-frame copies, retains original40/80,
  reaches seq256,105commits, all106non-timing outputs exact; query copy costs
  0.0164/0.0127ms remain inside the measured budget. Captured geometry is
  diagnosis, not a new actual drive or proof of valid repeated support.
- External exact support prototype32/32; neither runtime nor integration proof.
- R12 eight workflows/sixteen jobs succeeded; this does not apply to later code.

Detailed candidate chronology, rejected alternatives, source hashes and evidence
are in evidence/. Earlier eef-source accuracy must not be assigned to current
a0-source output. Latest actual drive remains failed trial28. NO completed first
lap, frozen route, second lap or requested successful two-lap video exists.

## Risks and blockers

Confirmed record/edge votes do not certify finite length at each location.
Mean-node chords can truncate genuine support, but using latest faces would
falsely claim repeated support. Exact per-position mean patches can grow and
become discontinuous; runtime model/schema/identity decisions are still needed.
Extraction is outside the engine80ms budget; end-to-end time remains unverified.
Local tests/CI are not ROS install, sensor calibration, motor-stop or vehicle
acceptance. Raw bags/images, coordinates, binaries, logs/videos and personal
paths stay local; only reviewed summaries are packaged.

## Next agent actions

1. Analyze original selected queries against only committed historical model
   segments. Preserve missing support and branch ambiguity; do not invent metadata.
2. Repair finite geometry from justified repeated support with bounded time/
   memory, unchanged gates, causal commits and explicit bundle/source identity.
3. Run fixed quality and complete adapter tests on exact final bytes; retain failures.
4. Obtain UI handback and verify original plant/scene; run actual consecutive
   TEACH/REPEAT without resets/stitching, with logs/errors/fixed-scale consensus video.
5. Publish reviewed version/tag/CI receipts. This failed checkpoint never grants
   physical vehicle driving authority.

