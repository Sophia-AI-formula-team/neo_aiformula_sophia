# Agent session: 2026-09-28-planner-neo-integration-004

Input handoff: user-request-20260928-neo-integration

Reviewed final record for packaging. No vehicle was operated.

## Objective and scope

Fix confirmed neo integration defects and validate three lane packages. User
requires inheritance of LYA's current reference parameter, not a speed recording
or a copied 2.0 default. Explicit configuration can override it. Feature-branch
publication and CI are authorized; main merge, vehicle operation, physical-limit
tuning and local dependency installation are not authorized.

## Input handoff

Read AGENTS.md, AGENT_CONTEXT.md, STATUS.md, PROTOCOL.md and
2026-09-27-planner-neo-003. User request supersedes the old unresolved integration
items, not GNSS endpoint-only and causal mapping constraints.

## Source and working tree

Base ef7e5ceb7dbf0b730e34f1f9e5de2a0e33807f23, initially clean isolated staging
checkout on feat/causal-lane-teach-repeat. Canonical repository is
Sophia-AI-formula-team/neo_aiformula_sophia, ID 1303517209. Fetched main remains
8914b613c7200709f7c8617aed2d408afd911a0d. Other dirty user checkouts and the
legacy repository were not modified.

## Work performed

- Preserve the full source Header on both road_detector outputs, without
  changing inference or pixels. Correct default mask topics and remove the
  mistaken blacklist that treated neo's full-image remap as an ROI.
- Select actual neo lya_0221 and share its reference via lya_profile.py. No
  repeated numeric YAML defaults; empty launch override preserves params-file
  precedence. Keep original LYA feedback law and original repeat yaw/lateral
  limits. Preserve valid TEACH/FALLBACK commands instead of shrinking to 0.8.
- Reject infeasible fixed-reference routes instead of silently slowing. Require
  an explicit approved teacher ceiling for any non-private output, without
  claiming the motor's bypassable max_command_v is a hardware cap.
- Add GNSS variant publisher-conflict guard, offline wiring/speed contracts and
  real installed-LYA process plus real Header DDS checks in disposable Foxy CI.
- Windows Python 3.13 has no rclpy/ros2/colcon. Starting existing Docker Desktop
  hidden failed and daemon pipe stayed unavailable; no images or dependencies
  were installed. Native ROS proof remains CI-only.
- Sparse-checkout expansion temporarily hid lane working files; explicitly
  restoring their sparse paths fixed reads without source deletion. Guessed
  filenames were replaced by actual rg-discovered config and workflow paths.
- First pytest command named a nonexistent fixed/test directory: zero tests.
  Broad road_detector test collection then failed on missing ament lint modules.
  Narrowed to the new output-contract test, matching CI. Functional regression
  first returned 486 passes, one skip, 88 AST-fixture errors because defaults
  now import the shared profile. Fixtures were repaired without removing original
  behavioral assertions. New launch-contract fixture initially had three stub
  assumption failures, fixed in the fixture, not production code.
- `python -B -m unittest discover -s tools/tests -q`: 33 tests, one Windows skip.
  Aggregate production-unit/header scope initially reached 597 passes. Review
  then found fixed-reference GNSS bundles were rejected by the legacy loader.
  Fixed the actual build/load/worker path, synchronized speed policy metadata and
  kept deployment limits independent of candidate loading. Final regression:
  619 passed, one Windows skip, 16.07 seconds. Final recheck at the dependency-fix
  baseline bab577b: 619 passed, one skip, 17.29 seconds (local_regression_04.xml).
  Python 3.8 parsing (24 changed source files), Bash syntax, YAML/XML and diff
  checks passed locally. Two draft-record patch attempts failed validation without
  changing files; corrected patches were then applied.
- Reference inheritance above the GNSS raw-motion gate (currently 3 m/s) now
  fails at startup with the actual conflicting values, not during the lap and
  not by silently reducing the requested reference or increasing the gate.
- Code commits 020df63 and 900b555 published to the canonical feature branch
  after checking the push URL and API repository ID. No PR or main update.
  Native runs: teach-repeat 36426750831, GNSS 36426750839; handoff 36426750810.
- First native LYA run failed: installed tf_transformations required the missing
  transforms3d import, child exit 255. Build/unit/old DDS checks had passed but
  this complete job had not. Read actual teacher log, checked official rosdep
  Focal rules and upstream tf_transformations/PyPI documentation. Declared the
  dependency and pinned transforms3d 0.4.2 with --no-deps only in disposable CI.
  Did not follow a log's install command blindly or install anything locally.
- Commit bab577b9cb71e5459301c785843ef54ba4a2cefa passed both new native runs:
  36427591132 and 36427591136. Downloaded their artifacts and read actual JSON
  observations plus unit stdout. Updated active context/runbook and issued new
  neo-004 handoff; old handoffs and sealed archives remain byte-preserved.

## Evidence and result

Local synthetic 20 m-radius track at current 2 m/s reference completed 125.643 m
in 64.85 s, cross-track RMSE 0.01651 m, maximum 0.10 m, steady mean command
1.99999984 m/s. Radius 5 m rejected by route gate; 9 m by controller gate.
Ideal kinematic simulation only, not real perception/localization/vehicle proof.
Final source baseline is bab577b9cb71e5459301c785843ef54ba4a2cefa.
See evidence/local_validation.json, native_ci.json, native_lya_header.json,
native_reference_fixed.json and native_gnss.json. Foxy package tests: 354 shared
+ 262 GNSS = 616, Header tests 4; native reference/fixed DDS 63 checks; GNSS
9 stationary synthetic DDS cases/151 checks; real LYA/default and global-YAML
override cases/24 checks. Default feedback observed 2/2.15, override 4/4.15;
real child/process-group exit and STOPPED verified in each case. Header was
transferred through production publication + cv_bridge + real DDS.

Artifacts keep complete CI logs temporarily; reviewed native reports and unit
stdout are copied into this permanent small package. The historical workflow
upload glob omitted package pytest.xml; counts are backed by archived stdout and
the CI's XML-count assertion, not invented from an empty colcon result. Raw local
XML with local paths is retained locally; its digest is recorded. Original
demo metrics/tracking and failed-native report are retained. Packaged text uses
LF line endings for deterministic Windows/Linux checkout and archive hashes.

## Risks and blockers

No model inference, new real-bag complete lap, physical stop or vehicle test.
Existing raw bag still has missing GNSS fields and motion dropouts. Older two
packages use fused VectorNav; GNSS variant uses raw motion and endpoint-only
GNSS. Neo teleop shares the motor topic; graph rejection is not a command mux.
Ownership, downstream watchdog and physical stop require field verification.
Motor zero-command bypass already exists. The 0.35 route-reference lateral gate
is not a hard bound on feedback-corrected commanded v*omega (demo peak 0.40244).
No physical gate was relaxed to fit the faster demo.

## Next agent actions

Planner publishes this reviewed session, ZIP and neo-004 task, then a separate
receipt referencing the evidence commit. Field agent must ACK the exact task
and baseline, use an experiment branch, and measure inputs/ownership/stop chain
before driving. Preserve both failed and successful attempts. Return verified
code diffs, effective parameters, logs and limitations in a reviewed session/ZIP.
This is RESULT_READY for code and isolated tests, partial for vehicle readiness;
no main merge, automatic new experiment, background wake-up or driving approval.
