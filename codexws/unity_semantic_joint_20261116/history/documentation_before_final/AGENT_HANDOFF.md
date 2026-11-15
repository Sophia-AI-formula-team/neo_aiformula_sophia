# Agent continuation index

Current staged policy is calibrated panel-only: C# `UseCalibratedSignalPanelOnly`
and Python `signal_panel_only`, default false outside the explicit Joint trial.
See `evidence/staged_panel_only_verification.json` for 13 boundary checks and
3268 frames with default-off exact output and only-mode original-helper parity.
V1/v2 five-file histories remain intact; the final six-file deployment excludes
the former identity helper and preserves the original `before/` rollback.
The raw model overlay remains diagnostic. This simulator-specific signal policy
does not prove official topo alignment, a physical red turn, or vehicle readiness.
Keep inner04's missing stationary-green evidence and inner06's rejected short-side
candidate separate; the latter was never deployed. Root owns fresh MCP results.

Goal: run the new 1115 joint-map semantic package with genuine original LYA,
complete at least two physical laps from inner and outer starts where possible,
show first-person perception plus semantic topology, and determine whether red
after mapping actually causes a turn into the interior route. Produce an edited
video around one minute without presenting failure recovery as red-turn success.

Source version: `0.4.0`, import commit
`6b71267e24e81459220c7e6d30d84c3f7a00e591`,
`workspace/src/aiformula/perception/semantic_planning_system` in this repository.
The Unity archive is independent and `COLCON_IGNORE`; it does not replace ROS
launch or duplicate that source. Use the current task branch; do not reset other
work, publish to the legacy repository, or infer merge permission from a PR.

## Index of existing material

| Material | Original location |
| --- | --- |
| Source and package docs | `workspace/src/aiformula/perception/semantic_planning_system/` |
| This adapter and guarded baseline | `codexws/unity_semantic_joint_20261116/` |
| Trial discovery / editable evidence index | `E:/Mess/Projects/Programming/aiformula/workspace/semantic_joint_trial_20261116/discovery.md` |
| Local actual runs / video work | `E:/Mess/Projects/Programming/aiformula/workspace/semantic_joint_trial_20261116/` |
| Canonical Simulator | `E:/Mess/School/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator` |
| Simulator instructions and current ownership notes | Simulator parent `AGENTS.md`, `workspace/CODING_MASTERPROMPT.md`, `workspace/handoff.md` |
| Original October accepted run evidence | `E:/Mess/Projects/Programming/aiformula/workspace/semantic_neo_topo_20261015/runs/` |
| MCP invocation helper | Simulator parent `workspace/tools/invoke_unity_bridge.py` |

At archive preparation, the worker completed 11 adapter-level message/config
smoke checks in 2.30 seconds. The runner parsed successfully. These checks did not
execute model inference, complete a Unity lap, or validate official map fusion.
Latest reviewed results are in [RESULTS.md](RESULTS.md): inner03 failed because
the harness counted cold startup against its progress budget; the corrected
inner04 completed one lap and a closed odometry reference but rejected official
fusion (`excluded_gap_too_long`) and never entered a red detour. Its mid-run
render cap is an explicitly mixed-setting diagnostic. The source algorithm is
unchanged. Root owns the proposed fixed-30 inner05; its outcome, the outer-start
case, two completed laps/verified interior turn and final video review remain
pending. Subsequent perception review identified same-box class-wise FCOS output,
not label remapping. The archived default-off narrow identity adapter passed 17
checks and saved-frame replay, with limits disclosed in
`evidence/PERCEPTION_INPUT_ANALYSIS.md`. It leaves raw detections/overlay and
semantic source unchanged; Joint explicitly opts in. Complete live results are
still pending. Never fill pending fields using earlier results.

Keep the two modes separate: joint experiment, Neo stop/wait, and bbox distance
have distinct runtime selection and evidence. The joint worker is in dedicated
`transport/`; dynamic reports must not be added to its source hash inventory.

Next agent:

1. Inspect actual Git status/HEAD, source version and manifest; check live Unity
   project/PID/ownership rather than relying on old process IDs in notes.
2. Read [PROBLEM_ANALYSIS.md](PROBLEM_ANALYSIS.md), then follow
   [DEPLOYMENT.md](DEPLOYMENT.md); do not install on an unknown modified baseline.
3. Obtain live compilation/MCP evidence, execute uniquely named inner and outer
   cases, preserve all outcomes and unchanged-source receipts.
4. Distinguish odometry reference ready, official graph match, detour planned,
   physical interior turn, and completed physical laps. Keep the 12-second red
   failure visible; label physical green recovery separately.
5. Report a necessary issue and its cause before applying a minimal fix. Keep
   unrelated functions and the original package algorithm unchanged unless the
   user-requested functionality requires a documented correction.
6. Record results/source hashes/videos in a concise result index and hand control
   back in stopped Edit Mode. Do not include temporary MCP bootstrap code in the
   runtime install manifest.
