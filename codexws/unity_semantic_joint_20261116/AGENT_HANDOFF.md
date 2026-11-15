# Agent continuation index — final 2026-11-16

Current result: original semantic **0.4.0** with final calibrated-panel-only Unity adapter completed **two physical laps from each start** (`inner_joint_07`, `outer_joint_03`) and real red-stop/green-resume. Official topo matching remains `ambiguous_correspondence`; no red-triggered interior detour or left turn occurred. Both `accepted=false` records are intentional. Read [RESULTS.md](RESULTS.md) before continuing.

The source import commit is `6b71267e24e81459220c7e6d30d84c3f7a00e591`. At archive preparation the task branch was `feat/semantic-joint-20261115`; publication is handled by the parent task. Recheck current Git rather than treating this preparation snapshot as the final published HEAD. Do not reset unrelated changes or merge a PR without separate authorization.

| Material | Source index |
| --- | --- |
| Original semantic source, manifests and ROS deployment | Repository `workspace/src/aiformula/perception/semantic_planning_system/` |
| This final six-file Unity integration | Repository `codexws/unity_semantic_joint_20261116/` |
| Local trial root, abbreviated TRIAL in receipts | `E:/Mess/Projects/Programming/aiformula/workspace/semantic_joint_trial_20261116` |
| Full real run inputs / RGB / per-frame logs | `TRIAL/runs/inner_joint_07/`, `TRIAL/runs/outer_joint_03/` |
| Full earlier failures and diagnostic inputs | `TRIAL/runs/`, `TRIAL/audit/` |
| Final edited first-person/semantic media | `TRIAL/media/final_panel_only/` |
| Original LYA Simulator, abbreviated SIMULATOR | `E:/Mess/School/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator` |
| Simulator rules / ownership | Parent `AGENTS.md`, `workspace/CODING_MASTERPROMPT.md`, `workspace/handoff.md` |
| MCP bridge helper | Simulator parent `workspace/tools/invoke_unity_bridge.py` |
| Source checkout, abbreviated REPOSITORY | `E:/Mess/Projects/Programming/aiformula/workspace/semantic_joint_20261116` |

The archive manifest pins every payload/history/evidence file. `.gitattributes` disables line conversion so reviewed SHA256 bytes survive Windows Git autocrlf. Use `transport/` as worker root; do not point at mutable `audit/`. The runner only inventories the worker file, source tree and named fixed dependencies. It requires new output names and never overwrites prior runs. Use MCP only, with exclusive ownership of the correct Editor; do not interfere with other Unity projects or chats; this task owns semantic testing only.

Final cleanup is recorded: clean Edit Mode, restored FPS/vSync/time scale/physics step, original scenes/settings/packages unchanged, task-owned bridge bootstrap removed, no remaining own model/semantic worker. Six reviewed integration files remain installed. `install.py` defaults read-only; it recognizes only clean original before bytes or exact final bytes. Historical v1/v2 are not generic allowed baselines.

Remaining work is algorithm research, not an unfinished installation: official junction correspondence is ambiguous; red stopping remains about 21–25 m Body-to-lamp-root and lacks the earlier 8–9 m approach; package persistence/restart recovery is not implemented. The old bbox mode stays separate. A proposed 75/100 matcher budget was not sufficient and not deployed. The four-point lane candidate was rejected for artificial target curvature and not deployed. One successful final pair does not erase earlier lane-support failure or establish robustness.

Before any new source repair, preserve the final videos/failure receipts, write a specific problem analysis, make a versioned source change, and rerun independent geometry plus actual closed-loop validation. Do not inject truth, hand-align the official map, discard competing candidates or loosen thresholds to obtain a successful demo. Read the report-first [decision chronology](PROBLEM_ANALYSIS.md). This archive contains code and small evidence only; raw recordings and video binaries remain at their indexed original paths.
