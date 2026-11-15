# Unity semantic joint-map experiment — final 2026-11-16

The final **inner_joint_07** and **outer_joint_03** MCP trials each completed two physical laps with original LYA. Both stopped for real red-panel observations and resumed from green observations while stationary. **Official topology fusion and the red-triggered interior left turn failed**: `ambiguous_correspondence` kept the planner in `red_wait_map_fusion`. Overall `accepted=false` remains in both results.

| Final run | Physical laps / travel | Learned odometry reference | Official topo / red turn |
| --- | --- | --- | --- |
| Inner 07 | 2 / 519.832642 m | 259.519390 m, closed | Failed / not entered |
| Outer 03 | 2 / 563.146700 m | 281.356355 m, closed | Failed / not entered |

After 12 seconds of red waiting, the evaluator saved `red_detour_failed=true`, switched only the physical lamp green, and let genuine perception/native control complete the second lap. Green recovery is not red-turn success. The package still stops immediately on confirmed red: measured stationary Body-to-lamp-root distances were about 24.84 m inner / 21.66 m outer. These are evaluation measurements, not camera/bumper/stop-line distances. The old 8–9 m approach and bbox-distance experiment were not mixed into this mode.

This `COLCON_IGNORE` archive contains the explicit `semantic_joint_1115` Unity adapter. The original **semantic 0.4.0**, official OSM and original LLP/LYA algorithm remain unchanged. Source is indexed at [the repository package](../../workspace/src/aiformula/perception/semantic_planning_system/) and import commit [6b71267](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/commit/6b71267e24e81459220c7e6d30d84c3f7a00e591); it is not copied here.

The final signal policy is **calibrated_rgb_panel_only**, an explicit simulator adapter using current RGB evidence and the existing two-frame confirmation. `signal_panel_only` defaults false outside this trial; Joint temporarily enables it and restores the original configuration. Raw model detections/colors/overlay remain visible. Disabled RGB is rejected, missing calibration yields unknown, and model color never substitutes for a missing confirmed panel in this mode. No signal distance enters semantic.

| Contents | Purpose |
| --- | --- |
| `overlay/Assets/` | Exactly four C# and two perception Python deployment files |
| `before/Assets/` | Exact originals of the five replaced files; harness was previously absent |
| `transport/semantic_joint_worker.py` | Original semantic/LYA JSON adapter; diagnostic worker only |
| `trials/run_joint_trial.py` | Bounded MCP runner, source hashes and real first-person recording |
| `manifest.json`, `install.py` | Exact hashes, read-only comparison by default, guarded install/rollback |
| [RESULTS.md](RESULTS.md) | Final verdicts, historical failures and limits |
| [PROBLEM_ANALYSIS.md](PROBLEM_ANALYSIS.md) | Problem-before-change chronology and rejected candidates |
| [DEPLOYMENT.md](DEPLOYMENT.md) | Reproduction and rollback |
| [AGENT_HANDOFF.md](AGENT_HANDOFF.md) | Current source/evidence index and remaining work |
| [evidence/README.md](evidence/README.md) | Small reports and receipts, without RGB or large logs |
| `history/` | Superseded v1/v2 signal policies and earlier documentation, never active payloads |
| `media/` | Reproducible renderer and small video evidence; binaries remain local |

`manifest.json` is the authoritative current pin list. The short-side four-point candidate was rejected because it created artificial target curvature; it was never deployed. Full 75/100-iteration matcher experiments still rejected an alternative correspondence and were not deployed. Final checks preserve the original source, scenes/settings, and restore the Editor to clean Edit Mode. These simulator results provide no hardware acceptance or restart-persistence proof; supplied 0.4.0 has no checkpoint persistence.
