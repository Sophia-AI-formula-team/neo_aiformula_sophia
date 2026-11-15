# Reviewed small evidence index

Final behavior is reported in [FINAL_PANEL_ONLY_TRIAL_RESULTS.md](FINAL_PANEL_ONLY_TRIAL_RESULTS.md), [OUTER03_FINAL_BEHAVIOR.md](OUTER03_FINAL_BEHAVIOR.md) and the compact [inner07](inner07_summary.json)/[outer03](outer03_summary.json) summaries. Both real MCP runs completed two laps and red-stop/stationary-green recovery; both retained failed official topology and red-detour acceptance. The receipts omit raw images, full trajectories, worker identities and large per-frame arrays. Full originals remain under the TRIAL index in [AGENT_HANDOFF.md](../AGENT_HANDOFF.md).

| Files | Evidence scope |
| --- | --- |
| `inner07_input_integrity.json`, `outer03_input_integrity.json` | All 125 recorded executable/configuration inputs unchanged within each final run; original receipt SHA256 retained |
| `final_editor_restored.json`, `final_protected_file_checks.json`, `final_console.json` | MCP clean Edit-state/restored settings; 35 protected originals unchanged and one explicitly expected transport overlay; final console snapshot |
| `final_bootstrap_cleanup.json` | Exact-hash removal of the two task-owned temporary MCP bootstrap files only |
| `panel_only_install.json`, `panel_only_obsolete_cleanup.json`, `panel_only_console.json` | Reviewed v2-to-final migration and old helper removal; historical unrelated obsolete-API compiler warning retained |
| `final_installer_readonly.json` | Default installer comparison of current six live final payloads; no writes |
| `TEST_REPORT.md`, `CRITICAL_ISSUES.md` | Original package test failures, isolated successes, reachability/alignment risks and first live adapter problems; not a blanket test pass |
| `smoke_callbacks_01/smoke_receipt.json`, `inner01_replay_diagnostics/receipt.json` | Original 11 adapter smoke checks and reconstructed real-frame provenance failure/pass; original worker revisions retained |
| `inner03_summary.json`, `inner04_summary.json` | Actual startup-budget failure, then one-lap mixed-render failure; not rewritten as final successful runs |
| `RENDER_LOAD_ANALYSIS.md`, `inner04_render_cap_transition.json` | Analysis before adjustment and exact within-run cap transition; no exclusive-GPU attribution |
| `PERCEPTION_INPUT_ANALYSIS.md`, `staged_perception_verification.json`, `staged_perception_v2_verification.json` | Historical v1/v2 studies and saved-frame checks; superseded policies, not final deployment |
| `JOINT_AMBIGUITY_STATIC_ANALYSIS.md`, `JOINT_AMBIGUITY_RUNTIME_ANALYSIS.md`, `CRITICAL_MATCHER_CONVERGENCE.md`, `fusion_budget_summary.json` | Initial static uncertainty, exact 15-iteration reproduction, pair convergence and still-ambiguous complete 75/100 search; no source change deployed |
| `INNER06_LANE_FAILURE_ANALYSIS.md`, `SHORT_SIDE_WIRING_REVIEW.md`, `rejected_short_side/` | Real lane-support failure and rejected four-point candidate; artificial omega_t prevents deployment despite finite output/default-off parity |
| `SIM_PANEL_ONLY_SIGNAL_ANALYSIS.md`, `panel_only_signal_replay_summary.json` | Five-run preimplementation signal study; inner04 stationary-green limitation preserved |
| `PANEL_ONLY_WIRING_DESIGN.md`, `staged_panel_only_verification.json`, `PANEL_ONLY_INDEPENDENT_REVIEW.md` | Problem/design before implementation, 13 boundary checks and 3268-frame default-off/only-mode parity, independent no-blocker review |
| `report_provenance.json` | Original report hash, archive hash and explicit environment-root normalization |

Historical reports deliberately retain conclusions at the time they were written and later appended updates. See the ordered [decision index](../PROBLEM_ANALYSIS.md); a historical pending sentence is not the final trial status. Source/fixture failures and unsuccessful live trials remain preserved. Signal JPEG replay is separate from real inference timing, actual vehicle motion and official-map correctness.

Archive preparation ran only file/AST/hash checks and a read-only installer comparison. It did not rerun model inference, fusion or Unity, and copied no RGB/video binary. Final media code, frame time index, hashes and QA are indexed under `../media/`; their source recordings remain local. Named roots TRIAL / REPOSITORY / SIMULATOR resolve through the handoff index rather than exposing personal account paths.
