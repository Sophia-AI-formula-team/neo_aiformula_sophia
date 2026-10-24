# RESULT_READY — R28 CI mixed; no runtime or driving acceptance

Canonical repo ID1303517209 only. Exact code8fcd4c0533a13d95400b9f377959733ed798f744/tag lane-mapping-20261012-r28-rc1. This append-only result follows the first published/in-progress receipt; sealed session024 and that receipt are unchanged.

| Workflow | Branch run | Tag run | Terminal result |
| --- | --- | --- | --- |
| Native kernels | 38171022750 | 38171022711 | both success |
| Agent handoff | 38171022778 | 38171022718 | both success |
| Lane Foxy | 38171022747 | 38171022764 | both success |
| Endpoint GNSS | 38171022797 | 38171022715 | branch success / tag failure |

Overall7/8 success. Failed tag job114563478846, step6. [Branch evidence run](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38171022797), [failed tag evidence run](https://github.com/Sophia-AI-formula-team/neo_aiformula_sophia/actions/runs/38171022715).

Both Endpoint artifacts build5 packages rc0, collect3340 reference+940 GNSS=4280 units allpass/zero skips, and4 road-detector Header tests allpass. Fixed collects0, not extra tests. Native9 actual calls and installed_prefix_verified=true, same C9dae4a7… and Linux binary472a3316… . Build/unit success is separate from DDS success.

Branch9/9 stationary synthetic actual-DDS cases,151 reached checks alltrue. Tag2/9 cases succeeds;121 reached checks alltrue but7 cases exit1, each accepts only1 camera with13/14 engine deadline rejections and terminal bootstrap_anchor_not_established. Worker alive/no worker failure. Journal and safety/engine samples are asynchronous, not an atomic combined state or commit-age proof. Preserve failures, no rerun-until-pass.

Tag invalid endpoint GNSS and runtime GNSS/orientation no-pose-influence checks were reached; later consensus, normal manual-estop/end-window and future-wheel/gyro stages were not all reached. Branch normal explicitly refuses a stationary incomplete route, bundle null; no REPEAT or driven lap. Both use private synthetic fixture teacher speed0.2m/s, not LYA's production default, and physical_stop_tested=false.

Worker logged compute range branch164.931–251.973ms/tag260.114–411.319ms includes projection/extraction+engine+optional consensus but excludes queue/transport/journal; it is NOT full simulator LaneSession200ms. No stable runtime, geometry, full200ms or two-lap acceptance follows.

Independent read-only review records72 artifact hashes; original raw logs retained locally and not published. Reviewed redacted reports will accompany session025. Main planner read the complete review and retains precise failed phases. Next: original gates remain; regress indexed allocation candidate and audit actual corner-model attribution. Car, simulation, camera and LYA settings remain unchanged; this is NOT vehicle release.
