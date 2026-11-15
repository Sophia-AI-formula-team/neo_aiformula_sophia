# Final live results

Final MCP trials used the same six-file panel-only integration, original semantic 0.4.0, official OSM and original LYA, at fixed 30 FPS / vSync 0. Both completed two real physical laps and passed red-stop/green-resume. Both failed official topology alignment and red-triggered interior turning, so `accepted=false` is correct.

| Measurement | Inner `inner_joint_07` | Outer `outer_joint_03` |
| --- | ---: | ---: |
| Physical laps | 2 | 2 |
| Recorded travel | 519.832642 m | 563.146700 m |
| Recorded simulation duration | 214.9798 s | 230.7783 s |
| Closed odometry reference | 259.519390 m | 281.356355 m |
| Official graph accepted | No: ambiguous_correspondence | No: ambiguous_correspondence |
| Interior detour / measured left turn | None / none | None / none |
| Worker shutdown | Clean exit 0, no pending fusion | Clean exit 0, no pending fusion |
| Fixed input files unchanged | 125 / 125 | 125 / 125 |

The first lap used the physical green signal. Only after odometry closure and physical start-plane return did the evaluator switch the lamp red. Native red voting stopped the vehicle. With official fusion unavailable, no detour path was created. After a full 12-second red interval the evaluator preserved the failed detour verdict, physically switched green and attempted the remaining lap. Real stationary green images then released the native wait. No evaluator truth, lap count, route or alignment entered control.

Inner stationary green frames 504–506 preceded motion at 507; outer frames 546–548 preceded motion at 549. An asynchronous trace may momentarily show `unknown` after a valid earlier green-majority transition; this does not mean unknown releases the red latch. Detailed evidence is in [the final report](evidence/FINAL_PANEL_ONLY_TRIAL_RESULTS.md), [outer analysis](evidence/OUTER03_FINAL_BEHAVIOR.md) and the compact [inner](evidence/inner07_summary.json)/[outer](evidence/outer03_summary.json) receipts.

## What mapping did and did not establish

Odometry reference closure is demonstrated. Official-node odometry positions remained empty and `graph_frame=official_unaligned`; interior road observation/mapping was not demonstrated. Inner final best/alternative RMSE were 0.0583205 / 0.2724613 m, score gap 0.2341708 < original 0.3, and geometry spread 1.3058268 > original 1 m. Outer values were 0.407436 / 0.507322 m, gap 0.107533 and spread 2.851812 m. A small best error alone cannot establish unique topology correspondence.

The original inner05 selected pair became nearly identical when continued, exposing insufficient refinement of that pair. Full search at 75 and 100 iterations retained another far, near-cost correspondence: still ambiguous, with about 37.93/37.95 m geometry spread. No source change, threshold relaxation, candidate deletion, manual map, or increased-budget algorithm was deployed. See [matcher report](evidence/CRITICAL_MATCHER_CONVERGENCE.md).

## Stop position and recovery limits

The new package stops immediately after red confirmation. Stationary Body-to-lamp-root planar distances were **24.838 m inner / 21.658 m outer**; projections along the car heading were **23.121 m / 21.396 m**. These post-run measurements never entered control and are not bumper, camera or stop-line distances. The earlier 8–9 m target and bbox mode were not used. Even successful fusion would still require checking whether a red stop outside the native 3 m junction-entry radius prevents reaching the detour entry.

This final pair does not prove robustness over all starts, visibility or timing. Supplied 0.4.0 explicitly reports persistence not implemented; clean shutdown is not checkpoint/restart recovery. No vehicle/hardware acceptance is claimed.

## Earlier failures remain separate evidence

| Attempt / candidate | Observed result and treatment |
| --- | --- |
| Inner 01 | Missing Joint vision provenance; reproduced and corrected in transport only |
| Inner 02 | Latency correctly produced zero, but legacy consumer permanently stopped Joint; separate fresh semantic-command consumer added |
| Inner 03 | Cold startup incorrectly consumed no-progress budget; independent bounded startup added |
| Inner 04 | One lap, 259.5646 m reference, excluded-gap fusion rejection; mixed render settings and later false red; failed second lap |
| Inner 05 | Two laps under earlier v1 signal policy; official match ambiguous, no red turn |
| Outer 01/02 | Earlier v1/v2 model-color policies admitted false red before first closure; retained failures |
| Inner 06 | One lap, 303.726624 m; real short lane support became unsupported, TTL held zero; not false-red or reference steering |
| Four-point side candidate | Default-off replay unchanged on 827 frames, but unequal A/B/C spacing plus native compensation produced omega_t 1.016826; rejected, never deployed |
| Final inner 07 / outer 03 | Two laps and red/green behavior passed; topology and red turn failed |

Original early reports and pins remain in `history/documentation_before_final/`; provisional statements there describe their historical stage. The [decision chronology](PROBLEM_ANALYSIS.md) links analyses written before each corresponding repair. The final first-person/semantic video retains red failure and labels subsequent green recovery; media code/receipts are archived separately from the binary recording.

## Cleanup

Final MCP evidence shows Edit Mode, clean scene, restored FPS -1/vSync 1/timeScale 1/fixedDeltaTime 0.02. Thirty-five protected original scene/settings/package files match their originals; the separately checked semantic transport is the expected overlay. Task-owned temporary MCP bootstrap and its metadata were exact-hash archived then removed. Final console receipt is empty. Cleanup is not a rollback of the six reviewed integration files.
