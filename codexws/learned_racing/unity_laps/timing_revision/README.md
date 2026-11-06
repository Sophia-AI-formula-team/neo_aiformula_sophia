# Moving-cone timing experiment

In the matched 37 m Unity segment at phase −4.05 rad, permitting an extra passing action increased actual body speed from 2.72 to 4.47 m/s before the cone's station. The entire car cleared the station at 9.51 s, 1.13 s before the cone entered the lateral footprint used by the passing car. Minimum obstacle clearance over this run was 0.99 m. With only the extra action disabled, the car steered and slowed, finishing the same distance in 15.46 s with 0.33 m clearance. Both runs completed. Their commands and perceived inputs match exactly up to the choice at 7.8 s.

## What chose the commands

The camera pipeline is the frozen A4 detector and outer-edge lane observer from the earlier lap experiment. A short, ego-motion-compensated history of A4 object detections supplies observed position, transverse velocity and fit residuals. The local planner compares the frozen controller's proposed pace with its ordinary clear-road cruise applied along the observed road line. It integrates a 1.45 m/s² acceleration/deceleration limit in 25 ms increments, allows time for command response, includes the rover front, rear and cone radius, then estimates lateral clearance over the resulting interval. A faster pass requires positive predicted clearance after a residual- and time-dependent allowance, an inward moving track confirmed by both halves of the recent observations, enough acceleration headroom and small heading/line errors. Otherwise the original passing controller chooses speed, yaw and lateral offset. The acceleration ramp and the original steering and wheel-command routines apply to the selected action. The threshold values and motion model are hand specified. The algorithm has no online weight update. The sinusoidal cone formula, known phase and future footprint are referee-only inputs to the evaluation.

The action may hold its choice briefly through near-vehicle occlusion using integrated body speed. Multiple tracks, weak motion evidence or significant observed drift restrict its use. The uncertainty budget is a modeling assumption. It does not bound an arbitrary future change in an obstacle's motion.

## Frozen and paired outcomes

Optional mode from source 6e96acdd7d72, compiled solver ca509cc0c9c239, shares 12 frozen values with f25947e8af56df. Both modes use the same A4 build, body, spawn seed 25, yaw, traffic placement and motion phase. The nominal approach command cap of 2.4 m/s leaves room to accelerate and is imposed on both arms. Actual approach speed typically settled near 2.72 m/s. The original, uncapped cruise was also tested. A new scenario, layout or course is required for evidence of broader transfer.

| Source | First-pass option enabled | Option disabled | Nearest cone, enabled / disabled |
|---|---|---|---|
| Display case, −4.05 rad | 13.51 s | 15.46 s | 0.991 / 0.329 m |
| Same case, original cruise | 9.59 s | 9.59 s | 2.254 / 2.254 m |
| Four registered phase holdouts | 3 of 4 segments complete | 3 of 4 complete | full results in pairs.json |
| Static cone | 16.48 s | 16.48 s | 1.396 / 1.396 m |
| Central moving cone | contact at 12.28 s | same contact | −0.014 / −0.014 m |
| Additional phase −1.65 rad | contact at 11.37 s | same contact | −0.006 / −0.006 m |

The held-out phases were −4.38, −3.54, −5.26, −1.10 rad in that order. The option activated only on the second. It saved 4.46 s while reducing measured clearance from 0.929 m to 0.218 m. The rear cleared 0.194 s before the cone entered the traversed lateral footprint. The fourth holdout contacted the obstacle with both options. Static and original-cruise pairs have exactly identical physical trajectories across the two arms. Both unsuccessful moving negative controls declined a go-first action and still contacted the obstacle under the frozen fallback.

The separate default-mode check re-ran the archived mixed_01 static/moving scene. Every physics row, perception measurement, policy decision and wheel command matched the original f25947e8af56df run. This entire experiment uses the isolated Unity editor.

## Evidence and replay

- `pairs.json` contains nine controlled comparisons, actual speed at the cone station, rear clearance, first subsequent cone occupation, byte-for-byte command/perception prefix checks, and digests/counts for each retained camera stream.
- `runs/` preserves every attempt including five rejected pilot candidates, seven final-rule development runs, the paired tests, an additional development control and the default regression. `events.jsonl` records solver, start, weights, scenario and outcome. `practice_v1.patch` restores the exact first candidate from frozen source for replaying p01–p05. The reconstructed pilot C# source hash is 1401252c26c980.
- Original synchronized images on the recording workstation remain in `.work/frames/<id>` (chase and unannotated RGB), with actual A4 detections in `.work/camera/<id>/a4_samples`. The explanatory, central moving and additional −1.65 rad pairs include camera streams.
- `timing_evidence.py` calculates geometric measures independently from the car's world poses and the referee's moving cone. Footprint re-entry uses the crossing car's width and heading at the cone station, and a 2 ms sampling interval. Station times use interpolation of recorded physics rows at 10 Hz.

From the unity_laps directory, activate the exclusive private simulator as described by isolated.py and set AIFORMULA_LAP_COPY to that project. A fresh clone has no retained frame directories. The commands below create a separate replay directory so the recorded cached JSONs and frames remain available.

```powershell
python -B timing_trials.py --output .work/timing_replay --film explanatory_go:-4.05:cross:both:2.4 explanatory_yield:-4.05:cross:yield_only:2.4
python -B timing_trials.py --output .work/timing_replay --film blocked_go:-4.05:blocked:both:2.4 blocked_yield:-4.05:blocked:yield_only:2.4 after_go:-1.65:cross:both:2.4 after_yield:-1.65:cross:yield_only:2.4
python -B timing_trials.py --output .work/timing_replay static_go:0:static:both:2.4 static_yield:0:static:yield_only:2.4 full_go:-4.05:cross:both:0 full_yield:-4.05:cross:yield_only:0
python -B timing_trials.py --output .work/timing_replay h1_go:-4.38:cross:both:2.4 h1_yield:-4.38:cross:yield_only:2.4 h2_go:-3.54:cross:both:2.4 h2_yield:-3.54:cross:yield_only:2.4 h3_go:-5.26:cross:both:2.4 h3_yield:-5.26:cross:yield_only:2.4 h4_go:-1.10:cross:both:2.4 h4_yield:-1.10:cross:yield_only:2.4
python -B timing_trials.py --output .work/timing_replay default_reference
python -B timing_evidence.py --output .work/timing_replay
```
