# Agent session: 2026-11-07-planner-longtraining

Input handoff: 2026-11-07-planner-learned-maneuvers

## Objective and scope

Run a longer Unity training experiment to reduce unnecessary steering while passing cones. Use physical contact and departure from the track as failures. Stop rewarding clearance above 0.5 m. Check natural straight-line cruising at 4 m/s using measured motion. Produce synchronized, inspectable simulation footage including unsuccessful runs. Authorization covers uploading the new videos to the user's existing private Drive location.

## Input handoff

Reviewed the prior learned-maneuvers session, the experiment README and WORKLOG, the V2 candidate and previous paired runs. V2 previously fell back on all four new test scenes. Measured steering could come from the lane estimate, changes in the passing target, and the wheel controller. Improving the passing action alone may leave steering variation.

## Source and working tree

The integration branch is experiment/unity-lap-training. It began at e8c35332d8db7c62da816a7a1aae896245370117. Training, independent scoring and film preparation used separate worktrees. The trainer alone controlled the private Unity copy. The camera, scene, motor and physical sources are compared by hash in codexws/learned_racing/unity_laps/long_training_evidence/plant.json. The parallel Unity development project and vehicle hardware were outside the scope of changes.

## Work performed

At 18:24 UTC on November 6, initialized this session and recorded the layout separation, planned experiments and cruise criteria in evidence/plan.json. Local timestamps for this session fall on November 7 JST. Recorded the constant-action addendum before training actions had been ranked. A separate scorer owned the primary holdout layout definition at 7a91a5f. Completed 560 paired training attempts and froze ck64 after 19 common development cases. The user then clarified the requirement to drive through gaps and prioritize speed. Created and verified a second isolated Unity copy, fitted a sequential opening planner, completed 84 practice runs, 72 recovery trials, and 24 blinded three-arm tests. A subsequent optional lane observation filter was assessed in 28 fresh paired trials and two known-case diagnostic replays. Both owned Unity editors were stopped. Detailed reproducibility and results are in the laboratory README, WORKLOG, and scorer reports.

## Evidence and result

The primary holdout achieved 23/24 local segments and 6/8 laps for both the reference and the frozen learner. Four jointly successful traffic laps improved median obstacle-window time 6.52%; L01 went out of bounds earlier, and L06 collided in both arms. Four clear measured straight gates had body means 3.361, 4.473, 4.476 and 4.477 m/s. The first fell below 4 following an A4 lane jump. The separate gap holdout completed 4/8 reference, 6/8 pace-only, and 7/8 gap; G08 was a gap collision after the pace-only arm succeeded. The lane filter improved 12/14 to 13/14 but did not repair the known straight, where progress speed fell 3.278 to 3.244 m/s. These are simulation outcomes and neither primary acceptance gate was met. Source hashes and scoring are archived under smooth_holdout, gap_holdout, and lane_gate_revision. The complete video inventory was uploaded to the user's private Drive folder; 19 current MP4s were confirmed, including training, full lap, passing and failures. Two earlier files with ambiguous captions were moved to Trash and replaced by v2. Local offline video review and its SHA inventory are available in the dedicated video worktree.

## Risks and blockers

A device under different battery, tire or surface conditions can respond differently. These Unity experiments do not measure physical-car performance. The selected action model is fitted to 448 of the 560 attempts, not all 560. Track and sensor transients still cause unnecessary slowdowns and failures. The separate optional planner and lane-filter code were combined and compiled, but their combination was not driven in Unity. Retain terminal collisions, uncompleted intervals and cases that fall back to the older controller.

## Next agent actions

Investigate multi-frame lane association errors using training data, with an untouched new evaluation set. Examine the G08 second-gate failure and excessive braking in G05. Preserve frozen datasets and do not tune on the reported independent layouts. Run new closed-loop checks before claiming any improvement or operating the real vehicle.
