# Unity driving curriculum · 2026-11-06

## Objective and scope

Random forward-facing spawn, complete a first lap using causal local observations, train for speed and traffic, and visualize every attempt. Runs are restricted to a private copy of the existing Unity simulator.

## Input handoff

Direct user task `2026-11-06-user-unity-lap-training`. The incoming 11 m two-cone experiment at `8294a2f005cc905930d8b85984838e050dcd79e2` established short-range steering selection. The repository context, original simulator instructions, calibration and validation records were read before extending the experiment.

## Source and working tree

Owned Neo worktree on `experiment/unity-lap-training`. The local source checkpoint for the first camera and search curriculum is `93f1f75`. New work lives in `codexws/learned_racing/unity_laps`. The installed scene, road surface, camera, tire model and articulated physical solver were compared by hash with the private project. Local camera releases are A3 v14 and A4 single-frame using the original GPU environment.

## Work performed

Ran `isolated.py` on a separate Unity 2022.3.22f1 project, `train.py` for no-cone and traffic practice, `traffic_training.py` for simultaneous layouts, `camera_training.py` for direct A4 practice and `camera_runs.py` for frozen evaluations. Crossed the single cone, staggered pair and moving cone in the same scene. Compared the successful frozen run to a muted detector input and a zero motion-extrapolation gain. `compare_motion.py` aligns the two actual Unity cameras on approach. `make_replays.py`, `report.py` and `verify.py` produce the material for review. Start/failure/finish events, seeds, settings, source hashes and timings are recorded in the experiment folder.

## Evidence and result

The evidence directory contains the measured summary, normalized source and artifact checks, per-run hashes and synthetic simulator previews. All 166 trials have a terminal log. Of these, 108 are training/qualification runs. The directly camera-trained policy achieved 80.18 s versus 171.26 s over the same clear 280.9 m lap. The previous physics-trained policy completed one mixed-traffic A4 lap in 68.97 s with 1.075 m minimum cone clearance. Muting the object channel caused first-cone contact at 57.83 m. Zeroing the lateral forecast caused moving-cone contact at 223.61 m. Three later blind routes with the directly camera-trained policy yielded two boundary stops and one cone contact, versus one finish with the initial policy.

## Risks and blockers

The test course and stock vehicle response are fixed. Cone ranging uses calibrated flat-ground projection and motion compensation uses simulation ego-motion. The strict boundary evaluator stops at the first car-body corner crossing. Three qualifying candidates failed on the third practice layout. Track re-acquisition after overtaking and generalization to new layouts are unresolved. A clear lap alone is insufficient to qualify traffic driving. Real vehicle response to a battery swap remains an open research question.

## Next agent actions

Use the failed qualification run and paired observations from replay to inspect lane association and yaw recovery on returning from an overtake. Compare any change against new predefined seeds and retain failures. The existing conditions are for simulator evidence only.
