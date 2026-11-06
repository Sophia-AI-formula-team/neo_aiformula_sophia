# A4 steering, free-road pace and yielding

## Objective and scope

Explain the actual per-wheel driving command. Improve repeatable safe lap completion, smoothness and speed in the existing Unity simulator. Separate hazard-zone decisions from unobstructed pace. No physical commands or remote publication.

## Input handoff

Continues 2026-11-06-planner-unity-lap-training. Reviewed its SESSION, camera_policy.json, paired runs and camera traces. Its faster camera policy missed three previously unused traffic layouts.

## Source and working tree

The owned local experiment/unity-lap-training worktree began clean at e7af050ccfc11838a062ab4e73c5c0ea882a807a. Source and evidence are committed at 1fcf3049949dda26ec4a530b2a422735dce93f31. The frozen solver uses 43423ae. The negative diagnostic source is isolated under reversal_diagnostic at 6d3328a. Only this reviewed handoff was pending after the source commit.

## Work performed

A separate editor ran the installed simulator scene and a 0.01 s physics tick. The A4 camera ran at 5 Hz. Logged the measured lane sections, tracked local objects, selected lateral offset, predicted speed, speed limit, yaw feedback and integer wheel targets. Ran risk_trials.py baseline, risk_curriculum.py, risk_select.py, risk_ablation.py, risk_film.py, command_replay.py, risk_audit.py and risk_results.py. Seeds, weights, timestamps and terminal outcomes are retained in risk_revision. The three additional reversal trials have an independent folder. The private editor exited. The original live editor was still running.

## Evidence and result

See [six paired cases and action sample](evidence/results.json), [integer motor and simulator source audit](evidence/audit.json), and [actual recorded command frame](evidence/command-example.jpg). Four new mixed-traffic layouts produced 2/4 completions with the old policy and 3/4 with the frozen policy. Over common free road the distance-weighted comparison by aggregate distance/time was 3.418 to 3.674 m/s. Two empty-course laps averaged 80.10 to 74.20 s. Measured free-road yaw-acceleration RMS decreased from 0.23 to 0.15 rad/s². In the shared successful 57846 run, spending 5.97 s instead of 4.92 s in the moving zone increased clearance from 0.09 m to 2.32 m. Six frozen acceptance runs produced 2414/2414 identical independently reconstructed CAN pairs. Decoded the 161 video frames and checked Edge playback, timeline updates and mobile width. The live-editor vehicle solver, scene and sensor source matched the private copy.

## Risks and blockers

In 20318 a moving object reversed its estimated lateral velocity. The old controller finished; the new one collided at 236.0 m. Raising only obstacle speed limits still collided. Applying double-sided motion uncertainty to every cone moved failure to the first stationary obstacle. In a previously used practice case a paired run with the legacy lane observer and the new controller performed better. Combined improvements cannot be assigned to the lane observer alone. These are small simulator tests on a fixed course with privileged ego motion for tracking. They do not establish real-vehicle control authority or behavior across battery states.

## Next agent actions

Use the local tracks around the 20318 reversal to test a history-based moving/static uncertainty model. Retain the same static, double-cone and moving regression cases. Freeze the candidate before evaluating additional seeded layouts, with separate availability, clearance, smoothness and speed metrics. The report and reproduction directions are in codexws/learned_racing/unity_laps/README.md.
