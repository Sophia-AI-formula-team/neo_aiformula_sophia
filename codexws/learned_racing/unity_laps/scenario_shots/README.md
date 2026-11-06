# A4 mixed traffic and corner footage

These are illustrative reproducible Unity captures using the frozen balanced policy and the same installed motor physics as the risk-aware experiment. Each successful new run is a short segment. These selected cases are outside the blind benchmark.

| Run | Actual scene | Result |
|---|---|---|
| `mixed_01` | Straight, one static cone followed 2.75 m later by a sinusoidally moving cone. Both evaluation zones overlap for 4.65 s. | Passed, 2.89 m minimum clearance. The tracker identified distinct targets matching both cones for 16 consecutive decisions. |
| `apex_01` | One static cone at outer-lane station 114.18 m, mid-corner. The local curvature is 0.0490/m; lane heading changes by 41.74° over a 15 m window. | Passed, 1.75 m minimum clearance. |
| `apex_pair_01` | Both a static and a laterally moving cone within the same corner, at 112.60 m and 115.45 m. Their zones overlap for 4.53 s. | Passed, 1.51 m minimum clearance. Both cones match distinct tracks for 22 consecutive decisions. |
| `failure_20318` | Full prior frozen holdout run with the originally randomized four-prop layout. | Contact at 71.74 s, 236.01 m, with the fourth (moving) cone. The complete physical trajectory, CAN commands, speed and yaw commands, and terminal result match the frozen record exactly. |

Each `runs/<id>.json` contains original telemetry, control decisions, tracker states, obstacle definitions and camera timestamps. Each `encounters/<id>.json` includes referee measurements, frame counts and checksums. Distinct matches use a one-to-one correspondence between current local tracks and the two real cone centers with maximum 1.25 m position error. The scene geometry used for grading is not provided to the driving policy.

The three synchronized image streams are kept locally in `.work/frames/<id>/*c.jpg` (Unity chase) and `*f.jpg` (front RGB) at 20 Hz, and `.work/camera/<id>/a4_samples/*.jpg` (actual A4 overlays) at 5 Hz. Stream digests concatenate each sorted basename, a zero byte, and the binary SHA256 of its contents. Run identifiers, weights, source version, field layout and terminal events are logged. Reproduce in a separate private Unity project with the parent directory's `scenario_shots.py` and these four IDs.
