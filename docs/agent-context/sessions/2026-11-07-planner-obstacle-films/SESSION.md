# Unity obstacle films

## Objective and scope

Show different obstacle situations with synchronized chase, real camera perception and local planner decisions. Work is confined to a private Unity copy and existing simulation logs. The driving policy is frozen.

## Input handoff

Continues 2026-11-06-planner-risk-aware-laps. Reviewed its paired results, command audit, source traces, simulator evidence and capture directories. Its original mixed-traffic lap placed the moving cone far from the static pair. The filmed static and moving obstacles from that lap were all on straights.

## Source and working tree

Task started at clean 3e14b95309ad288568efd3905c87490b317c1424 on experiment/unity-lap-training, using independent scenario and visualization worktrees. Source capture e202692 uses the same frozen A4 and f25947e8af56df solver. Real inferences, camera frames and decisions refer to the same Unity trial. The interactive report and individual films are in codexws/learned_racing/unity_laps/obstacle_films.

## Work performed

Three straight encounters reuse scored seed 93677. Staged and ran three additional short scenes with scenario_shots.py: close-spaced static and moving cones on a straight, one cone in a bend, and static and moving cones together in that bend. Reproduced seed 20318 to terminal contact. The private Unity editor exited normally. Every new run logs start and termination, scene geometry and three ordered camera-stream digests. Rebuilt the local archive and checked all twelve stream digests.

## Evidence and result

Each individual movie has a real chase image, A4 overlay and local decision view. A fixed ground reference distinguishes cone motion from camera motion. The film suite index records per-clip sources, timestamps and SHA256; the full H.264 has chapter marks. The two-cone straight encounter cleared by 2.89 m; the single-cone bend by 1.75 m; the two-cone bend by 1.51 m. On the mixed bend the two actual cones matched two distinct tracks for 22 consecutive control updates. The final failure segment ends at 71.74 s and 236.01 m. It matches the original blind evaluation in all physical rows, decisions and commands.

The completed suite is recorded at 86f6fe6cff6f41e460c16079a1ae157896763fa6. The final reel contains 1,469 frames at 20 fps (73.45 s, 2048 x 1280). Every chapter was decoded and checked against the joined sequence. Twenty-two frames were also checked against both source JPEG channels, ignoring the clearly labeled overlay in the chase camera. All seven insets were verified by reconstructing clearance from 734 logged poses. [Chapter evidence](evidence/verification.json) and [referee geometry check](evidence/referee_geometry_check.json).

## Risks and blockers

The three posed examples are successful segments on a fixed course. They are separate from blind performance estimates; the failed 20318 case remains a regression. Object tracks can persist briefly after a missed detection. Only the referee visualization uses obstacle truth. Terminal contact uses the simulator scoring envelope. This is simulator footage.

## Next agent actions

Open obstacle_films/index.html for chapter selection or all-scenes.mp4 for standalone playback. Use the archived data and capture/renderer commands in the lab README to reproduce or extend the cases. New performance claims require fresh, predefined layouts.
