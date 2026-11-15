# Live results through `inner_joint_04`

The fourth attempt completed one physical lap and built a closed **odometry
reference**. It did not complete two laps, accept an official topology match, or
enter a red-triggered interior route. Both recorded attempts below failed their
full acceptance criteria. The supplied semantic algorithm, official OSM,
original LYA, camera calibration and vehicle physics were unchanged.

| Attempt | Physical progress | Map / traffic result | Ending |
| --- | --- | --- | --- |
| `inner_joint_03` | 0.36709 m; 0 laps; 36.003 simulation seconds | No learned reference or red schedule | Harness incorrectly counted cold startup against its 35-second no-progress budget |
| `inner_joint_04` | 308.3987 m; 1 lap; 334.685 simulation seconds | 259.5646 m odometry reference; official fusion rejected; red wait, then recorded green recovery; no detour | Correct post-start 35-second no-progress watchdog after a later wait |

The source/configuration inventory remained unchanged within each run. This is
not a statement that runtime rendering settings were constant in run 04: its
own Unity render cap changed during the run as explicitly recorded below.

## `inner_joint_03`: evaluator startup timing

The first detection arrived about 30.52 seconds old and was rejected. A genuine
fresh frame started motion, a later 4.236-second inference was stale and caused
the native zero-command hold, and another fresh frame arrived around the old
watchdog deadline. The test stopped because its 35-second budget included cold
startup. This did not establish a mapping or traffic failure.

Before run 04, the harness gained an independent 90-wall-second startup bound.
The first actual nonzero applied command records `motion_control_started` and
starts the existing 35-simulation-second progress budget. It does not reset the
trajectory, traveled distance, lap count, clocks, or startup recordings. No
perception TTL or semantic algorithm was changed. Evidence:
[inner03_summary.json](evidence/inner03_summary.json).

## `inner_joint_04`: one lap, unavailable official map

The first applied motion was recorded at simulation time 27.5502 s. The package
reported a closed odometry reference near 260.6000 s; the physical start-plane
return was recorded at 260.9938 s after 259.4106 m of travel. These are separate
events. The closed reference length was 259.5646 m.

Official fusion never became available. Its final reason was
`excluded_gap_too_long`, with `mode=odom_only` and `graph_frame=official_unaligned`.
The summary contains 966 learned samples, 33 marked excluded, and an untrusted
reference gap of about 11.80 m. These facts do not establish the geometric cause
of every excluded sample. In particular, this mixed-render run cannot determine
whether a fixed rendering environment eliminates that fusion failure. The
official graph must remain visually distinct from the learned odometry path;
interior nodes are estimates from the official graph, not observed interior-road
mapping.

At the physical first-lap return the existing lamp was switched red. The package
observed red at 262.2558 s and held `red_wait_map_fusion`; no detour was generated
and no measured interior left turn occurred. After the stationary red interval,
the harness saved `red_detour_failed=true` at 274.6142 s, then switched the physical
lamp green solely to attempt completing the second lap. The package later
reported `green_resume`. This recovery does not count as red-triggered turning.

At 298.5900 s the package entered another `red_wait_map_fusion` state following
its received red observations. The physical lamp schedule was green. Possible
perception class/color cross-detection is **pending independent review**; no
perception correction or definitive causal attribution is made here. The car
then remained stopped and the post-start no-progress watchdog ended the trial.
The second lap remained incomplete. Worker shutdown was clean, exit 0; cleanup
reported no errors. Evidence: [inner04_summary.json](evidence/inner04_summary.json).

## Rendering-load diagnostic, not an algorithm fix

At wall time about 94 s, progress was only 7.429 m and recent inference duration
ranged from about 20 ms to 4.342 s. GPU utilization was observed at 100%, with
multiple graphics/compute applications active. That observation does not
identify a particular process as the cause and does not establish exclusive GPU
use.

At simulation time **168.4208 s**, only the owned Unity render settings changed
from `targetFrameRate=-1, vSyncCount=1` to `30, 0`. `Time.timeScale=1` and the
actual 0.01-second physics timestep stayed unchanged. Camera, weights, TTLs,
source, official map, and vehicle physics were not changed.

| Recorded segment | Frames | Median inference | Median capture age | Capture age over 0.5 s |
| --- | ---: | ---: | ---: | ---: |
| Before cap | 80 | 692.37 ms | 0.87939 s | 48 |
| Spanning transition | 1 | 1262.99 ms | 2.12527 s | 1 |
| After cap | 754 | 26.75 ms | 0.12993 s | 0 |

The post-cap inference p95 was 43.63 ms and capture-age p95 was 0.16496 s. These
are within-run observed segments with different rendering settings, not a
controlled proof of a single GPU cause or a full fixed-30-FPS acceptance run.
The next proposed `inner_joint_05` starts with the same cap fixed from the start;
its result remains pending. Root verifies the Unity surface/settings and owns
that run. Original recordings from run 04 remain intact.

The pre-adjustment analysis and exact transition are preserved in
[RENDER_LOAD_ANALYSIS.md](evidence/RENDER_LOAD_ANALYSIS.md) and
[inner04_render_cap_transition.json](evidence/inner04_render_cap_transition.json).

## Frozen code and remaining acceptance

| Component | SHA256 |
| --- | --- |
| Transport | `cf506d82e22f1cf10ea26011229bad71c3e368a97121d9673039bb25ea0911a8` |
| Joint command consumer | `ad9dfd2e2353babe55cc9b7e30f26040b20821a8249fd4ffd5ea6c1dfc626a08` |
| Harness | `90c1428401a36eb41f933c282ea93da8fefd95918b7dcc1f212073ff5d4b6be9` |
| Diagnostic worker | `cff1e44ffd6fe5fc2184c6659e933277a6db79454c1f7c04afe23f5ac8591674` |
| Original MapAssist | `9b8590588e707173d6e623f144c41e6c1dd97531f92ce4282415eb4dc7eedb5b` |

Semantic source remains version 0.4.0 at commit
`6b71267e24e81459220c7e6d30d84c3f7a00e591`. No algorithm fix is claimed. Fixed-cap
inner validation, the outer start, a verified official junction correspondence,
two completed physical laps, red-triggered interior entry, and final reviewed
first-person/semantic videos remain outstanding.
