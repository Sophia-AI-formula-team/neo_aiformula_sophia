# Semantic joint-matching experiment 0.4.0 — 2026-11-16

The user supplied `1115(1).zip` to test new topo-map functionality and its own
traffic/detour logic. The user explicitly chose an experimental source update
and a version increment, rather than merging the older traffic implementation
back into the new algorithm. Both `semantic_planner_core` and
`semantic_local_planner_ros` are version **0.4.0**. The source archive declares
0.1.0; only those four version fields were changed during source preparation.

The archive SHA256 is
`4126492d69cca6040e7617f06c0010c78308d4bc4a41bf2516df22b264dcc527`.
[SOURCE_MANIFEST.json](SOURCE_MANIFEST.json) records all 55 files, raw source
hashes, the version exceptions and current checkout hashes. Every supplied
runtime algorithm and test matches the archive after line-ending normalization.
The two supplier documents are preserved as evidence, not agent instructions:
[usage](SUPPLIER_USAGE.txt), [validation](SUPPLIER_VALIDATION.txt).

## Source placement

The repository source remains at
`workspace/src/aiformula/perception/semantic_planning_system/`:

- `semantic_planner_core/`: route fusion, joint alignment and topology utilities.
- `semantic_local_planner_ros/`: MapAssist node, its config, launch and tests.

The current worktree is
`E:/Mess/Projects/Programming/aiformula/workspace/semantic_joint_20261116`, branch
`feat/semantic-joint-20261115`, based on main commit
`9369bbabf097c405d27e15f738d70e72f917a3a0`. Keep the historical canonical checkout
and its unrelated changes intact. Unity receives a separately identified export
of this source; its transport/recording tools must not become the canonical ROS
package or mutate the archived input.

The exact original ZIP and all extracted files are retained locally under
`E:/Mess/Projects/Programming/aiformula/archive/semantic_1115_20261116/`.
The archive is for recovery/provenance, not a second ROS build root. Restrict
colcon discovery to the selected neo worktree's `workspace/src/aiformula` tree.

## Experimental compatibility boundary

This replaces the two selected packages' supplied files with a new experiment;
it is not a drop-in upgrade for every older integration. In particular:

- New MapAssist already contains traffic voting, wait/detour state and OA yield.
  The supplied defaults keep `enable_traffic_control=false`. A red-detour trial
  must explicitly record its traffic-enabled simulation configuration.
- The former `traffic_stop_required`, `stop_wait` and metric signal-approach
  interfaces are absent from the supplied MapAssist. Old motor gate / bbox /
  checkpoint test results do not establish compatibility with this experiment.
- The supplied setup omits the `race_map` executable and changes MapAssist's
  constructor/startup signatures. Inherited `race_map.py`, its docs and other
  repository-only files remain preserved, but are not part of the 1115 source
  manifest and must not be described as tested or supported by this import.
- The optional `lane_0927oa.py` supplement changes ABC indices from the current
  repository's `(0, 2, 4)` to `(4, 6, 8)`. It is archived, not installed over the
  current lane source. Any use in the simulation must be explicit and recorded.
- Historical 0.3.1 checkpoint/Unity receipts belong to their exact old source
  pin. This new input does not inherit those closure, recovery or stop-distance
  acceptance claims merely because its release number is higher.

## What needs to be observed

The requested simulation records first-person RGB and the actual semantic map
output while completing a lap, then observes whether red triggers an internal
route entry after the map is established. Distinguish:

1. odometry reference closure (`odom_reference_ready`);
2. asynchronous fusion pending versus accepted/rejected official topology;
3. correct outer/internal-node placement against the actual track;
4. selected detour and actual vehicle motion into that internal route.

A closed odometry loop alone is not proof of a correct fused topo map, and a
detour node list alone is not proof that the vehicle entered it. Preserve failed
attempts and show the actual state in the video. Do not hide ambiguity by
relaxing fusion gates or adding a manually calibrated control route.

## Verification status at source preparation

Completed here: ZIP hash, bounded path-safe extraction, 55-file archive readback,
source comparison, paired version-field equality and lane-supplement diff.
No Unity, ROS/DDS, hardware or algorithm tests were run by this preparation step.
The supplier reports 96/103 core tests passing and seven failures in the inherited
route-fusion suite; it reports additional focused tests separately. It explicitly
does not claim real rosbag, ROS2/Foxy colcon or vehicle validation. These are
supplier statements until independently reproduced. The root task records new
test and simulation evidence separately before final publication.

Before any necessary algorithm repair, record the concrete input, failure and
minimal proposed change in a new problem report. Keep the original archive and
its test outcomes immutable. Publication uses scoped commits and a reviewable
feature PR; no release tag or physical vehicle deployment is implied.
