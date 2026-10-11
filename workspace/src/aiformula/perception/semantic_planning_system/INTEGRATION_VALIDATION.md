# Integration validation — 2026-10-12

This record describes the new integration branch, built on core63. It does not
rename the earlier Unity runs as native ROS evidence.

| Check on the Windows authoring host | Result |
| --- | --- |
| `pytest` bridge pure policies + real-launch test module | 77 passed; 1 module skipped, 0.09 s |
| Real-launch imports | Skipped because ROS launch/ament are not installed |
| Native new Python syntax parsed with Python 3.8 grammar | 7 files passed |
| Protected source diff against 63eec2d | Empty for core/local planner, LLP, points, target filter, LYA, motor and allnodes |
| Package version check | core/local planner 0.2.0; new bridge 0.1.0; launcher 0.1.0 |

The pure policy tests cover source capture age, duplicates, out-of-order and
future frames, null range on real unknown/red, nonfinite values, source/steady
clock rollback, command and status timeouts, stale permitting status, and shadow
topic restrictions. They import no rclpy stub. They do not execute a ROS graph.

Authoring interpreter: Python 3.12.10. Command from the repository root:

```bash
PYTHONPATH=workspace/src/aiformula/control/semantic_drive_bridge python -m pytest \
  workspace/src/aiformula/control/semantic_drive_bridge/tests \
  workspace/src/aiformula/launchers/test/test_semantic_shadow_launch.py -q -p no:cacheprovider
```

On a sourced ROS host, rerun the real-launch module and complete the DDS tests
specified in the deployment guide. The current host has no ROS installation;
neither native DDS nor hardware acceptance was run. Frozen Unity evidence and
archive installer validation are recorded separately in `codexws/unity_semantic_integration`.
