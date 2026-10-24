# Recorded-prefix regression, not a completed lap

The fixture contains only observed simulation wheel/gyro samples and original
ideal-camera mask PNGs, with calibration and expected pose **digests**, not input
poses. The sensor model is not a physical vehicle measurement. No GNSS, world
truth, authored route, future frames, controller or teacher commands are inputs.

52 batches contain 566 wheel/gyro samples and 32 mapping masks. Replay recomputes
continuous raw odometry and one fixed teach origin, verifying every digest.
The unchanged baseline fails at sequence52 after31 accepted mapping frames;
the repair accepts32/32 with the same motion, matcher and teach configuration.
The simulator-specific keyframe refresh override remains0.25s/0.25m in both runs.
This is the exact observed prefix, **not** the future trajectory under changed
control and not proof of stopping, route preparation or a second lap.

Files:

- `trial13_sensor_fixture.json`: portable inputs, SHA256
  `7d9f6db31bf55d17c84146282e74d05569bbc34e1b8fd7c134aa92187f32d39d`.
- `trial13_baseline_prefix_02.json`: baseline Git blobs at85ddb702. Source content
  matches the capture after explicitly accounting only for LF/CRLF differences.
- `trial13_fixed_prefix_03.json`: repaired worktree output with actual/active
  reference frontier distinguished. Source hashes are recorded in the report.
- Repository `tools/replay_observed_keyframe.py` and its helper tests: standalone offline runner;
  requires already available Python, NumPy and Pillow. No ROS or Unity calls.

From the repository root, with an output path that does not exist:

```sh
python -B tools/replay_observed_keyframe.py replay --repo . --fixture docs/agent-context/sessions/2026-10-11-planner-keyframe-005/evidence/trial13_sensor_fixture.json --output fixed-replay-new.json
```

Add `--revision 85ddb70209e6c346c422f9ae8b2a88cb49da72e0` to reproduce the old failure.
Real processing clocks remain enabled, so a loaded machine can reject a deadline;
do not disable budgets to force the expected result. Reports never overwrite.

No raw bag, personal absolute paths, exact GNSS, world-truth logs or complete
source snapshots are included. Original failure/trial evidence stays local.
