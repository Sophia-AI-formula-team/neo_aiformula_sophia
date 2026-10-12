# Vehicle motor controller import — 2026-10-12

The user requested the motor controller from `Sophia-AI-formula-team/aiformula_sophia`, deployment to the vehicle, replacement of the neo package, version control and compilation.

## Source and behavior

- Upstream main pinned at `dd55f65ef2fe434512d9f7bf48fff7c17141fac3`, path `aiformula/control/motor_controller`.
- Neo package version changes from 0.1.1 to 0.2.0. Upstream's metadata was 0.0.0.
- The controller algorithm is byte-identical to upstream: SHA256 `224cee23e3151d94cdd5864963c57266b0ab2297ba68f155d24ce93193d44d0e`.
- This replaces selectable learned control with upstream fixed wheel-speed compensation. The old model assets remain in Git history and the vehicle backup. Topic remaps and CAN ID 0x210 retain the existing interface.
- Integration changes are package/version/dependency metadata, MIT attribution, documentation and removal of the broken `motortest` entry point (upstream supplies no corresponding module). Tracked bytecode is excluded. `UPSTREAM.json` records original source hashes.

## Deployment and evidence

Vehicle workspace: `/home/workspace`. Runtime receipts: `/home/control/aiformula_runtime/motor_controller_20261012_0_2_0`.

Only `motor_controller` was compiled, using `colcon build --base-paths /home/workspace/src/aiformula/control/motor_controller --packages-select motor_controller`. Build exit code was 0; installed algorithm hash matched the pinned source. The deployment receipt verifies that semantic build/install trees and allnodes source were unchanged. No vehicle controller was automatically restarted.

A real Foxy/rclpy test ran the installed console executable in ROS domain 93, localhost only, with private input/output topics. Five commands (zero, forward, turning, reverse turning, zero) produced the expected upstream CAN payloads using vehicle wheel parameters 0.60 m tread, 0.254 m diameter and 1.1 gear ratio. Seventy frames were received, approximately 100.5 Hz. Final payload was zero. No test command reached the hardware CAN topic. The test process needed SIGTERM after its SIGINT wait and exited -15; clean SIGINT shutdown is not established. This is isolated DDS acceptance, not physical driving or braking acceptance.

The upstream controller repeats cached commands and has no independent input watchdog. Importing it does not resolve the separately documented semantic control timeout requirement.

## Activation and rollback

Source/build/install backups are under `/home/control/aiformula_backups/20261012_motor_controller_0_2_0/{source,build,install}`. The old source main Python file was already missing before deployment; its installed Python implementation is preserved in the install backup. Do not claim the old source backup is directly rebuildable.

After stopping any existing vehicle launch, a fresh shell with `source /opt/ros/foxy/setup.bash` and `source /home/workspace/install/setup.bash`, followed by `ros2 launch launchers allnodes.launch.py`, loads the installed 0.2.0 controller. An already running Python process does not reload code when compilation finishes.

For rollback, stop the relevant launch, preserve the current controller source/build/install directories under unused names, and restore all three corresponding backup directories with symlinks preserved. Use a fresh shell and restart only after restoration. Keep the backup immutable. To rebuild the old version later, first recover the missing source module from its preserved installed copy or its exact Git revision.
