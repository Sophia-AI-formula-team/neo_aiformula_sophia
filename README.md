# AI Formula Sophia

Runnable ROS 2 source snapshot from the Sophia vehicle computer, captured on 2026-07-17 JST.

## Repository layout

- `workspace/src/aiformula`: the ROS 2 workspace source tree from `/home/workspace/src/aiformula` on the vehicle.
- `workspace/src/aiformula/control`: motor control, correction training, end-to-end control, GNSS following, PID control, and trajectory-following packages.
- `dependencies`: the source and platform dependency tree from `/home/dependencies`, including VectorNav, the ZED ROS 2 wrapper, ros2_socketcan, ZED X driver/configuration files, and captured package reports.
- `legacy/control`: archived control code kept for reference and excluded from active workspace builds.
- `codexws`: debug reports, test tools, patches, and source snapshots. `COLCON_IGNORE` prevents these diagnostic copies from creating duplicate ROS packages during a workspace build.

Generated `build/`, `install/`, `log/`, Python caches, bags, and diagnostic run outputs are intentionally excluded.

## Target platform

- NVIDIA Jetson, aarch64, L4T 35.4.1
- Ubuntu 20.04
- ROS 2 Foxy
- CUDA 11.4 and cuDNN 8.6
- ZED SDK 4.1.4 and ZED X driver package 1.0.5

See `dependencies/README.md` for the full captured dependency state.

## Build

```bash
source /opt/ros/foxy/setup.bash
rosdep install --from-paths dependencies workspace/src --ignore-src --rosdistro foxy -r -y
colcon build --base-paths dependencies workspace/src --symlink-install
source install/local_setup.bash
```

The ZED SDK, Jetson camera driver, CUDA stack, and physical device permissions must be installed separately; rosdep does not provide them.

## Run

Bring up the configured vehicle interfaces, then launch the consolidated stack:

```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch launchers allnodes.launch.py
```

LYA is intentionally separate from allnodes:

```bash
ros2 run trajectory_follower lya_0221
```

The launch controls real CAN-connected hardware. Verify the vehicle is safely suspended before starting motor control.

## 2026-07-17 lane fix

The deployed lane-line publisher scales its 1920-pixel `min_component_pixels` threshold to the incoming image width. At 640 pixels the threshold is 60 instead of 180. The test image produced 9 finite, non-zero points in each left, center, and right cloud. Details are in `codexws/DEBUG_REPORT_2026-07-17.md` and its Chinese version.

## Semantic integration (2026-10-12)

The existing vehicle launch remains unchanged. The new semantic wiring has a
dedicated native ROS **shadow** entry, with private intermediate topics and a
signal capture receiver/command watchdog under `control/semantic_drive_bridge`.
It does not launch motor control or teleop. Native perception calibration and
stamped lane/controller provenance still need target commissioning.

- [Semantic deployment and agent handoff](workspace/src/aiformula/perception/semantic_planning_system/README.md)
- [Native shadow launch guide](workspace/src/aiformula/launchers/SEMANTIC_DEPLOYMENT.md)
- [Verified Unity source overlay, installation and rollback](codexws/unity_semantic_integration/README.md)

The Unity archive preserves four final63 red/green inner/outer runs; red stop
distances were 8.636 m and 8.603 m with the original camera. It is an explicitly
separate simulation delivery, not ROS DDS or physical vehicle acceptance.
