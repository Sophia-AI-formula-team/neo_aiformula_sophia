# AI Formula Sophia

Runnable ROS 2 source snapshot from the Sophia vehicle computer, captured on 2026-07-17 JST.

## Lane teach/repeat and agent handoff

The canonical destination is **Sophia-AI-formula-team/neo_aiformula_sophia**
(repository ID 1303517209), branch `feat/causal-lane-teach-repeat`.
Start with [AGENT_CONTEXT.md](AGENT_CONTEXT.md), [current handoff](docs/agent-context/STATUS.md)
and [experiment recording/upload protocol](docs/agent-context/PROTOCOL.md).
The three lane-mapping packages are under `workspace/src/aiformula/control/`, beside LYA.
GNSS in `_gnss` is only a fixed-route start check, never a mapping input.
CI and handoff receipts do not authorize driving the vehicle.

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
