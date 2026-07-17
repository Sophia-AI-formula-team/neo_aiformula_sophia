# AI Formula Sophia

Runnable ROS 2 source snapshot from the Sophia vehicle computer, captured on 2026-07-17 JST.

## Repository layout

- `src/aiformula`: vehicle, sensing, perception, control, launch, and common packages from `/home/workspace/src/aiformula` on the vehicle.
- `src/trajectory_follower`: trajectory followers, including the separately launched `lya_0221` executable.
- `src/e2e_zw`, `src/gnss_follower`, `src/pid_controller`, `src/sine_cmd_publisher`, and `src/correction_controller_trainer`: other remote ROS packages.
- `src/aiformula/sensing/vectornav`, `src/aiformula/sensing/zed-ros2-wrapper`, and `src/ros2_socketcan`: vendored source dependencies required by the installed remote stack.
- `dependencies`: exact remote platform, ROS package, apt, and Python package snapshots.
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
rosdep install --from-paths src --ignore-src --rosdistro foxy -r -y
colcon build --symlink-install
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

