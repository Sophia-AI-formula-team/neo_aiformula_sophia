# Sophia AI Formula ROS 2 Snapshot

This student-maintained repository contains a public ROS 2 source snapshot used for AI Formula research in Sophia University's Control Engineering Laboratory. It is provided for research reference and is not an official Honda or Sophia University software release.

## Repository layout

- `workspace/src/aiformula`: the ROS 2 workspace source tree for vehicle integration, sensing, perception, control, launch, and supporting packages.
- `workspace/src/aiformula/control`: motor control, correction training, end-to-end control, GNSS following, PID control, and trajectory-following packages.
- `dependencies`: vendored source and platform records, including VectorNav, the ZED ROS 2 wrapper, ros2_socketcan, and captured package-version reports.
- `legacy/control`: archived control code kept for reference and excluded from active workspace builds.

Generated `build/`, `install/`, `log/`, Python caches, bag recordings, and internal debug reports are not part of the public tree.

## Target platform

- NVIDIA Jetson, aarch64, L4T 35.4.1
- Ubuntu 20.04
- ROS 2 Foxy
- CUDA 11.4 and cuDNN 8.6
- ZED SDK 4.1.4 and ZED X driver package 1.0.5

See `dependencies/README.md` for the captured dependency state. Hardware-specific SDKs, drivers, and permissions must be configured separately.

## Build

```bash
source /opt/ros/foxy/setup.bash
rosdep install --from-paths dependencies workspace/src --ignore-src --rosdistro foxy -r -y
colcon build --base-paths dependencies workspace/src --symlink-install
source install/local_setup.bash
```

The ZED SDK, Jetson camera driver, CUDA stack, and physical-device permissions must be installed separately; `rosdep` does not provide them.

## Run

On a properly configured vehicle, the consolidated stack can be launched with:

```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch launchers allnodes.launch.py
```

LYA is intentionally separate from allnodes:

```bash
ros2 run trajectory_follower lya_0221
```

This launch can actuate real CAN-connected hardware. Use it only with an emergency stop available and the vehicle secured for testing.

## Provenance and contribution boundary

This is an aggregated workspace rather than a record of single-author development. Its commit history documents public snapshot curation and later integration changes, but it does not establish original authorship of every package.

Owen Zi-Wen Zhou maintains the public snapshot and has contributed repository curation, integration, and documentation work recorded in the commit history. Earlier team code and vendored upstream projects retain their original authorship and license terms; repository ownership does not imply sole authorship.

Before publishing new material, remove credentials, private network or device details, internal deployment logs, bag recordings, and data or model files that are not cleared for redistribution.

## License

See [LICENSE](LICENSE) for the repository license. Vendored projects and other third-party material may carry separate licenses and notices.
