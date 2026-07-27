# Dependency record

This directory records the dependency state of the remote vehicle on 2026-07-17 JST.

## Authoritative dependency declarations

ROS and system dependencies are declared by the `package.xml`, `CMakeLists.txt`, `setup.py`, `pyproject.toml`, and `requirements.txt` files under `dependencies` and `workspace/src`.

Install resolvable ROS/system dependencies with:

```bash
source /opt/ros/foxy/setup.bash
rosdep update
rosdep install --from-paths dependencies workspace/src --ignore-src --rosdistro foxy -r -y
```

## Captured remote state

- `remote-platform.txt`: kernel, OS, ROS, Jetson, CUDA, cuDNN, camera-driver, Python, CMake, and colcon versions.
- `remote-ros-packages.tsv`: every installed `ros-foxy-*` Debian package and version.
- `remote-apt-manual.txt`: all packages marked manually installed on the remote. This is a reproducibility snapshot, not a minimal install list.
- `remote-pip-freeze.txt`: complete remote Python package snapshot. Do not install it blindly on non-aarch64 systems.
- `stereolabs-zedx`: the remote ZED X driver package and camera setting.
- `zed_camera_params`: the serial-specific camera intrinsic and extrinsic parameters.

## Non-rosdep requirements

- NVIDIA Jetson Linux/L4T 35.4.1
- CUDA 11.4 and cuDNN 8.6
- ZED SDK 4.1.4
- `stereolabs-zedx` 1.0.5 for L4T 35.4.1
- configured `can0` interface and permissions for `/dev/ttyUSB0`, joystick, and camera devices

## Vendored source provenance

- VectorNav: upstream commit `2a2789eaf8fb3ec99e5effc0df9c731d7933d74c`, with the Foxy-compatible `tf2_geometry_msgs.h` include used on the vehicle.
- ZED ROS 2 wrapper: upstream commit `1d015f3a4881aa50661e2ec60eadddc032a1cd5e`.
- ros2_socketcan: upstream commit `85da8c31286cad69de65dc50219e7d901ae9e94b`.

Their `.git` directories are excluded; complete source files are vendored directly under `dependencies`.
