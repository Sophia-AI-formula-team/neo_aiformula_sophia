## Launchers
This package manages launch files each node\
**Note:**  Topic name and Frame_id are maneged YAML file

| Items | /path/to/yaml_file |
| :---: | :----------------- |
| Topic Name | /config/topic_lint.yaml    |
| Frame id   | /config/frame_id_list.yaml |

### Launch Files
- `joy.launch.py`
  - Output: `gamepad command`

- `socket_can_bridge.launch.py`
  - Input: `speed command`, Output: `wheel rotation count`

- `teleop.launch.py`
  - Input: `gamepad command`, Output: `speed command`

- `vectornav.launch.py`
  - Output: `Imu`

- `zedx_camera.launch.py`
  - Output: `Image (raw / undostorted), Imu`

### Vehicle perception profiles

`allnodes.launch.py` starts A3 V14 by default through `yolop_a3.launch.py`,
reusing its existing camera and lane publisher. The default camera is
`SN48797506` at nHD; its intrinsic calibration is included in the `vehicle`
package. The motor controller also receives the selected gamepad button layout.

The A3 launcher expects the separately deployed runtime at
`/home/control/aiformula_runtime/yolop_a3_v14_20261012`, including `run_a3.py`,
the Python environment, ROS adapter and V14 checkpoint. Its standalone
`deployment_root` argument can select another prepared runtime. The startup
guard rejects duplicate publishers on the perception output topics.

The default is still `yolop_profile:=V14` so an existing installation does
not change on checkout. With a prepared bundle and matching Python/ROS
environment, explicitly select `yolop_profile:=A4` or
`yolop_profile:=A4_lite`. Both use the existing camera/calibration and
remaps; no motor or control settings change. The launcher selects
`<a4_bundle_parent>/<profile>/venv/bin/python3` and that bundle's
`run_vehicle.py`. See the perception package deployment guide for shadow
validation, dependency requirements and rollback.

The archived detector is opt-in with
`launch_yolop_a3:=false launch_legacy_road_detector:=true`; it requires the
archived source configured by `legacy_yolop_path` and its weights. Enabling
both detectors is rejected. `launch_rviz:=false` disables the RViz startup.
