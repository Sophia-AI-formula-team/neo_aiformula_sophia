# Semantic-Costmap-Guided Lightweight Local Planning System

This workspace contains a minimal layered prototype for a low-speed ROS2 robot vehicle. It does not modify perception models or lane detection code. Real perception should connect by publishing masks and semantic states to the topics listed below.

## Architecture

- `semantic_planner_core`: pure Python planning library with no ROS2 imports.
- `semantic_gate_ros`: turns traffic light state plus odometry into JSON semantic constraints and a saved `theta_ref`.
- `semantic_costmap_ros`: converts external road/offroad/lane masks into a local semantic costmap.
- `semantic_local_planner_ros`: scores short-horizon motion primitives and publishes a planner command.
- `semantic_lane_recovery_ros`: gates lane-follow recovery so lane detection cannot directly take over during diversion.
- `semantic_safety_ros`: final authority over the configured final velocity topic; stops on stale data, invalid primitives, red forward violations, or unsafe commands.
- `semantic_test_tools`: fake publishers and a pure Python demo.

## Topic Graph

Inputs from external perception:

- `/traffic_light/state` (`std_msgs/String`): `red`, `green`, or `unknown`.
- `/aiformula_perception/road_detector/mask_image` (`sensor_msgs/Image`): default road/lane mask input from the AIFormula road detector.
- `/semantic_perception/offroad_mask` (`sensor_msgs/Image`): green/off-road forbidden mask from a semantic adapter.
- `/lane_guidance_mask` (`sensor_msgs/Image`, optional): soft recovery guidance only.
- `/lane_result` (`std_msgs/String`): JSON lane result.
- `/aiformula_sensing/gyro_odometry_publisher/odom` (`nav_msgs/Odometry`): default current orientation input.

Internal outputs:

- `/semantic_constraints` (`std_msgs/String`): JSON `SemanticConstraints`.
- `/theta_ref` (`std_msgs/Float32`): heading saved when red is stable.
- `/local_semantic_costmap` (`sensor_msgs/Image`): grayscale semantic costmap.
- `/selected_primitive` (`std_msgs/String`): selected motion primitive.
- `/selected_path_debug` (`nav_msgs/Path`): sampled path points in normalized image space.
- `/planner_cmd_vel` (`geometry_msgs/Twist`): planner output before final safety.
- `/planner_score_debug` (`std_msgs/String`): JSON score details.
- `/lane_recovery_state` (`std_msgs/String`): `ignore_lane`, `candidate`, or `recoverable`.
- `/aiformula_control/game_pad/cmd_vel` (`geometry_msgs/Twist`): default final supervised command for the current AIFormula motor-controller launch setup.
- `/safety_state` (`std_msgs/String`): safety supervisor state.

## AIFormula Topic Alignment

The defaults in `config/semantic_system.yaml` were checked against `SophiaControl/AIformula_sophia`:

- Existing odometry: `/aiformula_sensing/gyro_odometry_publisher/odom`.
- Existing road detector output: `/aiformula_perception/road_detector/mask_image`.
- Existing lane line outputs: `/aiformula_perception/lane_line_publisher/lane_lines/left`, `/right`, and `/center`.
- Existing motor controller input path: `/aiformula_control/game_pad/cmd_vel`.

Two semantic inputs are still adapters, not existing native topics in that repository:

- `/traffic_light/state`: should be produced by a traffic-light or red-pixel semantic adapter as `red`, `green`, or `unknown`.
- `/semantic_perception/offroad_mask`: should be produced by a semantic segmentation/green-region adapter. Without this mask the costmap node stays conservative.

When connecting to the real vehicle, avoid running another controller that also publishes `/aiformula_control/game_pad/cmd_vel` at the same time. Use either this safety supervisor as the final publisher or add a command mux.

## Training-Free Color Mask Adapter

If YOLOP drivable-area segmentation is unavailable, run `color_semantic_mask_node`.
It uses simple HSV/value thresholds:

- dark/black lower-image pixels -> `/semantic_perception/road_mask`
- green lower-image pixels -> `/semantic_perception/offroad_mask`
- debug overlay -> `/semantic_perception/color_mask_debug`

This is not a trained perception model. It is a conservative adapter for early
vehicle tests on a black road with green forbidden regions. Tune
`road_value_max`, `road_saturation_max`, `green_h_min`, `green_h_max`,
`green_s_min`, `green_v_min`, `road_roi_top_ratio`, and
`offroad_roi_top_ratio` in
`semantic_vehicle_shadow_demo.yaml` for the actual lighting.

## Running With ROS2

From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch semantic_bringup semantic_system.launch.py
```

For a vehicle-side static shadow demo that uses the real camera image but never
publishes to the motor controller command topic:

```bash
ros2 launch semantic_bringup semantic_vehicle_shadow_demo.launch.py
```

This demo publishes final supervised velocity to `/semantic_demo/cmd_vel_shadow`
instead of `/aiformula_control/game_pad/cmd_vel`, so it is suitable for checking
camera masks, costmaps, selected primitives, and safety decisions while the robot
is physically unable to move.

To force a red-gate test without moving the robot, publish:

```bash
ros2 topic pub /traffic_light/state std_msgs/msg/String "{data: red}" -r 5
```

Then watch:

```bash
ros2 topic echo /semantic_constraints
ros2 topic echo /selected_primitive
ros2 topic echo /planner_score_debug
ros2 topic echo /semantic_demo/cmd_vel_shadow
```

For RViz2, add a `MarkerArray` display on `/semantic_planner/primitive_markers`
with fixed frame `base_footprint`. Green is selected, blue is valid but not selected,
and red is invalid. See `VEHICLE_SHADOW_DEMO_GUIDE.txt` for a full vehicle-side
startup checklist.

Fake input tools can be started separately, for example:

```bash
ros2 run semantic_test_tools fake_mask_publisher_node --ros-args -p mode:=red_gate_left_open
ros2 run semantic_test_tools fake_traffic_light_node --ros-args -p state:=red
ros2 run semantic_test_tools fake_lane_result_node --ros-args -p mode:=valid
```

## Pure Python Tests Without ROS2

The core package is intentionally ROS-free:

```bash
python -m pytest src/semantic_planning_system/semantic_planner_core/tests
```

The non-ROS demo can also be run from the workspace root:

```bash
python src/semantic_planning_system/semantic_test_tools/scripts/run_core_demo.py
```

It saves debug images and planner tables under `debug_outputs/`.

## Debugging the Pure Python Planner

Run the demo without ROS2 from the workspace root:

```bash
python src/semantic_planning_system/semantic_test_tools/scripts/run_core_demo.py
```

The script cleans and regenerates:

- `debug_outputs/red_gate_left_blocked/`
- `debug_outputs/red_gate_left_open/`

Each scenario directory contains:

- `road_mask.png`: white pixels are drivable road.
- `offroad_mask.png`: white pixels are off-road or green-region hard forbidden cells.
- `costmap.png`: low values are preferred, medium values are unknown, white lethal cells are forbidden.
- `selected_path_overlay.png`: only the selected primitive path is drawn in green.
- `all_primitives_overlay.png`: every moving primitive path is drawn; green is selected, blue is valid but not selected, red is invalid.
- `planner_debug.csv`: one row per primitive.

Expected results:

- `red_gate_left_blocked`: red gate is active and the left diversion area is lethal, so the planner should select `STOP`.
- `red_gate_left_open`: red gate is active but left diversion is open, so the planner should select one of `SMALL_LEFT`, `MEDIUM_LEFT`, `STRONG_LEFT`, or `LEFT_THEN_ALIGN`; it must not select `SLOW_FORWARD`.

Read `planner_debug.csv` by checking `valid`, `selected`, `lethal_hit`, `red_gate_hit`, and `reason`. Red-light forward blocking and off-road/lethal cells are hard constraints. `theta_ref` only affects `heading_cost` as a soft prior and cannot make a forbidden primitive valid.

In the current `red_gate_left_open` synthetic case, `STRONG_LEFT` is the only valid moving primitive. The other left primitives still pass through the red forward forbidden ROI early in their sampled paths, so their CSV rows show `red_gate_hit=True`. This is useful for checking whether the open corridor should be widened or whether those primitives should start farther left.

## Parameters

Parameters are centralized in `config/semantic_system.yaml`. Key groups:

- Traffic light stability: `red_stable_frames`, `green_stable_frames`.
- Costmap costs and red forward ROI: `low_cost`, `unknown_cost`, `lethal_cost`, `red_forward_*`.
- Planner scoring: `heading_weight`, `smoothness_weight`, `curvature_weight`, `semantic_cost_weight`, `progress_weight`.
- Lane recovery gate: `stable_required_frames`, `min_confidence`, `horizontal_angle_threshold_deg`, `max_center_jump_ratio`.
- Safety limits: `max_linear_velocity`, `max_angular_velocity`, stale timeout, clamping policy.

## Known Limitations

- The first version is image-space only; no BEV projection is implemented.
- Motion primitives are simple normalized image-space samples, not dynamically simulated trajectories.
- The semantic costmap assumes perception masks are already aligned.
- Lane guidance is only a soft cost reduction after recovery is allowed; it never directly controls velocity.
- ROS2 wrappers are syntactically structured but require a ROS2 environment with `rclpy`, `cv_bridge`, and standard message packages.

## Future Work

- Add calibrated image-to-ground projection or BEV costmaps.
- Add robot footprint checking and time-parameterized trajectory rollout.
- Add richer diagnostics and visualization overlays.
- Integrate with a real low-level controller and hardware stop chain.
- Tune costs and primitive geometry on recorded robot data before live testing.
