# CorrectionControl Model Folder

Place the legacy body-space `correction_control.pt` in this folder.

For the current RPM correction controller docs, see `../README.md`.

The active `motor_controller` executable still loads this body-space checkpoint in all controller states so logger runs can compare old model output even when correction is not applied.

Controller states:

- `controller_state=0`: ideal diff-drive conversion, no empirical tuning and no correction applied.
- `controller_state=1`: copied BKUP tuning, no correction applied.
- `controller_state=2`: live RPM correction when trainer weights are available; ideal RPM while weights/history warm up.

On the standard PS4/DualShock4 Joy mapping used by the launchers:

- Triangle selects state `0`.
- Circle selects state `1`.
- Cross/X selects state `2`.

RPM runtime flow:

```text
base cmd_vel + recent response history + accepted live RPM weights
  -> dynamic affine parameters [a_right, a_left, b_right, b_left]
  -> corrected right/left RPM
  -> CAN frame encoding
  -> CAN frame
```

The model subscribes to:

- `sub_speed_command`: `geometry_msgs/msg/Twist`
- `/aiformula_sensing/vectornav/velocity_body`: `geometry_msgs/msg/TwistWithCovarianceStamped`, using `twist.twist.linear.x` as measured forward velocity
- `/aiformula_sensing/gyro_odometry_publisher/odom`: `nav_msgs/msg/Odometry`, using `twist.twist.angular.z` as measured yaw rate
- `/aiformula_control/joy_node/joy`: `sensor_msgs/msg/Joy`, using button rising edges for state switching

Outgoing CAN frames keep the same ID and 8-byte payload layout as the backed-up differential-drive controller:

- bytes 0-3: right wheel RPM, signed int32 little-endian
- bytes 4-7: left wheel RPM, signed int32 little-endian

The debug comparison CSV defaults to this same `model_controller` folder as `correction_control_debug.csv`, so it can be copied from beside the deployed model weight after a vehicle run.

The controller also publishes `std_msgs/msg/Float64MultiArray` debug rows on:

```text
/aiformula_control/motor_controller/correction_debug
```

The trainer consumes that topic to write timestamped Desktop log CSVs.

Live RPM weights are received on:

```text
/aiformula_control/correction_controller_trainer/rpm_weights
```

Only accepted trainer checkpoints are published on that topic. If the online trainer keeps learning but the checkpoint score does not improve, the controller continues using the previous accepted weight.
