# motor_controller

Selectable ROS 2 motor controller for the Aiformula vehicle.

This package owns the final command path from body velocity command to CAN motor RPM. It supports three controller states so a run can compare the old parameter-tuned behavior against the new live RPM correction model.

## Controller States

`controller_state` defaults to `1` in `config/motor_controller.yaml` for backward compatibility.

- `0`: ideal differential-drive conversion only. No empirical tuning and no model correction.
- `1`: copied BKUP parameter-tuned controller. This is the normal safe default.
- `2`: corrected controller RPM mode. It applies a live RPM affine model when accepted weights and history are available.

The standard PS4 controller mapping is:

- Triangle: state `0`
- Circle: state `1`
- Cross/X: state `2`

When the state leaves `2`, live RPM weights are unloaded. State `0` and state `1` ignore live weight messages.

## RPM Correction Flow

In state `2`, the controller can receive a live checkpoint from `correction_controller_trainer`:

```text
/aiformula_control/correction_controller_trainer/rpm_weights
```

The message type is `std_msgs/msg/ByteMultiArray`; the payload is a Torch checkpoint containing:

- GRU history model weights
- RPM affine output head
- feature/command column names
- normalization and clamp config
- checkpoint metrics

The model predicts separate affine parameters for each wheel:

```text
right_rpm = a_right * ideal_right_rpm + b_right
left_rpm  = a_left  * ideal_left_rpm  + b_left
```

The left and right wheels have separate `a` and `b` values. That is intentional: the robot may need asymmetric correction.

Live weights stay loaded while the controller remains in state `2`. The RPM history gate is intentionally tolerant: it uses recent samples, resamples/pads them to the model's required history length, and briefly holds the last valid RPM correction through short callback gaps. If no accepted live weights have arrived, state `2` falls back to the ideal RPM path. It does not apply stale weights from state `1`.

## Inputs And Outputs

Subscribed inputs:

- `sub_speed_command`: remapped by launch to gamepad `cmd_vel`
- `/aiformula_sensing/vectornav/velocity_body`: measured forward velocity
- `/aiformula_sensing/gyro_odometry_publisher/odom`: measured yaw rate
- `/aiformula_control/joy_node/joy`: PS4 state switching
- `/aiformula_control/correction_controller_trainer/rpm_weights`: accepted live RPM checkpoints

Published outputs:

- `pub_can`: remapped by launch to the vehicle CAN command topic
- `/aiformula_control/motor_controller/correction_debug`: `std_msgs/msg/Float64MultiArray` debug stream

CAN encoding is unchanged from the BKUP controller:

- CAN ID: `0x210`
- bytes `0..3`: right wheel RPM, signed int32 little-endian
- bytes `4..7`: left wheel RPM, signed int32 little-endian

A full stop command always sends zero RPM to both wheels.

## Safety Limits

`max_command_v` gates input linear velocity before any state-specific controller math. The default is `4.0 m/s`.

No standalone omega clamp is applied in the command gate. This is deliberate because the observed problem is underturning, and clamping yaw would hide the failure mode during correction training.

The RPM model output is still clipped by the checkpoint config, default `max_abs_rpm=650`.

## Debug Data

The debug topic and optional CSV include both body-level and RPM-level fields. Important RPM fields:

- `rpm_weights_loaded`: live checkpoint has been loaded
- `rpm_correction_ready`: weights and recent history are ready for a fresh model prediction
- `rpm_model_applied`: RPM correction was used for this command
- `rpm_model_right_rpm`, `rpm_model_left_rpm`: corrected wheel RPM output
- `rpm_a_right`, `rpm_a_left`, `rpm_b_right`, `rpm_b_left`: current affine parameters
- `rpm_history_len`, `rpm_history_ready`: model history status

The trainer uses this debug stream to verify that online samples are collected only from observed state `2`.

## Related Package

`pid_ws/src/correction_controller_trainer` generates trajectories, collects aligned CSV logs, trains RPM checkpoints online, and publishes accepted live weights to this controller.
