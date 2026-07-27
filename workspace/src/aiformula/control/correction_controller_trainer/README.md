# correction_controller_trainer

Listens to joystick `cmd_vel` commands, records aligned robot logs, and trains/publishes accepted live RPM correction weights.

This package is the runtime data collector and online trainer for the RPM correction controller. By default, command authority stays with the normal joystick/teleop stack. The trainer only observes the joystick `cmd_vel`, VectorNav, odometry, CAN, and `motor_controller` debug streams. It trains the live RPM model during observed state `2`, writes CSVs for offline analysis, and publishes only accepted checkpoints to `motor_controller`.

Trajectory publishing still exists for repeatable bench tests, but it is opt-in with `command_source:=trajectory`.

Use either entrypoint for the default joystick trainer:

```bash
ros2 run correction_controller_trainer joy_trainer
```

`scripted_trainer` is kept as a backward-compatible alias, but its default `command_source` is now also `joy`.

```text
default: joystick cmd_vel observer/trainer, no command publishing
optional trajectory mode: s0 -> 4 s stop -> s1 -> 4 s stop -> s2 -> 4 s stop
```

Controller states:

- `s0`: ideal differential-drive conversion only.
- `s1`: copied BKUP controller tuning, with correction model loaded but not applied.
- `s2`: live RPM correction when online weights are available; ideal RPM while weights/history warm up.

Outputs are written to a temporary correction-control run folder on the robot desktop. In default joystick mode, CSVs go under `joy/`:

```text
~/Desktop/correctioncontrol_temp/run_YYYYMMDD_HHMMSS/
  online_training_samples.csv
  online_training_history.csv
  run_manifest.json
  online_checkpoint_events.csv
  weights/
    corrected_controller_rpm_latest.pt
    corrected_controller_rpm_final.pt
    corrected_controller_rpm_startpoint.pt
    checkpoints/
      checkpoint_YYYYMMDD_HHMMSS_u000123.pt
  plots/
    tracking_cmd_vs_measured.png
    rpm_target_vs_model.png
    rpm_split.png
    online_loss.png
  artifacts/
    online_summary_metrics.json
  online_training_divergence_events.csv
  joy/
    train_YYYYMMDD_HHMMSS.csv
    log_YYYYMMDD_HHMMSS.csv
```

If `command_source:=trajectory state_sequence:=0,1,2` is used, the run directory contains `s0/`, `s1/`, and `s2/`:

```text
~/Desktop/correctioncontrol_temp/run_YYYYMMDD_HHMMSS/
  s0/
    train_YYYYMMDD_HHMMSS.csv
    log_YYYYMMDD_HHMMSS.csv
  s1/
    train_YYYYMMDD_HHMMSS.csv
    log_YYYYMMDD_HHMMSS.csv
  s2/
    train_YYYYMMDD_HHMMSS.csv
    log_YYYYMMDD_HHMMSS.csv
```

`train_*.csv` matches the existing training pipeline:

```text
timestamp,cmd_v,cmd_omega,vn_body_vx,odom_omega_z
```

`log_*.csv` includes the aligned ideal command, VectorNav velocity, odometry trajectory, CAN payload,
and `motor_controller` debug fields such as base command, applied command, model-corrected command,
`a_v`, `a_omega`, `b_v`, `b_omega`, and whether the correction was applied.

The CSV writer is atomic and runs during the trajectory, so interruption should still leave readable files for the completed portion of the run.

## Online RPM Training

During the run, the trainer also trains a small RPM-space affine correction model:

```text
right_rpm = a_right * ideal_right_rpm + b_right
left_rpm  = a_left  * ideal_left_rpm  + b_left
```

The live training target is not copied from state 1 anymore. It is calculated from tracking error:

```text
target_rpm = current_sent_rpm + adaptation_gain * ideal_rpm(cmd_body - measured_body)
```

where:

- `cmd_body` is the ideal `cmd_vel` being published by this trainer.
- `measured_body` uses VectorNav `linear.x` and odometry `angular.z`.
- `current_sent_rpm` comes from aligned motor-controller debug CAN RPM.

This lets the model learn the RPM command nudge that would reduce `cmd_v - measured_v` and `cmd_omega - measured_omega` while the robot is running.

The generated `weights/corrected_controller_rpm_final.pt` contains the model, config, feature columns, command columns, and target columns. Training keeps running in memory, but `weights/corrected_controller_rpm_latest.pt` is only overwritten when the checkpoint evaluator says the current model is reliable and meaningfully better than the last accepted checkpoint. Retained snapshots go under `weights/checkpoints/` every `online_archive_checkpoint_period_sec` seconds, default `10.0`, with a cap from `online_max_archived_checkpoints`.

`online_checkpoint_events.csv` records every checkpoint evaluation. Accepted rows mean a `.pt` file and live weight topic were updated. Rejected rows keep the reason, such as too few moving samples, not better than the current command, or reliable but not enough improvement over the previous best.

The trainer also publishes the latest accepted weight checkpoint as `std_msgs/ByteMultiArray` on:

```text
/aiformula_control/correction_controller_trainer/rpm_weights
```

The motor controller subscribes to this topic with transient-local QoS. In controller state `2`, it keeps live RPM weights loaded, resamples recent history into the model's fixed input frame, and briefly holds the last valid corrected RPM through short history timing gaps. While weights/history warm up, state `2` uses the ideal RPM path.

Online training is gated to observed motor-controller state `2` by default, and it only trains from rows where motor-controller debug reports that RPM correction was actually applied. If the controller is switched back to state `1`, new online samples and weight publications pause. The motor controller unloads live RPM weights when it leaves state `2` and ignores live weight messages while in state `0` or `1`.

If the model converges and later updates do not improve the checkpoint score, the trainer keeps collecting samples and training internally, but it stops replacing `.pt` files until a new reliable improvement appears. Because checkpoint windows move during a live run, a later candidate can also replace the old checkpoint when it is clearly better than the current sent RPM in the current window and adds a meaningful turn-split boost.

If live training diverges, the trainer writes `online_training_divergence_events.csv`, keeps writing logs, and stops publishing new weights. It does not switch the motor controller to an older fallback path.

## Runtime Contract

The trainer does not publish a separate controller-state topic. It watches the `motor_controller` debug stream and only adds online training samples when the debug row reports `controller_state == 2`.

In default joystick mode, the trainer does not publish `cmd_vel`, does not set `controller_state`, and does not send sine waves. Joystick axes drive the vehicle through the existing teleop path. PS4 buttons still switch state inside `motor_controller`.

Checkpoint acceptance requires:

- enough recent reliable samples
- enough optimizer updates
- model error better than the current sent command
- RPM split and wheel RMSE below configured thresholds
- meaningful improvement over the previous accepted checkpoint, or a strong current-window gain with a stronger turn split

Accepted checkpoints update both:

- `weights/corrected_controller_rpm_latest.pt`
- `/aiformula_control/correction_controller_trainer/rpm_weights`

Rejected checkpoints still create an `online_checkpoint_events.csv` row, but they do not overwrite `.pt` files and do not publish live weights.

Plots can be regenerated later with:

```powershell
ros2 run correction_controller_trainer visualize_online_run ~/Desktop/correctioncontrol_temp/run_YYYYMMDD_HHMMSS
```

A runtime-compatible startpoint can be trained locally from existing log CSVs:

```powershell
python -B pid_ws/src/correction_controller_trainer/correction_controller_trainer/train_rpm_startpoint.py --log-root CorrectionControl/Training/data
```

That produces:

```text
pid_ws/src/correction_controller_trainer/startpoint_weights/run_YYYYMMDD_HHMMSS/
  weights/corrected_controller_rpm_startpoint.pt
  weights/corrected_controller_rpm_final.pt
  weights/corrected_controller_rpm_latest.pt
  weights/checkpoints/
  online_training_samples.csv
  online_training_history.csv
  online_checkpoint_events.csv
  online_training_divergence_events.csv
```

The default test trajectory is:

```text
settle stop for 2 s
straight holds at 1, 2, and 3 m/s
gentle ramps 1 -> 3 m/s and 3 -> 1 m/s
left/right constant turns at 0.8 m/s and +/-0.35 rad/s
left/right circles at 1.0 m/s and +/-0.35 rad/s
two-cycle sine waves at 1.5, 2.0, and 2.5 m/s
variable-speed wave from 1.0 to 3.0 m/s
state stop for 4 s
```

This trajectory is only used when `command_source:=trajectory`. It is intentionally moderate: speeds stay between 1 and 3 m/s during motion, and yaw commands stay at or below about 0.35 rad/s.

Pass `trajectory_json` to override the run.
Supported segment kinds:

- `hold`: constant `v`, `omega`.
- `ramp`: linear interpolation from `start_v/start_omega` to `end_v/end_omega`.
- `sine`: sinusoid around `offset_v/offset_omega` with `amplitude_v/amplitude_omega`.

Parameters:

- `command_source`: `joy` or `trajectory`, default `joy`. `joy` subscribes to joystick `cmd_vel`; `trajectory` publishes scripted commands.
- `state_sequence`: comma-separated or JSON list of states, default `2`.
- `inter_state_stop_sec`: zero-command stop after each state, default `4.0`.
- `final_stop_burst_sec`: emergency shutdown stop if interrupted, default `4.0`.
- `autosave_period_sec`: refresh CSV outputs during a run, default `2.0`.
- `output_root`: default `~/Desktop/correctioncontrol_temp`.
- `state_service_wait_sec`: wait for `/aiformula_control/motor_controller/set_parameters`, default `20.0`.
- `online_rpm_training_enabled`: train and save the online RPM model, default `true`.
- `online_weights_topic`: published live checkpoint topic, default `/aiformula_control/correction_controller_trainer/rpm_weights`.
- `online_train_only_in_state2`: pause online training unless motor-controller debug reports state `2`, default `true`.
- `online_train_only_when_model_applied`: pause online training unless motor-controller debug reports `rpm_model_applied == 1`, default `true`.
- `online_checkpoint_period_sec`: evaluate checkpoint candidates and refresh CSV artifacts, default `5.0`.
- `online_archive_checkpoint_period_sec`: keep one retained accepted checkpoint at this interval, default `10.0`.
- `online_max_archived_checkpoints`: cap retained checkpoints, default `60`.
- `online_initial_weights_path`: startpoint checkpoint to load before the run, default `~/Desktop/correctioncontrol_temp/corrected_controller_rpm_startpoint.pt`. If the file is missing, the trainer warns and starts from identity.
- `online_adaptation_gain`: forward-speed-error RPM target gain, default `0.35`.
- `online_omega_adaptation_gain`: yaw-error RPM split target gain, default `4.5`.
- `online_max_delta_rpm`: maximum per-wheel per-sample target RPM nudge, default `360.0`.
- `online_max_abs_rpm`: target RPM clamp, default `650.0`.
- `online_split_loss_weight`: extra loss on right-left RPM split, default `10.0`.
- `online_omega_error_loss_weight`: extra sample weight for turning/yaw-error rows, default `5.0`.
- `online_divergence_loss_threshold`: halt online weight updates above this loss, default `250000.0`.
- `online_checkpoint_eval_window`: recent replay samples used to score a checkpoint, default `400`.
- `online_checkpoint_min_samples`: minimum reliable samples required before accepting a checkpoint, default `120`.
- `online_checkpoint_min_updates`: minimum optimizer updates before accepting a checkpoint, default `20`.
- `online_checkpoint_min_improvement_rpm`: minimum absolute score improvement before replacing the best checkpoint, default `1.0`.
- `online_checkpoint_min_relative_improvement`: minimum relative score improvement before replacing the best checkpoint, default `0.03`.
- `online_checkpoint_max_split_rmse`: reject checkpoints with right-left split RMSE above this value, default `40.0`.
- `online_checkpoint_max_wheel_rmse`: reject checkpoints with average wheel RMSE above this value, default `90.0`.
- `online_checkpoint_moving_only`: score checkpoints using moving commands only, default `true`.
- `online_checkpoint_min_motion_v`: motion gate for checkpoint scoring, default `0.05`.
- `online_checkpoint_min_motion_omega`: turning gate for checkpoint scoring, default `0.03`.
- `online_checkpoint_min_turn_samples`: minimum turn-error rows required before accepting a checkpoint, default `60`.
- `online_checkpoint_min_omega_error`: minimum yaw error for turn correction acceptance checks, default `0.15`.
- `online_checkpoint_min_turn_split_boost_rpm`: minimum median RPM split boost in the yaw-error direction, default `45.0`.
- `online_checkpoint_min_current_score_gain`: minimum score gain over current sent RPM in the current window, default `15.0`.
- `online_checkpoint_min_current_relative_gain`: minimum relative score gain over current sent RPM in the current window, default `0.20`.
- `online_checkpoint_min_turn_split_boost_improvement_rpm`: minimum extra median turn-split boost needed to replace an older checkpoint when absolute score is not lower, default `20.0`.
- `online_checkpoint_max_turn_actual_over_cmd_p95`: reject checkpoints when recent measured yaw overshoot p95 is too high, default `1.35`.

In joystick mode, the node never publishes `cmd_vel`, including on shutdown. In trajectory mode, it publishes zero `cmd_vel` before shutdown, including on interruption. It also autosaves both CSV files while running and rewrites CSVs atomically, so Ctrl-C or an operator stop should still leave readable outputs for the completed portion of the run.
