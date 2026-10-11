# semantic_drive_bridge 0.1.0

Two real `rclpy` executables provide native ROS 2 **shadow-only** wiring for
semantic core revision `63eec2da9f9d2ef90e1db29ce2c6c0b3c1a18a96`.
They import installed ROS message types and do not install ROS stubs, extract
classes with AST, launch a model, or use the Simulator's stdio transport.

This package does not yet provide a deployed RGB metric-observation producer,
atomic source-stamped ABC/controller transport, or a real-vehicle control
contract. It does not port the Simulator's current-corridor policy. Those
remaining interfaces are the reason its command output is restricted to
`/semantic_drive/shadow_*`; there is no parameter to enable a live output.

## Signal capture ingress

`signal_capture_bridge` parameters (read-only after startup):

| Parameter | Default | Role |
|---|---|---|
| `input_topic` | `/semantic_drive/signal_capture` | String containing the capture payload below |
| `output_topic` | `/semantic_drive/map_signal` | String containing the unchanged core atomic schema |
| `expected_frame_id` | empty, startup rejected | Exact required source optical-frame name |
| `max_age_sec` | `0.5` | Positive maximum source-to-receiver ROS age |
| `diagnostics_topic` | `/semantic_drive/signal_bridge_status` | JSON diagnostics; never a substitute observation |

Input example (the numbers illustrate the schema, not a current observation):

```json
{"header":{"stamp":{"sec":10,"nanosec":250000000},"frame_id":"zed_left_camera_optical_frame"},"sequence":10250000000,"state":"red","distance_m":12.0,"uncertainty_m":0.2}
```

`sequence` must be an integer exactly equal to the positive source Header stamp
in nanoseconds. The source must emit only one observation for each real capture.
States are exactly `red`, `green`, or `unknown`. Distance is a finite positive
capture-time body-forward depth from the vehicle-front reference, or explicit
`null` when unavailable; valid distance requires finite nonnegative uncertainty.
When distance is null, unused uncertainty (including null) is normalized to zero
as in the core parser. Nonfinite JSON constants are always rejected. Both range
keys are required. A real red with null range remains red and reaches the core's
existing missing-range fault behavior. A real unknown is not changed into red.
Metric calibration and its error bound belong to the upstream producer; the
Simulator's saved heights/front offset are not measured hardware calibration.

At receipt, the bridge computes `age_sec = (receiver_ROS_now - source_stamp)`.
It checks the exact configured frame, integer stamp/sequence, finite values,
strictly newer source stamp, future timestamps, and maximum age. Successful
output is exactly `{sequence,state,age_sec,distance_m,uncertainty_m}`. Errors,
duplicates, and rollback produce diagnostics only: they never generate an
unknown vote or republish cached color/range. The last well-formed source
identity is consumed even if its subsequent age/payload check fails, so fixing
and retransmitting that same capture cannot create a fresh vote. A malformed
future stamp can advance this watermark; restart the complete chain after
correcting the producer instead of relying on counter reset.

Both nodes sample the ROS clock on receipt and on a 20 Hz steady-clock timer.
Any ROS clock rollback latches a fault until process/chain restart. A paused
`/clock` cannot pause the steady watchdog. Source clock and receiver clock must
share the same ROS time domain; this package does not synchronize clocks.
Signal timeouts abstain and allow the core's existing observation timeout to
hold motion. Restart the producer, bridge and semantic state together after a
time reset; restarting only a sequence counter is unsupported.

**Remaining age boundary:** receipt validation includes producer/inference and
DDS delay up to this bridge. The subsequent bridge-to-MapAssist DDS hop is not
included in the published `age_sec`. Core adds elapsed time only after its own
callback receives that JSON. A source-stamped same-executor core ingress would
close that gap; this release does not claim it has done so.

## Shadow command watchdog

`command_watchdog` parameters (read-only after startup):

| Parameter | Default | Role |
|---|---|---|
| `input_topic` | `/semantic_drive/gated_cmd_vel` | Core-gated `geometry_msgs/Twist` |
| `status_topic` | `/semantic_drive/map_status` | Core `std_msgs/String` status input |
| `output_topic` | `/semantic_drive/shadow_cmd_vel` | Shadow-only Twist output |
| `timeout_sec` | `0.25` | Positive steady receipt timeout for command and status separately |
| `diagnostics_topic` | `/semantic_drive/watchdog_status` | Watchdog JSON output |

The output name must remain under `/semantic_drive/shadow_*`. Both configured
name and ROS-resolved `Publisher.topic_name` are checked, including CLI remaps.
`/aiformula_control/game_pad/cmd_vel` and all other non-shadow destinations are
rejected. The invalid publisher is destroyed without publishing any command.

The node publishes at 20 Hz using `STEADY_TIME`. All six Twist components must
be finite. Fresh permitting status is required before accepting a command.
Status is parsed against the real core fields `velocity_gate_reason`,
`traffic_mode`, `avoidance_active`, `path_output_allowed`, and `ros_time_sec`.
Only `forwarding_controller` or `signal_approach_speed_limit` can permit a
command, with an allowed normal/approach/detour path or explicit OA ownership.
Unknown/disabled/hold reasons, missing or malformed status, stale/future status
timestamps, duplicate/out-of-order status, or either receipt timeout produce
zero. ROS and steady clock rollbacks latch zero until restart. Status timestamp
must be positive and increase; its ROS age must also fit `timeout_sec`.

Blocked/malformed/expired input invalidates the cached command; later good
status alone cannot replay it. A new command is required. While both inputs
remain fresh, the latest accepted command is repeated on the 20 Hz shadow
output. `Twist` has no source stamp: this is a **receipt watchdog**, not proof
of image-to-actuator age or ABC/command identity. The core emits status at 5 Hz;
the default 0.25 s limit intentionally leaves only 0.05 s nominal scheduling
headroom. Delayed status can cause conservative shadow zero output.

Clean destruction attempts one zero command. This is best effort, not delivery
acknowledgment. SIGKILL, process death, host/power loss and actuator watchdog
behavior are unverified; a hardware-side command timeout is still required for
physical operation. The private naming check is not protection against an
external relay intentionally forwarding shadow traffic to a vehicle.

All publishers/subscriptions use reliable, volatile KEEP_LAST depth 1. Consumers
must have compatible QoS. No queued-history replay is requested.

## Build and inspect in an actual ROS environment

```sh
colcon build --packages-select semantic_drive_bridge
. install/setup.sh
ros2 run semantic_drive_bridge signal_capture_bridge --ros-args \
  -p expected_frame_id:=zed_left_camera_optical_frame
ros2 run semantic_drive_bridge command_watchdog
```

These commands do not create the signal producer, launch inference, or enable
vehicle control. A separate reviewed shadow launch configures the core and
legacy vision chain. Use the same ROS time domain and unique private publishers.

The package's policy source targets Python >=3.8; the separate bundled A3 model
runtime requires Python >=3.10 and compatible rclpy/cv_bridge binaries. Source
compatibility of this bridge does not make that model Foxy-compatible.

## Verification boundary

`tests/test_policy.py` imports only pure validation policies, with explicit fake
clock *values*. It does not replace `rclpy`, import a fake ROS module, start DDS,
or constitute a ROS runtime acceptance test. Run these tests with this package
on PYTHONPATH or after installing it. They cover source identity and freshness,
actual core status fields, rollback latches, independent steady deadlines,
finite Twist checks, fault recovery requiring new commands, and output bounds.

At implementation time this Windows host has no verified real rclpy/DDS runtime;
its available WSL environment is docker-desktop. Real colcon build, executable
startup, resolved-topic/remap rejection, paused `/clock` behavior, real DDS
delivery and clean shutdown still require verification in the target ROS
environment. Pure policy PASS must be reported separately. Earlier Simulator
four-case success and stdio/shim tests do not validate this new ROS package.
